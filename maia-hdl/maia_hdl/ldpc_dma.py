#
# SPDX-License-Identifier: MIT
#

"""DDR in and out for the FPGA LDPC decoder (ldpc_axi.py with dma=True):
the CPU writes the LLR words into DDR (0.2 ms for a normal frame, against
3.5 ms through the AXI4-Lite window), the engine loads them into the
decoder's RAM over an AXI3 read master, starts the decoder, and writes the
decisions back packed 32 to a word (bit j of word k: the sign of RAM
variable 32 k + j) over an AXI3 write master (2025 words, against 16200
reads of the RAM).

Ports: in_addr, out_addr (bytes, 128-aligned), in_words (32-bit words,
<= 16200), out_words (a multiple of 32, <= 2048); go (pulse); busy;
dec_* (the decoder's CPU port and start/busy); the AXI master `axi`.

Cells (DVB-T2, QPSK; tezuka_fw_simple trxd dvbt2/stream.rs, bit for bit):
with `cells` the words in are the FEC block's cells after the
deinterleavers, two a word (I the low byte, Q the next, i8), 2 in_words
cells; the engine makes the LLRs itself, word j's from I of cell j and,
with `rot`, Q of cell j + 1 (cyclically) rotated back (c14, s14, Q14):
LLR = clamp(((z >> 7) kq + 2^16) >> 17, +-31), bits 2 j (from the I
axis) and 2 j + 1, and writes each byte where the decoder's layout has
that variable (the info part natural; the parity bit p = c q + r at word
k / 4 + 90 r + c / 4, byte c % 4). `load_only`: no decode (tests).
"""

from amaranth import *
from amaranth.lib.memory import Memory
from amaranth.lib.fifo import SyncFIFOBuffered

from . import axi
from .ldpc_dec import RATES, N

BURST = 16
FIFO = 64


class LdpcDma(Elaboratable):
    def __init__(self):
        self.in_addr = Signal(32)
        self.out_addr = Signal(32)
        self.in_words = Signal(15)
        self.out_words = Signal(12)
        self.go = Signal()
        self.busy = Signal()
        self.dec_addr = Signal(14)
        self.dec_wdata = Signal(32)
        self.dec_we = Signal(4)
        self.dec_re = Signal()
        self.dec_rdata = Signal(32)
        self.dec_start = Signal()
        self.dec_busy = Signal()
        self.dec_rate = Signal()
        self.cells = Signal()
        self.rot = Signal()
        self.load_only = Signal()
        self.kq = Signal(17)
        self.c14 = Signal(signed(16))
        self.s14 = Signal(signed(16))
        # 16QAM: four LLRs a cell (the inner/outer level at a14, Q14) and the
        # column-twist bit deinterleaver
        self.qam16 = Signal()
        self.a14 = Signal(20)
        self.axi = axi.AxiInterface(
            axi.AxiDevice.MANAGER,
            [axi.AxiChannel(axi.AxiDirection.READ, 32, 64, id_bits=6),
             axi.AxiChannel(axi.AxiDirection.WRITE, 32, 64, id_bits=6)],
            axi.AxiVersion.AXI3, name='m_axi')

    def elaborate(self, platform):
        m = Module()
        a = self.axi
        m.d.comb += [a.arid.eq(0), a.arsize.eq(3), a.arlock.eq(0), a.arcache.eq(0b0011),
                     a.arburst.eq(0b01), a.arlen.eq(BURST - 1), a.arprot.eq(0),
                     a.awid.eq(0), a.awsize.eq(3), a.awlock.eq(0), a.awcache.eq(0b0011),
                     a.awburst.eq(0b01), a.awlen.eq(BURST - 1), a.awprot.eq(0),
                     a.wid.eq(0), a.wstrb.eq(0xFF), a.bready.eq(1)]

        # ---- load: bursts into a FIFO, two decoder words a beat
        m.submodules.fifo = fifo = SyncFIFOBuffered(width=64, depth=FIFO)
        r_addr = Signal(32)
        r_left = Signal(14)            # beats still to request
        outstanding = Signal(8)
        ar_pending = Signal()
        loading = Signal()
        room = Signal()
        m.d.comb += room.eq(fifo.level + outstanding + BURST <= FIFO)
        issue = Signal()
        m.d.comb += [issue.eq(loading & ~ar_pending & room & (r_left != 0)),
                     a.araddr.eq(r_addr), a.arvalid.eq(ar_pending),
                     a.rready.eq(fifo.w_rdy), fifo.w_data.eq(a.rdata),
                     fifo.w_en.eq(a.rvalid)]
        got = Signal()
        m.d.comb += got.eq(a.rvalid & a.rready)
        m.d.sync += outstanding.eq(outstanding + Mux(issue, BURST, 0) - got)
        with m.If(issue):
            m.d.sync += ar_pending.eq(1)
        with m.If(ar_pending & a.arready):
            m.d.sync += [ar_pending.eq(0), r_addr.eq(r_addr + BURST * 8),
                         r_left.eq(Mux(r_left > BURST, r_left - BURST, 0))]

        # ---- the decisions, packed, in a 64-bit buffer for the write bursts
        m.submodules.obuf = obuf = Memory(shape=64, depth=1024, init=[])
        ob_rd = obuf.read_port()
        ob_wr = obuf.write_port()

        w = Signal(15)                 # decoder words loaded
        half = Signal()
        k = Signal(12)                 # decision words / beats
        i = Signal(15)                 # RAM words read (store)
        i1 = Signal(15)
        rd_ok = Signal()
        acc = Signal(32)
        lo = Signal(32)
        nb = Signal(4)                 # beat in the burst
        bursts = Signal(8)             # write bursts still unanswered
        aw_pending = Signal()
        wd = Signal(64)
        wvalid = Signal()
        m.d.comb += [a.awaddr.eq(self.out_addr + Cat(C(0, 2), k)), a.awvalid.eq(aw_pending),
                     ob_rd.addr.eq(k[1:]),
                     a.wdata.eq(wd), a.wvalid.eq(wvalid), a.wlast.eq(nb == BURST - 1)]
        with m.If(a.bvalid):
            m.d.sync += bursts.eq(bursts - 1)

        # ---- cells -> LLRs (4 stages) -> the decoder RAM, a byte a cycle
        n_cells = Signal(16)
        m.d.comb += n_cells.eq(Cat(C(0, 1), self.in_words))
        cq = Signal(2)                 # cell in the FIFO's 64-bit word
        cell_av = Signal()
        cell = Signal(16)
        cell_take = Signal()
        m.d.comb += [cell_av.eq(fifo.r_rdy), cell.eq(fifo.r_data.word_select(cq, 16))]
        with m.If(cell_take):
            m.d.sync += cq.eq(cq + 1)
        cur_i = Signal(signed(8))
        cur_q = Signal(signed(8))
        nxt_i = Signal(signed(8))
        nxt_q = Signal(signed(8))
        first_q = Signal(signed(8))
        nq = Signal(signed(8))
        jc = Signal(16)                # cells done
        ph = Signal(2)                 # LLR of the cell: 0 I, 1 Q (16QAM: 2 |I| - a, 3 |Q| - a)
        # the target of variable v (issue stage)
        v = Signal(17)
        pr = Signal(7)
        pc = Signal(11)
        pr90 = Signal(14)
        k_c = Signal(17)
        q_c = Signal(7)
        m.d.comb += [k_c.eq(Mux(self.dec_rate, RATES[1][0], RATES[0][0])),
                     q_c.eq(Mux(self.dec_rate, RATES[1][1], RATES[0][1]))]
        t_addr = Signal(14)
        t_byte = Signal(2)
        with m.If(v < k_c):
            m.d.comb += [t_addr.eq(v[2:]), t_byte.eq(v[:2])]
        with m.Else():
            m.d.comb += [t_addr.eq(k_c[2:] + pr90 + pc[2:]), t_byte.eq(pc[:2])]
        # 16QAM: the column-twist deinterleaver as eight column counters (the
        # codeword position of this row's bit in column e: e 8100 + i_e,
        # i_e = (row - twist[e]) mod 8100), for parity columns as t, s with
        # position - K = 360 t + s: parity bit q s + t, the decoder's layout
        # word K / 4 + 90 t + s / 4, byte s % 4
        ROWS = 8100
        TWIST = [0, 0, 2, 4, 4, 5, 7, 7]
        MUXINV = [7, 1, 3, 5, 2, 4, 6, 0]      # LLR m of a row pair -> column
        col_i = [Signal(13, name=f'col_i{e}') for e in range(8)]
        col_pos = [Signal(17, name=f'col_pos{e}') for e in range(8)]
        col_t = [Signal(8, name=f'col_t{e}') for e in range(8)]
        col_t90 = [Signal(14, name=f'col_t90{e}') for e in range(8)]
        col_s = [Signal(9, name=f'col_s{e}') for e in range(8)]

        def col_consts(rate, e, idx):
            k = RATES[rate][0]
            if idx < k:
                return idx, 0, 0
            t, s_ = divmod(idx - k, 360)
            return idx, t, s_

        col_init = []
        col_wrap = []
        for e in range(8):
            i0 = (-TWIST[e]) % ROWS
            col_init.append([col_consts(r, e, e * ROWS + i0) for r in (0, 1)])
            col_wrap.append([col_consts(r, e, e * ROWS) for r in (0, 1)])

        def col_load(which):
            st = []
            for e in range(8):
                c0, c1 = which[e]
                i_v = (-TWIST[e]) % ROWS if which is col_init else 0
                st += [col_i[e].eq(i_v),
                       col_pos[e].eq(Mux(self.dec_rate, c1[0], c0[0])),
                       col_t[e].eq(Mux(self.dec_rate, c1[1], c0[1])),
                       col_t90[e].eq(Mux(self.dec_rate, 90 * c1[1], 90 * c0[1])),
                       col_s[e].eq(Mux(self.dec_rate, c1[2], c0[2]))]
            return st

        def col_advance():
            st = []
            for e in range(8):
                c0, c1 = col_wrap[e]
                with m.If(col_i[e] == ROWS - 1):
                    st_w = [col_i[e].eq(0),
                            col_pos[e].eq(Mux(self.dec_rate, c1[0], c0[0])),
                            col_t[e].eq(Mux(self.dec_rate, c1[1], c0[1])),
                            col_t90[e].eq(Mux(self.dec_rate, 90 * c1[1], 90 * c0[1])),
                            col_s[e].eq(Mux(self.dec_rate, c1[2], c0[2]))]
                    m.d.sync += st_w
                with m.Else():
                    m.d.sync += [col_i[e].eq(col_i[e] + 1), col_pos[e].eq(col_pos[e] + 1)]
                    with m.If(col_pos[e] < k_c):
                        pass                   # info (the columns never straddle K)
                    with m.Elif(col_s[e] == 359):
                        m.d.sync += [col_s[e].eq(0), col_t[e].eq(col_t[e] + 1),
                                     col_t90[e].eq(col_t90[e] + 90)]
                    with m.Else():
                        m.d.sync += col_s[e].eq(col_s[e] + 1)

        # the column of this LLR (m = 4 h + ph, h the cell in its row pair)
        llr_m = Signal(3)
        m.d.comb += llr_m.eq(Cat(ph, jc[0]))
        col_sel = Signal(3)
        with m.Switch(llr_m):
            for mm in range(8):
                with m.Case(mm):
                    m.d.comb += col_sel.eq(MUXINV[mm])
        q_addr = Signal(14)
        q_byte = Signal(2)
        info_cols = Signal(4)
        m.d.comb += info_cols.eq(Mux(self.dec_rate, RATES[1][0] // ROWS, RATES[0][0] // ROWS))
        with m.Switch(col_sel):
            for e in range(8):
                with m.Case(e):
                    with m.If(e < info_cols):
                        m.d.comb += [q_addr.eq(col_pos[e][2:]), q_byte.eq(col_pos[e][:2])]
                    with m.Else():
                        m.d.comb += [q_addr.eq(k_c[2:] + col_t90[e] + col_s[e][2:]),
                                     q_byte.eq(col_s[e][:2])]
        tgt_addr = Signal(14)
        tgt_byte = Signal(2)
        m.d.comb += [tgt_addr.eq(Mux(self.qam16, q_addr, t_addr)),
                     tgt_byte.eq(Mux(self.qam16, q_byte, t_byte))]

        issue = Signal()
        # stage 1: the rotated axis value (1b: 16QAM |z| - a); 2: x kq;
        # 3: rounded, clamped; 4: write
        a_i = Signal(signed(8))
        a_q = Signal(signed(8))
        m.d.comb += [a_i.eq(cur_i), a_q.eq(Mux(self.rot, nq, cur_q))]
        s1 = Signal()
        s1_z = Signal(signed(26))
        s1_addr = Signal(14)
        s1_byte = Signal(2)
        with m.If(~self.rot):
            m.d.sync += s1_z.eq(Mux(ph[0], a_q, a_i) * 16384)
        with m.Elif(~ph[0]):
            m.d.sync += s1_z.eq(a_i * self.c14 - a_q * self.s14)
        with m.Else():
            m.d.sync += s1_z.eq(a_i * self.s14 + a_q * self.c14)
        s1_outer = Signal()
        m.d.sync += [s1.eq(issue), s1_addr.eq(tgt_addr), s1_byte.eq(tgt_byte),
                     s1_outer.eq(self.qam16 & ph[1])]
        s1b = Signal()
        s1b_z = Signal(signed(27))
        s1b_addr = Signal(14)
        s1b_byte = Signal(2)
        abs_z = Signal(26)
        m.d.comb += abs_z.eq(Mux(s1_z < 0, -s1_z, s1_z))
        m.d.sync += [s1b.eq(s1), s1b_addr.eq(s1_addr), s1b_byte.eq(s1_byte),
                     s1b_z.eq(Mux(s1_outer, abs_z - self.a14, s1_z))]
        s2 = Signal()
        s2_y = Signal(signed(40))
        s2_addr = Signal(14)
        s2_byte = Signal(2)
        m.d.sync += [s2.eq(s1b), s2_y.eq((s1b_z >> 7) * Cat(self.kq, C(0, 1)).as_signed() + 65536),
                     s2_addr.eq(s1b_addr), s2_byte.eq(s1b_byte)]
        s3 = Signal()
        s3_q = Signal(signed(8))
        s3_addr = Signal(14)
        s3_byte = Signal(2)
        y17 = Signal(signed(23))
        m.d.comb += y17.eq(s2_y >> 17)
        m.d.sync += [s3.eq(s2), s3_q.eq(Mux(y17 > 31, 31, Mux(y17 < -31, -31, y17))),
                     s3_addr.eq(s2_addr), s3_byte.eq(s2_byte)]
        llr_pipe = Signal()
        m.d.comb += llr_pipe.eq(s1 | s1b | s2 | s3)
        llr_we = Signal()
        m.d.comb += llr_we.eq(s3)

        with m.FSM():
            with m.State('IDLE'):
                with m.If(self.go):
                    m.d.sync += [loading.eq(1), r_addr.eq(self.in_addr),
                                 r_left.eq(((self.in_words + 31) >> 5) << 4),
                                 w.eq(0), half.eq(0), cq.eq(0), jc.eq(0), ph.eq(0),
                                 v.eq(0), pr.eq(0), pc.eq(0), pr90.eq(0)]
                    with m.If(self.cells):
                        m.next = 'C_FIRST'
                    with m.Else():
                        m.next = 'LOAD'
            # cells: the first one (its Q closes the block's rotation)
            with m.State('C_FIRST'):
                m.d.comb += self.busy.eq(1)
                # (dec_rate is set with go: the column counters from here)
                m.d.sync += col_load(col_init)
                with m.If(cell_av):
                    m.d.comb += [cell_take.eq(1), fifo.r_en.eq(cq == 3)]
                    m.d.sync += [cur_i.eq(cell[:8]), cur_q.eq(cell[8:]), first_q.eq(cell[8:])]
                    m.next = 'C_NEXT'
            # the next cell (the last one's rotation takes the first's Q)
            with m.State('C_NEXT'):
                m.d.comb += self.busy.eq(1)
                with m.If(jc + 1 == n_cells):
                    m.d.sync += nq.eq(first_q)
                    m.next = 'C_LLR'
                with m.Elif(cell_av):
                    m.d.comb += [cell_take.eq(1), fifo.r_en.eq(cq == 3)]
                    m.d.sync += [nxt_i.eq(cell[:8]), nxt_q.eq(cell[8:]), nq.eq(cell[8:])]
                    m.next = 'C_LLR'
            # two LLRs of cell jc into the pipeline, one a cycle
            with m.State('C_LLR'):
                m.d.comb += [self.busy.eq(1), issue.eq(1)]
                m.d.sync += [v.eq(v + 1), ph.eq(ph + 1)]
                with m.If(v >= k_c):
                    with m.If(pr == q_c - 1):
                        m.d.sync += [pr.eq(0), pr90.eq(0), pc.eq(pc + 1)]
                    with m.Else():
                        m.d.sync += [pr.eq(pr + 1), pr90.eq(pr90 + 90)]
                cell_end = Signal()
                m.d.comb += cell_end.eq(Mux(self.qam16, ph == 3, ph == 1))
                with m.If(self.qam16 & (ph == 3) & jc[0]):
                    col_advance()
                with m.If(cell_end):
                    m.d.sync += ph.eq(0)
                    m.d.sync += [cur_i.eq(nxt_i), cur_q.eq(nxt_q), jc.eq(jc + 1)]
                    with m.If(jc + 1 == n_cells):
                        m.next = 'C_DRAIN'
                    with m.Else():
                        m.next = 'C_NEXT'
            with m.State('C_DRAIN'):
                m.d.comb += self.busy.eq(1)
                with m.If(fifo.r_rdy):
                    m.d.comb += fifo.r_en.eq(1)          # past the block: drop
                with m.If(~llr_pipe & (r_left == 0) & ~ar_pending & (outstanding == 0) & ~fifo.r_rdy):
                    m.d.sync += loading.eq(0)
                    m.next = 'C_DONE'
            with m.State('C_DONE'):
                m.d.comb += self.busy.eq(1)
                with m.If(self.load_only):
                    m.next = 'IDLE'
                with m.Else():
                    m.next = 'START'
            with m.State('LOAD'):
                m.d.comb += self.busy.eq(1)
                with m.If(fifo.r_rdy & (w < self.in_words)):
                    m.d.comb += [self.dec_addr.eq(w), self.dec_wdata.eq(fifo.r_data.word_select(half, 32)),
                                 self.dec_we.eq(0xF), fifo.r_en.eq(half)]
                    m.d.sync += [w.eq(w + 1), half.eq(~half)]
                with m.Elif(fifo.r_rdy & (w >= self.in_words)):
                    m.d.comb += fifo.r_en.eq(1)          # past the end: drop
                with m.If((w >= self.in_words) & (r_left == 0) & ~ar_pending & (outstanding == 0)
                          & ~fifo.r_rdy):
                    m.d.sync += loading.eq(0)
                    m.next = 'C_DONE'
            with m.State('START'):
                m.d.comb += [self.busy.eq(1), self.dec_start.eq(1)]
                m.next = 'WAIT0'
            with m.State('WAIT0'):
                m.d.comb += self.busy.eq(1)
                m.next = 'WAIT'
            with m.State('WAIT'):
                m.d.comb += self.busy.eq(1)
                with m.If(~self.dec_busy):
                    m.d.sync += [i.eq(0), k.eq(0), rd_ok.eq(0), acc.eq(0)]
                    m.next = 'STORE'
            # read the RAM a word a cycle (i), the four sign bits of each a
            # cycle later (i1)
            with m.State('STORE'):
                m.d.comb += self.busy.eq(1)
                total = Signal(15)
                m.d.comb += total.eq(Cat(C(0, 3), self.out_words))
                issuing = Signal()
                m.d.comb += issuing.eq(i < total)
                with m.If(issuing):
                    m.d.comb += [self.dec_addr.eq(i), self.dec_re.eq(1)]
                    m.d.sync += i.eq(i + 1)
                m.d.sync += [rd_ok.eq(issuing), i1.eq(i)]
                with m.If(rd_ok):
                    j = Signal(3)
                    m.d.comb += j.eq(i1[:3])
                    d = self.dec_rdata
                    nacc = Signal(32)
                    m.d.comb += nacc.eq(acc | (Cat(d[7], d[15], d[23], d[31]) << Cat(C(0, 2), j)))
                    m.d.sync += acc.eq(Mux(j == 7, 0, nacc))
                    with m.If(j == 7):
                        with m.If(k[0]):
                            m.d.comb += [ob_wr.addr.eq(k[1:]), ob_wr.data.eq(Cat(lo, nacc)), ob_wr.en.eq(1)]
                        with m.Else():
                            m.d.sync += lo.eq(nacc)
                        m.d.sync += k.eq(k + 1)
                with m.If(~issuing & ~rd_ok):
                    m.d.sync += [k.eq(0), nb.eq(0), bursts.eq(0)]
                    m.next = 'AW'
            # the buffer out: a burst of 16 beats (32 words) at a time
            with m.State('AW'):
                m.d.comb += self.busy.eq(1)
                m.d.sync += aw_pending.eq(1)
                m.next = 'AW_WAIT'
            with m.State('AW_WAIT'):
                m.d.comb += self.busy.eq(1)
                with m.If(a.awready):
                    m.d.sync += [aw_pending.eq(0), wd.eq(ob_rd.data), wvalid.eq(1),
                                 bursts.eq(bursts + 1 - a.bvalid)]
                    m.next = 'W'
            with m.State('W'):
                m.d.comb += self.busy.eq(1)
                with m.If(wvalid & a.wready):
                    m.d.sync += [wvalid.eq(0), k.eq(k + 2), nb.eq(nb + 1)]
                    with m.If(nb == BURST - 1):
                        with m.If(k + 2 >= self.out_words):
                            m.next = 'B'
                        with m.Else():
                            m.next = 'AW'
                    with m.Else():
                        m.next = 'W_NEXT'
            with m.State('W_NEXT'):
                m.d.comb += self.busy.eq(1)
                m.next = 'W_LOAD'
            with m.State('W_LOAD'):
                m.d.comb += self.busy.eq(1)
                m.d.sync += [wd.eq(ob_rd.data), wvalid.eq(1)]
                m.next = 'W'
            # every burst answered: the decisions are in DDR
            with m.State('B'):
                m.d.comb += self.busy.eq(1)
                with m.If((bursts == 0) | ((bursts == 1) & a.bvalid)):
                    m.next = 'IDLE'
        with m.If(llr_we):
            m.d.comb += [self.dec_addr.eq(s3_addr),
                         self.dec_wdata.eq(Cat(s3_q, s3_q, s3_q, s3_q)),
                         self.dec_we.eq(C(1, 4) << s3_byte)]
        return m
