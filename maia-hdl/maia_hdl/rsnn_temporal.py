#
# SPDX-License-Identifier: MIT
#

"""The CW-RS keying detector's temporal layers in the FPGA (tezuka_fw_simple
trxd src/rsnn.rs `Stream::feed`, bit for bit): after the front end
(rsnn_front.py) gives a frame's 1x1 outputs, each dilated layer
(kernel 5, dilation d) takes it in and, once the frame 2d later is there,
gives out[t] = in[t] + max(b + sum_k sum_i w[o][k][i] q[t + (k - 2) d][i], 0)
>> 11, q the input clamped to +-32767, sums in 32 bits (wrapping, as i32).

Each layer keeps its last 4d + 1 input frames (i32) in one ring memory
(frames before 0 read as the zeros the reset left). The weights stay in
DDR (the CPU writes them once, [layer][out][tap][in], 16 bits, 128-byte
aligned) and stream in over an AXI3 read port each frame, in the order the
multiplier takes them (one a cycle); biases in block RAM. The CPU reads the
last layer's outputs (a frame's, when there is one) and does the 1x1 to
the logit itself.

Limits: c1 even and <= 64, up to 8 layers, the rings' frames (the sum of
4d + 1) <= 520.
"""

from amaranth import *
from amaranth.lib.memory import Memory
from amaranth.lib.fifo import SyncFIFOBuffered

from . import axi
from .rsnn_front import SplitMem

C1MAX = 64
LMAX = 8
RING_FRAMES = 520
BIAS_DEPTH = LMAX * C1MAX
FIFO_WORDS = 64
BURST = 16


class RsnnTemporal(Elaboratable):
    """Ports (one clock domain):

    n_layers, c1: the configuration; l_waddr, l_dil, l_base, l_len, l_we:
    a layer's dilation, first ring frame and ring length (4 d + 1);
    w_base (bytes, 128-aligned), w_words (64-bit words to fetch a frame);
    b_waddr, b_wdata, b_we: biases ([layer][out], x AQ WQ);
    in_waddr, in_wdata, in_we: the frame's input (the front's outputs);
    go (pulse): a frame in; reset (pulse): clear the rings; busy; valid:
    this frame gave the last layer's outputs (o_raddr -> o_rdata, a cycle
    later); frames (count of those).
    """
    def __init__(self):
        self.n_layers = Signal(4)
        self.c1 = Signal(7, init=C1MAX)
        self.l_waddr = Signal(3)
        self.l_dil = Signal(7)
        self.l_base = Signal(10)
        self.l_len = Signal(10)
        self.l_we = Signal()
        self.l_raddr = Signal(3)
        self.l_rdata = Signal(27)
        self.w_base = Signal(32)
        self.w_words = Signal(16)
        self.b_waddr = Signal(9)
        self.b_wdata = Signal(32)
        self.b_we = Signal()
        self.in_waddr = Signal(6)
        self.in_wdata = Signal(32)
        self.in_we = Signal()
        self.go = Signal()
        self.reset = Signal()
        self.busy = Signal()
        self.valid = Signal()
        self.frames = Signal(16)
        self.o_raddr = Signal(6)
        self.o_rdata = Signal(32)
        self.axi = axi.AxiInterface(
            axi.AxiDevice.MANAGER,
            [axi.AxiChannel(axi.AxiDirection.READ, 32, 64, id_bits=6)],
            axi.AxiVersion.AXI3, name='m_axi')

    def elaborate(self, platform):
        m = Module()
        c1 = self.c1

        # ---- memories
        m.submodules.ring = ring = SplitMem(32, [32768, RING_FRAMES * C1MAX - 32768])
        m.submodules.bmem = bmem = Memory(shape=32, depth=BIAS_DEPTH, init=[])
        b_rd = bmem.read_port()
        b_wr = bmem.write_port()
        m.submodules.inm = inm = Memory(shape=32, depth=C1MAX, init=[])
        in_rd = inm.read_port(domain='comb')
        in_wr = inm.write_port()
        m.submodules.obuf = obuf = Memory(shape=32, depth=C1MAX, init=[])
        ob_rd = obuf.read_port(domain='comb')
        ob_rd2 = obuf.read_port(domain='comb')
        ob_wr = obuf.write_port()
        m.d.comb += [b_wr.addr.eq(self.b_waddr), b_wr.data.eq(self.b_wdata), b_wr.en.eq(self.b_we),
                     in_wr.addr.eq(self.in_waddr), in_wr.data.eq(self.in_wdata),
                     in_wr.en.eq(self.in_we),
                     ob_rd2.addr.eq(self.o_raddr)]
        m.d.sync += self.o_rdata.eq(ob_rd2.data)

        # ---- per-layer configuration and state
        dil = Array(Signal(7, name=f'dil{i}') for i in range(LMAX))
        base = Array(Signal(10, name=f'base{i}') for i in range(LMAX))
        rlen = Array(Signal(10, name=f'rlen{i}') for i in range(LMAX))
        head = Array(Signal(10, name=f'head{i}') for i in range(LMAX))
        count = Array(Signal(9, name=f'count{i}') for i in range(LMAX))
        m.d.comb += self.l_rdata.eq(Cat(dil[self.l_raddr], base[self.l_raddr], rlen[self.l_raddr]))
        with m.If(self.l_we):
            m.d.sync += [dil[self.l_waddr].eq(self.l_dil), base[self.l_waddr].eq(self.l_base),
                         rlen[self.l_waddr].eq(self.l_len)]

        # ---- the weight stream: AXI3 bursts of 16 x 64 bits into a FIFO
        a = self.axi
        m.submodules.wfifo = wfifo = SyncFIFOBuffered(width=64, depth=FIFO_WORDS)
        fetch_on = Signal()
        f_addr = Signal(32)
        f_left = Signal(17)            # words still to request
        outstanding = Signal(8)        # words requested, not yet in
        ar_pending = Signal()
        discard = Signal()
        m.d.comb += [a.arid.eq(0), a.arsize.eq(3), a.arlock.eq(0), a.arcache.eq(0b0011),
                     a.arburst.eq(0b01), a.arlen.eq(BURST - 1), a.arprot.eq(0),
                     a.araddr.eq(f_addr), a.arvalid.eq(ar_pending)]
        room = Signal()
        m.d.comb += room.eq(wfifo.level + outstanding + BURST <= FIFO_WORDS)
        issue = Signal()
        m.d.comb += issue.eq(fetch_on & ~ar_pending & room & (f_left != 0))
        got = Signal()
        m.d.comb += [a.rready.eq(discard | wfifo.w_rdy), got.eq(a.rvalid & a.rready),
                     wfifo.w_data.eq(a.rdata), wfifo.w_en.eq(a.rvalid & ~discard)]
        acc_ar = Signal()
        m.d.comb += acc_ar.eq(ar_pending & a.arready)
        m.d.sync += outstanding.eq(outstanding + Mux(issue, BURST, 0) - got)
        with m.If(issue):
            m.d.sync += ar_pending.eq(1)
        with m.If(acc_ar):
            m.d.sync += [ar_pending.eq(0), f_addr.eq(f_addr + BURST * 8),
                         f_left.eq(Mux(f_left > BURST, f_left - BURST, 0))]

        # a weight a cycle out of the 64-bit words (the low one first)
        sub = Signal(2)
        w_avail = Signal()
        w_take = Signal()
        w_val = Signal(signed(16))
        m.d.comb += [w_avail.eq(wfifo.r_rdy),
                     w_val.eq(wfifo.r_data.word_select(sub, 16)),
                     wfifo.r_en.eq((w_take & (sub == 3)) | (discard & wfifo.r_rdy))]
        with m.If(w_take):
            m.d.sync += sub.eq(sub + 1)

        # ---- the MAC pipeline (issue -> ring read -> clamp -> product -> sum)
        op = Signal()
        s1 = Signal()
        s1_w = Signal(signed(16))
        s2 = Signal()
        s2_q = Signal(signed(16))
        s2_w = Signal(signed(16))
        s3 = Signal()
        s3_p = Signal(signed(32))
        acc = Signal(32)
        m.d.sync += [s1.eq(op), s1_w.eq(w_val), s2.eq(s1), s2_w.eq(s1_w), s3.eq(s2),
                     s3_p.eq(s2_q * s2_w)]
        rv = Signal(signed(32))
        m.d.comb += rv.eq(ring.rdata)
        with m.If(s1):
            m.d.sync += s2_q.eq(Mux(rv > 32767, 32767, Mux(rv < -32767, -32767, rv[:16])))
        with m.If(s3):
            m.d.sync += acc.eq(acc + s3_p)
        pipe_empty = Signal()
        m.d.comb += pipe_empty.eq(~op & ~s1 & ~s2 & ~s3)

        # ---- the sequencer
        l = Signal(3)
        i = Signal(7)
        o = Signal(7)
        k = Signal(3)
        from_obuf = Signal()
        s_new = Signal(10)
        slot = Array(Signal(10, name=f'slot{j}') for j in range(5))
        center = Signal(10)
        res = Signal(32)
        clr = Signal(16)
        valid = Signal()
        m.d.comb += self.valid.eq(valid)
        cur_base = Signal(10)
        cur_len = Signal(10)
        cur_dil = Signal(7)
        cur_head = Signal(10)
        m.d.comb += [cur_base.eq(base[l]), cur_len.eq(rlen[l]), cur_dil.eq(dil[l]),
                     cur_head.eq(head[l])]

        def back(off):
            # the slot `off` frames before the newest (off < ring length)
            v = Signal(11)
            m.d.comb += v.eq(cur_head - off)
            return Mux(cur_head >= off, v[:10], (v + cur_len)[:10])

        with m.FSM(name='temporal'):
            with m.State('IDLE'):
                with m.If(self.reset):
                    m.d.sync += [clr.eq(0), valid.eq(0), self.frames.eq(0)]
                    m.d.sync += [head[j].eq(0) for j in range(LMAX)]
                    m.d.sync += [count[j].eq(0) for j in range(LMAX)]
                    m.next = 'CLEAR'
                with m.Elif(self.go & (self.n_layers != 0)):
                    m.d.sync += [valid.eq(0), l.eq(0), from_obuf.eq(0), fetch_on.eq(1),
                                 f_addr.eq(self.w_base), f_left.eq(self.w_words), sub.eq(0)]
                    m.next = 'PUSH_START'
            with m.State('CLEAR'):
                m.d.comb += self.busy.eq(1)
                m.d.comb += [ring.waddr.eq(clr), ring.wdata.eq(0), ring.we.eq(1)]
                m.d.sync += clr.eq(clr + 1)
                with m.If(clr == RING_FRAMES * C1MAX - 1):
                    m.next = 'IDLE'
            # the frame into layer l's ring (at the slot after the newest)
            with m.State('PUSH_START'):
                m.d.comb += self.busy.eq(1)
                m.d.sync += [s_new.eq(Mux(cur_head + 1 == cur_len, 0, cur_head + 1)), i.eq(0)]
                m.next = 'PUSH'
            with m.State('PUSH'):
                m.d.comb += self.busy.eq(1)
                m.d.comb += [in_rd.addr.eq(i[:6]), ob_rd.addr.eq(i[:6]),
                             ring.waddr.eq(Cat(i[:6], (cur_base + s_new)[:10])),
                             ring.wdata.eq(Mux(from_obuf, ob_rd.data, in_rd.data)), ring.we.eq(1)]
                m.d.sync += i.eq(i + 1)
                with m.If(i == c1 - 1):
                    m.d.sync += [head[l].eq(s_new),
                                 count[l].eq(Mux(count[l] == 511, 511, count[l] + 1))]
                    m.next = 'PUSHED'
            with m.State('PUSHED'):
                m.d.comb += self.busy.eq(1)
                # an output once the frame 2d after it is in
                with m.If(count[l] >= Cat(C(1, 1), cur_dil)):
                    m.d.sync += [slot[j].eq(back((4 - j) * cur_dil)) for j in range(5)]
                    m.d.sync += [center.eq(back(2 * cur_dil)), o.eq(0)]
                    m.next = 'RES_RD'
                with m.Else():
                    m.next = 'FINISH'
            # one output: residual and bias, 5 c1 products, the result
            with m.State('RES_RD'):
                m.d.comb += self.busy.eq(1)
                m.d.comb += [ring.raddr.eq(Cat(o[:6], (cur_base + center)[:10])),
                             b_rd.addr.eq(Cat(o[:6], l))]
                m.next = 'RES_LAT'
            with m.State('RES_LAT'):
                m.d.comb += self.busy.eq(1)
                m.d.sync += [res.eq(ring.rdata), acc.eq(b_rd.data), k.eq(0), i.eq(0)]
                m.next = 'MAC'
            with m.State('MAC'):
                m.d.comb += self.busy.eq(1)
                m.d.comb += ring.raddr.eq(Cat(i[:6], (cur_base + slot[k])[:10]))
                with m.If(w_avail):
                    m.d.comb += [op.eq(1), w_take.eq(1)]
                    m.d.sync += i.eq(i + 1)
                    with m.If(i == c1 - 1):
                        m.d.sync += [i.eq(0), k.eq(k + 1)]
                        with m.If(k == 4):
                            m.next = 'MAC_DRAIN'
            with m.State('MAC_DRAIN'):
                m.d.comb += self.busy.eq(1)
                with m.If(pipe_empty):
                    m.next = 'OUT'
            with m.State('OUT'):
                m.d.comb += self.busy.eq(1)
                relu = Signal(32)
                m.d.comb += relu.eq(Mux(acc[31], 0, Cat(acc[11:31], C(0, 12))))
                m.d.comb += [ob_wr.addr.eq(o[:6]), ob_wr.data.eq(res + relu), ob_wr.en.eq(1)]
                m.d.sync += o.eq(o + 1)
                m.next = 'RES_RD'
                with m.If(o == c1 - 1):
                    with m.If(l + 1 == self.n_layers):
                        m.d.sync += [valid.eq(1), self.frames.eq(self.frames + 1)]
                        m.next = 'FINISH'
                    with m.Else():
                        m.d.sync += [l.eq(l + 1), from_obuf.eq(1)]
                        m.next = 'PUSH_START'
            # stop the weight stream: drop what is still coming, empty the FIFO
            with m.State('FINISH'):
                m.d.comb += self.busy.eq(1)
                m.d.sync += fetch_on.eq(0)
                with m.If(~ar_pending):
                    m.d.sync += discard.eq(1)
                    m.next = 'DRAIN'
            with m.State('DRAIN'):
                m.d.comb += self.busy.eq(1)
                with m.If((outstanding == 0) & ~wfifo.r_rdy & ~ar_pending):
                    m.d.sync += discard.eq(0)
                    m.next = 'IDLE'
        return m
