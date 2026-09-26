#
# SPDX-License-Identifier: MIT
#

"""DVB-S2 LDPC decoder, normal frames (64800 bits), rates 1/2 and 3/4.

Layered normalized min-sum, one edge per clock, bit-exact with trxd's
reference model (tezuka_fw_simple src/trxd/src/dvbs2/ldpc_fpga.rs):

- checks in DVB-S2's natural groups j, j + q, j + 2q, ... (no variable is
  shared within a group, so the pipeline runs through a group; it drains
  between groups, which the parity chain links);
- a check's info edges: variable g * 360 + ((k - s) mod 360) for each
  (g, s) of group j (ROM), then p(m - 1) (m > 0) and p(m);
- per check a compressed state: min1, min2 (normalized), index, signs;
- pass 1 per edge: R from the state, Q = sat(P - R), running min1/min2,
  sign product; Q into a FIFO. Pass 2, one check behind, per edge:
  R' = n1 or n2 (n = x * 15 / 16) with the sign product of the others,
  P = sat(Q + R'); the check's hard-decision parity counts towards the
  early stop (an iteration with every check satisfied);
- posteriors 8 bits (LLRs in: 6 bits, sign-extended), messages 5 bits.

CPU side (idle only): the posterior RAM is 16200 words of 4 bytes (byte
v % 4 of word v // 4 is variable v): write the LLRs, start, read the signs.
"""

from amaranth import *
from amaranth.lib.fifo import SyncFIFOBuffered
from amaranth.lib.memory import Memory

N = 64800
RATES = {  # rate: (k, q, info edges per check)
    0: (32400, 90, 5),   # 1/2
    1: (48600, 45, 12),  # 3/4
}
P_BITS = 8
M_BITS = 5
IDX_BITS = 4
SIGN_BITS = 16
STATE_BITS = 2 * M_BITS + IDX_BITS + SIGN_BITS


def group_rom(tables):
    """ROM image: for each rate, for each group j, its (g, s) pairs,
    packed as g << 9 | s; the base address of each rate; and for each rate
    and group the most checks pass 1 may have in flight: checks k and k + d
    of a group share a variable when one table row has two addresses in
    the group (d from their shifts), so fewer than d, and at most 6."""
    words, bases, limits = [], [], []
    for rate in sorted(RATES):
        k, q, dci = RATES[rate]
        groups = [[] for _ in range(q)]
        for g, row in enumerate(tables[rate]):
            for a in row:
                groups[a % q].append((g, a // q))
        bases.append(len(words))
        lim = []
        for j in range(q):
            assert len(groups[j]) == dci
            words += [(g << 9) | s for g, s in groups[j]]
            d = 6
            for i1, (g1, s1) in enumerate(groups[j]):
                for g2, s2 in groups[j][i1 + 1:]:
                    if g1 == g2:
                        d = min(d, (s1 - s2) % 360, (s2 - s1) % 360)
            lim.append(max(1, d))
        limits.append(lim)
    # pass 1 reads past a group's last info edge (while on its parity edges)
    words += [0, 0]
    return words, bases, limits


class LdpcDecoder(Elaboratable):
    """Attributes (all in the one clock domain):

    start (in, pulse), rate (in: 0 = 1/2, 1 = 3/4), max_iter (in, 6 bits);
    busy, converged (out), iterations (out, 6 bits);
    cpu_addr (in, 14 bits word), cpu_wdata (in, 32), cpu_we (in, 4 byte
    enables), cpu_re (in), cpu_rdata (out, 32, one cycle after cpu_re).
    """
    def __init__(self, tables):
        self.rom_words, self.rom_bases, self.limits = group_rom(tables)
        self.start = Signal()
        self.rate = Signal()
        self.max_iter = Signal(6, init=50)
        self.busy = Signal()
        self.converged = Signal()
        self.iterations = Signal(6)
        self.cpu_addr = Signal(14)
        self.cpu_wdata = Signal(32)
        self.cpu_we = Signal(4)
        self.cpu_re = Signal()
        self.cpu_rdata = Signal(32)

    def elaborate(self, platform):
        m = Module()
        pmax = (1 << (P_BITS - 1)) - 1
        mmax = (1 << M_BITS) - 1

        def sat(x):
            return Mux(x > pmax, pmax, Mux(x < -pmax, -pmax, x))

        # ---- memories
        m.submodules.post = post = Memory(shape=32, depth=N // 4, init=[])
        prd = post.read_port()
        pwr = post.write_port(granularity=8)
        m.submodules.state = state = Memory(shape=STATE_BITS, depth=32400, init=[])
        srd = state.read_port()
        swr = state.write_port()
        m.submodules.rom = rom = Memory(shape=17, depth=len(self.rom_words),
                                        init=self.rom_words)
        rrd = rom.read_port()

        # ---- configuration of the selected rate
        k_c = Signal(17)
        q_c = Signal(7)
        dci = Signal(4)
        base = Signal(range(len(self.rom_words) + 1))
        with m.If(self.rate):
            m.d.comb += [k_c.eq(RATES[1][0]), q_c.eq(RATES[1][1]),
                         dci.eq(RATES[1][2]), base.eq(self.rom_bases[1])]
        with m.Else():
            m.d.comb += [k_c.eq(RATES[0][0]), q_c.eq(RATES[0][1]),
                         dci.eq(RATES[0][2]), base.eq(self.rom_bases[0])]

        # ---- pass-1 address generator (stage 0)
        run = Signal()          # decoding
        gen = Signal()          # generating edges of the current check
        it = Signal(6)
        j = Signal(7)
        kk = Signal(9)
        mm = Signal(16)         # check index j + kk * q
        e = Signal(4)           # edge within the check
        dc = Signal(5)
        m.d.comb += dc.eq(dci + 2 - (mm == 0))
        unsat = Signal(16)

        # stage 1: ROM word (for info edges) valid; compute the variable
        s1_valid = Signal()
        s1_e = Signal(4)
        s1_kk = Signal(9)
        s1_mm = Signal(16)
        s1_last = Signal()
        s1_first = Signal()
        m.d.comb += rrd.addr.eq(base + j * dci + e)

        # stage 2: variable known, post/state reads issued
        s2_valid = Signal()
        s2_v = Signal(17)
        s2_e = Signal(4)
        s2_last = Signal()
        s2_first = Signal()
        s2_mm = Signal(16)

        # stage 3: post byte and state available
        s3_valid = Signal()
        s3_v = Signal(17)
        s3_e = Signal(4)
        s3_last = Signal()
        s3_first = Signal()
        s3_mm = Signal(16)

        # FIFOs to pass 2
        m.submodules.qfifo = qfifo = SyncFIFOBuffered(width=P_BITS + 17 + 4 + 1, depth=64)
        m.submodules.rfifo = rfifo = SyncFIFOBuffered(
            width=2 * M_BITS + IDX_BITS + 1 + 16, depth=8)

        inflight = Signal(8)    # checks started in pass 1 not finished in pass 2
        # Checks of a group may share a variable after all: a table row with
        # two addresses in one group links checks k and k + d. Pass 1 must
        # not get d checks ahead of pass 2 (per group, from the tables: one
        # group of rate 3/4 has d = 2; everywhere else 6 is safe).
        inflight_max = Signal(3)
        lim_half = Array(C(x, 3) for x in self.limits[0])
        lim_3q = Array(C(x, 3) for x in self.limits[1] + [6] * (len(self.limits[0]) - len(self.limits[1])))
        m.d.comb += inflight_max.eq(Mux(self.rate, lim_3q[j], lim_half[j]))
        start_check = Signal()
        finish_check = Signal()

        # ---- control
        pass2_idle = Signal()
        with m.FSM():
            with m.State('IDLE'):
                m.d.comb += self.busy.eq(0)
                with m.If(self.start):
                    m.d.sync += [run.eq(1), it.eq(1), j.eq(0), kk.eq(0), mm.eq(0),
                                 e.eq(0), unsat.eq(0), self.converged.eq(0)]
                    m.next = 'GROUP'
            with m.State('GROUP'):
                m.d.comb += self.busy.eq(1)
                # Start the group's checks once the previous group is done.
                with m.If((inflight == 0) & pass2_idle & ~s1_valid & ~s2_valid & ~s3_valid):
                    m.d.sync += [kk.eq(0), mm.eq(j), e.eq(0), gen.eq(1)]
                    m.next = 'RUN'
            with m.State('RUN'):
                m.d.comb += self.busy.eq(1)
                with m.If(gen):
                    # room downstream: FIFO space for a whole check plus the pipe
                    with m.If((qfifo.level < 64 - 20) & (inflight < inflight_max)):
                        m.d.comb += start_check.eq(e == 0)
                        with m.If(e == dc - 1):
                            m.d.sync += e.eq(0)
                            with m.If(kk == 359):
                                m.d.sync += gen.eq(0)
                            with m.Else():
                                m.d.sync += [kk.eq(kk + 1), mm.eq(mm + q_c)]
                        with m.Else():
                            m.d.sync += e.eq(e + 1)
                with m.Else():
                    with m.If(j == q_c - 1):
                        m.next = 'ENDIT'
                    with m.Else():
                        m.d.sync += j.eq(j + 1)
                        m.next = 'GROUP'
            with m.State('ENDIT'):
                m.d.comb += self.busy.eq(1)
                with m.If((inflight == 0) & pass2_idle & ~s1_valid & ~s2_valid & ~s3_valid):
                    with m.If(unsat == 0):
                        m.d.sync += [self.converged.eq(1), run.eq(0),
                                     self.iterations.eq(it)]
                        m.next = 'IDLE'
                    with m.Elif(it == self.max_iter):
                        m.d.sync += [run.eq(0), self.iterations.eq(it)]
                        m.next = 'IDLE'
                    with m.Else():
                        m.d.sync += [it.eq(it + 1), j.eq(0), unsat.eq(0)]
                        m.next = 'GROUP'

        issuing = Signal()
        m.d.comb += issuing.eq(gen & (qfifo.level < 64 - 20) & (inflight < inflight_max)
                               & (self.busy) & ~(self.start))
        # stage 0 -> 1
        m.d.sync += [s1_valid.eq(issuing), s1_e.eq(e), s1_kk.eq(kk), s1_mm.eq(mm),
                     s1_last.eq(e == dc - 1), s1_first.eq(e == 0)]
        with m.If(~issuing):
            m.d.sync += s1_valid.eq(0)

        # stage 1 -> 2: the variable
        g = Signal(8)
        sh = Signal(9)
        m.d.comb += [g.eq(rrd.data[9:]), sh.eq(rrd.data[:9])]
        rot = Signal(10)
        m.d.comb += rot.eq(s1_kk + 360 - sh)
        kidx = Signal(9)
        m.d.comb += kidx.eq(Mux(rot >= 360, rot - 360, rot))
        v1 = Signal(17)
        with m.If(s1_e < dci):
            m.d.comb += v1.eq(g * 360 + kidx)
        with m.Elif((s1_e == dci) & (s1_mm != 0)):
            m.d.comb += v1.eq(k_c + s1_mm - 1)
        with m.Else():
            m.d.comb += v1.eq(k_c + s1_mm)
        m.d.sync += [s2_valid.eq(s1_valid), s2_v.eq(v1), s2_e.eq(s1_e),
                     s2_last.eq(s1_last), s2_first.eq(s1_first), s2_mm.eq(s1_mm)]

        # post / state reads (the CPU uses the post ports while idle)
        with m.If(self.busy):
            m.d.comb += [prd.addr.eq(v1[2:]), srd.addr.eq(s1_mm)]
        with m.Else():
            m.d.comb += [prd.addr.eq(self.cpu_addr), self.cpu_rdata.eq(prd.data)]

        # stage 2 -> 3
        m.d.sync += [s3_valid.eq(s2_valid), s3_v.eq(s2_v), s3_e.eq(s2_e),
                     s3_last.eq(s2_last), s3_first.eq(s2_first), s3_mm.eq(s2_mm)]
        # post data for s2's read arrives now (read port: 1 cycle)
        pbyte = Signal(signed(8))
        m.d.comb += pbyte.eq(prd.data.word_select(s2_v[:2], 8))
        p2 = Signal(signed(P_BITS + 1))
        st2 = Signal(STATE_BITS)
        m.d.sync += [p2.eq(pbyte), st2.eq(srd.data)]

        # stage 3: pass 1 arithmetic
        st_hold = Signal(STATE_BITS)
        with m.If(s3_valid & s3_first):
            m.d.sync += st_hold.eq(st2)
        stv = Signal(STATE_BITS)
        m.d.comb += stv.eq(Mux(s3_first, st2, st_hold))
        smin1 = stv[:M_BITS]
        smin2 = stv[M_BITS:2 * M_BITS]
        sidx = stv[2 * M_BITS:2 * M_BITS + IDX_BITS]
        ssigns = stv[2 * M_BITS + IDX_BITS:]
        rmag = Signal(M_BITS)
        m.d.comb += rmag.eq(Mux(it == 1, 0, Mux(s3_e == sidx, smin2, smin1)))
        rsign = Signal()
        m.d.comb += rsign.eq(ssigns.bit_select(s3_e, 1) & (it != 1))
        r_old = Signal(signed(M_BITS + 1))
        m.d.comb += r_old.eq(Mux(rsign, -rmag, rmag))
        qv = Signal(signed(P_BITS + 2))
        m.d.comb += qv.eq(sat(p2 - r_old))
        qa = Signal(P_BITS)
        m.d.comb += qa.eq(Mux(qv < 0, -qv, qv))
        a = Signal(M_BITS)
        m.d.comb += a.eq(Mux(qa > mmax, mmax, qa))
        m1 = Signal(M_BITS)
        m2 = Signal(M_BITS)
        ix = Signal(IDX_BITS)
        sp = Signal()
        c_m1 = Signal(M_BITS)
        c_m2 = Signal(M_BITS)
        c_ix = Signal(IDX_BITS)
        c_sp = Signal()
        m.d.comb += [c_m1.eq(Mux(s3_first, mmax, m1)), c_m2.eq(Mux(s3_first, mmax, m2)),
                     c_ix.eq(Mux(s3_first, 0, ix)), c_sp.eq(Mux(s3_first, 0, sp))]
        n_m1 = Signal(M_BITS)
        n_m2 = Signal(M_BITS)
        n_ix = Signal(IDX_BITS)
        with m.If(a < c_m1):
            m.d.comb += [n_m1.eq(a), n_m2.eq(c_m1), n_ix.eq(s3_e)]
        with m.Elif(a < c_m2):
            m.d.comb += [n_m1.eq(c_m1), n_m2.eq(a), n_ix.eq(c_ix)]
        with m.Else():
            m.d.comb += [n_m1.eq(c_m1), n_m2.eq(c_m2), n_ix.eq(c_ix)]
        n_sp = Signal()
        m.d.comb += n_sp.eq(c_sp ^ (qv < 0))
        with m.If(s3_valid):
            m.d.sync += [m1.eq(n_m1), m2.eq(n_m2), ix.eq(n_ix), sp.eq(n_sp)]
        m.d.comb += [
            qfifo.w_data.eq(Cat(qv[:P_BITS], s3_v, s3_e, s3_last)),
            qfifo.w_en.eq(s3_valid),
        ]

        def nrm(x):
            return (x * 15) >> 4

        n1 = Signal(M_BITS)
        n2 = Signal(M_BITS)
        m.d.comb += [n1.eq(nrm(n_m1)), n2.eq(nrm(n_m2))]
        m.d.comb += [
            rfifo.w_data.eq(Cat(n1, n2, n_ix, n_sp, s3_mm)),
            rfifo.w_en.eq(s3_valid & s3_last),
        ]

        # ---- pass 2
        # The head check's result must be there before its edges run: it is
        # written with the check's last edge, so pass 2 waits for it.
        p2_active = Signal()
        r_n1 = Signal(M_BITS)
        r_n2 = Signal(M_BITS)
        r_ix = Signal(IDX_BITS)
        r_sp = Signal()
        r_mm = Signal(16)
        signs = Signal(SIGN_BITS)
        parity = Signal()
        m.d.comb += pass2_idle.eq(~p2_active & ~rfifo.r_rdy & ~qfifo.r_rdy)
        with m.If(~p2_active & rfifo.r_rdy):
            m.d.comb += rfifo.r_en.eq(1)
            d = rfifo.r_data
            m.d.sync += [r_n1.eq(d[:M_BITS]), r_n2.eq(d[M_BITS:2 * M_BITS]),
                         r_ix.eq(d[2 * M_BITS:2 * M_BITS + IDX_BITS]),
                         r_sp.eq(d[2 * M_BITS + IDX_BITS]),
                         r_mm.eq(d[2 * M_BITS + IDX_BITS + 1:]),
                         signs.eq(0), parity.eq(0), p2_active.eq(1)]
        qd = qfifo.r_data
        q_q = Signal(signed(P_BITS))
        q_v = Signal(17)
        q_e = Signal(4)
        q_last = Signal()
        m.d.comb += [q_q.eq(qd[:P_BITS]), q_v.eq(qd[P_BITS:P_BITS + 17]),
                     q_e.eq(qd[P_BITS + 17:P_BITS + 21]), q_last.eq(qd[P_BITS + 21])]
        sq = Signal()
        m.d.comb += sq.eq(q_q < 0)
        sr = Signal()
        m.d.comb += sr.eq(r_sp ^ sq)
        rmag2 = Signal(M_BITS)
        m.d.comb += rmag2.eq(Mux(q_e == r_ix, r_n2, r_n1))
        rnew = Signal(signed(M_BITS + 1))
        m.d.comb += rnew.eq(Mux(sr, -rmag2, rmag2))
        pnew = Signal(signed(P_BITS + 2))
        m.d.comb += pnew.eq(sat(q_q + rnew))
        do2 = Signal()
        m.d.comb += do2.eq(p2_active & qfifo.r_rdy)
        m.d.comb += qfifo.r_en.eq(do2)
        new_signs = Signal(SIGN_BITS)
        m.d.comb += new_signs.eq(signs | (sr << q_e))
        new_par = Signal()
        m.d.comb += new_par.eq(parity ^ (pnew < 0))
        with m.If(do2):
            m.d.sync += [signs.eq(new_signs), parity.eq(new_par)]
        # posterior write (the CPU writes through the same port while idle)
        with m.If(self.busy):
            m.d.comb += [pwr.addr.eq(q_v[2:]),
                         pwr.data.eq(Cat(*([pnew[:8]] * 4))),
                         pwr.en.eq(Mux(do2, 1 << q_v[:2], 0))]
        with m.Else():
            m.d.comb += [pwr.addr.eq(self.cpu_addr), pwr.data.eq(self.cpu_wdata),
                         pwr.en.eq(self.cpu_we)]
        # check done: its state and the syndrome
        with m.If(do2 & q_last):
            m.d.comb += [swr.addr.eq(r_mm),
                         swr.data.eq(Cat(r_n1, r_n2, r_ix, new_signs)),
                         swr.en.eq(1),
                         finish_check.eq(1)]
            m.d.sync += p2_active.eq(0)
            with m.If(new_par):
                m.d.sync += unsat.eq(unsat + 1)
        m.d.sync += inflight.eq(inflight + start_check - finish_check)
        with m.If(self.start & ~self.busy):
            m.d.sync += inflight.eq(0)
        return m
