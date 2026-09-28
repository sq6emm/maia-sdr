#
# SPDX-License-Identifier: MIT
#

"""DVB-S2 LDPC decoder, normal frames (64800 bits), rates 1/2 and 3/4, four
checks at a time: the schedule and arithmetic of ldpc_dec.py (layered
normalized min-sum, bit-exact with trxd's model dvbs2/ldpc_fpga.rs), four
lanes wide, about four times faster.

Checks of a group (j, j + q, j + 2q, ...) share no variable, apart from a
few pairs k, k + d (one table row with two addresses in the group; d >= 11
but in one group of rate 3/4, d = 2), so their order does not matter and
four consecutive ones (k = 4K .. 4K + 3, lane l = k mod 4) run together:

- info edge (g, s) of check k reads variable g 360 + ((k - s) mod 360):
  four consecutive positions of row g, so one from each of four banks
  (bank = variable mod 4, address = variable / 4), rotated by -s mod 4;
- parity p = k q + j (and p - 1): parity bit p = c q + r lives in bank
  c mod 4 at address K/4 + r 90 + c / 4 (the parity as q rows of 360
  columns): for four consecutive checks, four consecutive columns of one
  row (p - 1 of group 0 is row q - 1, one column back: rotated by 3);
- a check's state in bank l at address j 90 + K.

The group with d = 2 runs two checks at a time; batches of a group with
links stay far enough apart (pass 1 of a batch waits for pass 2 of the one
d checks back), as the serial decoder does per check.

CPU side (idle only): 16200 words of 4 bytes, byte b of word w in bank b at
address w. Info variable v: word v / 4, byte v % 4 (as ldpc_dec.py). Parity
bit p = c q + r: word K / 4 + r 90 + c / 4, byte c % 4 (write the parity
LLRs in that order; trxd's fpga_ldpc does).
"""

from amaranth import *
from amaranth.lib.fifo import SyncFIFOBuffered
from amaranth.lib.memory import Memory

from .ldpc_dec import N, RATES, P_BITS, M_BITS, IDX_BITS, SIGN_BITS, STATE_BITS

L = 4           # lanes
W = 360 // L    # words of a row in a bank
SDEPTH = 90 * W  # state words a bank (rate 1/2: q = 90)


def group_rom4(tables):
    """ROM image: for each rate and group j its (g, s) pairs as
    (g 90) << 9 | s; each rate's base; per rate and group the batch width
    (4, or 2 where d < 4) and how many batches may be in flight (pass 1 of
    batch b + n waits for pass 2 of batch b when a link d spans n batches;
    at most 7)."""
    words, bases, widths, limits = [], [], [], []
    for rate in sorted(RATES):
        k, q, dci = RATES[rate]
        groups = [[] for _ in range(q)]
        for g, row in enumerate(tables[rate]):
            for a in row:
                groups[a % q].append((g, a // q))
        bases.append(len(words))
        wd, lim = [], []
        for j in range(q):
            assert len(groups[j]) == dci
            words += [((g * W) << 9) | s for g, s in groups[j]]
            d = 360
            for i1, (g1, s1) in enumerate(groups[j]):
                for g2, s2 in groups[j][i1 + 1:]:
                    if g1 == g2:
                        d = min(d, (s1 - s2) % 360, (s2 - s1) % 360)
            w = L if d >= L else 2 if d >= 2 else 1
            # checks c and c + d: batches c // w and (c + d) // w, at
            # least n = d // w apart when w divides... take the minimum
            n = min((c + d) // w - c // w for c in range(w))
            wd.append(w)
            lim.append(max(1, min(7, n)))
        widths.append(wd)
        limits.append(lim)
    words += [0, 0]
    return words, bases, widths, limits


def cpu_layout(rate):
    """For each variable v, its CPU word and byte (tests; trxd does the
    same): the info part natural, the parity as described above."""
    k, q, _ = RATES[rate]
    word = [0] * N
    byte = [0] * N
    for v in range(k):
        word[v], byte[v] = v // 4, v % 4
    for p in range(N - k):
        c, r = p // q, p % q
        word[k + p] = k // 4 + r * W + c // 4
        byte[k + p] = c % 4
    return word, byte


class LdpcDecoder4(Elaboratable):
    """The interface of ldpc_dec.LdpcDecoder."""
    def __init__(self, tables):
        (self.rom_words, self.rom_bases, self.widths,
         self.limits) = group_rom4(tables)
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
        prd, pwr, srd, swr = [], [], [], []
        for b in range(L):
            post = Memory(shape=8, depth=N // 4, init=[])
            m.submodules[f'post{b}'] = post
            prd.append(post.read_port())
            pwr.append(post.write_port())
            st = Memory(shape=STATE_BITS, depth=SDEPTH, init=[])
            m.submodules[f'state{b}'] = st
            srd.append(st.read_port())
            swr.append(st.write_port())
        m.submodules.rom = rom = Memory(shape=23, depth=len(self.rom_words),
                                        init=self.rom_words)
        rrd = rom.read_port()

        # ---- configuration of the selected rate
        k4_c = Signal(14)       # k / 4: the parity's first word
        q_c = Signal(7)
        dci = Signal(4)
        base = Signal(range(len(self.rom_words) + 1))
        with m.If(self.rate):
            m.d.comb += [k4_c.eq(RATES[1][0] // 4), q_c.eq(RATES[1][1]),
                         dci.eq(RATES[1][2]), base.eq(self.rom_bases[1])]
        with m.Else():
            m.d.comb += [k4_c.eq(RATES[0][0] // 4), q_c.eq(RATES[0][1]),
                         dci.eq(RATES[0][2]), base.eq(self.rom_bases[0])]

        # ---- pass-1 generator (stage 0): batch (K, first lane), edge e
        gen = Signal()
        it = Signal(6)
        j = Signal(7)
        kk = Signal(9)          # the batch's first check (multiple of its width)
        e = Signal(4)
        dc = Signal(5)
        m.d.comb += dc.eq(dci + 2)
        unsat = Signal(16)
        width = Signal(3)
        wid_half = Array(C(x, 3) for x in self.widths[0])
        wid_3q = Array(C(x, 3) for x in self.widths[1] + [L] * (len(self.widths[0]) - len(self.widths[1])))
        m.d.comb += width.eq(Mux(self.rate, wid_3q[j], wid_half[j]))
        inflight_max = Signal(3)
        lim_half = Array(C(x, 3) for x in self.limits[0])
        lim_3q = Array(C(x, 3) for x in self.limits[1] + [7] * (len(self.limits[0]) - len(self.limits[1])))
        m.d.comb += inflight_max.eq(Mux(self.rate, lim_3q[j], lim_half[j]))

        # lanes of the batch: lane l = check mod 4; active when in the batch
        # (width 2: kk mod 4 is 0 or 2); the check of lane l is 4K + l
        act = Signal(L)
        with m.Switch(width):
            with m.Case(1):
                m.d.comb += act.eq(1 << kk[:2])
            with m.Case(2):
                m.d.comb += act.eq(Mux(kk[1], 0b1100, 0b0011))
            with m.Default():
                m.d.comb += act.eq(0b1111)
        kq = Signal(7)          # K = kk / 4
        m.d.comb += kq.eq(kk[2:])

        # stage 1: ROM word valid
        s1_valid = Signal()
        s1_e = Signal(4)
        s1_K = Signal(7)
        s1_j = Signal(7)
        s1_act = Signal(L)
        s1_last = Signal()
        s1_first = Signal()
        m.d.comb += rrd.addr.eq(base + j * dci + e)

        # stage 2: addresses issued
        s2_valid = Signal()
        s2_e = Signal(4)
        s2_act = Signal(L)      # lanes with this edge (mm = 0 has no p(mm - 1))
        s2_cact = Signal(L)     # lanes with this check
        s2_rot = Signal(2)
        s2_addr = [Signal(14, name=f's2_addr{l}') for l in range(L)]
        s2_last = Signal()
        s2_first = Signal()
        s2_saddr = Signal(13)

        # stage 3: data available
        s3_valid = Signal()
        s3_e = Signal(4)
        s3_act = Signal(L)
        s3_cact = Signal(L)
        s3_rot = Signal(2)
        s3_addr = [Signal(14, name=f's3_addr{l}') for l in range(L)]
        s3_last = Signal()
        s3_first = Signal()
        s3_saddr = Signal(13)

        # FIFOs to pass 2: per edge the four Q values, lane addresses, rotation,
        # lanes, edge index, last; per batch the four check results
        QW = L * (P_BITS + 14) + 2 + L + 4 + 1
        m.submodules.qfifo = qfifo = SyncFIFOBuffered(width=QW, depth=64)
        RW = L * (2 * M_BITS + IDX_BITS + 1) + 13 + L
        m.submodules.rfifo = rfifo = SyncFIFOBuffered(width=RW, depth=8)

        inflight = Signal(4)    # batches started in pass 1, not done in pass 2
        start_batch = Signal()
        finish_batch = Signal()

        # A new batch waits for the in-flight limit (its own later edges
        # must not: with a limit of 1 it would wait for itself).
        may_issue = Signal()
        m.d.comb += may_issue.eq((qfifo.level < 64 - 20) & ((e != 0) | (inflight < inflight_max)))

        # ---- control
        fails = Signal(L)
        m.d.sync += unsat.eq(unsat + fails[0] + fails[1] + fails[2] + fails[3])
        pass2_idle = Signal()
        drained = Signal()
        s4_valid = Signal()
        rf_pending = Signal()
        w_pending = Signal()
        m.d.comb += drained.eq((inflight == 0) & pass2_idle & ~s1_valid & ~s2_valid & ~s3_valid
                               & ~s4_valid & ~rf_pending & ~w_pending & (fails == 0))
        with m.FSM():
            with m.State('IDLE'):
                m.d.comb += self.busy.eq(0)
                with m.If(self.start):
                    m.d.sync += [it.eq(1), j.eq(0), kk.eq(0), e.eq(0), unsat.eq(0),
                                 self.converged.eq(0)]
                    m.next = 'GROUP'
            with m.State('GROUP'):
                m.d.comb += self.busy.eq(1)
                with m.If(drained):
                    m.d.sync += [kk.eq(0), e.eq(0), gen.eq(1)]
                    m.next = 'RUN'
            with m.State('RUN'):
                m.d.comb += self.busy.eq(1)
                with m.If(gen):
                    with m.If(may_issue):
                        m.d.comb += start_batch.eq(e == 0)
                        with m.If(e == dc - 1):
                            m.d.sync += e.eq(0)
                            with m.If(kk + width >= 360):
                                m.d.sync += gen.eq(0)
                            with m.Else():
                                m.d.sync += kk.eq(kk + width)
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
                with m.If(drained):
                    with m.If(unsat == 0):
                        m.d.sync += [self.converged.eq(1), self.iterations.eq(it)]
                        m.next = 'IDLE'
                    with m.Elif(it == self.max_iter):
                        m.d.sync += self.iterations.eq(it)
                        m.next = 'IDLE'
                    with m.Else():
                        m.d.sync += [it.eq(it + 1), j.eq(0), unsat.eq(0)]
                        m.next = 'GROUP'

        issuing = Signal()
        m.d.comb += issuing.eq(gen & may_issue & self.busy & ~self.start)
        # stage 0 -> 1
        m.d.sync += [s1_valid.eq(issuing), s1_e.eq(e), s1_K.eq(kq), s1_j.eq(j),
                     s1_act.eq(act), s1_last.eq(e == dc - 1), s1_first.eq(e == 0)]

        # stage 1 -> 2: the lanes' addresses and the rotation
        g90 = Signal(14)
        sh = Signal(9)
        m.d.comb += [g90.eq(rrd.data[9:]), sh.eq(rrd.data[:9])]
        # info: position of lane 0: t = (4K - s) mod 360; lane l at t + l
        t0 = Signal(10)
        m.d.comb += t0.eq(Cat(C(0, 2), s1_K) + 360 - sh)
        t = Signal(9)
        m.d.comb += t.eq(Mux(t0 >= 360, t0 - 360, t0))
        rot = Signal(2)
        addr = [Signal(14, name=f's1_addr{l}') for l in range(L)]
        eact = Signal(L)
        m.d.comb += eact.eq(s1_act)
        jm1 = Signal(7)
        m.d.comb += jm1.eq(s1_j - 1)
        with m.If(s1_e < dci):
            # lane l reads bank (l - s) mod 4 = l + rot
            m.d.comb += rot.eq(t[:2])
            for l in range(L):
                pl = Signal(10, name=f'pos{l}')
                m.d.comb += pl.eq(t + l)
                pw = Signal(9, name=f'posw{l}')
                m.d.comb += pw.eq(Mux(pl >= 360, pl - 360, pl))
                m.d.comb += addr[l].eq(g90 + pw[2:])
        with m.Elif((s1_e == dci) & (s1_j != 0)):
            # p(mm - 1), row j - 1, column 4K + l
            m.d.comb += rot.eq(0)
            for l in range(L):
                m.d.comb += addr[l].eq(k4_c + jm1 * W + s1_K)
        with m.Elif(s1_e == dci):
            # group 0: p(mm - 1) is row q - 1, column 4K + l - 1 (bank l - 1);
            # check 0 (lane 0 of K = 0) has none
            m.d.comb += rot.eq(3)
            for l in range(L):
                if l == 0:
                    m.d.comb += addr[l].eq(k4_c + (q_c - 1) * W + s1_K - 1)
                else:
                    m.d.comb += addr[l].eq(k4_c + (q_c - 1) * W + s1_K)
            with m.If(s1_K == 0):
                m.d.comb += eact.eq(s1_act & 0b1110)
        with m.Else():
            # p(mm): row j, column 4K + l
            m.d.comb += rot.eq(0)
            for l in range(L):
                m.d.comb += addr[l].eq(k4_c + s1_j * W + s1_K)
        m.d.sync += [s2_valid.eq(s1_valid), s2_e.eq(s1_e), s2_act.eq(eact),
                     s2_cact.eq(s1_act), s2_rot.eq(rot),
                     s2_last.eq(s1_last), s2_first.eq(s1_first),
                     s2_saddr.eq(s1_j * W + s1_K)]
        m.d.sync += [s2_addr[l].eq(addr[l]) for l in range(L)]

        # reads: bank b serves lane (b - rot) mod 4 (the CPU while idle)
        def lane_of(b, r):
            return Array(C((b - x) % L, 2) for x in range(L))[r]

        def bank_of(l, r):
            return Array(C((l + x) % L, 2) for x in range(L))[r]

        # (from the stage-2 registers: the ROM -> address -> RAM path in one
        # cycle missed 100 MHz)
        for b in range(L):
            with m.If(self.busy):
                la = Array(s2_addr)
                m.d.comb += prd[b].addr.eq(la[lane_of(b, s2_rot)])
            with m.Else():
                m.d.comb += prd[b].addr.eq(self.cpu_addr)
            m.d.comb += srd[b].addr.eq(s2_saddr)
        m.d.comb += self.cpu_rdata.eq(Cat(*[prd[b].data for b in range(L)]))

        # stage 2 -> 3
        m.d.sync += [s3_valid.eq(s2_valid), s3_e.eq(s2_e), s3_act.eq(s2_act),
                     s3_cact.eq(s2_cact), s3_rot.eq(s2_rot),
                     s3_last.eq(s2_last), s3_first.eq(s2_first), s3_saddr.eq(s2_saddr)]
        m.d.sync += [s3_addr[l].eq(s2_addr[l]) for l in range(L)]
        # the banks' bytes for s2's reads arrive now (stage 3): to lanes
        banks_rd = Array(prd[b].data for b in range(L))
        p2 = [Signal(signed(P_BITS + 1), name=f'p2_{l}') for l in range(L)]
        st2 = [Signal(STATE_BITS, name=f'st2_{l}') for l in range(L)]
        for l in range(L):
            byte = Signal(signed(8), name=f'pbyte{l}')
            m.d.comb += byte.eq(banks_rd[bank_of(l, s3_rot)])
            m.d.sync += [p2[l].eq(byte), st2[l].eq(srd[l].data)]
        # stage 3 -> 4
        s4_e = Signal(4)
        s4_act = Signal(L)
        s4_cact = Signal(L)
        s4_rot = Signal(2)
        s4_addr = [Signal(14, name=f's4_addr{l}') for l in range(L)]
        s4_last = Signal()
        s4_first = Signal()
        s4_saddr = Signal(13)
        m.d.sync += [s4_valid.eq(s3_valid), s4_e.eq(s3_e), s4_act.eq(s3_act),
                     s4_cact.eq(s3_cact), s4_rot.eq(s3_rot),
                     s4_last.eq(s3_last), s4_first.eq(s3_first), s4_saddr.eq(s3_saddr)]
        m.d.sync += [s4_addr[l].eq(s3_addr[l]) for l in range(L)]

        # stage 4: pass 1, per lane
        qvs, n_out = [], []
        for l in range(L):
            st_hold = Signal(STATE_BITS, name=f'st_hold{l}')
            with m.If(s4_valid & s4_first):
                m.d.sync += st_hold.eq(st2[l])
            stv = Signal(STATE_BITS, name=f'stv{l}')
            m.d.comb += stv.eq(Mux(s4_first, st2[l], st_hold))
            smin1 = stv[:M_BITS]
            smin2 = stv[M_BITS:2 * M_BITS]
            sidx = stv[2 * M_BITS:2 * M_BITS + IDX_BITS]
            ssigns = stv[2 * M_BITS + IDX_BITS:]
            rmag = Signal(M_BITS, name=f'rmag{l}')
            m.d.comb += rmag.eq(Mux(it == 1, 0, Mux(s4_e == sidx, smin2, smin1)))
            rsign = Signal(name=f'rsign{l}')
            m.d.comb += rsign.eq(ssigns.bit_select(s4_e, 1) & (it != 1))
            r_old = Signal(signed(M_BITS + 1), name=f'r_old{l}')
            m.d.comb += r_old.eq(Mux(rsign, -rmag, rmag))
            qv = Signal(signed(P_BITS + 2), name=f'qv{l}')
            m.d.comb += qv.eq(sat(p2[l] - r_old))
            qa = Signal(P_BITS, name=f'qa{l}')
            m.d.comb += qa.eq(Mux(qv < 0, -qv, qv))
            a = Signal(M_BITS, name=f'a{l}')
            m.d.comb += a.eq(Mux(qa > mmax, mmax, qa))
            m1 = Signal(M_BITS, name=f'm1_{l}')
            m2 = Signal(M_BITS, name=f'm2_{l}')
            ix = Signal(IDX_BITS, name=f'ix{l}')
            sp = Signal(name=f'sp{l}')
            c_m1 = Signal(M_BITS, name=f'c_m1_{l}')
            c_m2 = Signal(M_BITS, name=f'c_m2_{l}')
            c_ix = Signal(IDX_BITS, name=f'c_ix{l}')
            c_sp = Signal(name=f'c_sp{l}')
            m.d.comb += [c_m1.eq(Mux(s4_first, mmax, m1)), c_m2.eq(Mux(s4_first, mmax, m2)),
                         c_ix.eq(Mux(s4_first, 0, ix)), c_sp.eq(Mux(s4_first, 0, sp))]
            n_m1 = Signal(M_BITS, name=f'n_m1_{l}')
            n_m2 = Signal(M_BITS, name=f'n_m2_{l}')
            n_ix = Signal(IDX_BITS, name=f'n_ix{l}')
            n_sp = Signal(name=f'n_sp{l}')
            with m.If(~s4_act[l]):
                # no edge here (check 0's missing p(-1)): state unchanged
                m.d.comb += [n_m1.eq(c_m1), n_m2.eq(c_m2), n_ix.eq(c_ix), n_sp.eq(c_sp)]
            with m.Elif(a < c_m1):
                m.d.comb += [n_m1.eq(a), n_m2.eq(c_m1), n_ix.eq(s4_e),
                             n_sp.eq(c_sp ^ (qv < 0))]
            with m.Elif(a < c_m2):
                m.d.comb += [n_m1.eq(c_m1), n_m2.eq(a), n_ix.eq(c_ix),
                             n_sp.eq(c_sp ^ (qv < 0))]
            with m.Else():
                m.d.comb += [n_m1.eq(c_m1), n_m2.eq(c_m2), n_ix.eq(c_ix),
                             n_sp.eq(c_sp ^ (qv < 0))]
            with m.If(s4_valid):
                m.d.sync += [m1.eq(n_m1), m2.eq(n_m2), ix.eq(n_ix), sp.eq(n_sp)]
            qvs.append(qv)

            def nrm(x):
                return (x * 15) >> 4

            n1 = Signal(M_BITS, name=f'n1_{l}')
            n2 = Signal(M_BITS, name=f'n2_{l}')
            m.d.comb += [n1.eq(nrm(n_m1)), n2.eq(nrm(n_m2))]
            n_out.append(Cat(n1, n2, n_ix, n_sp))
        m.d.comb += [
            qfifo.w_data.eq(Cat(*[qvs[l][:P_BITS] for l in range(L)],
                                *[s4_addr[l] for l in range(L)],
                                s4_rot, s4_act, s4_e, s4_last)),
            qfifo.w_en.eq(s4_valid),
        ]
        # (registered: the pass-1 compare -> normalize -> FIFO path was long)
        rf_data = Signal(RW)
        m.d.sync += [rf_data.eq(Cat(*n_out, s4_saddr, s4_cact)),
                     rf_pending.eq(s4_valid & s4_last)]
        m.d.comb += [rfifo.w_data.eq(rf_data), rfifo.w_en.eq(rf_pending)]

        # ---- pass 2
        RL = 2 * M_BITS + IDX_BITS + 1
        p2_active = Signal()
        r_res = Signal(L * RL)
        r_saddr = Signal(13)
        r_cact = Signal(L)
        signs = [Signal(SIGN_BITS, name=f'signs{l}') for l in range(L)]
        parity = Signal(L)
        m.d.comb += pass2_idle.eq(~p2_active & ~rfifo.r_rdy & ~qfifo.r_rdy)
        with m.If(~p2_active & rfifo.r_rdy):
            m.d.comb += rfifo.r_en.eq(1)
            d = rfifo.r_data
            m.d.sync += [r_res.eq(d[:L * RL]), r_saddr.eq(d[L * RL:L * RL + 13]),
                         r_cact.eq(d[L * RL + 13:]), parity.eq(0), p2_active.eq(1)]
            m.d.sync += [signs[l].eq(0) for l in range(L)]
        qd = qfifo.r_data
        o = 0
        q_q = []
        for l in range(L):
            x = Signal(signed(P_BITS), name=f'q_q{l}')
            m.d.comb += x.eq(qd[o:o + P_BITS])
            q_q.append(x)
            o += P_BITS
        q_addr = []
        for l in range(L):
            x = Signal(14, name=f'q_addr{l}')
            m.d.comb += x.eq(qd[o:o + 14])
            q_addr.append(x)
            o += 14
        q_rot = Signal(2)
        q_act = Signal(L)
        q_e = Signal(4)
        q_last = Signal()
        m.d.comb += [q_rot.eq(qd[o:o + 2]), q_act.eq(qd[o + 2:o + 2 + L]),
                     q_e.eq(qd[o + 2 + L:o + 6 + L]), q_last.eq(qd[o + 6 + L])]
        do2 = Signal()
        m.d.comb += do2.eq(p2_active & qfifo.r_rdy)
        m.d.comb += qfifo.r_en.eq(do2)
        pnew = []
        new_signs = []
        new_par = Signal(L)
        for l in range(L):
            res = r_res[l * RL:(l + 1) * RL]
            r_n1 = res[:M_BITS]
            r_n2 = res[M_BITS:2 * M_BITS]
            r_ix = res[2 * M_BITS:2 * M_BITS + IDX_BITS]
            r_sp = res[2 * M_BITS + IDX_BITS]
            sq = Signal(name=f'sq{l}')
            m.d.comb += sq.eq(q_q[l] < 0)
            sr = Signal(name=f'sr{l}')
            m.d.comb += sr.eq(r_sp ^ sq)
            rmag2 = Signal(M_BITS, name=f'rmag2_{l}')
            m.d.comb += rmag2.eq(Mux(q_e == r_ix, r_n2, r_n1))
            rnew = Signal(signed(M_BITS + 1), name=f'rnew{l}')
            m.d.comb += rnew.eq(Mux(sr, -rmag2, rmag2))
            pn = Signal(signed(P_BITS + 2), name=f'pnew{l}')
            m.d.comb += pn.eq(sat(q_q[l] + rnew))
            pnew.append(pn)
            ns = Signal(SIGN_BITS, name=f'new_signs{l}')
            m.d.comb += ns.eq(Mux(q_act[l], signs[l] | (sr << q_e), signs[l]))
            new_signs.append(ns)
            m.d.comb += new_par[l].eq(parity[l] ^ (q_act[l] & (pn < 0)))
            with m.If(do2):
                m.d.sync += signs[l].eq(ns)
        with m.If(do2):
            m.d.sync += parity.eq(new_par)
        # posterior writes: lane l to bank l + rot (the CPU while idle)
        lane_addr = Array(q_addr)
        lane_data = Array(pn[:8] for pn in pnew)
        lane_act = Array(q_act[l] for l in range(L))
        # (registered: pass 2 -> rotation -> RAM in one cycle missed 100 MHz;
        # nothing reads a variable in the cycle after its write: checks of a
        # group share none, the next group waits for the drain)
        m.d.sync += w_pending.eq(do2)
        for b in range(L):
            lsel = lane_of(b, q_rot)
            wa = Signal(14, name=f'wa{b}')
            wdat = Signal(8, name=f'wd{b}')
            we = Signal(name=f'we{b}')
            m.d.sync += [wa.eq(lane_addr[lsel]), wdat.eq(lane_data[lsel]),
                         we.eq(do2 & lane_act[lsel])]
            with m.If(self.busy):
                m.d.comb += [pwr[b].addr.eq(wa),
                             pwr[b].data.eq(wdat),
                             pwr[b].en.eq(we)]
            with m.Else():
                m.d.comb += [pwr[b].addr.eq(self.cpu_addr),
                             pwr[b].data.eq(self.cpu_wdata.word_select(b, 8)),
                             pwr[b].en.eq(self.cpu_we[b])]
        # batch done: the lanes' states and the syndrome
        for l in range(L):
            res = r_res[l * RL:(l + 1) * RL]
            m.d.sync += [swr[l].addr.eq(r_saddr),
                         swr[l].data.eq(Cat(res[:2 * M_BITS + IDX_BITS], new_signs[l])),
                         swr[l].en.eq(do2 & q_last & r_cact[l])]
        m.d.sync += fails.eq(Mux(do2 & q_last, new_par & r_cact, 0))
        with m.If(do2 & q_last):
            m.d.comb += finish_batch.eq(1)
            m.d.sync += p2_active.eq(0)
        m.d.sync += inflight.eq(inflight + start_batch - finish_batch)
        with m.If(self.start & ~self.busy):
            m.d.sync += inflight.eq(0)
        self.dbg = dict(j=j, kk=kk, e=e, gen=gen, it=it, inflight=inflight,
                        inflight_max=inflight_max, width=width, qlevel=qfifo.level,
                        p2_active=p2_active, rrdy=rfifo.r_rdy, qrdy=qfifo.r_rdy)
        return m
