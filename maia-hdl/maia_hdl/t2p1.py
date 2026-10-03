#
# SPDX-License-Identifier: MIT
#

"""DVB-T2 P1 detector and guard-interval correlator (tezuka_fw_simple): in
the T2 OFDM front end (t2ofdm.py), on the raw samples (before its NCO), so
that the ARM needs neither every sample (acquisition) nor each symbol's
guard interval (frequency tracking).

P1 (EN 302 755 9.8.2): C A B, 2048 samples; C (542) and B (482) are copies
of the start and the end of A (1024), shifted in frequency by 1/1024 of the
sample rate. With y[m] = x[m] e^(-j 2 pi m / 1024) (the counter m modulo
1024: a common phase the magnitudes below ignore):

  C(s) = sum_{m = s}^{s + 541}         y[m] conj(x[m + 1024])
  B(s) = sum_{m = s + 1566}^{s + 2047} y[m] conj(x[m - 482])
  E(s) = sum_{m = s}^{s + 2047}        |x[m]|^2

(trxd stream.rs structure_peak_in: the same sums). They run as the samples
come (sample t = s + 2047 completes window s): from one delay line of x and
y (taps 482, 964, 1024, 1506, 2048), each term added once and taken out
once, bit for bit the same, so the sums are exact. Per block of
``block_len`` samples the window with the largest
``(|C| + |B|) - k E`` (``k`` = ``k_q8`` / 256; |z| as max + 3/8 min) is
reported: its start s, ``|C| + |B|`` and ``E`` (each >> 10).

Guard intervals: over a frame, sum conj(x[t - 2048]) x[t] for the samples
``tail`` marks (each symbol's copied tail: its guard interval is 2048
samples back), reported at the frame's end (``last``, with the frame's
start ``frame_start``), >> 16.

Reports go out as words for the ring, in the carrier stream (bit 16 set) as
headers do: a header (bit 0 set; payload = re[15:1] | im[15:1] << 15) with
symbol number ``P1_J`` or ``GI_J`` (never a real symbol) in payload bits
7:0, then ``REPORT_WORDS`` words of 16 bits each (payload bits 15:0, the
other bits 0): P1: s (2 words, low first), |C| + |B| (2), E (2); GI: the
correlation's re (2), im (2), the frame's start (2).

Numbers: x 16 bits; y = round(x e^(-j..) 2^-15) with the 1024-entry Q15
table (t2ofdm.nco_tables), saturated; products 32 bits; C, B, E, GI 48 bits.
"""

from amaranth import *
from amaranth.lib.fifo import SyncFIFOBuffered
from amaranth.lib.memory import Memory

import numpy as np

N = 2048
TAPS = (482, 964, 1024, 1506, 2048)
P1_J = 253
GI_J = 252
REPORT_WORDS = 6
ACC = 48


NCO_BITS = 10


def nco_tables():
    """t2ofdm.nco_tables: the same 1024-entry Q15 table."""
    ph = 2 * np.pi * np.arange(2**NCO_BITS) / 2**NCO_BITS
    return (np.round(32767 * np.cos(ph)).astype(int),
            np.round(32767 * np.sin(ph)).astype(int))


def sat16(v):
    return max(-32768, min(32767, v))


def rot(x_re, x_im, t, cos_t, sin_t):
    """y = x e^(-j 2 pi t / 1024), Q15, rounded, saturated."""
    c, s = int(cos_t[t & 1023]), int(sin_t[t & 1023])
    yr = (x_re * c + x_im * s + (1 << 14)) >> 15
    yi = (x_im * c - x_re * s + (1 << 14)) >> 15
    return sat16(yr), sat16(yi)


def cmul_conj(a, b):
    """a conj(b), integers."""
    return (a[0] * b[0] + a[1] * b[1], a[1] * b[0] - a[0] * b[1])


def mag(z):
    a, b = abs(z[0]), abs(z[1])
    hi, lo = max(a, b), min(a, b)
    return hi + ((3 * lo) >> 3)


def wrap(v, bits):
    v &= (1 << bits) - 1
    return v - (1 << bits) if v >> (bits - 1) else v


def words16(v):
    return [v & 0xFFFF, (v >> 16) & 0xFFFF]


def pack_header(payload):
    re = ((payload & 0x7FFF) << 1) | 1
    im = (((payload >> 15) & 0x7FFF) << 1) | 1
    return re | im << 16


def pack_value(v16):
    """A report word: 16 bits as payload (bits 15:0), carrier stream bit."""
    re = (v16 & 0x7FFF) << 1
    im = (((v16 >> 15) & 1) << 1) | 1
    return re | im << 16


class Model:
    """Bit-exact model: ``push(x_re, x_im, t, tail, last, frame_start)`` per
    sample returns the report words that sample completes."""
    def __init__(self, block_len, k_q8):
        self.block_len = block_len
        self.k = k_q8
        self.cos, self.sin = nco_tables()
        self.mem = {}
        self.count = 0          # samples since enable
        self.c = [0, 0]
        self.b = [0, 0]
        self.e = 0
        self.gi = [0, 0]
        self.blk = 0
        self.best = None

    def tap(self, d):
        if self.count < d:
            return (0, 0), (0, 0)
        return self.mem[self.count - d]

    def push(self, x_re, x_im, t, tail=False, last=False, frame_start=0):
        out = []
        x0 = (x_re, x_im)
        y0 = rot(x_re, x_im, t, self.cos, self.sin)
        (x482, y482) = self.tap(482)
        (x964, _) = self.tap(964)
        (x1024, _) = self.tap(1024)
        (_, y1506) = self.tap(1506)
        (x2048, y2048) = self.tap(2048)
        self.e = wrap(self.e + x0[0] ** 2 + x0[1] ** 2 - x2048[0] ** 2 - x2048[1] ** 2, ACC)
        badd, brem = cmul_conj(y0, x482), cmul_conj(y482, x964)
        cadd, crem = cmul_conj(y1506, x482), cmul_conj(y2048, x1024)
        for i in range(2):
            self.b[i] = wrap(self.b[i] + badd[i] - brem[i], ACC)
            self.c[i] = wrap(self.c[i] + cadd[i] - crem[i], ACC)
        if tail and self.count >= N:
            g = cmul_conj(x0, x2048)   # x0 conj(x2048)
            for i in range(2):
                self.gi[i] = wrap(self.gi[i] + g[i], ACC)
        self.mem[self.count] = (x0, y0)
        self.mem.pop(self.count - 2048 - 1, None)
        self.count += 1
        if self.count > N:
            cb = ((mag(self.c) + mag(self.b)) >> 10) & 0xFFFFFFFF
            e = (self.e >> 10) & 0xFFFFFFFF
            score = cb - ((self.k * e) >> 8)
            if self.best is None or score > self.best[0]:
                self.best = (score, (t - (N - 1)) & 0xFFFFFFFF, cb & 0xFFFFFFFF, e & 0xFFFFFFFF)
        self.blk += 1
        if self.blk == self.block_len:
            self.blk = 0
            if self.best is not None:
                _, s, cb, e = self.best
                out += [pack_header(P1_J)] + [pack_value(w) for w in words16(s) + words16(cb) + words16(e)]
            self.best = None
        if last:
            gr = (self.gi[0] >> 16) & 0xFFFFFFFF
            gim = (self.gi[1] >> 16) & 0xFFFFFFFF
            out += [pack_header(GI_J)] + [pack_value(w) for w in words16(gr) + words16(gim) + words16(frame_start & 0xFFFFFFFF)]
            self.gi = [0, 0]
        return out


class T2P1(Elaboratable):
    """Clock domain ``sync``. Samples in (``strobe``, ``x_re``, ``x_im``,
    ``t`` their counter, ``tail``, ``last`` + ``frame_start``) at most one
    every ~16 cycles on average (a small FIFO takes bursts); report words
    out (``o_data``, ``o_en`` when ``o_rdy``)."""
    def __init__(self):
        self.enable = Signal()
        self.block_len = Signal(20)
        self.k_q8 = Signal(8)
        self.strobe = Signal()
        self.x_re = Signal(signed(16))
        self.x_im = Signal(signed(16))
        self.t = Signal(32)
        self.tail = Signal()
        self.last = Signal()
        self.frame_start = Signal(32)
        self.o_data = Signal(32)
        self.o_en = Signal()
        self.o_rdy = Signal()
        self.overflow = Signal()     # sticky until disabled: samples lost

    def elaborate(self, platform):
        m = Module()
        cos_t, sin_t = nco_tables()
        m.submodules.cos = cos_m = Memory(shape=signed(16), depth=1024, init=[int(v) for v in cos_t])
        m.submodules.sin = sin_m = Memory(shape=signed(16), depth=1024, init=[int(v) for v in sin_t])
        cos_rd = cos_m.read_port()
        sin_rd = sin_m.read_port()
        # the delay line: x (31:0) and y (63:32), a sample an address
        m.submodules.dl = dl = Memory(shape=64, depth=N, init=[])
        dl_rd = dl.read_port()
        dl_wr = dl.write_port()

        # samples in: a FIFO (x 32, t 32, tail, last) and the frame start
        fin = SyncFIFOBuffered(width=66, depth=8)
        m.submodules.fin = fin
        fs_hold = Signal(32)
        m.d.comb += [fin.w_data.eq(Cat(self.x_re, self.x_im, self.t, self.tail, self.last)),
                     fin.w_en.eq(self.enable & self.strobe)]
        with m.If(self.enable & self.strobe & self.last):
            m.d.sync += fs_hold.eq(self.frame_start)
        with m.If(~self.enable):
            m.d.sync += self.overflow.eq(0)
        with m.Elif(fin.w_en & ~fin.w_rdy):
            m.d.sync += self.overflow.eq(1)

        # report words out: a FIFO (room for a P1 and a GI report)
        fout = SyncFIFOBuffered(width=32, depth=16)
        m.submodules.fout = fout
        # valid / ready: o_en a word is there, taken when o_rdy
        m.d.comb += [self.o_data.eq(fout.r_data), self.o_en.eq(fout.r_rdy),
                     fout.r_en.eq(self.o_rdy & fout.r_rdy)]

        x0r = Signal(signed(16))
        x0i = Signal(signed(16))
        y0r = Signal(signed(16))
        y0i = Signal(signed(16))
        t = Signal(32)
        tail = Signal()
        last = Signal()
        count = Signal(12)       # samples before this one, saturating at 2049
        wptr = Signal(11)        # delay-line write address (wraps)
        full = Signal()
        m.d.comb += full.eq(count > N)
        ntap = len(TAPS)
        tx = [Signal(32, name=f"tx{d}") for d in TAPS]
        ty = [Signal(32, name=f"ty{d}") for d in TAPS]
        tapd = Array(C(dd, 12) for dd in TAPS)
        rc = Signal(range(ntap + 2))   # read cycle
        pc = Signal(range(9))          # product cycle

        c_re = Signal(signed(ACC))
        c_im = Signal(signed(ACC))
        b_re = Signal(signed(ACC))
        b_im = Signal(signed(ACC))
        e = Signal(signed(ACC))
        g_re = Signal(signed(ACC))
        g_im = Signal(signed(ACC))

        def split(v):
            return v[:16].as_signed(), v[16:32].as_signed()

        # ---- products: item i's operands a, b (a conj(b)) ----
        x0 = (x0r, x0i)
        y0 = (y0r, y0i)
        items = [
            (x0, x0),                        # 0: |x0|^2
            (split(tx[4]), split(tx[4])),    # 1: |x2048|^2
            (y0, split(tx[0])),              # 2: B add   y0 conj(x482)
            (split(ty[0]), split(tx[1])),    # 3: B take  y482 conj(x964)
            (split(ty[3]), split(tx[0])),    # 4: C add   y1506 conj(x482)
            (split(ty[4]), split(tx[2])),    # 5: C take  y2048 conj(x1024)
            (x0, split(tx[4])),              # 6: GI      x0 conj(x2048)
        ]
        ar = Signal(signed(16))
        ai = Signal(signed(16))
        br = Signal(signed(16))
        bi = Signal(signed(16))
        with m.Switch(pc):
            for i, ((a0, a1), (b0, b1)) in enumerate(items):
                with m.Case(i):
                    m.d.comb += [ar.eq(a0), ai.eq(a1), br.eq(b0), bi.eq(b1)]
        q = [Signal(signed(32), name=f"q{i}") for i in range(4)]
        qi = Signal(range(8))      # the item whose products q holds
        qv = Signal()
        pre = Signal(signed(33))
        pim = Signal(signed(33))
        m.d.comb += [pre.eq(q[0] + q[1]), pim.eq(q[2] - q[3])]

        # score
        def absv(v):
            return Mux(v < 0, -v, v)

        def magn(re, im):
            a = Signal(ACC)
            b = Signal(ACC)
            hi = Signal(ACC)
            lo = Signal(ACC)
            mm = Signal(ACC + 1)
            m.d.comb += [a.eq(absv(re)), b.eq(absv(im)),
                         hi.eq(Mux(a > b, a, b)), lo.eq(Mux(a > b, b, a)),
                         mm.eq(hi + ((lo * 3) >> 3))]
            return mm
        cb_full = Signal(ACC + 2)
        m.d.comb += cb_full.eq(magn(c_re, c_im) + magn(b_re, b_im))
        cb10 = Signal(32)
        e10 = Signal(32)
        m.d.comb += [cb10.eq(cb_full >> 10), e10.eq(e.as_unsigned() >> 10)]
        ke = Signal(32)
        m.d.sync += ke.eq((e10 * self.k_q8) >> 8)
        score = Signal(signed(34))
        m.d.comb += score.eq(Cat(cb10, C(0, 2)).as_signed() - Cat(ke, C(0, 2)).as_signed())

        blk = Signal(20)
        have_best = Signal()
        best_score = Signal(signed(34))
        best_s = Signal(32)
        best_cb = Signal(32)
        best_e = Signal(32)
        wq = Signal(range(REPORT_WORDS + 2))
        rep_vals = Array([Signal(16, name=f"rv{i}") for i in range(REPORT_WORDS)])
        want_p1 = Signal()
        want_gi = Signal()

        m.d.comb += [fout.w_en.eq(0), fin.r_en.eq(0), dl_wr.en.eq(0),
                     cos_rd.addr.eq(t[:10]), sin_rd.addr.eq(t[:10]),
                     dl_rd.addr.eq((wptr - tapd[rc])[:11])]

        hdr = lambda j: Cat(C(1, 1), C(j, 15), C(1, 1), C(0, 15))
        val = lambda v: Cat(C(0, 1), v[:15], C(1, 1), v[15], C(0, 14))

        with m.FSM():
            with m.State('IDLE'):
                with m.If(fin.r_rdy & self.enable):
                    d = fin.r_data
                    m.d.comb += fin.r_en.eq(1)
                    m.d.sync += [x0r.eq(d[:16].as_signed()), x0i.eq(d[16:32].as_signed()),
                                 t.eq(d[32:64]), tail.eq(d[64]), last.eq(d[65]), rc.eq(0)]
                    m.next = 'RD'
            with m.State('RD'):
                # cycle rc: address of tap rc out (comb), data of tap rc - 1
                # in; the table data for t arrives on the first cycle
                with m.If(rc == 1):
                    pr = Signal(signed(33))
                    pi = Signal(signed(33))
                    m.d.comb += [pr.eq(x0r * cos_rd.data + x0i * sin_rd.data),
                                 pi.eq(x0i * cos_rd.data - x0r * sin_rd.data)]
                    yr = Signal(signed(19))
                    yi = Signal(signed(19))
                    m.d.comb += [yr.eq((pr + (1 << 14)) >> 15), yi.eq((pi + (1 << 14)) >> 15)]
                    m.d.sync += [y0r.eq(Mux(yr > 32767, 32767, Mux(yr < -32768, -32768, yr))),
                                 y0i.eq(Mux(yi > 32767, 32767, Mux(yi < -32768, -32768, yi)))]
                for i in range(ntap):
                    with m.If(rc == i + 1):
                        with m.If(count >= TAPS[i]):
                            m.d.sync += [tx[i].eq(dl_rd.data[:32]), ty[i].eq(dl_rd.data[32:])]
                        with m.Else():
                            m.d.sync += [tx[i].eq(0), ty[i].eq(0)]
                with m.If(rc == ntap):
                    m.d.sync += [rc.eq(0), pc.eq(0), qv.eq(0)]
                    m.next = 'MUL'
                with m.Else():
                    m.d.sync += rc.eq(rc + 1)
            with m.State('MUL'):
                # the taps are read: x0, y0 into the delay line now
                with m.If(pc == 0):
                    m.d.comb += [dl_wr.addr.eq(wptr), dl_wr.data.eq(Cat(x0r, x0i, y0r, y0i)),
                                 dl_wr.en.eq(1)]
                    m.d.sync += wptr.eq(wptr + 1)
                # item pc's products into q (pc < 7) while item qi's are summed
                with m.If(pc < len(items)):
                    m.d.sync += [q[0].eq(ar * br), q[1].eq(ai * bi),
                                 q[2].eq(ai * br), q[3].eq(ar * bi),
                                 qi.eq(pc), qv.eq(1)]
                with m.Else():
                    m.d.sync += qv.eq(0)
                with m.If(qv):
                    with m.Switch(qi):
                        with m.Case(0):
                            m.d.sync += e.eq(e + pre)
                        with m.Case(1):
                            m.d.sync += e.eq(e - pre)
                        with m.Case(2):
                            m.d.sync += [b_re.eq(b_re + pre), b_im.eq(b_im + pim)]
                        with m.Case(3):
                            m.d.sync += [b_re.eq(b_re - pre), b_im.eq(b_im - pim)]
                        with m.Case(4):
                            m.d.sync += [c_re.eq(c_re + pre), c_im.eq(c_im + pim)]
                        with m.Case(5):
                            m.d.sync += [c_re.eq(c_re - pre), c_im.eq(c_im - pim)]
                        with m.Case(6):
                            with m.If(tail & (count >= N)):
                                m.d.sync += [g_re.eq(g_re + pre), g_im.eq(g_im + pim)]
                with m.If(pc == len(items)):
                    m.d.sync += count.eq(Mux(count > N, count, count + 1))
                    # (e has its last update at pc 2: ke, registered from
                    # it, is settled by now)
                    m.next = 'SCORE2'
                with m.Else():
                    m.d.sync += pc.eq(pc + 1)
            with m.State('SCORE2'):
                with m.If(full & (~have_best | (score > best_score))):
                    m.d.sync += [have_best.eq(1), best_score.eq(score),
                                 best_s.eq(t - (N - 1)), best_cb.eq(cb10), best_e.eq(e10)]
                end = Signal()
                m.d.comb += end.eq(blk == self.block_len - 1)
                with m.If(end):
                    m.d.sync += [blk.eq(0), want_p1.eq(have_best | full)]
                with m.Else():
                    m.d.sync += blk.eq(blk + 1)
                with m.If(last):
                    m.d.sync += want_gi.eq(1)
                m.d.sync += wq.eq(0)
                with m.If(end | last | want_p1 | want_gi):
                    m.next = 'REPORT'
                with m.Else():
                    m.next = 'IDLE'
            with m.State('REPORT'):
                with m.If(want_p1):
                    with m.If(fout.w_rdy):
                        with m.If(wq == 0):
                            m.d.comb += [fout.w_data.eq(hdr(P1_J)), fout.w_en.eq(1)]
                            m.d.sync += [rep_vals[0].eq(best_s[:16]), rep_vals[1].eq(best_s[16:]),
                                         rep_vals[2].eq(best_cb[:16]), rep_vals[3].eq(best_cb[16:]),
                                         rep_vals[4].eq(best_e[:16]), rep_vals[5].eq(best_e[16:]),
                                         wq.eq(1)]
                        with m.Else():
                            m.d.comb += [fout.w_data.eq(val(rep_vals[wq - 1])), fout.w_en.eq(1)]
                            with m.If(wq == REPORT_WORDS):
                                m.d.sync += [wq.eq(0), want_p1.eq(0), have_best.eq(0)]
                            with m.Else():
                                m.d.sync += wq.eq(wq + 1)
                with m.Elif(want_gi):
                    with m.If(fout.w_rdy):
                        with m.If(wq == 0):
                            gr = Signal(32)
                            gim = Signal(32)
                            m.d.comb += [gr.eq(g_re >> 16), gim.eq(g_im >> 16),
                                         fout.w_data.eq(hdr(GI_J)), fout.w_en.eq(1)]
                            m.d.sync += [rep_vals[0].eq(gr[:16]), rep_vals[1].eq(gr[16:]),
                                         rep_vals[2].eq(gim[:16]), rep_vals[3].eq(gim[16:]),
                                         rep_vals[4].eq(fs_hold[:16]), rep_vals[5].eq(fs_hold[16:]),
                                         g_re.eq(0), g_im.eq(0), wq.eq(1)]
                        with m.Else():
                            m.d.comb += [fout.w_data.eq(val(rep_vals[wq - 1])), fout.w_en.eq(1)]
                            with m.If(wq == REPORT_WORDS):
                                m.d.sync += [wq.eq(0), want_gi.eq(0)]
                            with m.Else():
                                m.d.sync += wq.eq(wq + 1)
                with m.Else():
                    m.next = 'IDLE'

        with m.If(~self.enable):
            m.d.sync += [count.eq(0), wptr.eq(0), c_re.eq(0), c_im.eq(0), b_re.eq(0),
                         b_im.eq(0), e.eq(0), g_re.eq(0), g_im.eq(0), blk.eq(0),
                         have_best.eq(0), want_p1.eq(0), want_gi.eq(0), wq.eq(0)]
        return m
