#
# SPDX-License-Identifier: MIT
#

"""DVB-S2 known-symbol accumulator (tezuka_fw_simple): the per-frame work
of trxd's ring-mode receiver (dvbs2/rx.rs frame()) on the header and pilot
blocks, done as the symbols go into the ring, so that the ARM reads one
entry a block instead of making every known symbol itself.

It sees the words the recorder writes into the ring (the header detector's
symbols: re in bits 15:0, im in 31:16, both signed; bit 16 is the detector's
flag and is taken as part of im, as trxd does) with their absolute index (k:
words since the recorder started, as trxd counts the ring).

Frames: trxd loads a header's position ``base`` (any frame of ours already
seen), the frame length ``frame_len`` (header, data and pilot symbols),
``pilots`` and the number of pilot blocks ``npil``. The unit steps ``base``
on by whole frames to the frame in progress and from the next frame start on
follows frame after frame (a frame of another length, a slip: trxd loads
again). In each frame the known blocks are the header (symbols 0..89, its
references from ``hdr_q``) and the pilot blocks (36 symbols after every 1440
data symbols, ``npil`` of them; reference the PL scrambling sequence R(d),
d = position - 90).

AFC mixer: a phase accumulator (32 bits a turn): symbol k is multiplied by
e^(j 2 pi phi_k / 2^32) and phi_{k+1} = phi_k + dth, every word; dth is
``dth`` taken at each frame start (always while no frame runs). The
1024-entry Q15 table (t2p1.nco_tables) at (phi + 2^21) >> 22:

  y = sat16((x c - x' s + 2^14) >> 15) + j sat16((x s + x' c + 2^14) >> 15)

Every reference is e^(j pi/4) j^q, so a block's correlation with them is
e^(-j pi/4) times the sum of y j^(-q) (quarter turns, no multiplier); trxd
applies the e^(-j pi/4).

Per block an entry (6 words) goes into a FIFO: the absolute index of its
first symbol (low 32 bits), sum re, sum im (32 bits, signed), the sum of
|x|^2 >> 8 (32 bits), phi at the first symbol and the frame's dth. trxd
reads them (``level``, the words, ``pop``).
"""

from amaranth import *
from amaranth.lib.fifo import SyncFIFOBuffered
from amaranth.lib.memory import Memory

from .t2p1 import nco_tables, sat16
from .s2front import gold_states

SLOT = 90
PILOT = 36
GROUP = 16 * SLOT          # data symbols between pilot blocks
PERIOD = GROUP + PILOT
ENTRY_WORDS = 6
FIFO_DEPTH = 512
FEATURE = 1 << 16          # read at 0x7C: the S2 tracker (T2's are bits 7:0)


def gold_step(x, y):
    bx = ((x >> 7) ^ x) & 1
    x = ((bx << 18) | x) >> 1
    by = ((y >> 10) ^ (y >> 7) ^ (y >> 5) ^ y) & 1
    y = ((by << 18) | y) >> 1
    return x, y


def mix(x_re, x_im, phi, cos_t, sin_t):
    t = ((phi + (1 << 21)) >> 22) & 1023
    c, s = int(cos_t[t]), int(sin_t[t])
    yr = sat16((x_re * c - x_im * s + (1 << 14)) >> 15)
    yi = sat16((x_re * s + x_im * c + (1 << 14)) >> 15)
    return yr, yi


def rot_q(yr, yi, q):
    """y j^(-q)."""
    return [(yr, yi), (yi, -yr), (-yr, -yi), (-yi, yr)][q & 3]


def s16(v):
    v &= 0xFFFF
    return v - 0x10000 if v & 0x8000 else v


def block_entry(words, k0, refs, phi0, dth, cos_t=None, sin_t=None):
    """One block's entry from its words (the first at absolute index k0),
    their references' quarter turns and the mixer phase at the first: what
    trxd's dvbs2/s2trk.rs block() computes."""
    if cos_t is None:
        cos_t, sin_t = nco_tables()
    sr = si = p = 0
    phi = phi0
    for w, q in zip(words, refs):
        xr, xi = s16(w), s16(w >> 16)
        yr, yi = mix(xr, xi, phi, cos_t, sin_t)
        zr, zi = rot_q(yr, yi, q)
        sr += zr
        si += zi
        p += xr * xr + xi * xi
        phi = (phi + dth) & 0xFFFFFFFF
    m = 0xFFFFFFFF
    return [k0 & m, sr & m, si & m, (p >> 8) & m, phi0 & m, dth & m]


class Model:
    """The unit, word by word. ``load(base, frame_len, pilots, npil)``
    takes effect before the next word; ``dth`` and ``hdr_q`` are the
    registers."""

    def __init__(self, hdr_q, dth=0):
        self.hdr_q = list(hdr_q)
        self.dth = dth
        self.cos_t, self.sin_t = nco_tables()
        self.k = 0
        self.phi = 0
        self.dth_cur = dth
        self.synced = False
        self.started = False
        self.pending = None
        self.entries = []

    def load(self, base, frame_len, pilots, npil):
        self.pending = (base, frame_len, pilots, npil)

    def push(self, w):
        k = self.k
        if self.pending is not None:
            base, self.frame_len, self.pilots, self.npil = self.pending
            self.pending = None
            # step on by whole frames to the one in progress (a base in
            # the future: wait for it)
            if ((k - base) & 0xFFFFFFFF) < 1 << 31:
                while ((k - base) & 0xFFFFFFFF) >= self.frame_len:
                    base = (base + self.frame_len) & 0xFFFFFFFF
            self.base = base
            self.synced = True
            self.started = False
        known, q, first, last = False, 0, False, False
        if self.synced:
            pos = (k - self.base) & 0xFFFFFFFF
            if pos >= 1 << 31:
                pos = None              # base still ahead
            elif pos == self.frame_len:
                self.base = (self.base + self.frame_len) & 0xFFFFFFFF
                pos = 0
            if pos == 0:
                self.started = True
                self.dth_cur = self.dth
            if pos is not None and self.started:
                if pos < SLOT:
                    known, q = True, self.hdr_q[pos]
                    first, last = pos == 0, pos == SLOT - 1
                else:
                    d = pos - SLOT
                    if pos == SLOT:
                        self.g1, self.g2 = gold_states()
                    q = ((self.g1[0] ^ self.g1[1]) & 1) | (((self.g2[0] ^ self.g2[1]) & 1) << 1)
                    self.g1 = gold_step(*self.g1)
                    self.g2 = gold_step(*self.g2)
                    g, r = divmod(d, PERIOD)
                    if self.pilots and r >= GROUP and g < self.npil:
                        known = True
                        first, last = r == GROUP, r == PERIOD - 1
        if not self.started:
            self.dth_cur = self.dth
        if known:
            xr, xi = s16(w), s16(w >> 16)
            yr, yi = mix(xr, xi, self.phi, self.cos_t, self.sin_t)
            zr, zi = rot_q(yr, yi, q)
            if first:
                self.acc = [k, 0, 0, 0, self.phi, self.dth_cur]
            self.acc[1] += zr
            self.acc[2] += zi
            self.acc[3] += xr * xr + xi * xi
            if last:
                m = 0xFFFFFFFF
                a = self.acc
                self.entries.append([a[0] & m, a[1] & m, a[2] & m, (a[3] >> 8) & m, a[4], a[5] & m])
        self.phi = (self.phi + self.dth_cur) & 0xFFFFFFFF
        self.k += 1


class S2Trk(Elaboratable):
    """See the module's docstring. ``word``/``valid``: what the recorder
    writes (``run_start`` when it starts: the index restarts at 0)."""

    def __init__(self):
        self.enable = Signal()
        self.word = Signal(32)
        self.valid = Signal()
        self.run_start = Signal()
        self.load = Signal()
        self.base = Signal(32)
        self.frame_len = Signal(17)
        self.pilots = Signal()
        self.npil = Signal(5)
        self.dth = Signal(32)
        self.hdr_waddr = Signal(7)
        self.hdr_wdata = Signal(2)
        self.hdr_we = Signal()
        self.pop = Signal()
        self.level = Signal(range(FIFO_DEPTH + 1))
        self.entry = Signal(32 * ENTRY_WORDS)
        self.overflow = Signal()
        self.synced = Signal()
        self.counter = Signal(32)

    def elaborate(self, platform):
        m = Module()
        cos_t, sin_t = nco_tables()
        m.submodules.table = table = Memory(
            shape=32, depth=1024,
            init=[(int(c) & 0xFFFF) | ((int(s) & 0xFFFF) << 16)
                  for c, s in zip(cos_t, sin_t)])
        trd = table.read_port()
        m.submodules.hdr = hdr = Memory(shape=2, depth=128, init=[])
        hw = hdr.write_port()
        hrd = hdr.read_port(domain='comb')
        m.d.comb += [hw.addr.eq(self.hdr_waddr), hw.data.eq(self.hdr_wdata),
                     hw.en.eq(self.hdr_we)]
        m.submodules.fifo = fifo = SyncFIFOBuffered(
            width=32 * ENTRY_WORDS, depth=FIFO_DEPTH)
        m.d.comb += [self.level.eq(fifo.r_level), self.entry.eq(fifo.r_data),
                     fifo.r_en.eq(self.pop)]

        k = self.counter
        with m.If(self.run_start):
            m.d.sync += k.eq(0)
        with m.Elif(self.valid):
            m.d.sync += k.eq(k + 1)

        # ---- frame position (stage A, the word's cycle) ----
        base = Signal(32)
        frame_len = Signal(17)
        pilots = Signal()
        npil = Signal(5)
        aligning = Signal()
        synced = self.synced
        started = Signal()
        ahead = Signal()            # base not reached yet
        phi = Signal(32)
        dth_cur = Signal(32)
        r = Signal(range(PERIOD))
        g = Signal(5)
        (x1i, y1i), (x2i, y2i) = gold_states()
        gx1, gy1 = Signal(18), Signal(18)
        gx2, gy2 = Signal(18), Signal(18)

        diff = Signal(32)
        m.d.comb += diff.eq(k - base)
        with m.If(self.load):
            m.d.sync += [base.eq(self.base), frame_len.eq(self.frame_len),
                         pilots.eq(self.pilots), npil.eq(self.npil),
                         aligning.eq(1), synced.eq(0), started.eq(0)]
        with m.Elif(aligning & ~self.valid):
            # whole frames on, one a cycle, between words
            with m.If(~diff[31] & (diff >= frame_len)):
                m.d.sync += base.eq(base + frame_len)
            with m.Else():
                m.d.sync += [aligning.eq(0), synced.eq(1)]
        with m.If(~self.enable | self.run_start):
            m.d.sync += [synced.eq(0), started.eq(0), aligning.eq(0)]

        # this word's position
        wpos = Signal(17)
        at_start = Signal()
        m.d.comb += [ahead.eq(diff[31]),
                     wpos.eq(Mux(diff[:17] == frame_len, 0, diff[:17])),
                     at_start.eq(synced & ~ahead & (wpos == 0))]

        def gold(x, y):
            bx = (x >> 7)[0] ^ x[0]
            by = (y >> 10)[0] ^ (y >> 7)[0] ^ (y >> 5)[0] ^ y[0]
            return Cat(x[1:], bx), Cat(y[1:], by)

        known = Signal()
        q = Signal(2)
        first = Signal()
        last = Signal()
        run = Signal()
        m.d.comb += run.eq(synced & ~ahead & (started | at_start))
        hdr_part = Signal()
        m.d.comb += [hdr_part.eq(wpos < SLOT), hrd.addr.eq(wpos[:7])]
        pil = Signal()
        m.d.comb += pil.eq(pilots & ~hdr_part & (r >= GROUP) & (g < npil))
        gq = Signal(2)
        m.d.comb += gq.eq(Cat(gx1[0] ^ gy1[0], gx2[0] ^ gy2[0]))
        with m.If(hdr_part):
            m.d.comb += [known.eq(run), q.eq(hrd.data),
                         first.eq(wpos == 0), last.eq(wpos == SLOT - 1)]
        with m.Else():
            m.d.comb += [known.eq(run & pil), q.eq(gq),
                         first.eq(r == GROUP), last.eq(r == PERIOD - 1)]
        dth_next = Signal(32)
        m.d.comb += dth_next.eq(Mux(at_start | ~started, self.dth, dth_cur))
        with m.If(self.valid):
            m.d.sync += [phi.eq(phi + dth_next), dth_cur.eq(dth_next)]
            with m.If(synced & ~ahead):
                with m.If(diff[:17] == frame_len):
                    m.d.sync += base.eq(base + frame_len)
                with m.If(at_start):
                    m.d.sync += started.eq(1)
                # position 89 -> the counters for 90 ready; past 90 step
                with m.If(wpos == SLOT - 1):
                    m.d.sync += [r.eq(0), g.eq(0),
                                 gx1.eq(x1i), gy1.eq(y1i),
                                 gx2.eq(x2i), gy2.eq(y2i)]
                with m.Elif(~hdr_part):
                    nx1, ny1 = gold(gx1, gy1)
                    nx2, ny2 = gold(gx2, gy2)
                    m.d.sync += [gx1.eq(nx1), gy1.eq(ny1),
                                 gx2.eq(nx2), gy2.eq(ny2)]
                    with m.If(r == PERIOD - 1):
                        m.d.sync += [r.eq(0), g.eq(g + 1)]
                    with m.Else():
                        m.d.sync += r.eq(r + 1)

        # ---- stage A -> B: the word, its table entry (read), flags ----
        m.d.comb += trd.addr.eq((phi + (1 << 21))[22:32])
        b_v = Signal()
        b_xr = Signal(signed(16))
        b_xi = Signal(signed(16))
        b_q = Signal(2)
        b_first = Signal()
        b_last = Signal()
        b_k = Signal(32)
        b_phi = Signal(32)
        b_dth = Signal(32)
        m.d.sync += [b_v.eq(self.valid & known & self.enable),
                     b_xr.eq(self.word[:16]), b_xi.eq(self.word[16:]),
                     b_q.eq(q), b_first.eq(first), b_last.eq(last),
                     b_k.eq(k), b_phi.eq(phi), b_dth.eq(dth_next)]
        # ---- stage B -> C: products ----
        c_tab = trd.data
        cc = c_tab[:16].as_signed()
        ss = c_tab[16:].as_signed()
        c_v = Signal()
        c_rr = Signal(signed(33))
        c_ii = Signal(signed(33))
        c_p = Signal(33)
        c_q = Signal(2)
        c_first = Signal()
        c_last = Signal()
        c_k = Signal(32)
        c_phi = Signal(32)
        c_dth = Signal(32)
        m.d.sync += [c_v.eq(b_v),
                     c_rr.eq(b_xr * cc - b_xi * ss),
                     c_ii.eq(b_xr * ss + b_xi * cc),
                     c_p.eq(b_xr * b_xr + b_xi * b_xi),
                     c_q.eq(b_q), c_first.eq(b_first), c_last.eq(b_last),
                     c_k.eq(b_k), c_phi.eq(b_phi), c_dth.eq(b_dth)]
        # ---- stage C: round, saturate, quarter turns, accumulate ----

        def sat(v):
            return Mux(v > 32767, 32767, Mux(v < -32768, -32768, v))

        yr = Signal(signed(17))
        yi = Signal(signed(17))
        m.d.comb += [yr.eq(sat((c_rr + (1 << 14)) >> 15)),
                     yi.eq(sat((c_ii + (1 << 14)) >> 15))]
        zr = Signal(signed(17))
        zi = Signal(signed(17))
        with m.Switch(c_q):
            with m.Case(0):
                m.d.comb += [zr.eq(yr), zi.eq(yi)]
            with m.Case(1):
                m.d.comb += [zr.eq(yi), zi.eq(-yr)]
            with m.Case(2):
                m.d.comb += [zr.eq(-yr), zi.eq(-yi)]
            with m.Case(3):
                m.d.comb += [zr.eq(-yi), zi.eq(yr)]
        a_re = Signal(signed(32))
        a_im = Signal(signed(32))
        a_p = Signal(48)
        a_k = Signal(32)
        a_phi = Signal(32)
        a_dth = Signal(32)
        push = Signal()
        m.d.sync += push.eq(0)
        with m.If(c_v):
            with m.If(c_first):
                m.d.sync += [a_re.eq(zr), a_im.eq(zi), a_p.eq(c_p),
                             a_k.eq(c_k), a_phi.eq(c_phi), a_dth.eq(c_dth)]
            with m.Else():
                m.d.sync += [a_re.eq(a_re + zr), a_im.eq(a_im + zi),
                             a_p.eq(a_p + c_p)]
            m.d.sync += push.eq(c_last)
        m.d.comb += [fifo.w_data.eq(Cat(a_k, a_re, a_im, a_p[8:40], a_phi,
                                        a_dth)),
                     fifo.w_en.eq(push)]
        with m.If(push & ~fifo.w_rdy):
            m.d.sync += self.overflow.eq(1)
        with m.If(self.load):
            m.d.sync += self.overflow.eq(0)
        return m
