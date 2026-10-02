#
# SPDX-License-Identifier: MIT
#

"""DVB-S2 long frames straight from the receive ring (trxd dvbs2/s2ring.rs,
bit for bit): the LDPC DDR engine (ldpc_dma.py) reads a PLFRAME's symbols
where the recorder left them and this block turns them into the engine's
cells, so the CPU no longer touches the data symbols.

Input: the ring words of the frame after its PLHEADER, two a 64-bit beat
(low word first), each re (15:0) and im (31:16) as i16; the first `lead`
words (the beats are 128-byte aligned) are skipped. With `pilots` every
1440 data symbols are followed by 36 pilot symbols, dropped (but counted by
the scrambler). Data symbols come out as cells until `n_cells` are done;
the rest of the input is swallowed.

Each data symbol t of group s (the data between two known blocks) is turned
by the angle A_s + t B_s (32 bits a turn, the CPU's per-frame carrier fit,
written into a 32-entry table) rounded to 16 bits, plus (4 - R_j) mod 4
quarter turns (descrambling; R_j the PL scrambling sequence, Gold code
n = 0, restarted at each frame, j the symbol's position after the
header), by a 12-step CORDIC after a +-90 degree pre-rotation, then scaled
by the frame's gain G (cell = (x G + 2^19) >> 20, saturated to i8). The
CORDIC's gain (1.64676) is in G.
"""

import math

from amaranth import *
from amaranth.lib.memory import Memory
from amaranth.lib.fifo import SyncFIFOBuffered

STAGES = 12
ATAN = [round(math.atan(2.0 ** -i) * 32768 / math.pi) for i in range(STAGES)]
GROUP = 1440
PILOT = 36
GAIN_SHIFT = 20
CF_DEPTH = 32


def gold_states():
    """(x, y) initial states of the two Gold generators: offset 0 and
    offset 131072 (the second component of R_n)."""
    x, y = 1, 0x3FFFF
    first = (x, y)
    for _ in range(131072):
        bx = ((x >> 7) ^ x) & 1
        x = ((bx << 18) | x) >> 1
        by = ((y >> 10) ^ (y >> 7) ^ (y >> 5) ^ y) & 1
        y = ((by << 18) | y) >> 1
    return first, (x, y)


def pl_scrambling(n):
    """trxd dvbs2/mod.rs pl_scrambling: R_j for j < n."""
    (x1, y1), (x2, y2) = gold_states()
    out = []
    for _ in range(n):
        out.append(((x1 ^ y1) & 1) | ((x2 ^ y2) & 1) << 1)
        for which in (0, 1):
            x, y = (x1, y1) if which == 0 else (x2, y2)
            bx = ((x >> 7) ^ x) & 1
            x = ((bx << 18) | x) >> 1
            by = ((y >> 10) ^ (y >> 7) ^ (y >> 5) ^ y) & 1
            y = ((by << 18) | y) >> 1
            if which == 0:
                x1, y1 = x, y
            else:
                x2, y2 = x, y
    return out


def cordic(x, y, th):
    """Rotate (x, y) by th (16 bits a turn, signed)."""
    if th >= 16384:
        x, y, th = -y, x, th - 16384
    elif th < -16384:
        x, y, th = y, -x, th + 16384
    for i in range(STAGES):
        if th >= 0:
            x, y, th = x - (y >> i), y + (x >> i), th - ATAN[i]
        else:
            x, y, th = x + (y >> i), y - (x >> i), th + ATAN[i]
    return x, y


def model_cells(words, n_cells, pilots, segs, gain):
    """The cells of a frame from its ring words (after the header)."""
    def s16(v):
        v &= 0xFFFF
        return v - 0x10000 if v & 0x8000 else v

    def sat(v):
        v = (v * gain + (1 << (GAIN_SHIFT - 1))) >> GAIN_SHIFT
        return max(-128, min(127, v))
    r = pl_scrambling(len(words))
    out = []
    j = 0
    s = t = 0
    pos = 0
    while len(out) < n_cells:
        w = words[j]
        data = (not pilots) or pos < GROUP
        if data:
            a, b = segs[s]
            acc = (a + t * b) & 0xFFFFFFFF
            th = (((acc + 0x8000) >> 16) + (((4 - r[j]) & 3) << 14)) & 0xFFFF
            th = th - 0x10000 if th & 0x8000 else th
            x, y = cordic(s16(w), s16(w >> 16), th)
            out.append((sat(x), sat(y)))
            t += 1
        j += 1
        pos += 1
        if pilots and pos == GROUP + PILOT:
            pos = 0
            s += 1
            t = 0
    return out


class S2Front(Elaboratable):
    """Ring symbols (64-bit beats) in, cells out (16 bits: I low byte)."""

    def __init__(self):
        self.start = Signal()          # pulse: a frame begins
        self.lead = Signal(5)
        self.n_cells = Signal(16)
        self.pilots = Signal()
        self.gain = Signal(17)
        # the segment table (CPU side): angle, step, address, write
        self.seg_we = Signal()
        self.seg_addr = Signal(5)
        self.seg_angle = Signal(32)
        self.seg_step = Signal(32)
        # beats in (a FIFO's read side)
        self.beat = Signal(64)
        self.beat_rdy = Signal()
        self.beat_en = Signal()
        # cells out (a FIFO's read side)
        self.cell = Signal(16)
        self.cell_rdy = Signal()
        self.cell_en = Signal()

    def elaborate(self, platform):
        m = Module()
        m.submodules.tab = tab = Memory(shape=64, depth=32, init=[])
        tw = tab.write_port()
        tr = tab.read_port(domain='comb')
        m.d.comb += [tw.addr.eq(self.seg_addr), tw.data.eq(Cat(self.seg_angle, self.seg_step)),
                     tw.en.eq(self.seg_we)]

        m.submodules.cf = cf = SyncFIFOBuffered(width=16, depth=CF_DEPTH)
        m.d.comb += [self.cell.eq(cf.r_data), self.cell_rdy.eq(cf.r_rdy), cf.r_en.eq(self.cell_en)]

        # ---- the input side: one ring word a cycle
        half = Signal()                # word of the beat
        skip = Signal(5)               # lead words still to drop
        done_in = Signal()             # n_cells taken: swallow the rest
        cells_in = Signal(16)          # data symbols sent into the pipeline
        inflight = Signal(range(CF_DEPTH + STAGES + 8))
        pos = Signal(11)               # position in the 1476-symbol group
        seg = Signal(5)
        acc = Signal(32)
        step = Signal(32)
        (x1i, y1i), (x2i, y2i) = gold_states()
        gx1 = Signal(18, init=x1i)
        gy1 = Signal(18, init=y1i)
        gx2 = Signal(18, init=x2i)
        gy2 = Signal(18, init=y2i)

        word = Signal(32)
        m.d.comb += word.eq(Mux(half, self.beat[32:], self.beat[:32]))
        room = Signal()
        m.d.comb += room.eq(cf.level + inflight < CF_DEPTH - 2)
        take = Signal()
        m.d.comb += take.eq(self.beat_rdy & (done_in | (skip != 0) | room))
        m.d.comb += self.beat_en.eq(take & half)
        with m.If(take):
            m.d.sync += half.eq(~half)

        is_data = Signal()
        m.d.comb += is_data.eq(~self.pilots | (pos < GROUP))
        emit = Signal()
        m.d.comb += emit.eq(take & ~done_in & (skip == 0) & is_data)

        def gold_step(x, y):
            bx = x[7] ^ x[0]
            by = y[10] ^ y[7] ^ y[5] ^ y[0]
            return [x.eq(Cat(x[1:], bx)), y.eq(Cat(y[1:], by))]

        r_now = Signal(2)
        m.d.comb += r_now.eq(Cat(gx1[0] ^ gy1[0], gx2[0] ^ gy2[0]))
        # the angle of this symbol: A_s + t B_s, rounded, plus the
        # descrambling quarter turns (below)
        th0 = Signal(16)

        with m.If(self.start):
            m.d.sync += [half.eq(0), skip.eq(self.lead), done_in.eq(0), cells_in.eq(0),
                         pos.eq(0), seg.eq(0), acc.eq(0), step.eq(0),
                         gx1.eq(x1i), gy1.eq(y1i), gx2.eq(x2i), gy2.eq(y2i)]
        with m.Elif(take & ~done_in):
            with m.If(skip != 0):
                m.d.sync += skip.eq(skip - 1)
            with m.Else():
                # past the lead: a frame symbol (data or pilot)
                m.d.sync += gold_step(gx1, gy1) + gold_step(gx2, gy2)
                with m.If(self.pilots & (pos == GROUP + PILOT - 1)):
                    m.d.sync += [pos.eq(0), seg.eq(seg + 1)]
                with m.Else():
                    m.d.sync += pos.eq(pos + 1)
                with m.If(is_data):
                    m.d.sync += cells_in.eq(cells_in + 1)
                    with m.If(cells_in + 1 == self.n_cells):
                        m.d.sync += done_in.eq(1)
        # the accumulator: A_s at the group's first data symbol, + B_s after
        # each (the table read is combinational on seg)
        m.d.comb += tr.addr.eq(seg)
        first = Signal()
        m.d.comb += first.eq(~self.pilots & (cells_in == 0) | self.pilots & (pos == 0))
        cur_acc = Signal(32)
        cur_step = Signal(32)
        m.d.comb += [cur_acc.eq(Mux(first, tr.data[:32], acc)),
                     cur_step.eq(Mux(first, tr.data[32:], step))]
        m.d.comb += th0.eq(cur_acc[16:] + cur_acc[15] + ((C(4, 3) - r_now)[:2] << 14))
        with m.If(emit):
            m.d.sync += [acc.eq(cur_acc + cur_step), step.eq(cur_step)]

        # ---- CORDIC pipeline
        v = Signal()
        x = Signal(signed(19))
        y = Signal(signed(19))
        z = Signal(signed(17))
        re = Signal(signed(16))
        im = Signal(signed(16))
        th = Signal(signed(16))
        m.d.comb += [re.eq(word[:16]), im.eq(word[16:]), th.eq(th0)]
        m.d.sync += v.eq(emit)
        with m.If(th >= 16384):
            m.d.sync += [x.eq(-im), y.eq(re), z.eq(th - 16384)]
        with m.Elif(th < -16384):
            m.d.sync += [x.eq(im), y.eq(-re), z.eq(th + 16384)]
        with m.Else():
            m.d.sync += [x.eq(re), y.eq(im), z.eq(th)]
        for i in range(STAGES):
            nv = Signal(name=f'cv{i}')
            nx = Signal(signed(19), name=f'cx{i}')
            ny = Signal(signed(19), name=f'cy{i}')
            nz = Signal(signed(17), name=f'cz{i}')
            m.d.sync += nv.eq(v)
            with m.If(z >= 0):
                m.d.sync += [nx.eq(x - (y >> i)), ny.eq(y + (x >> i)), nz.eq(z - ATAN[i])]
            with m.Else():
                m.d.sync += [nx.eq(x + (y >> i)), ny.eq(y - (x >> i)), nz.eq(z + ATAN[i])]
            v, x, y, z = nv, nx, ny, nz
        # ---- gain (two DSP products, registered in and out) and saturation
        g = Signal(signed(18))
        m.d.comb += g.eq(Cat(self.gain, C(0, 1)))
        pv = Signal()
        px = Signal(signed(19))
        py = Signal(signed(19))
        m.d.sync += [pv.eq(v), px.eq(x), py.eq(y)]
        mv = Signal()
        mx = Signal(signed(37))
        my = Signal(signed(37))
        m.d.sync += [mv.eq(pv), mx.eq(px * g + (1 << (GAIN_SHIFT - 1))),
                     my.eq(py * g + (1 << (GAIN_SHIFT - 1)))]
        sv = Signal()
        sx = Signal(signed(8))
        sy = Signal(signed(8))
        qx = Signal(signed(17))
        qy = Signal(signed(17))
        m.d.comb += [qx.eq(mx >> GAIN_SHIFT), qy.eq(my >> GAIN_SHIFT)]
        m.d.sync += [sv.eq(mv),
                     sx.eq(Mux(qx > 127, 127, Mux(qx < -128, -128, qx))),
                     sy.eq(Mux(qy > 127, 127, Mux(qy < -128, -128, qy)))]
        m.d.comb += [cf.w_data.eq(Cat(sx, sy)), cf.w_en.eq(sv)]
        # in flight: into the pipeline, out into the FIFO
        m.d.sync += inflight.eq(inflight + emit - sv)
        with m.If(self.start):
            m.d.sync += inflight.eq(0)
        return m
