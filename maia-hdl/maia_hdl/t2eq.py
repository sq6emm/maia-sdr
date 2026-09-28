#
# SPDX-License-Identifier: MIT
#

"""DVB-T2 equalizer (tezuka_fw_simple): between the T2 front end's carrier
stream (t2ofdm.py) and the DATV ring. For each data symbol it does what the
A9 did carrier by carrier:

- z_k = c_k G_k (G the channel inverse the ARM loads, two banks: the ARM
  fills one while the other is in use and flips ``gbank``, taken at each
  symbol's start), kept by carrier index k in a symbol RAM;
- the phase slope across the carriers (timing): the angle of
  sum z'_(k+D) conj(z'_k) over the symbol's scattered pilots (z' the pilot
  with its sign taken out: prbs[k] ^ pn[j]; D = dx dy, or dx in the frame
  closing symbol), times 1/D;
- the common phase: the angle of sum z'_k e^(-j slope k) over the pilots;
- the cells z_k e^(-j (a + slope k)), 7 bits a component (a cell unit is
  20), two to a ring word, carriers 0..1704 in order.

Symbols before ``p2`` (the P2 symbols: the ARM estimates the channel from
them) and everything while ``enable`` is off pass through as they come.

Input: the front end's carrier words (bit 16 set; a header has bit 0 set:
payload = re[15:1] | im[15:1] << 15, symbol j in bits 7:0). Output: the
same headers with payload bit 29 (word bit 31) set on equalized symbols
(the frame start in bits 28:8 then, 21 bits), followed by 853 cell words:
bits 7:1 I and 14:8 Q of carrier 2i, 23:17 and 30:24 of carrier 2i + 1
(carrier 1705 is 0), bit 16 set.

Numbers: c 16 bits; G 16 bits (the ARM scales it so that z has a unit of
1280, ``gshift`` bits down); z = sat16((c G + half) >> gshift); angles in
turns (16 bits; 32 for phases); e^(j phase) from a 1024-entry table (Q15);
angles by a 16-step CORDIC after scaling the sums to 19 bits.
"""

import numpy as np
from amaranth import *
from amaranth.lib.fifo import SyncFIFOBuffered
from amaranth.lib.memory import Memory

N = 2048
CARRIERS = 1705
LUT_BITS = 10
Z_UNIT = 1280
CELL_SHIFT = 21          # (z e) >> 21: z unit 1280 x 2^15 -> 20
CELL_MAX = 63
NORM_BITS = 19
CORDIC_STEPS = 16
ACC_BITS = 48

PN_SEQUENCE = [77, 194, 175, 123, 216, 195, 201, 161, 231, 108, 154, 9, 10, 241, 195, 17, 79, 7, 252, 162, 128, 142, 148, 98, 233, 173, 123, 113, 45, 111, 74, 200, 165, 155, 176, 105, 204, 80, 191, 17, 73, 146, 126, 107, 177, 201, 252, 140, 24, 187, 148, 155, 48, 205, 9, 221, 215, 73, 231, 4, 245, 123, 65, 222, 199, 231, 177, 118, 225, 44, 86, 87, 67, 43, 81, 176, 184, 18, 223, 14, 20, 136, 126, 36, 216, 12, 151, 240, 147, 116, 173, 118, 39, 14, 88, 254, 23, 116, 178, 120, 29, 141, 56, 33, 227, 147, 242, 234, 15, 253, 77, 36, 222, 32]


def pn_bits():
    """pn[j] for j = 0..255 (trxd ofdm.rs: the PN sequence MSB first)."""
    return [(PN_SEQUENCE[j // 8] >> (7 - j % 8)) & 1 for j in range(256)]


def prbs_bits():
    """prbs[k], k = 0..1704 (trxd ofdm.rs Ofdm::oversampled)."""
    sr = 0x7ff
    out = []
    for _ in range(CARRIERS):
        b = (sr ^ (sr >> 2)) & 1
        out.append(sr & 1)
        sr >>= 1
        if b:
            sr |= 0x400
    return out


def bitrev(i, bits=11):
    return int(format(i, f'0{bits}b')[::-1], 2)


def carrier_of_position():
    """The carrier k of each carrier word of a symbol, in the order the
    front end sends them (bit-reversed bins, active ones only)."""
    left = (N - CARRIERS) // 2 + 1
    k_of_bin = {}
    for k in range(CARRIERS):
        b = left + k
        k_of_bin[b - N // 2 if b >= N // 2 else b + N // 2] = k
    return [k_of_bin[bitrev(i)] for i in range(N) if bitrev(i) in k_of_bin]


def lut_tables():
    ph = 2 * np.pi * np.arange(2**LUT_BITS) / 2**LUT_BITS
    return ([int(v) for v in np.round(32767 * np.cos(ph))],
            [int(v) for v in np.round(32767 * np.sin(ph))])


def atan_table():
    return [int(round(np.arctan(2.0**-i) / (2 * np.pi) * 65536))
            for i in range(CORDIC_STEPS)]


def s16(x):
    x &= 0xFFFF
    return x - 65536 if x & 0x8000 else x


def sat(x, m):
    return max(-m, min(m, x))


def cordic(x, y):
    """Angle of (x, y) in turns x 65536 (0..65535), as the HDL."""
    while max(abs(x), abs(y)) >= 1 << NORM_BITS:
        x >>= 1
        y >>= 1
    ang = 0
    if x < 0:
        x, y, ang = -x, -y, 32768
    for i, at in enumerate(atan_table()):
        if y > 0:
            x, y, ang = x + (y >> i), y - (x >> i), ang + at
        else:
            x, y, ang = x - (y >> i), y + (x >> i), ang - at
    return ang & 0xFFFF


class Model:
    """Bit-exact model. ``regs``: enable, p2, dx, dy, fc_j, rec_d, rec_fc,
    gshift; ``g``: two lists of 1705 (re, im) (banks), ``gbank``."""
    def __init__(self):
        self.pos_k = carrier_of_position()
        self.prbs = prbs_bits()
        self.pn = pn_bits()
        self.cos, self.sin = lut_tables()

    def e(self, phase):
        idx = (phase >> (32 - LUT_BITS)) & (2**LUT_BITS - 1)
        return self.cos[idx], self.sin[idx]

    def symbol(self, j, carriers, regs, g):
        """carriers: 1705 (re, im) in the front end's order -> the 853 cell
        words."""
        z = [None] * CARRIERS
        gs = regs['gshift']
        for pos, (cr, ci) in enumerate(carriers):
            k = self.pos_k[pos]
            gr, gi = g[k]
            half = 1 << (gs - 1)
            z[k] = (sat((cr * gr - ci * gi + half) >> gs, 32767),
                    sat((cr * gi + ci * gr + half) >> gs, 32767))
        dx, dy = regs['dx'], regs['dy']
        if j == regs['fc_j']:
            d, k0, rec = dx, 0, regs['rec_fc']
        else:
            d, k0, rec = dx * dy, dx * (j & (dy - 1)), regs['rec_d']
        pil = []
        for k in range(k0, CARRIERS, d):
            neg = self.prbs[k] ^ self.pn[j & 255]
            zr, zi = z[k]
            pil.append((k, -zr if neg else zr, -zi if neg else zi))
        dsr = dsi = 0
        for (_, ar, ai), (_, br, bi) in zip(pil, pil[1:]):
            # b conj(a)
            dsr += br * ar + bi * ai
            dsi += bi * ar - br * ai
        ang = cordic(dsr, dsi)
        step = (-s16(ang) * rec) & 0xFFFFFFFF     # phase a carrier
        cr_ = ci_ = 0
        for k, pr, pi in pil:
            c, s = self.e((step * k) & 0xFFFFFFFF)
            cr_ += pr * c - pi * s
            ci_ += pr * s + pi * c
        a = cordic(cr_, ci_)
        phase = (-(a << 16)) & 0xFFFFFFFF
        cells = []
        for k in range(CARRIERS):
            c, s = self.e(phase)
            zr, zi = z[k]
            vr = zr * c - zi * s
            vi = zr * s + zi * c
            h = 1 << (CELL_SHIFT - 1)
            cells.append((sat((vr + h) >> CELL_SHIFT, CELL_MAX),
                          sat((vi + h) >> CELL_SHIFT, CELL_MAX)))
            phase = (phase + step) & 0xFFFFFFFF
        cells.append((0, 0))
        words = []
        for i in range(0, CARRIERS + 1, 2):
            (a0, b0), (a1, b1) = cells[i], cells[i + 1]
            words.append(((a0 & 0x7F) << 1) | ((b0 & 0x7F) << 8) | (1 << 16)
                         | ((a1 & 0x7F) << 17) | ((b1 & 0x7F) << 24))
        return words, cells[:CARRIERS]


class T2Eq(Elaboratable):
    """See the module docstring. One clock domain (``sync``)."""
    def __init__(self):
        self.pos_k = carrier_of_position()
        # registers
        self.enable = Signal()
        self.p2 = Signal(8, init=8)
        self.dx = Signal(6, init=6)
        self.dy = Signal(3, init=2)
        self.fc_j = Signal(8, init=255)
        self.rec_d = Signal(16)
        self.rec_fc = Signal(16)
        self.gshift = Signal(5, init=16)
        self.gbank = Signal()
        self.g_waddr = Signal(11)
        self.g_wbank = Signal()
        self.g_wdata = Signal(32)
        self.g_we = Signal()
        self.symbols = Signal(16)   # out: symbols equalized
        # stream in (from the carrier FIFO) and out
        self.i_data = Signal(32)
        self.i_rdy = Signal()
        self.i_en = Signal()        # out: take i_data
        self.o_data = Signal(32)
        self.o_rdy = Signal()       # out FIFO has room
        self.o_en = Signal()        # out: o_data valid

    def elaborate(self, platform):
        m = Module()
        cos_t, sin_t = lut_tables()
        m.submodules.gmem = gmem = Memory(shape=32, depth=2 * N, init=[])
        g_rd = gmem.read_port()
        g_wr = gmem.write_port()
        # two symbol banks: the collector fills one while the processor
        # empties the other (a symbol's carriers follow the last one's with
        # no gap: the FFT's output runs on)
        m.submodules.sym = sym = Memory(shape=32, depth=2 * N, init=[])
        z_rd = sym.read_port()
        z_wr = sym.write_port()
        m.submodules.posk = posk = Memory(shape=11, depth=N, init=self.pos_k + [0] * (N - len(self.pos_k)))
        k_rd = posk.read_port()
        prbs = prbs_bits()
        m.submodules.prbs = prbs_m = Memory(shape=1, depth=N, init=prbs + [0] * (N - len(prbs)))
        prbs_rd = prbs_m.read_port()
        m.submodules.pn = pn_m = Memory(shape=1, depth=256, init=pn_bits())
        pn_rd = pn_m.read_port()
        m.submodules.cos = cos_m = Memory(shape=signed(16), depth=2**LUT_BITS, init=cos_t)
        m.submodules.sin = sin_m = Memory(shape=signed(16), depth=2**LUT_BITS, init=sin_t)
        cos_rd = cos_m.read_port()
        sin_rd = sin_m.read_port()
        atans = Array(C(v, 16) for v in atan_table())

        # G writes (the ARM)
        m.d.comb += [g_wr.addr.eq(Cat(self.g_waddr, self.g_wbank)),
                     g_wr.data.eq(self.g_wdata), g_wr.en.eq(self.g_we)]

        def sat16(x):
            return Mux(x > 32767, 32767, Mux(x < -32767, -32767, x))

        def satc(x):
            return Mux(x > CELL_MAX, CELL_MAX, Mux(x < -CELL_MAX, -CELL_MAX, x))

        is_hdr = self.i_data[0]
        hdr_j = self.i_data[1:9]

        # ---- handoff: collector -> processor
        p_busy = Signal()
        p_start = Signal()
        p_bank = Signal()
        p_eq = Signal()
        p_j = Signal(8)
        p_hdr = Signal(32)

        # ---- collector
        cbank = Signal()
        c_eq = Signal()
        c_j = Signal(8)
        c_hdr = Signal(32)
        cnt = Signal(12)
        ck = Signal(11)
        c_re = Signal(signed(16))
        c_im = Signal(signed(16))
        gbank_c = Signal()
        prod = [Signal(signed(34), name=f'cprod{i}') for i in range(4)]
        m.d.comb += [k_rd.addr.eq(cnt), g_rd.addr.eq(Cat(ck, gbank_c))]
        with m.FSM(name='collect'):
            with m.State('WAIT'):
                with m.If(self.i_rdy):
                    m.d.comb += self.i_en.eq(1)
                    # (carriers without a header: dropped)
                    with m.If(is_hdr):
                        eq = self.enable & (hdr_j >= self.p2)
                        m.d.sync += [c_eq.eq(eq), c_j.eq(hdr_j), cnt.eq(0), gbank_c.eq(self.gbank),
                                     c_hdr.eq(Mux(eq, self.i_data | (1 << 31), self.i_data & 0x7FFFFFFF))]
                        m.next = 'COLLECT'
            with m.State('COLLECT'):
                with m.If(cnt == CARRIERS):
                    m.next = 'HAND'
                with m.Elif(self.i_rdy):
                    with m.If(is_hdr):
                        m.next = 'WAIT'      # cut short: dropped; the header anew
                    with m.Else():
                        m.d.comb += self.i_en.eq(1)
                        with m.If(c_eq):
                            m.d.sync += [c_re.eq(Cat(C(0, 1), self.i_data[1:16]).as_signed()),
                                         c_im.eq(Cat(C(0, 1), self.i_data[17:32]).as_signed())]
                            m.next = 'C_K'
                        with m.Else():
                            # as it came, by position
                            m.d.comb += [z_wr.addr.eq(Cat(cnt[:11], cbank)), z_wr.data.eq(self.i_data),
                                         z_wr.en.eq(1)]
                            m.d.sync += cnt.eq(cnt + 1)
            with m.State('C_K'):
                m.d.sync += ck.eq(k_rd.data)
                m.next = 'C_G'
            with m.State('C_G'):
                m.next = 'C_MUL'
            with m.State('C_MUL'):
                gr = g_rd.data[:16].as_signed()
                gi = g_rd.data[16:].as_signed()
                m.d.sync += [prod[0].eq(c_re * gr), prod[1].eq(c_im * gi),
                             prod[2].eq(c_re * gi), prod[3].eq(c_im * gr)]
                m.next = 'C_WR'
            with m.State('C_WR'):
                sh = self.gshift
                rnd = Signal(signed(36))
                m.d.comb += rnd.eq((C(1, 36) << sh) >> 1)
                re = Signal(signed(36))
                im = Signal(signed(36))
                m.d.comb += [re.eq((prod[0] - prod[1] + rnd) >> sh),
                             im.eq((prod[2] + prod[3] + rnd) >> sh)]
                m.d.comb += [z_wr.addr.eq(Cat(ck, cbank)),
                             z_wr.data.eq(Cat(sat16(re)[:16], sat16(im)[:16])), z_wr.en.eq(1)]
                m.d.sync += cnt.eq(cnt + 1)
                m.next = 'COLLECT'
            with m.State('HAND'):
                with m.If(~p_busy):
                    m.d.comb += p_start.eq(1)
                    m.d.sync += [p_bank.eq(cbank), p_eq.eq(c_eq), p_j.eq(c_j), p_hdr.eq(c_hdr),
                                 cbank.eq(~cbank)]
                    m.next = 'WAIT'

        # ---- processor
        k = Signal(12)
        d = Signal(8)
        k0 = Signal(12)
        rec = Signal(16)
        acc_re = Signal(signed(ACC_BITS))
        acc_im = Signal(signed(ACC_BITS))
        prev_re = Signal(signed(17))
        prev_im = Signal(signed(17))
        first = Signal()
        step = Signal(32)
        phase = Signal(32)
        ang = Signal(16)
        cx = Signal(signed(ACC_BITS + 2))
        cy = Signal(signed(ACC_BITS + 2))
        ci = Signal(5)
        after = Signal()
        cell0 = Signal(14)
        half = Signal()
        pp = [Signal(signed(34), name=f'pprod{i}') for i in range(4)]
        zr = Signal(signed(16))
        zi = Signal(signed(16))
        neg = Signal()
        m.d.comb += [z_rd.addr.eq(Cat(k[:11], p_bank)), prbs_rd.addr.eq(k), pn_rd.addr.eq(p_j),
                     cos_rd.addr.eq(phase[32 - LUT_BITS:]),
                     sin_rd.addr.eq(phase[32 - LUT_BITS:])]
        with m.FSM(name='process'):
            with m.State('IDLE'):
                with m.If(p_start):
                    m.d.sync += p_busy.eq(1)
                    m.next = 'START'
            with m.State('START'):
                with m.If(p_eq):
                    m.d.sync += [first.eq(1), acc_re.eq(0), acc_im.eq(0)]
                    with m.If(p_j == self.fc_j):
                        m.d.sync += [d.eq(self.dx), k0.eq(0), k.eq(0), rec.eq(self.rec_fc)]
                    with m.Else():
                        m.d.sync += [d.eq(self.dx * self.dy),
                                     k0.eq(self.dx * (p_j & (self.dy - 1))),
                                     k.eq(self.dx * (p_j & (self.dy - 1))),
                                     rec.eq(self.rec_d)]
                    m.next = 'SLOPE_RD'
                with m.Else():
                    m.d.sync += k.eq(0)
                    m.next = 'RAW_HDR'
            # -- raw: the words as they came
            with m.State('RAW_HDR'):
                with m.If(self.o_rdy):
                    m.d.comb += [self.o_data.eq(p_hdr), self.o_en.eq(1)]
                    m.next = 'RAW_RD'
            with m.State('RAW_RD'):
                m.next = 'RAW_OUT'           # z_rd for k next cycle
            with m.State('RAW_OUT'):
                with m.If(self.o_rdy):
                    m.d.comb += [self.o_data.eq(z_rd.data), self.o_en.eq(1)]
                    with m.If(k == CARRIERS - 1):
                        m.d.sync += p_busy.eq(0)
                        m.next = 'IDLE'
                    with m.Else():
                        m.d.sync += k.eq(k + 1)
                        m.next = 'RAW_RD'
            # -- slope: pilots k0, k0 + d, ...
            with m.State('SLOPE_RD'):
                with m.If(k >= CARRIERS):
                    m.d.sync += [cx.eq(acc_re), cy.eq(acc_im), after.eq(0)]
                    m.next = 'NORM'
                with m.Else():
                    m.next = 'SLOPE_ACC'
            with m.State('SLOPE_ACC'):
                pr = Signal(signed(17))
                pi = Signal(signed(17))
                zz_r = z_rd.data[:16].as_signed()
                zz_i = z_rd.data[16:].as_signed()
                nn = prbs_rd.data ^ pn_rd.data
                m.d.comb += [pr.eq(Mux(nn, -zz_r, zz_r)), pi.eq(Mux(nn, -zz_i, zz_i))]
                with m.If(~first):
                    m.d.sync += [acc_re.eq(acc_re + pr * prev_re + pi * prev_im),
                                 acc_im.eq(acc_im + pi * prev_re - pr * prev_im)]
                m.d.sync += [prev_re.eq(pr), prev_im.eq(pi), first.eq(0), k.eq(k + d)]
                m.next = 'SLOPE_RD'
            # -- CORDIC (after = 0: the slope, 1: the common phase)
            with m.State('NORM'):
                ax = Signal(ACC_BITS + 2)
                ay = Signal(ACC_BITS + 2)
                m.d.comb += [ax.eq(Mux(cx < 0, -cx, cx)), ay.eq(Mux(cy < 0, -cy, cy))]
                with m.If((ax >= (1 << NORM_BITS)) | (ay >= (1 << NORM_BITS))):
                    m.d.sync += [cx.eq(cx >> 1), cy.eq(cy >> 1)]
                with m.Else():
                    with m.If(cx < 0):
                        m.d.sync += [cx.eq(-cx), cy.eq(-cy), ang.eq(32768)]
                    with m.Else():
                        m.d.sync += ang.eq(0)
                    m.d.sync += ci.eq(0)
                    m.next = 'CORDIC'
            with m.State('CORDIC'):
                with m.If(ci == CORDIC_STEPS):
                    with m.If(~after):
                        m.d.sync += [step.eq(-(ang.as_signed() * rec)),
                                     k.eq(k0), acc_re.eq(0), acc_im.eq(0)]
                        m.next = 'CPE_PH'
                    with m.Else():
                        m.d.sync += [phase.eq(-(Cat(C(0, 16), ang))), k.eq(0), half.eq(0)]
                        m.next = 'OUT_HDR'
                with m.Else():
                    with m.If(cy > 0):
                        m.d.sync += [cx.eq(cx + (cy >> ci)), cy.eq(cy - (cx >> ci)),
                                     ang.eq(ang + atans[ci])]
                    with m.Else():
                        m.d.sync += [cx.eq(cx - (cy >> ci)), cy.eq(cy + (cx >> ci)),
                                     ang.eq(ang - atans[ci])]
                    m.d.sync += ci.eq(ci + 1)
            # -- common phase
            with m.State('CPE_PH'):
                with m.If(k >= CARRIERS):
                    m.d.sync += [cx.eq(acc_re), cy.eq(acc_im), after.eq(1)]
                    m.next = 'NORM'
                with m.Else():
                    m.d.sync += phase.eq(step * k)
                    m.next = 'CPE_RD'
            with m.State('CPE_RD'):
                m.d.sync += [zr.eq(z_rd.data[:16].as_signed()),
                             zi.eq(z_rd.data[16:].as_signed()),
                             neg.eq(prbs_rd.data ^ pn_rd.data)]
                m.next = 'CPE_ACC'
            with m.State('CPE_ACC'):
                pr = Signal(signed(17))
                pi = Signal(signed(17))
                m.d.comb += [pr.eq(Mux(neg, -zr, zr)), pi.eq(Mux(neg, -zi, zi))]
                m.d.sync += [acc_re.eq(acc_re + pr * cos_rd.data - pi * sin_rd.data),
                             acc_im.eq(acc_im + pr * sin_rd.data + pi * cos_rd.data),
                             k.eq(k + d)]
                m.next = 'CPE_PH'
            # -- the cells
            with m.State('OUT_HDR'):
                with m.If(self.o_rdy):
                    m.d.comb += [self.o_data.eq(p_hdr), self.o_en.eq(1)]
                    m.next = 'OUT_RD'
            with m.State('OUT_RD'):
                m.next = 'OUT_MUL'
            with m.State('OUT_MUL'):
                zzr = z_rd.data[:16].as_signed()
                zzi = z_rd.data[16:].as_signed()
                m.d.sync += [pp[0].eq(zzr * cos_rd.data), pp[1].eq(zzi * sin_rd.data),
                             pp[2].eq(zzr * sin_rd.data), pp[3].eq(zzi * cos_rd.data)]
                m.next = 'OUT_CELL'
            with m.State('OUT_CELL'):
                hh = 1 << (CELL_SHIFT - 1)
                vr = Signal(signed(36))
                vi = Signal(signed(36))
                m.d.comb += [vr.eq((pp[0] - pp[1] + hh) >> CELL_SHIFT),
                             vi.eq((pp[2] + pp[3] + hh) >> CELL_SHIFT)]
                cell = Cat(satc(vr)[:7], satc(vi)[:7])
                last = Signal()
                m.d.comb += last.eq(k == CARRIERS - 1)
                with m.If(~half):
                    with m.If(last):
                        with m.If(self.o_rdy):
                            m.d.comb += [self.o_data.eq(Cat(C(0, 1), cell, C(0, 1), C(1, 1))),
                                         self.o_en.eq(1)]
                            m.d.sync += [self.symbols.eq(self.symbols + 1), p_busy.eq(0)]
                            m.next = 'IDLE'
                    with m.Else():
                        m.d.sync += [cell0.eq(cell), half.eq(1), k.eq(k + 1),
                                     phase.eq(phase + step)]
                        m.next = 'OUT_RD'
                with m.Else():
                    with m.If(self.o_rdy):
                        m.d.comb += [self.o_data.eq(Cat(C(0, 1), cell0, C(0, 1), C(1, 1), cell, C(0, 1))),
                                     self.o_en.eq(1)]
                        m.d.sync += [half.eq(0), k.eq(k + 1), phase.eq(phase + step)]
                        with m.If(last):
                            m.d.sync += [self.symbols.eq(self.symbols + 1), p_busy.eq(0)]
                            m.next = 'IDLE'
                        with m.Else():
                            m.next = 'OUT_RD'
        return m
