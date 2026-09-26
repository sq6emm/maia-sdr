#
# SPDX-License-Identifier: MIT
#

"""DVB-S2 header screening for the DATV receiver (tezuka_fw_simple).

After SymSync, one symbol per strobe: correlates the last 26 symbols with
the SOF in two coherent chunks of 13 and flags the symbol (bit 0 of its
imaginary part) when the chunk magnitudes add up to at least 12/16 of the
window's summed magnitude. Bit-exact with trxd's model
(``src/dvbs2/hdrdet.rs``), which explains the arithmetic. The CPU then
evaluates its full header metric only at flagged positions.

One window step per clock (26 steps and a few more per symbol); the symbol
leaves once its flag is known. With ``enable`` low symbols pass unchanged.
"""

from amaranth import *

SOF = 0x18D2E82
SOF_LEN = 26
CHUNK = 13
THR16 = 12


def sof_signs():
    out = []
    for s in range(SOF_LEN):
        bit = (SOF >> (SOF_LEN - 1 - s)) & 1
        q = 2 * bit + (s & 1)
        out.append({0: (1, 1), 1: (-1, 1), 2: (-1, -1), 3: (1, -1)}[q])
    return out


def mag(re, im, m, width):
    """max(|re|, |im|) + (3 min >> 3), combinational."""
    a = Signal(width, name=None)
    b = Signal(width, name=None)
    hi = Signal(width, name=None)
    lo = Signal(width, name=None)
    out = Signal(width + 1, name=None)
    m.d.comb += [
        a.eq(Mux(re < 0, -re, re)),
        b.eq(Mux(im < 0, -im, im)),
        hi.eq(Mux(a > b, a, b)),
        lo.eq(Mux(a > b, b, a)),
        out.eq(hi + ((3 * lo) >> 3)),
    ]
    return out


class HdrDet(Elaboratable):
    def __init__(self, width=16):
        self.w = width
        self.enable = Signal()
        self.strobe_in = Signal()
        self.re_in = Signal(signed(width))
        self.im_in = Signal(signed(width))
        self.strobe_out = Signal()
        self.re_out = Signal(signed(width))
        self.im_out = Signal(signed(width))

    def elaborate(self, platform):
        m = Module()
        w = self.w
        buf_re = Array(Signal(signed(w), name=f'buf_re{i}') for i in range(32))
        buf_im = Array(Signal(signed(w), name=f'buf_im{i}') for i in range(32))
        signs = sof_signs()
        sa = Array(Const(a, signed(2)) for a, _ in signs)
        sb = Array(Const(b, signed(2)) for _, b in signs)

        n = Signal(5)          # next write slot
        filled = Signal(range(SOF_LEN + 1))
        i = Signal(range(SOF_LEN + 8))
        ar = Signal(signed(24))
        ai = Signal(signed(24))
        num = Signal(26)
        den = Signal(26)
        y_re = Signal(signed(w))
        y_im = Signal(signed(w))

        # Pipeline (one window step a clock, 16 ns): 1 buffer read and
        # signs; 2 accumulate, |s|; 3 |acc| at a chunk end, den sum;
        # 4 num sum.
        slot = Signal(5)
        m.d.comb += slot.eq(n - SOF_LEN + i)
        re1 = Signal(signed(w))
        im1 = Signal(signed(w))
        a1 = Signal(signed(2))
        b1 = Signal(signed(2))
        v1 = Signal()
        end1 = Signal()
        m.d.sync += [
            re1.eq(buf_re[slot]), im1.eq(buf_im[slot]),
            a1.eq(sa[i[:5]]), b1.eq(sb[i[:5]]),
            v1.eq(0), end1.eq(0),
        ]
        t_re = Signal(signed(w + 2))
        t_im = Signal(signed(w + 2))
        m.d.comb += [
            t_re.eq(Mux(a1 > 0, re1, -re1) + Mux(b1 > 0, im1, -im1)),
            t_im.eq(Mux(a1 > 0, im1, -im1) - Mux(b1 > 0, re1, -re1)),
        ]
        ar_next = Signal(signed(24))
        ai_next = Signal(signed(24))
        m.d.comb += [ar_next.eq(ar + t_re), ai_next.eq(ai + t_im)]
        mag_s = mag(re1, im1, m, w + 1)
        # stage 2 -> 3
        ms2 = Signal(w + 2)
        v2 = Signal()
        acc_re = Signal(signed(24))
        acc_im = Signal(signed(24))
        accv2 = Signal()
        m.d.sync += [ms2.eq(mag_s), v2.eq(v1), accv2.eq(0)]
        with m.If(v1):
            with m.If(end1):
                m.d.sync += [acc_re.eq(ar_next), acc_im.eq(ai_next),
                             accv2.eq(1), ar.eq(0), ai.eq(0)]
            with m.Else():
                m.d.sync += [ar.eq(ar_next), ai.eq(ai_next)]
        mag_acc = mag(acc_re, acc_im, m, 24)
        # stage 3 -> 4
        ma3 = Signal(25)
        accv3 = Signal()
        m.d.sync += [ma3.eq(mag_acc), accv3.eq(accv2)]
        with m.If(v2):
            m.d.sync += den.eq(den + ms2)
        with m.If(accv3):
            m.d.sync += num.eq(num + ma3)

        with m.If(~self.enable):
            m.d.sync += [n.eq(0), filled.eq(0)]
            m.d.comb += [self.strobe_out.eq(self.strobe_in),
                         self.re_out.eq(self.re_in),
                         self.im_out.eq(self.im_in)]

        with m.FSM(name='hdrdet'):
            with m.State('IDLE'):
                with m.If(self.enable & self.strobe_in):
                    m.d.sync += [
                        buf_re[n].eq(self.re_in),
                        buf_im[n].eq(self.im_in),
                        n.eq(n + 1),
                        y_re.eq(self.re_in),
                        y_im.eq(self.im_in),
                        filled.eq(Mux(filled < SOF_LEN, filled + 1, filled)),
                        i.eq(0), ar.eq(0), ai.eq(0), num.eq(0), den.eq(0),
                    ]
                    m.next = 'SUM'
            with m.State('SUM'):
                m.d.sync += i.eq(i + 1)
                with m.If(i < SOF_LEN):
                    m.d.sync += [v1.eq(1),
                                 end1.eq((i == CHUNK - 1) | (i == SOF_LEN - 1))]
                with m.If(i == SOF_LEN + 3):
                    m.next = 'CMP'
            with m.State('CMP'):
                flag = Signal()
                m.d.sync += flag.eq((filled >= SOF_LEN) & (16 * num >= THR16 * den)
                                    & (den > 0))
                m.next = 'OUT'
            with m.State('OUT'):
                with m.If(self.enable):
                    m.d.comb += [self.strobe_out.eq(1),
                                 self.re_out.eq(y_re),
                                 self.im_out.eq(Cat(flag, y_im[1:]))]
                m.next = 'IDLE'
        return m
