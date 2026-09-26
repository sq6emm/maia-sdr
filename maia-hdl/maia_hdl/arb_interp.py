#
# SPDX-License-Identifier: MIT
#

"""Arbitrary-rate pulse-shaping interpolator (DVB-S2 transmit).

Symbols in (AXI-Stream, one complex symbol per transfer), samples out at any
rate above twice the symbol rate. Each output sample is computed directly
from the last ``span`` symbols:

    y = sum_k sym[k] * h[k * P + phase]      (sym[0] the newest symbol)

where ``phase`` is the output time since the newest symbol in 1/P of a
symbol, from a phase accumulator advanced by ``step`` = symbol rate / output
rate * 2**32 per output sample; a carry takes in the next symbol. ``h`` is
the pulse (RRC) sampled at P phases per symbol and loaded by the CPU, so any
symbol rate and roll-off work without rebuilding: 250 kS/s from the
3.072 MS/s DAC stream, for instance, which no whole interpolation factor
reaches.

The output side is back-pressured (AXI-Stream): put a FIFO in front of the
DAC and the symbol rate follows the DAC clock exactly.
"""

from amaranth import *
from amaranth.lib.memory import Memory
import numpy as np


class ArbInterpolator(Elaboratable):
    """Arbitrary-rate interpolator.

    Parameters
    ----------
    span : int
        Pulse length in symbols (taps per output sample).
    phases_log2 : int
        log2 of the phases per symbol in the coefficient table.
    iq_width : int
        Width of the input symbol and output sample components.
    coeff_width : int
        Coefficient width. Coefficients are fractions: the sum of products
        is shifted right by ``coeff_width - 1`` (rounded) and saturated.

    Attributes
    ----------
    step : Signal(32), in
        Symbol rate / output rate * 2**32 (below 2**31: at least 2 samples
        per symbol).
    coeff_waddr, coeff_wdata, coeff_wren : in
        Coefficient table write port; address ``k * P + phase``.
    in_re, in_im, in_valid, in_ready : symbols (AXI-Stream handshake)
    out_re, out_im, out_valid, out_ready : samples (AXI-Stream handshake)
    """
    def __init__(self, span=16, phases_log2=8, iq_width=16, coeff_width=18):
        self.span = span
        self.phases_log2 = phases_log2
        self.iw = iq_width
        self.cw = coeff_width
        self.depth = span << phases_log2
        self.aw = (self.depth - 1).bit_length()

        self.step = Signal(32)
        self.coeff_waddr = Signal(self.aw)
        self.coeff_wdata = Signal(signed(coeff_width))
        self.coeff_wren = Signal()

        self.in_re = Signal(signed(iq_width))
        self.in_im = Signal(signed(iq_width))
        self.in_valid = Signal()
        self.in_ready = Signal()

        self.out_re = Signal(signed(iq_width))
        self.out_im = Signal(signed(iq_width))
        self.out_valid = Signal()
        self.out_ready = Signal()

    def model(self, step, coeffs, re, im, nout):
        """Integer model: ``nout`` samples from the symbols (numpy arrays)."""
        span, P = self.span, 1 << self.phases_log2
        coeffs = np.asarray(coeffs, dtype=np.int64)
        hist_re = np.zeros(span, dtype=np.int64)
        hist_im = np.zeros(span, dtype=np.int64)
        acc, n = 0, 0
        shift = self.cw - 1
        lim = 2**(self.iw - 1)
        out = np.zeros((nout, 2), dtype=np.int64)
        for m in range(nout):
            acc += step
            if acc >= 2**32:
                acc -= 2**32
                hist_re = np.roll(hist_re, 1)
                hist_im = np.roll(hist_im, 1)
                hist_re[0], hist_im[0] = re[n], im[n]
                n += 1
            ph = acc >> (32 - self.phases_log2)
            h = coeffs[np.arange(span) * P + ph]
            sr = int(np.sum(h * hist_re)) + (1 << (shift - 1))
            si = int(np.sum(h * hist_im)) + (1 << (shift - 1))
            out[m] = [min(max(sr >> shift, -lim), lim - 1),
                      min(max(si >> shift, -lim), lim - 1)]
        return out

    def elaborate(self, platform):
        m = Module()
        span, pl2 = self.span, self.phases_log2

        m.submodules.coeffs = coeffs = Memory(
            shape=signed(self.cw), depth=self.depth, init=[])
        wr = coeffs.write_port()
        rd = coeffs.read_port()      # synchronous: data one cycle after addr
        m.d.comb += [wr.addr.eq(self.coeff_waddr),
                     wr.data.eq(self.coeff_wdata),
                     wr.en.eq(self.coeff_wren)]

        hist_re = Array(Signal(signed(self.iw), name=f'hre{k}')
                        for k in range(span))
        hist_im = Array(Signal(signed(self.iw), name=f'him{k}')
                        for k in range(span))

        acc = Signal(32)
        phase = Signal(pl2)
        k = Signal(range(span + 1))      # tap being addressed
        k_d = Signal(range(span + 1))    # tap whose coefficient is on rd.data
        k_valid = Signal()
        prod_valid = Signal()
        accw = self.iw + self.cw + (span - 1).bit_length() + 1
        sum_re = Signal(signed(accw))
        sum_im = Signal(signed(accw))
        prod_re = Signal(signed(self.iw + self.cw))
        prod_im = Signal(signed(self.iw + self.cw))
        shift = self.cw - 1
        lim = 2**(self.iw - 1)

        def sat(x):
            return Mux(x > lim - 1, lim - 1, Mux(x < -lim, -lim, x))

        m.d.comb += rd.addr.eq(Cat(phase, k[:(self.aw - pl2)]))

        # A sample taken empties the output register (DONE below, later in
        # the code, wins when it refills it in the same cycle).
        with m.If(self.out_valid & self.out_ready):
            m.d.sync += self.out_valid.eq(0)

        with m.FSM():
            with m.State('IDLE'):
                # Room in the output register: the next sample's time.
                with m.If(~self.out_valid | self.out_ready):
                    nxt = Signal(33)
                    m.d.comb += nxt.eq(acc + self.step)
                    m.d.sync += [acc.eq(nxt[:32]),
                                 phase.eq(nxt[32 - pl2:32])]
                    with m.If(nxt[32]):
                        m.next = 'FETCH'
                    with m.Else():
                        m.d.sync += [k.eq(0), k_valid.eq(0),
                                     prod_valid.eq(0),
                                     sum_re.eq(1 << (shift - 1)),
                                     sum_im.eq(1 << (shift - 1))]
                        m.next = 'MAC'
            with m.State('FETCH'):
                m.d.comb += self.in_ready.eq(1)
                with m.If(self.in_valid):
                    m.d.sync += [hist_re[0].eq(self.in_re),
                                 hist_im[0].eq(self.in_im)]
                    m.d.sync += [hist_re[j].eq(hist_re[j - 1])
                                 for j in range(1, span)]
                    m.d.sync += [hist_im[j].eq(hist_im[j - 1])
                                 for j in range(1, span)]
                    m.d.sync += [k.eq(0), k_valid.eq(0), prod_valid.eq(0),
                                 sum_re.eq(1 << (shift - 1)),
                                 sum_im.eq(1 << (shift - 1))]
                    m.next = 'MAC'
            with m.State('MAC'):
                # Three-stage pipeline: address k -> coefficient (rd.data)
                # -> product -> sum.
                with m.If(k < span):
                    m.d.sync += k.eq(k + 1)
                m.d.sync += [k_d.eq(k), k_valid.eq(k < span)]
                with m.If(k_valid):
                    m.d.sync += [prod_re.eq(rd.data * hist_re[k_d]),
                                 prod_im.eq(rd.data * hist_im[k_d])]
                m.d.sync += prod_valid.eq(k_valid)
                with m.If(prod_valid):
                    m.d.sync += [sum_re.eq(sum_re + prod_re),
                                 sum_im.eq(sum_im + prod_im)]
                with m.If((k == span) & ~k_valid & ~prod_valid):
                    m.next = 'DONE'
            with m.State('DONE'):
                m.d.sync += [self.out_re.eq(sat(sum_re >> shift)),
                             self.out_im.eq(sat(sum_im >> shift))]
                m.d.sync += self.out_valid.eq(1)
                m.next = 'IDLE'

        return m


def rrc_table(span, phases_log2, rolloff, coeff_width=18):
    """RRC coefficient table for ArbInterpolator: h(t) at t = k + p/P
    symbols after each symbol, centred on span/2, scaled so no output can
    exceed full scale for full-scale symbols (the worst phase's sum of
    |h| is 1)."""
    P = 1 << phases_log2
    t = (np.arange(span * P) / P) - span / 2
    b = rolloff
    h = np.empty_like(t)
    for i, x in enumerate(t):
        if abs(x) < 1e-12:
            h[i] = 1 - b + 4 * b / np.pi
        elif abs(abs(x) - 1 / (4 * b)) < 1e-9:
            h[i] = b / np.sqrt(2) * ((1 + 2 / np.pi) * np.sin(np.pi / (4 * b))
                                     + (1 - 2 / np.pi) * np.cos(np.pi / (4 * b)))
        else:
            h[i] = ((np.sin(np.pi * x * (1 - b))
                     + 4 * b * x * np.cos(np.pi * x * (1 + b)))
                    / (np.pi * x * (1 - (4 * b * x)**2)))
    # A gentle taper on the ends of the truncated pulse (Kaiser, beta 3):
    # side lobes of the cut RRC down, pass band untouched.
    h = h * np.kaiser(len(h), 3.0)
    worst = max(np.sum(np.abs(h[p::P])) for p in range(P))
    scale = (2**(coeff_width - 1) - 1) / worst
    return [int(round(v * scale)) for v in h]
