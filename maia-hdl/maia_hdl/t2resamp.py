#
# SPDX-License-Identifier: MIT
#

"""DVB-T2 receive resampler (tezuka_fw_simple).

ADC samples (3.072 MS/s) in, samples at the T2 elementary rate (131/71 MS/s
for a 1.7 MHz channel, or any rate below the input's) out, for the DDC
ring. Bit-exact with trxd's model (``src/dvbt2/resamp.rs``):

- ``d``, the time of the next output less the newest input's, Q.30 input
  samples: each input takes 1.0 off; when that leaves ``d <= 0`` an output
  is due at phase ``-d`` (in [0, 1)), and ``d += step`` (input samples per
  output, Q2.30);
- ``y = sat16((sum_k h[k P + p] x[n - k] + 2**13) >> 14)`` over the last 32
  inputs (x[n] the newest), ``p = (-d) >> 23`` (P = 128 phases);
- ``h`` (18 bits) loaded by the CPU at address ``k * 128 + p``.

Four multiply lanes (taps k = 4 j + lane): 8 cycles of products and about
six of pipeline an output, well inside the ~20 cycles between inputs at
62.5 MHz. An input arriving meanwhile waits in a holding register. With
``enable`` low the state resets (history cleared, d = step).
"""

from amaranth import *
from amaranth.lib.memory import Memory

SPAN = 32
PHASES_LOG2 = 7
LANES = 4
FRAC = 30
CW = 18
SHIFT = CW - 4    # coefficients unity at 2**17; output 3 bits up


class T2Resampler(Elaboratable):
    def __init__(self, width=16):
        self.w = width
        self.enable = Signal()
        self.step = Signal(32)
        self.coeff_waddr = Signal(12)
        self.coeff_wdata = Signal(signed(CW))
        self.coeff_wren = Signal()
        self.strobe_in = Signal()
        self.re_in = Signal(signed(width))
        self.im_in = Signal(signed(width))
        self.strobe_out = Signal()
        self.re_out = Signal(signed(16))
        self.im_out = Signal(signed(16))

    def elaborate(self, platform):
        m = Module()
        w = self.w
        per_lane = SPAN // LANES
        jbits = (per_lane - 1).bit_length()

        rds = []
        for lane in range(LANES):
            mem = Memory(shape=signed(CW), depth=per_lane << PHASES_LOG2,
                         init=[])
            m.submodules[f'coeffs{lane}'] = mem
            wr = mem.write_port()
            rd = mem.read_port()
            a = self.coeff_waddr
            m.d.comb += [
                wr.addr.eq(Cat(a[:PHASES_LOG2],
                               a[PHASES_LOG2 + 2:PHASES_LOG2 + 2 + jbits])),
                wr.data.eq(self.coeff_wdata),
                wr.en.eq(self.coeff_wren
                         & (a[PHASES_LOG2:PHASES_LOG2 + 2] == lane)),
            ]
            rds.append(rd)

        hist_re = Array(Signal(signed(w), name=f'hre{k}') for k in range(SPAN))
        hist_im = Array(Signal(signed(w), name=f'him{k}') for k in range(SPAN))

        pend = Signal()
        pend_re = Signal(signed(w))
        pend_im = Signal(signed(w))
        d = Signal(signed(34))
        nd = Signal(signed(34))
        phase = Signal(PHASES_LOG2)
        j = Signal(range(per_lane + 1))
        j_d = Signal(jbits)
        j_valid = Signal()
        prod_valid = Signal()
        pw = w + CW
        prods_re = [Signal(signed(pw), name=f'pre{l}') for l in range(LANES)]
        prods_im = [Signal(signed(pw), name=f'pim{l}') for l in range(LANES)]
        # Adder tree, one register level a stage: two-input adds only
        # (Vivado 2023.1 crashes absorbing a three-input one into the DSPs).
        pair_re = [Signal(signed(pw + 1), name=f'pare{l}') for l in range(2)]
        pair_im = [Signal(signed(pw + 1), name=f'paim{l}') for l in range(2)]
        quad_re = Signal(signed(pw + 2))
        quad_im = Signal(signed(pw + 2))
        pair_valid = Signal()
        quad_valid = Signal()
        accw = pw + (SPAN - 1).bit_length() + 1
        sum_re = Signal(signed(accw))
        sum_im = Signal(signed(accw))
        lim = 2**15

        def sat(x):
            return Mux(x > lim - 1, lim - 1, Mux(x < -lim, -lim, x))

        for rd in rds:
            m.d.comb += rd.addr.eq(Cat(phase, j[:jbits]))

        m.d.sync += self.strobe_out.eq(0)
        m.d.comb += nd.eq(d - (1 << FRAC))

        with m.If(~self.enable):
            m.d.sync += [d.eq(self.step), pend.eq(0)]
            m.d.sync += [hist_re[k].eq(0) for k in range(SPAN)]
            m.d.sync += [hist_im[k].eq(0) for k in range(SPAN)]
        with m.Else():
            with m.FSM():
                with m.State('IDLE'):
                    with m.If(pend):
                        m.d.sync += pend.eq(0)
                        m.d.sync += [hist_re[0].eq(pend_re),
                                     hist_im[0].eq(pend_im)]
                        m.d.sync += [hist_re[k].eq(hist_re[k - 1])
                                     for k in range(1, SPAN)]
                        m.d.sync += [hist_im[k].eq(hist_im[k - 1])
                                     for k in range(1, SPAN)]
                        with m.If(nd > 0):
                            m.d.sync += d.eq(nd)
                        with m.Else():
                            m.d.sync += [
                                d.eq(nd + self.step),
                                phase.eq((-nd)[FRAC - PHASES_LOG2:FRAC]),
                                j.eq(0), j_valid.eq(0), prod_valid.eq(0),
                                pair_valid.eq(0), quad_valid.eq(0),
                                sum_re.eq(1 << (SHIFT - 1)),
                                sum_im.eq(1 << (SHIFT - 1)),
                            ]
                            m.next = 'MAC'
                with m.State('MAC'):
                    # address j -> coefficients -> products -> sum
                    with m.If(j < per_lane):
                        m.d.sync += j.eq(j + 1)
                    m.d.sync += [j_d.eq(j[:jbits]), j_valid.eq(j < per_lane)]
                    with m.If(j_valid):
                        for lane in range(LANES):
                            k = Cat(C(lane, 2), j_d)
                            m.d.sync += [
                                prods_re[lane].eq(rds[lane].data * hist_re[k]),
                                prods_im[lane].eq(rds[lane].data * hist_im[k]),
                            ]
                    m.d.sync += [prod_valid.eq(j_valid),
                                 pair_valid.eq(prod_valid),
                                 quad_valid.eq(pair_valid)]
                    m.d.sync += [
                        pair_re[0].eq(prods_re[0] + prods_re[1]),
                        pair_re[1].eq(prods_re[2] + prods_re[3]),
                        pair_im[0].eq(prods_im[0] + prods_im[1]),
                        pair_im[1].eq(prods_im[2] + prods_im[3]),
                        quad_re.eq(pair_re[0] + pair_re[1]),
                        quad_im.eq(pair_im[0] + pair_im[1]),
                    ]
                    with m.If(quad_valid):
                        m.d.sync += [sum_re.eq(sum_re + quad_re),
                                     sum_im.eq(sum_im + quad_im)]
                    with m.If((j == per_lane) & ~j_valid & ~prod_valid
                              & ~pair_valid & ~quad_valid):
                        m.next = 'DONE'
                with m.State('DONE'):
                    m.d.sync += [self.re_out.eq(sat(sum_re >> SHIFT)),
                                 self.im_out.eq(sat(sum_im >> SHIFT)),
                                 self.strobe_out.eq(1)]
                    m.next = 'IDLE'
            # After IDLE's use of the holding register: a new input wins.
            with m.If(self.strobe_in):
                m.d.sync += [pend.eq(1), pend_re.eq(self.re_in),
                             pend_im.eq(self.im_in)]
        return m
