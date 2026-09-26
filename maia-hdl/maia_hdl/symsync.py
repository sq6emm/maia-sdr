#
# SPDX-License-Identifier: MIT
#

"""Symbol timing recovery for the DATV receiver (tezuka_fw_simple).

Between the DDC (matched-filtered samples at 2 to 2.7 per symbol, not
necessarily a whole ratio) and the recorder: one sample per symbol out, at
the symbol centres. Gardner timing error, cubic (Catmull-Rom) interpolation,
a PI loop. Bit-exact with trxd's model (``src/dvbs2/symsync.rs``), which
documents the arithmetic:

- positions in samples Q.24: ``t`` the next strobe, ``omega`` samples per
  symbol (clamped to the nominal +- 1/128);
- interpolation at ``mu`` (Q0.16) from a, b, c, d (b at floor(pos)), with
  doubled coefficients C1 = c - a, C2 = 2a - 5b + 4c - d,
  C3 = (d - a) + 3(b - c) and Horner steps v = ((v * mu) >> 16) + C...,
  y = sat16((v_last + 2b) >> 1);
- e = Re((prev - y) conj(mid)), mid at t - omega / 2;
  agc += (|y|^2 - agc) >> 10; en = clamp((e << 16) >> msb(agc), +-2**16);
- omega += (en << 8) >> ki_shift; t += omega + ((en << 8) >> kp_shift).

A strobe falls on the sample with index floor(t) + 2. Work per strobe is
sequential on one multiplier (about 45 cycles), far below the input sample
period (80 or more cycles at 62.5 MHz). With ``enable`` low the input passes
straight through and the loop state is reset.
"""

from amaranth import *

FRAC = 24
MU_BITS = 16


class SymSync(Elaboratable):
    def __init__(self, width=16):
        self.w = width
        self.enable = Signal()
        self.omega_nom = Signal(32)   # Q8.24
        self.kp_shift = Signal(5)
        self.ki_shift = Signal(5)

        self.strobe_in = Signal()
        self.re_in = Signal(signed(width))
        self.im_in = Signal(signed(width))

        self.strobe_out = Signal()
        self.re_out = Signal(signed(width))
        self.im_out = Signal(signed(width))

        # The loop's samples per symbol (Q8.24), for the CPU to read.
        self.omega_out = Signal(32)

    def elaborate(self, platform):
        m = Module()
        w = self.w

        hist_re = Array(Signal(signed(w), name=f'hist_re{i}') for i in range(8))
        hist_im = Array(Signal(signed(w), name=f'hist_im{i}') for i in range(8))
        n = Signal(16)               # samples received
        t = Signal(40)               # Q16.24, wrapping
        omega = Signal(signed(34))
        prev_re = Signal(signed(w))
        prev_im = Signal(signed(w))
        agc = Signal(signed(34))
        pending = Signal()           # a sample came in: check for a strobe
        busy = Signal()

        nom = Signal(signed(34))
        m.d.comb += [nom.eq(self.omega_nom),
                     self.omega_out.eq(omega[:32])]

        # Sample in: history and count, busy or not (a new sample never
        # overwrites one the current strobe reads: those are n-5 .. n-1).
        with m.If(self.enable & self.strobe_in):
            m.d.sync += [hist_re[n[:3]].eq(self.re_in),
                         hist_im[n[:3]].eq(self.im_in),
                         n.eq(n + 1),
                         pending.eq(1)]

        # Shared multiplier, one cycle.
        mul_a = Signal(signed(40))
        mul_b = Signal(signed(40))
        prod = Signal(signed(80))
        m.d.sync += prod.eq(mul_a * mul_b)

        # Interpolation state.
        pos = Signal(40)
        mu = Signal(signed(MU_BITS + 1))
        comp = Signal()              # 0 re, 1 im
        which = Signal()             # 0 y (at t), 1 mid
        a = Signal(signed(w))
        b = Signal(signed(w))
        c = Signal(signed(w))
        d = Signal(signed(w))
        c1 = Signal(signed(22))
        c2 = Signal(signed(22))
        v = Signal(signed(24))
        y_re = Signal(signed(w))
        y_im = Signal(signed(w))
        mid_re = Signal(signed(w))
        mid_im = Signal(signed(w))
        e = Signal(signed(40))
        p = Signal(signed(34))
        en = Signal(signed(18))

        idx = Signal(3)
        m.d.comb += [idx.eq(pos[FRAC:FRAC + 3]),
                     mu.eq(pos[FRAC - MU_BITS:FRAC])]

        def sat(x):
            lo, hi = -(1 << (w - 1)), (1 << (w - 1)) - 1
            return Mux(x > hi, hi, Mux(x < lo, lo, x))

        vfinal = Signal(signed(24))
        m.d.comb += vfinal.eq(((prod >> MU_BITS) + 2 * b) >> 1)

        # msb(agc) for agc >= 1 (agc < 2**33): position of the top one.
        agc1 = Signal(33)
        s = Signal(6)
        m.d.comb += agc1.eq(Mux(agc < 1, 1, agc))
        for i in range(33):
            with m.If(agc1[i]):
                m.d.comb += s.eq(i)

        e_sh = Signal(signed(60))
        m.d.comb += e_sh.eq((e << 16) >> s)
        one = 1 << 16

        diff = Signal(signed(16))
        m.d.comb += diff.eq((n - 1)[:16] - t[FRAC:FRAC + 16])

        omega_new = Signal(signed(34))
        lo_om = Signal(signed(34))
        hi_om = Signal(signed(34))
        m.d.comb += [lo_om.eq(nom - (nom >> 7)), hi_om.eq(nom + (nom >> 7))]
        om_try = Signal(signed(36))
        m.d.comb += om_try.eq(omega + ((en << 8) >> self.ki_shift))
        m.d.comb += omega_new.eq(Mux(om_try < lo_om, lo_om,
                                     Mux(om_try > hi_om, hi_om, om_try)))

        with m.If(~self.enable):
            m.d.sync += [n.eq(0), t.eq(0), omega.eq(nom), prev_re.eq(0),
                         prev_im.eq(0), agc.eq(0), pending.eq(0)]
            m.d.sync += [hist_re[i].eq(0) for i in range(8)]
            m.d.sync += [hist_im[i].eq(0) for i in range(8)]
            m.d.comb += [self.strobe_out.eq(self.strobe_in),
                         self.re_out.eq(self.re_in),
                         self.im_out.eq(self.im_in)]

        with m.FSM(name='symsync') as fsm:
            with m.State('IDLE'):
                with m.If(self.enable & pending & ~self.strobe_in):
                    m.d.sync += pending.eq(0)
                    with m.If(diff >= 2):
                        m.d.sync += [pos.eq(t), which.eq(0), comp.eq(0)]
                        m.next = 'LOAD'
            with m.State('LOAD'):
                # a, b, c, d for this component at floor(pos).
                for sig, k in ((a, -1), (b, 0), (c, 1), (d, 2)):
                    i = (idx + k)[:3]
                    m.d.sync += sig.eq(Mux(comp, hist_im[i], hist_re[i]))
                m.next = 'COEF'
            with m.State('COEF'):
                m.d.sync += [
                    c1.eq(c - a),
                    c2.eq(2 * a - 5 * b + 4 * c - d),
                    v.eq((d - a) + 3 * (b - c)),
                ]
                m.next = 'M1'
            with m.State('M1'):
                m.d.comb += [mul_a.eq(v), mul_b.eq(mu)]
                m.next = 'H1'
            with m.State('H1'):
                m.d.sync += v.eq((prod >> MU_BITS) + c2)
                m.next = 'M2'
            with m.State('M2'):
                m.d.comb += [mul_a.eq(v), mul_b.eq(mu)]
                m.next = 'H2'
            with m.State('H2'):
                m.d.sync += v.eq((prod >> MU_BITS) + c1)
                m.next = 'M3'
            with m.State('M3'):
                m.d.comb += [mul_a.eq(v), mul_b.eq(mu)]
                m.next = 'H3'
            with m.State('H3'):
                res = sat(vfinal)
                with m.If(which == 0):
                    with m.If(comp == 0):
                        m.d.sync += y_re.eq(res)
                    with m.Else():
                        m.d.sync += y_im.eq(res)
                with m.Else():
                    with m.If(comp == 0):
                        m.d.sync += mid_re.eq(res)
                    with m.Else():
                        m.d.sync += mid_im.eq(res)
                with m.If(comp == 0):
                    m.d.sync += comp.eq(1)
                    m.next = 'LOAD'
                with m.Elif(which == 0):
                    m.d.sync += [comp.eq(0), which.eq(1),
                                 pos.eq(t - (omega[:34].as_unsigned() >> 1))]
                    m.next = 'LOAD'
                with m.Else():
                    m.next = 'E1'
            with m.State('E1'):
                m.d.comb += [mul_a.eq(prev_re - y_re), mul_b.eq(mid_re)]
                m.next = 'E2'
            with m.State('E2'):
                m.d.sync += e.eq(prod)
                m.d.comb += [mul_a.eq(prev_im - y_im), mul_b.eq(mid_im)]
                m.next = 'E3'
            with m.State('E3'):
                m.d.sync += e.eq(e + prod)
                m.d.comb += [mul_a.eq(y_re), mul_b.eq(y_re)]
                m.next = 'P1'
            with m.State('P1'):
                m.d.sync += p.eq(prod)
                m.d.comb += [mul_a.eq(y_im), mul_b.eq(y_im)]
                m.next = 'P2'
            with m.State('P2'):
                m.d.sync += agc.eq(agc + ((p + prod - agc) >> 10))
                m.next = 'EN'
            with m.State('EN'):
                # agc is the updated one here (as in the model).
                m.d.sync += en.eq(Mux(e_sh > one, one, Mux(e_sh < -one, -one, e_sh)))
                m.next = 'LOOP'
            with m.State('LOOP'):
                m.d.sync += [
                    omega.eq(omega_new),
                    t.eq(t + omega_new + ((en << 8) >> self.kp_shift)),
                    prev_re.eq(y_re),
                    prev_im.eq(y_im),
                ]
                m.next = 'OUT'
            with m.State('OUT'):
                with m.If(self.enable):
                    m.d.comb += [self.strobe_out.eq(1),
                                 self.re_out.eq(y_re),
                                 self.im_out.eq(y_im)]
                m.next = 'IDLE'
        return m
