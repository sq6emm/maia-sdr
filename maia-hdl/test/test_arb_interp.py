#
# SPDX-License-Identifier: MIT
#

"""ArbInterpolator: the HDL against its model, and the model's signal
quality for DVB-S2 at 250 kS/s from the 3.072 MS/s DAC stream."""

import random
import unittest

import numpy as np

from maia_hdl.arb_interp import ArbInterpolator, rrc_table
from .amaranth_sim import AmaranthSim

FS = 3_072_000


class TestArbInterpolator(AmaranthSim):
    def test_hdl_matches_model(self):
        rnd = random.Random(2)
        self.dut = ArbInterpolator(span=6, phases_log2=5)
        dut = self.dut
        coeffs = [rnd.randint(-2**16, 2**16) for _ in range(dut.depth)]
        nsym = 60
        re = np.array([rnd.randint(-2**15, 2**15 - 1) for _ in range(nsym)])
        im = np.array([rnd.randint(-2**15, 2**15 - 1) for _ in range(nsym)])
        step = int(250_000 / FS * 2**32)
        nout = int((nsym - 2) * FS / 250_000)
        expected = dut.model(step, coeffs, re, im, nout)
        got = []

        async def bench(ctx):
            ctx.set(dut.step, step)
            for a, c in enumerate(coeffs):
                ctx.set(dut.coeff_waddr, a)
                ctx.set(dut.coeff_wdata, c)
                ctx.set(dut.coeff_wren, 1)
                await ctx.tick()
            ctx.set(dut.coeff_wren, 0)
            n = 0
            for _ in range(nout * 40):
                # A slow symbol source and a reader that stalls at random.
                sym_ok = n < nsym and rnd.random() < 0.6
                ctx.set(dut.in_valid, int(sym_ok))
                if n < nsym:
                    ctx.set(dut.in_re, int(re[n]))
                    ctx.set(dut.in_im, int(im[n]))
                ready = rnd.random() < 0.5
                ctx.set(dut.out_ready, int(ready))
                take = sym_ok and ctx.get(dut.in_ready)
                out = ready and ctx.get(dut.out_valid)
                if out:
                    got.append((ctx.get(dut.out_re), ctx.get(dut.out_im)))
                await ctx.tick()
                if take:
                    n += 1
                if len(got) >= nout:
                    break

        self.simulate(bench)
        self.assertEqual(len(got), nout)
        np.testing.assert_array_equal(np.array(got), expected)

    def quality(self, points, sr=250_000, rolloff=0.35):
        rnd = np.random.default_rng(1)
        dut = ArbInterpolator()
        coeffs = rrc_table(dut.span, dut.phases_log2, rolloff)
        nsym = 1500
        amp = 2**15 - 1
        idx = rnd.integers(0, len(points), nsym)
        sym = np.array(points)[idx]
        re = np.round(sym.real * amp).astype(int)
        im = np.round(sym.imag * amp).astype(int)
        step = round(sr / FS * 2**32)
        nout = int((nsym - dut.span - 2) * FS / sr)
        y = dut.model(step, coeffs, re, im, nout)
        z = (y[:, 0] + 1j * y[:, 1]) / amp
        # Out of band: power beyond (1 + rolloff) / 2 * sr plus a margin.
        spec = np.abs(np.fft.fftshift(np.fft.fft(z * np.hanning(len(z)))))**2
        f = np.fft.fftshift(np.fft.fftfreq(len(z), 1 / FS))
        inband = np.abs(f) < (1 - rolloff) / 2 * sr
        outband = np.abs(f) > (1 + rolloff) / 2 * sr * 1.3
        oob_db = 10 * np.log10(spec[outband].max() / spec[inband].mean())
        # Matched filter (float RRC at the output rate) and sample at the
        # symbol instants: the phase accumulator gives them exactly.
        sps = FS / sr
        tt = np.arange(-10 * sps, 10 * sps + 1) / sps
        b = rolloff
        with np.errstate(divide='ignore', invalid='ignore'):
            g = ((np.sin(np.pi * tt * (1 - b)) + 4 * b * tt * np.cos(np.pi * tt * (1 + b)))
                 / (np.pi * tt * (1 - (4 * b * tt)**2)))
        g[np.abs(tt) < 1e-9] = 1 - b + 4 * b / np.pi
        sing = np.abs(np.abs(tt) - 1 / (4 * b)) < 1e-9
        g[sing] = b / np.sqrt(2) * ((1 + 2 / np.pi) * np.sin(np.pi / (4 * b))
                                     + (1 - 2 / np.pi) * np.cos(np.pi / (4 * b)))
        mf = np.convolve(z, g, mode='same')
        # Symbol n (newest at output m when acc wraps) peaks span/2 symbols later.
        acc, n, tpk = 0, 0, []
        for m in range(nout):
            acc += step
            if acc >= 2**32:
                acc -= 2**32
                tpk.append((n, m - acc / 2**32 * sps + dut.span / 2 * sps))
                n += 1
        est, ref = [], []
        for n, t in tpk[20:-20]:
            i = int(np.floor(t))
            if i + 1 >= len(mf):
                break
            fr = t - i
            est.append(mf[i] * (1 - fr) + mf[i + 1] * fr)
            ref.append(sym[n])
        est, ref = np.array(est), np.array(ref)
        gain = np.vdot(ref, est) / np.vdot(ref, ref)
        err = est / gain - ref
        mer_db = 10 * np.log10(np.mean(np.abs(ref)**2) / np.mean(np.abs(err)**2))
        return mer_db, oob_db

    def test_qpsk_250k_quality(self):
        pts = np.exp(1j * (np.pi / 4 + np.pi / 2 * np.arange(4)))
        mer, oob = self.quality(pts)
        print(f'QPSK 250 kS/s: MER {mer:.1f} dB, out of band {oob:.1f} dB')
        self.assertGreater(mer, 35)
        self.assertLess(oob, -40)

    def test_8psk_250k_quality(self):
        pts = np.exp(1j * np.pi / 4 * np.arange(8))
        mer, oob = self.quality(pts)
        print(f'8PSK 250 kS/s: MER {mer:.1f} dB, out of band {oob:.1f} dB')
        self.assertGreater(mer, 35)
        self.assertLess(oob, -40)


if __name__ == '__main__':
    unittest.main()
