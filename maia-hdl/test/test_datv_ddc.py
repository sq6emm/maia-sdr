#
# SPDX-License-Identifier: MIT
#

"""DATV receive front end: the DDC as trxd configures it.

trxd (tezuka_fw_simple, src/trxd/src/dvbs2/ddc.rs) designs the three FIR
stages (channel low-pass filters and the RRC matched filter), lays out the
coefficient RAM, and carries a bit-exact model of the DDC that its DVB-S2
receiver is tested against. This test ties that model to the HDL:

- trxd's mixer output equals Mixer.model (checked against the HDL in
  test_mixer.py);
- trxd's FIR cascade equals FIR4DSP.model / FIR2DSP.model with its taps;
- FIRDecimator3Stage, simulated with the very coefficient RAM image and
  registers trxd writes, equals those models once its sample buffers have
  filled (the HDL does not clear them), at the decimation phase it starts
  in after reset (arbitrary, and of no consequence to a receiver).

Vectors: vectors/datv_ddc.json, written by trxd's
`DDC_VECTORS=<file> cargo test dump_vectors -- --ignored`.
"""

import json
import os
import unittest

import numpy as np

from maia_hdl.fir import FIR2DSP, FIR4DSP, FIRDecimator3Stage
from maia_hdl.mixer import Mixer
from .amaranth_sim import AmaranthSim

CYCLES_PER_SAMPLE = 61
VECTORS = os.path.join(os.path.dirname(__file__), 'vectors', 'datv_ddc.json')


def load_cases():
    with open(VECTORS) as f:
        return json.load(f)


def pad(taps, d):
    taps = np.array(taps, 'int')
    return np.concatenate([taps, np.zeros((-len(taps)) % d, 'int')])


def stage_models(case, phases=(0, 0, 0)):
    """The cascade by maia-hdl's stage models, dropping phases[k] samples
    before stage k (its decimation phase)."""
    x = np.array(case['mixed'])
    (d1, d2, d3), (t1, t2, t3) = case['decimation'], case['taps']
    re, im = x[phases[0]:, 0], x[phases[0]:, 1]
    re, im = FIR4DSP(in_width=12, out_width=16, macc_trunc=17,
                     len_log2=8).model(pad(t1, d1), d1, re, im)
    if not case['bypass2']:
        re, im = FIR2DSP(in_width=16, out_width=16, macc_trunc=18,
                         len_log2=7).model(pad(t2, d2), d2,
                                           re[phases[1]:], im[phases[1]:])
    re, im = FIR4DSP(in_width=16, out_width=16, macc_trunc=18,
                     len_log2=8).model(pad(t3, d3), d3,
                                       re[phases[2]:], im[phases[2]:])
    return np.stack([re, im], axis=1)


class TestDatvDDC(AmaranthSim):
    def test_mixer_matches_trxd(self):
        for case in load_cases():
            with self.subTest(rs=case['rs']):
                x = np.array(case['input'])
                mixer = Mixer('clk3x', 12, nco_width=28)
                fw = case['freq_word']
                if fw >= 2**27:
                    fw -= 2**28
                re, im = mixer.model(fw, x[:, 0], x[:, 1])
                expected = np.array(case['mixed'])
                np.testing.assert_array_equal(re, expected[:, 0])
                np.testing.assert_array_equal(im, expected[:, 1])

    def test_stage_models_match_trxd(self):
        for case in load_cases():
            with self.subTest(rs=case['rs']):
                ref = stage_models(case)
                out = np.array(case['output'])
                n = min(len(ref), len(out))
                np.testing.assert_array_equal(ref[:n], out[:n])

    def test_hdl_matches_the_models(self):
        for case in load_cases():
            with self.subTest(rs=case['rs']):
                self.run_case(case)

    def run_case(self, case):
        self.dut = FIRDecimator3Stage()
        mixed = np.array(case['mixed'])
        # Enough input for a good stretch of output.
        dec = np.prod([d for d, byp in zip(
            case['decimation'],
            [False, case['bypass2'], case['bypass3']]) if not byp])
        nin = min(len(mixed), 300 * dec)
        # The last few outputs wait in the pipeline for later input.
        nout = nin // dec - 8
        got = np.zeros((nout, 2), 'int')

        async def set_inputs(ctx):
            d1, d2, d3 = case['decimation']
            o1, o2, o3 = case['operations_minus_one']
            odd1, odd3 = case['odd_operations']
            ctx.set(self.dut.decimation1, d1)
            ctx.set(self.dut.decimation2, d2)
            ctx.set(self.dut.decimation3, d3)
            ctx.set(self.dut.bypass2, int(case['bypass2']))
            ctx.set(self.dut.bypass3, int(case['bypass3']))
            ctx.set(self.dut.operations_minus_one1, o1)
            ctx.set(self.dut.operations_minus_one2, o2)
            ctx.set(self.dut.operations_minus_one3, o3)
            ctx.set(self.dut.odd_operations1, int(odd1))
            ctx.set(self.dut.odd_operations3, int(odd3))
            for addr, coeff in case['coeffs']:
                await ctx.tick()
                ctx.set(self.dut.coeff_wren, 1)
                ctx.set(self.dut.coeff_waddr, addr)
                ctx.set(self.dut.coeff_wdata, int(coeff) & (2**18 - 1))
            await ctx.tick()
            ctx.set(self.dut.coeff_wren, 0)
            # Samples at the real rate: 3.072 MS/s into the 187.5 MHz clock
            # is one per 61 cycles. The stages do not backpressure each
            # other, so feeding faster than the clock budget that
            # maia-httpd (and trxd) check would drop samples between them.
            for re, im in mixed[:nin]:
                ctx.set(self.dut.in_valid, 1)
                ctx.set(self.dut.re_in, int(re))
                ctx.set(self.dut.im_in, int(im))
                while True:
                    await ctx.tick()
                    if ctx.get(self.dut.in_ready):
                        break
                ctx.set(self.dut.in_valid, 0)
                await ctx.tick().repeat(CYCLES_PER_SAMPLE - 1)

        async def get_outputs(ctx):
            budget = (CYCLES_PER_SAMPLE + 10) * nin + 10000
            for j in range(nout):
                while True:
                    await ctx.tick()
                    budget -= 1
                    self.assertGreater(budget, 0, f'stuck after {j} outputs')
                    if ctx.get(self.dut.strobe_out):
                        got[j] = (ctx.get(self.dut.re_out),
                                  ctx.get(self.dut.im_out))
                        break

        self.simulate([set_inputs, get_outputs])
        # Outputs until every stage's buffer holds real samples.
        (d1, d2, d3), (t1, t2, t3) = case['decimation'], case['taps']
        d2 = 1 if case['bypass2'] else d2
        warm = (len(t3) // d3 + len(t2) // (d2 * d3)
                + len(t1) // (d1 * d2 * d3) + 4)
        best = None
        for p1 in range(d1):
            for p2 in range(d2):
                for p3 in range(d3):
                    ref = stage_models(case, (p1, p2, p3))
                    for off in range(-3, 4):
                        a = got[warm + max(0, off):]
                        b = ref[warm + max(0, -off):]
                        n = min(len(a), len(b))
                        bad = np.count_nonzero(a[:n] != b[:n])
                        if best is None or bad < best[0]:
                            best = (bad, (p1, p2, p3), off, n)
        bad, phases, off, n = best
        self.assertGreater(n, 200)
        self.assertEqual(bad, 0, f'best alignment {phases}, {off}: '
                                 f'{bad} of {n} outputs differ')


if __name__ == '__main__':
    unittest.main()
