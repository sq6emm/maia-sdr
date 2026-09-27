#
# SPDX-License-Identifier: MIT
#

"""T2Resampler against trxd's bit-exact model (vectors/t2resamp.json,
written by trxd's ``t2resamp_vectors`` test): full-scale noise, then a
strong tone. Inputs come every 20 clocks or so (3.072 MS/s at 62.5 MHz),
sometimes 19."""

import json
import pathlib
import random
import unittest

from maia_hdl.t2resamp import T2Resampler
from .amaranth_sim import AmaranthSim

VECTORS = pathlib.Path(__file__).parent / 'vectors' / 't2resamp.json'


class TestT2Resampler(AmaranthSim):
    def run_case(self, case, gaps):
        self.dut = dut = T2Resampler()
        rnd = random.Random(1)
        got = []

        async def bench(ctx):
            ctx.set(dut.step, case['step'])
            for a, c in enumerate(case['coeffs']):
                ctx.set(dut.coeff_waddr, a)
                ctx.set(dut.coeff_wdata, c)
                ctx.set(dut.coeff_wren, 1)
                await ctx.tick()
            ctx.set(dut.coeff_wren, 0)
            await ctx.tick()   # disabled: the state loads
            ctx.set(dut.enable, 1)
            await ctx.tick()
            for x in case['input']:
                ctx.set(dut.re_in, x[0])
                ctx.set(dut.im_in, x[1])
                ctx.set(dut.strobe_in, 1)
                if ctx.get(dut.strobe_out):
                    got.append([ctx.get(dut.re_out), ctx.get(dut.im_out)])
                await ctx.tick()
                ctx.set(dut.strobe_in, 0)
                for _ in range(gaps(rnd) - 1):
                    if ctx.get(dut.strobe_out):
                        got.append([ctx.get(dut.re_out), ctx.get(dut.im_out)])
                    await ctx.tick()
            for _ in range(40):
                if ctx.get(dut.strobe_out):
                    got.append([ctx.get(dut.re_out), ctx.get(dut.im_out)])
                await ctx.tick()

        self.simulate(bench)
        exp = case['output']
        n = min(len(got), len(exp))
        bad = [i for i in range(n) if got[i] != exp[i]]
        self.assertEqual(bad, [], f"first mismatch at {bad[:3]}: "
                         f"got {[got[i] for i in bad[:3]]} "
                         f"expected {[exp[i] for i in bad[:3]]}")
        self.assertEqual(len(got), len(exp))

    def test_vectors(self):
        for case in json.loads(VECTORS.read_text()):
            with self.subTest(case=case['name']):
                self.run_case(case, lambda r: 19 + (r.random() < 0.65))


if __name__ == '__main__':
    unittest.main()
