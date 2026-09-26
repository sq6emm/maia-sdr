#
# SPDX-License-Identifier: MIT
#

"""SymSync against trxd's bit-exact model (vectors/symsync.json, written by
trxd's ``symsync_vectors`` test): a board recording at 250 kS/s and a weak
synthetic signal at 333 kS/s. Samples arrive every ~80 clocks with jitter,
sometimes back to back while a strobe is being worked out."""

import json
import pathlib
import random
import unittest

from maia_hdl.symsync import SymSync
from .amaranth_sim import AmaranthSim

VECTORS = pathlib.Path(__file__).parent / 'vectors' / 'symsync.json'


class TestSymSync(AmaranthSim):
    def run_case(self, case, gaps):
        self.dut = dut = SymSync()
        rnd = random.Random(1)
        got = []
        inp = case['input']

        async def bench(ctx):
            ctx.set(dut.omega_nom, case['omega'])
            ctx.set(dut.kp_shift, case['kp_shift'])
            ctx.set(dut.ki_shift, case['ki_shift'])
            await ctx.tick()
            ctx.set(dut.enable, 1)
            await ctx.tick()
            for x in inp:
                ctx.set(dut.re_in, x[0])
                ctx.set(dut.im_in, x[1])
                ctx.set(dut.strobe_in, 1)
                if ctx.get(dut.strobe_out):
                    got.append([ctx.get(dut.re_out), ctx.get(dut.im_out)])
                await ctx.tick()
                ctx.set(dut.strobe_in, 0)
                for _ in range(gaps(rnd)):
                    if ctx.get(dut.strobe_out):
                        got.append([ctx.get(dut.re_out), ctx.get(dut.im_out)])
                    await ctx.tick()
            for _ in range(100):
                if ctx.get(dut.strobe_out):
                    got.append([ctx.get(dut.re_out), ctx.get(dut.im_out)])
                await ctx.tick()
            self.omega_end = ctx.get(dut.omega_out)

        self.simulate(bench)
        exp = case['output']
        n = min(len(got), len(exp))
        bad = [i for i in range(n) if got[i] != exp[i]]
        self.assertEqual(len(got), len(exp), f"{case['name']}: symbol count")
        self.assertEqual(bad, [], f"{case['name']}: first mismatch at {bad[:3]}: "
                         f"got {[got[i] for i in bad[:3]]} expected {[exp[i] for i in bad[:3]]}")
        self.assertEqual(self.omega_end, case['omega_end'] & 0xFFFFFFFF)

    def test_vectors(self):
        cases = json.loads(VECTORS.read_text())
        for case in cases:
            with self.subTest(case=case['name']):
                # Regular spacing near the slowest the design must take.
                self.run_case(case, lambda r: 70 + r.randrange(20))

    def test_back_to_back_samples(self):
        # A second sample right after the first while the strobe's work is
        # still going on (a hiccup upstream): the result must not change.
        case = json.loads(VECTORS.read_text())[0]
        case = dict(case, input=case['input'][:3000])
        self.run_case_prefix(case)

    def run_case_prefix(self, case):
        self.dut = dut = SymSync()
        rnd = random.Random(3)
        got = []

        async def bench(ctx):
            ctx.set(dut.omega_nom, case['omega'])
            ctx.set(dut.kp_shift, case['kp_shift'])
            ctx.set(dut.ki_shift, case['ki_shift'])
            await ctx.tick()   # disabled for a cycle: the loop state loads
            ctx.set(dut.enable, 1)
            await ctx.tick()
            for k, x in enumerate(case['input']):
                ctx.set(dut.re_in, x[0])
                ctx.set(dut.im_in, x[1])
                ctx.set(dut.strobe_in, 1)
                await ctx.tick()
                ctx.set(dut.strobe_in, 0)
                gap = 2 if k % 97 == 5 else 70
                for _ in range(gap):
                    if ctx.get(dut.strobe_out):
                        got.append([ctx.get(dut.re_out), ctx.get(dut.im_out)])
                    await ctx.tick()
            for _ in range(100):
                if ctx.get(dut.strobe_out):
                    got.append([ctx.get(dut.re_out), ctx.get(dut.im_out)])
                await ctx.tick()

        self.simulate(bench)
        exp = case['output'][:len(got)]
        self.assertGreater(len(got), 1400)
        bad = [i for i in range(len(exp)) if got[i] != exp[i]]
        self.assertEqual(bad, [], f'first mismatch at {bad[:3]}')

    def test_bypass(self):
        self.dut = dut = SymSync()

        async def bench(ctx):
            ctx.set(dut.enable, 0)
            for v in (5, -7, 1234):
                ctx.set(dut.re_in, v)
                ctx.set(dut.im_in, -v)
                ctx.set(dut.strobe_in, 1)
                self.assertEqual(ctx.get(dut.strobe_out), 1)
                self.assertEqual(ctx.get(dut.re_out), v)
                self.assertEqual(ctx.get(dut.im_out), -v)
                await ctx.tick()

        self.simulate(bench)


if __name__ == '__main__':
    unittest.main()
