#
# SPDX-License-Identifier: MIT
#

"""HdrDet against trxd's bit-exact model (vectors/hdrdet.json, written by
trxd's ``hdrdet_vectors`` test): symbols from the timing recovery model on a
board recording (several DVB-S2 headers) and noise; the flagged symbols."""

import json
import pathlib
import unittest

from maia_hdl.hdrdet import HdrDet
from .amaranth_sim import AmaranthSim

VECTORS = pathlib.Path(__file__).parent / 'vectors' / 'hdrdet.json'


class TestHdrDet(AmaranthSim):
    def run_case(self, case):
        self.dut = dut = HdrDet()
        got_flags = []
        got_syms = []

        async def bench(ctx):
            await ctx.tick()
            ctx.set(dut.enable, 1)
            await ctx.tick()
            for x in case['input']:
                ctx.set(dut.re_in, x[0])
                ctx.set(dut.im_in, x[1])
                ctx.set(dut.strobe_in, 1)
                await ctx.tick()
                ctx.set(dut.strobe_in, 0)
                for _ in range(40):
                    if ctx.get(dut.strobe_out):
                        re, im = ctx.get(dut.re_out), ctx.get(dut.im_out)
                        got_flags.append(im & 1)
                        got_syms.append((re, im >> 1))
                    await ctx.tick()

        self.simulate(bench)
        inp = case['input']
        self.assertEqual(len(got_flags), len(inp))
        flags = [k for k, f in enumerate(got_flags) if f]
        self.assertEqual(flags, case['flags'], case['name'])
        self.assertEqual(got_syms, [(x[0], x[1] >> 1) for x in inp])

    def test_vectors(self):
        for case in json.loads(VECTORS.read_text()):
            with self.subTest(case=case['name']):
                self.run_case(case)

    def test_bypass(self):
        self.dut = dut = HdrDet()

        async def bench(ctx):
            for v in (5, -7, 1235):
                ctx.set(dut.re_in, v)
                ctx.set(dut.im_in, v + 1)
                ctx.set(dut.strobe_in, 1)
                self.assertEqual(ctx.get(dut.strobe_out), 1)
                self.assertEqual(ctx.get(dut.im_out), v + 1)
                await ctx.tick()

        self.simulate(bench)


if __name__ == '__main__':
    unittest.main()
