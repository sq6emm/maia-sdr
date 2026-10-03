#
# SPDX-License-Identifier: MIT
#

"""EqRingFilter: of a stream of symbols (equalized and not), the ring gets
the unequalized ones and equalized symbol ring_j only, whatever the
consumer's pace."""

import random
import unittest

from maia_hdl.t2ofdm import EqRingFilter
from .amaranth_sim import AmaranthSim


def header(j, eq):
    payload = j | 0x1234 << 8
    return 1 | (payload & 0x7FFF) << 1 | 1 << 16 | (payload >> 15) << 17 | eq << 31


def symbol(j, eq, n):
    return [header(j, eq)] + [(j << 20 | i) << 1 & ~(1 << 16) & 0xFFFE_FFFE for i in range(n)]


class TestEqRingFilter(AmaranthSim):
    def test_filter(self):
        stream = []
        for j in range(1, 9):
            stream += symbol(j, 1, 5 + j)
            if j % 3 == 0:
                stream += symbol(j, 0, 4)       # an unequalized one between
        for ring_j in (0, 4):
            want = []
            want, keep = [], True
            for w in stream:
                if w & 1 and w & (1 << 16):
                    j, eq = (w >> 1) & 0xFF, w >> 31
                    keep = not (eq and ring_j and j != ring_j)
                if keep:
                    want.append(w)
            self.check(stream, ring_j, want)

    def check(self, stream, ring_j, want):
        self.dut = dut = EqRingFilter()
        got = []
        random.seed(ring_j)

        async def bench(ctx):
            ctx.set(dut.ring_j, ring_j)
            i, cycle = 0, 0
            while i < len(stream) and cycle < 20 * len(stream):
                valid = random.random() < 0.8
                take = random.random() < 0.5
                ctx.set(dut.valid, valid)
                ctx.set(dut.data, stream[i] if valid else 0xDEAD_BEEF)
                ctx.set(dut.take, take)
                await ctx.delay(1e-9)
                if ctx.get(dut.avail) and take:
                    got.append(stream[i])
                if ctx.get(dut.pop):
                    self.assertTrue(valid)
                    i += 1
                await ctx.tick()
                cycle += 1

        self.simulate(bench)
        self.assertEqual(got, want, f'ring_j {ring_j}')


if __name__ == '__main__':
    unittest.main()
