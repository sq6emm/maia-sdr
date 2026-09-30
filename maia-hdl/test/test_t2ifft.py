#
# SPDX-License-Identifier: MIT
#

"""T2Ifft against its model (maia_hdl.t2ifft.Model): two short frames (P1 and
three symbols each) with random input gaps and output back-pressure; frames
shorter than nsym (the next sync word ends them); and the passthrough with
the block off."""

import random
import unittest

import numpy as np

from maia_hdl.t2ifft import T2Ifft, Model, P1, NCAR, SYNC
from .amaranth_sim import AmaranthSim

NSYM = 3


def word(i, q):
    return (i & 0xFFFF) | (q & 0xFFFF) << 16


def unword(w):
    i, q = w & 0xFFFF, (w >> 16) & 0xFFFF
    return (i - 65536 if i & 0x8000 else i, q - 65536 if q & 0x8000 else q)


class TestT2Ifft(AmaranthSim):
    def test_frames(self):
        self.run_frames([NSYM, NSYM])

    def test_short_frames(self):
        # 2, then the full 3, then 1 symbol: each frame ended by the next sync
        self.run_frames([2, NSYM, 1, 2])

    def run_frames(self, syms):
        rng = np.random.default_rng(7)
        # stray words first (IQ still queued when T2 started), then per frame
        # the sync word, P1 and the symbols' carriers
        samples = [(int(a), int(b)) for a, b in zip(rng.integers(-500, 500, 37), rng.integers(-500, 500, 37))]
        sync = (0x7FFF, -0x7FFF)
        for nsym in syms:
            samples.append(sync)
            samples += [(int(a), int(b)) for a, b in zip(rng.integers(-20000, 20000, P1), rng.integers(-20000, 20000, P1))]
            # carriers: complex amplitude within the FFT's limit (32767)
            for _ in range(nsym):
                samples += [(int(a), int(b)) for a, b in zip(rng.integers(-3000, 3000, NCAR), rng.integers(-3000, 3000, NCAR))]
        # (a sync word after the last frame closes it)
        samples.append(sync)
        want = Model(nsym=NSYM).run(samples)
        dut = T2Ifft(nsym=NSYM)
        self.dut = dut
        got = []
        random.seed(3)

        async def bench(ctx):
            ctx.set(dut.enable, 1)
            await ctx.tick()
            at = 0
            cycles = 0
            while len(got) < len(want) and cycles < 400_000:
                # input: valid with gaps
                if at < len(samples) and random.random() < 0.7:
                    ctx.set(dut.s_tdata, word(*samples[at]))
                    ctx.set(dut.s_tvalid, 1)
                else:
                    ctx.set(dut.s_tvalid, 0)
                ready = random.random() < 0.6
                ctx.set(dut.m_tready, ready)
                await ctx.delay(1e-9)
                took = ctx.get(dut.s_tvalid) and ctx.get(dut.s_tready)
                if ready and ctx.get(dut.m_tvalid):
                    got.append(unword(ctx.get(dut.m_tdata)))
                await ctx.tick()
                if took:
                    at += 1
                cycles += 1
            self.cycles, self.at = cycles, at
            self.state = {n: ctx.get(getattr(dut, n)) for n in ['_item', '_in_sym', '_feeding', '_full', '_wb', '_rb', '_wcnt', '_ocnt', '_bin'] if hasattr(dut, n)}

        self.simulate(bench)
        print('cycles', self.cycles, 'input taken', self.at, 'of', len(samples), 'out', len(got), 'of', len(want), 'state', self.state)
        self.assertEqual(len(got), len(want))
        bad = [k for k in range(len(want)) if got[k] != want[k]]
        self.assertEqual(bad, [], f'first mismatches at {bad[:4]}: got {[got[k] for k in bad[:2]]} want {[want[k] for k in bad[:2]]}')

    def test_passthrough(self):
        dut = T2Ifft(nsym=NSYM)
        self.dut = dut
        got = []

        async def bench(ctx):
            ctx.set(dut.enable, 0)
            for k in range(20):
                ctx.set(dut.s_tdata, word(k, -k))
                ctx.set(dut.s_tvalid, 1)
                ctx.set(dut.m_tready, k % 3 != 0)
                await ctx.delay(1e-9)
                self.assertEqual(ctx.get(dut.m_tvalid), 1)
                self.assertEqual(ctx.get(dut.s_tready), int(k % 3 != 0))
                got.append(unword(ctx.get(dut.m_tdata)))
                await ctx.tick()

        self.simulate(bench)
        self.assertEqual(got, [(k, -k) for k in range(20)])


if __name__ == '__main__':
    unittest.main()
