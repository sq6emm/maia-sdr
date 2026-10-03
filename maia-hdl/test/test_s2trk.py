#
# SPDX-License-Identifier: MIT
#

"""S2Trk against its model (bit-exact): short frames (two pilot groups and
a tail) of random words, loaded from a frame two frames back, dth changed
mid-stream (taken at the next frame start), words at an uneven pace, the
FIFO read now and then. The model's entries against block_entry (what trxd
computes from the ring's words)."""

import random
import unittest

import numpy as np

from maia_hdl.s2trk import (S2Trk, Model, block_entry, SLOT, PILOT, GROUP,
                            PERIOD, ENTRY_WORDS)
from maia_hdl.s2front import pl_scrambling
from .amaranth_sim import AmaranthSim

NPIL = 2
L = SLOT + NPIL * PERIOD + 500


def stream(n, seed):
    rng = np.random.default_rng(seed)
    re = rng.integers(-20000, 20000, n)
    im = rng.integers(-20000, 20000, n)
    return [(int(a) & 0xFFFF) | ((int(b) & 0xFFFF) << 16) for a, b in zip(re, im)]


def refs_of(pos_list, hdr_q, scr):
    return [hdr_q[p] if p < SLOT else scr[p - SLOT] for p in pos_list]


class TestS2Trk(AmaranthSim):
    def setUp(self):
        rng = random.Random(4)
        self.hdr_q = [rng.randrange(4) for _ in range(SLOT)]
        self.words = stream(4 * L + 300, 5)
        self.k_load = 2 * L + 1000          # the load: before this word
        self.base = 7                        # frames start at 7 + n L
        self.dth = [(0x01234567, 0), (0xFEDCBA98, 3 * L - 200)]

    def model(self):
        md = Model(self.hdr_q, self.dth[0][0])
        for k, w in enumerate(self.words):
            for v, at in self.dth:
                if at == k:
                    md.dth = v
            if k == self.k_load:
                md.load(self.base, L, True, NPIL)
            md.push(w)
        return md.entries

    def test_model_blocks(self):
        """Each entry is the block's own sum (block_entry) at the entry's
        phase: the model and trxd's per-block function agree."""
        ents = self.model()
        scr = pl_scrambling(L)
        starts = [self.base + f * L for f in range(5)]
        blocks = 0
        for e in ents:
            k0 = e[0]
            f = max(s for s in starts if s <= k0)
            p0 = k0 - f
            n = SLOT if p0 == 0 else PILOT
            refs = refs_of(range(p0, p0 + n), self.hdr_q, scr)
            self.assertEqual(e, block_entry(self.words[k0:k0 + n], k0, refs, e[4], e[5]))
            blocks += 1
        # frame 3 whole (the load in frame 2: from frame 3 on), frame 4's
        # header (the stream ends in it)
        self.assertEqual(blocks, 1 + NPIL + 1)
        self.assertEqual([e[0] for e in ents][:3], [self.base + 3 * L, self.base + 3 * L + SLOT + GROUP,
                                                   self.base + 3 * L + SLOT + PERIOD + GROUP])

    def test_hdl(self):
        want = self.model()
        self.dut = dut = S2Trk()
        got = []
        rnd = random.Random(6)

        async def bench(ctx):
            ctx.set(dut.enable, 1)
            for a, q in enumerate(self.hdr_q):
                ctx.set(dut.hdr_waddr, a)
                ctx.set(dut.hdr_wdata, q)
                ctx.set(dut.hdr_we, 1)
                await ctx.tick()
            ctx.set(dut.hdr_we, 0)
            ctx.set(dut.dth, self.dth[0][0])
            ctx.set(dut.run_start, 1)
            await ctx.tick()
            ctx.set(dut.run_start, 0)
            for k, w in enumerate(self.words):
                for v, at in self.dth:
                    if at == k:
                        ctx.set(dut.dth, v)
                if k == self.k_load:
                    ctx.set(dut.base, self.base)
                    ctx.set(dut.frame_len, L)
                    ctx.set(dut.pilots, 1)
                    ctx.set(dut.npil, NPIL)
                    ctx.set(dut.load, 1)
                    await ctx.tick()
                    ctx.set(dut.load, 0)
                    for _ in range(8):
                        await ctx.tick()
                ctx.set(dut.word, w)
                ctx.set(dut.valid, 1)
                await ctx.tick()
                ctx.set(dut.valid, 0)
                for _ in range(rnd.choice([0, 1, 2, 5])):
                    await ctx.tick()
                if rnd.random() < 0.01 and ctx.get(dut.level) > 0:
                    e = ctx.get(dut.entry)
                    got.append([(e >> (32 * i)) & 0xFFFFFFFF for i in range(ENTRY_WORDS)])
                    ctx.set(dut.pop, 1)
                    await ctx.tick()
                    ctx.set(dut.pop, 0)
            for _ in range(10):
                await ctx.tick()
            while ctx.get(dut.level) > 0:
                e = ctx.get(dut.entry)
                got.append([(e >> (32 * i)) & 0xFFFFFFFF for i in range(ENTRY_WORDS)])
                ctx.set(dut.pop, 1)
                await ctx.tick()
                ctx.set(dut.pop, 0)
                await ctx.tick()
            self.assertEqual(ctx.get(dut.overflow), 0)

        self.simulate(bench)
        self.assertEqual(got, want)


if __name__ == '__main__':
    unittest.main()
