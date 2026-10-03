#
# SPDX-License-Identifier: MIT
#

"""T2P1 against its model (bit-exact), and the model against P1s placed in
noise: the reported window starts where the P1s are. Samples at the front
end's pace (one about every 34 cycles), with bursts; guard-interval tails
and frame ends marked."""

import random
import unittest

import numpy as np

from maia_hdl.t2p1 import T2P1, Model, P1_J, GI_J, REPORT_WORDS
from .amaranth_sim import AmaranthSim

BLOCK = 5000
K_Q8 = 64


def p1_symbol(rng, amp=3000.0):
    """C A B with C, B copies of A's last 542 / 482 samples shifted by
    1/1024 (as trxd's structure_peak_in pairs them: C 1024 samples before
    its copy, B 482 after)."""
    a = (rng.normal(size=1024) + 1j * rng.normal(size=1024)) * amp / np.sqrt(2)
    c = a[482:] * np.exp(2j * np.pi * np.arange(542) / 1024)
    b = a[542:] * np.exp(2j * np.pi * np.arange(482) / 1024)
    return np.concatenate([c, a, b])


def signal(n, at, rng, noise=300.0):
    x = (rng.normal(size=n) + 1j * rng.normal(size=n)) * noise / np.sqrt(2)
    for s in at:
        x[s:s + 2048] += p1_symbol(rng)
    re = np.clip(np.round(x.real), -32768, 32767).astype(int)
    im = np.clip(np.round(x.imag), -32768, 32767).astype(int)
    return list(zip(re.tolist(), im.tolist()))


def decode(words):
    """(kind, values) records out of the report words."""
    out, i = [], 0
    while i < len(words):
        w = words[i]
        assert w & 1 and w & (1 << 16), f'not a header: {w:08x}'
        payload = ((w >> 1) & 0x7FFF) | ((w >> 17) & 0x7FFF) << 15
        vals = []
        for v in words[i + 1:i + 1 + REPORT_WORDS]:
            vals.append(((v >> 1) & 0x7FFF) | ((v >> 17) & 1) << 15)
        i += 1 + REPORT_WORDS
        v32 = [vals[2 * k] | vals[2 * k + 1] << 16 for k in range(3)]
        out.append((payload & 0xFF, v32))
    return out


class TestT2P1(AmaranthSim):
    def stream(self, n, at, seed):
        rng = np.random.default_rng(seed)
        x = signal(n, at, rng)
        # tails: 200-sample runs every 2300 (as guard tails would be), and
        # frame ends
        tail = [(t % 2300) >= 2100 for t in range(n)]
        last = {6000: 1000, 11000: 6000}
        return x, tail, last

    def test_model_finds_the_p1s(self):
        at = [1500, 6500, 11500]
        x, tail, last = self.stream(15000, at, 1)
        md = Model(BLOCK, K_Q8)
        words = []
        for t, (re, im) in enumerate(x):
            words += md.push(re, im, t, tail[t], t in last, last.get(t, 0))
        recs = decode(words)
        p1 = [v[0] for k, v in recs if k == P1_J]
        self.assertEqual(p1, at)
        gi = [v for k, v in recs if k == GI_J]
        self.assertEqual([v[2] for v in gi], [1000, 6000])

    def test_hdl(self):
        n = 12000
        at = [2500, 7500]
        x, tail, last = self.stream(n, at, 2)
        md = Model(BLOCK, K_Q8)
        want = []
        for t, (re, im) in enumerate(x):
            want += md.push(re, im, t, tail[t], t in last, last.get(t, 0))
        self.dut = dut = T2P1()
        got = []
        random.seed(3)

        async def bench(ctx):
            ctx.set(dut.enable, 1)
            ctx.set(dut.block_len, BLOCK)
            ctx.set(dut.k_q8, K_Q8)
            ctx.set(dut.o_rdy, 1)
            t, cycle, nxt = 0, 0, 0
            while (t < n or len(got) < len(want)) and cycle < 34 * n + 20000:
                strobe = t < n and cycle >= nxt
                ctx.set(dut.strobe, strobe)
                if strobe:
                    re, im = x[t]
                    ctx.set(dut.x_re, re)
                    ctx.set(dut.x_im, im)
                    ctx.set(dut.t, t)
                    ctx.set(dut.tail, tail[t])
                    ctx.set(dut.last, t in last)
                    ctx.set(dut.frame_start, last.get(t, 0))
                    # mostly 34 cycles apart, now and then two close together
                    nxt = cycle + (3 if random.random() < 0.05 else 34)
                    t += 1
                rdy = random.random() < 0.7
                ctx.set(dut.o_rdy, rdy)
                await ctx.delay(1e-9)
                if ctx.get(dut.o_en) and rdy:
                    got.append(ctx.get(dut.o_data))
                await ctx.tick()
                cycle += 1
            self.assertEqual(ctx.get(dut.overflow), 0)

        self.simulate(bench)
        self.assertEqual(len(got), len(want))
        for i, (g, w) in enumerate(zip(got, want)):
            self.assertEqual(g, w, f'word {i}: {g:08x} != {w:08x}')
        p1 = [v[0] for k, v in decode(got) if k == P1_J]
        self.assertEqual(p1[:2], at)


if __name__ == '__main__':
    unittest.main()
