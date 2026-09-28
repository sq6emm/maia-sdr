#
# SPDX-License-Identifier: MIT
#

"""T2Ofdm against its model (maia_hdl.t2ofdm.Model): raw-everything first,
then the schedule loaded (short frames: one symbol), NCO on, word streams
(raw, carriers) compared."""

import random
import unittest

import numpy as np

from maia_hdl.t2ofdm import T2Ofdm, Model, t2_active_bins
from .amaranth_sim import AmaranthSim
from .common_edge import CommonEdgeTb

import os
GAP = int(os.environ.get("T2GAP", "6"))   # cycles between samples (34 on the board)


def whole(words):
    """The carrier stream's whole symbols (a header and all 1705 carriers):
    t2eq passes only those (a symbol cut short, or not yet out, is left
    out; the ARM could not use it)."""
    out, cur = [], None
    for w in words:
        if w & 1:
            if cur is not None and len(cur) == 1 + len(t2_active_bins()):
                out += cur
            cur = [w]
        elif cur is not None:
            cur.append(w)
    if cur is not None and len(cur) == 1 + len(t2_active_bins()):
        out += cur
    return out


class TestT2Ofdm(AmaranthSim):
    def test_model(self):
        rng = np.random.default_rng(3)
        nsamp = 1200 + 3 * 4352 + 600
        x = [(int(a), int(b)) for a, b in zip(rng.integers(-9000, 9000, nsamp), rng.integers(-9000, 9000, nsamp))]
        regs = dict(frame_len=2048 + 2304, nsym=1, gi=256, early=64, track=64,
                    freq=0x0123_4567, shift=1)
        # A start already past (3 frames and a bit): the front end catches up.
        late = (1200 - 3 * 4352) % 2**32
        events = {0: dict(regs, scheduled=0),
                  1000: dict(scheduled=1, next_start=late, load=True)}
        raw_m, car_m = Model(t2_active_bins()).run(x, events)

        ofdm = T2Ofdm()
        self.dut = ofdm
        got = []

        async def tick(ctx):
            # Every cycle's output looked at, then the clock.
            if ctx.get(ofdm.strobe_out):
                got.append(ctx.get(ofdm.re_out) | ctx.get(ofdm.im_out) << 16)
            await ctx.tick()

        async def bench(ctx):
            for k, v in regs.items():
                ctx.set(getattr(ofdm, k), v)
            ctx.set(ofdm.enable, 1)
            await ctx.tick()
            for n, (re, im) in enumerate(x):
                if n == 1000:
                    ctx.set(ofdm.scheduled, 1)
                    ctx.set(ofdm.next_start, late)
                    ctx.set(ofdm.load, 1)
                    await tick(ctx)
                    ctx.set(ofdm.load, 0)
                    # (the start is taken on a cycle without a sample)
                    await tick(ctx)
                ctx.set(ofdm.re_in, re)
                ctx.set(ofdm.im_in, im)
                ctx.set(ofdm.strobe_in, 1)
                await tick(ctx)
                ctx.set(ofdm.strobe_in, 0)
                for _ in range(GAP - 1):
                    await tick(ctx)
            for _ in range(200):
                await tick(ctx)
            self.assertEqual(ctx.get(ofdm.overflow), 0)

        self.simulate(bench)
        raw_h = [w for w in got if not w & (1 << 16)]
        car_h = [w for w in got if w & (1 << 16)]
        import os
        if os.environ.get('T2OFDM_DEBUG'):
            from maia_hdl.t2ofdm import pack
            where = {pack(*v, 0, 0): n for n, v in enumerate(x)}
            desc = lambda w: f'H{(w >> 1 & 0x7FFF) | (w >> 17 & 0x7FFF) << 15}' if w & 1 else str(where.get(w, '?'))
            print('hdl  ', ' '.join(desc(w) for w in raw_h[995:1015]))
            print('model', ' '.join(desc(w) for w in raw_m[995:1015]))
            print('hdl tail', ' '.join(desc(w) for w in raw_h[-8:]), len(raw_h), len(raw_m))
        self.assertEqual(raw_h, raw_m[:len(raw_h)])
        self.assertEqual(len(raw_h), len(raw_m))
        # An FFT's output finishes during the window after next (the FFT's
        # delay is 2085 samples): of three windows, one and most of another.
        per = 1 + len(t2_active_bins())
        car_m = whole(car_m)
        self.assertGreaterEqual(len(car_h), per, f'{len(car_h)} carrier words')
        self.assertEqual(len(car_h), len(car_m))
        n = min(len(car_h), len(car_m))
        bad = [i for i in range(n) if car_h[i] != car_m[i]]
        self.assertEqual(bad, [], f'first carrier mismatch at {bad[:4]} (of {n}): '
                         f'got {[hex(car_h[i]) for i in bad[:4]]} expected {[hex(car_m[i]) for i in bad[:4]]}')


class TestT2OfdmRestart(AmaranthSim):
    """Raw-everything asked for in the middle of an FFT window, a schedule
    again later: the FFT and its window labels restart (before the fix the
    labels ran two windows behind and the framing slipped)."""
    def test_restart(self):
        fl = 2048 + 2304
        self.restart(1200 + 4 * fl - 900, 1200 + 4 * fl)

    def test_restart_past(self):
        """The schedule given again with a start already past, the counter
        in a window of that frame (as the receiver does after acquiring
        through the ring's backlog): that frame is left out, the next one
        framed right (the symbol counters had started from 0 mid-frame)."""
        fl = 2048 + 2304
        self.restart(1200 + 3 * fl + 2048 + 300, 1200 + 3 * fl)

    def restart(self, b, start_b):
        rng = np.random.default_rng(5)
        fl = 2048 + 2304
        a = 1200 + 2 * fl + 2048 + 256 + 700
        nsamp = 1200 + 7 * fl
        x = [(int(p), int(q)) for p, q in zip(rng.integers(-9000, 9000, nsamp), rng.integers(-9000, 9000, nsamp))]
        regs = dict(frame_len=fl, nsym=1, gi=256, early=64, track=64, freq=0x0123_4567, shift=1)
        events = {0: dict(regs, scheduled=0),
                  1000: dict(scheduled=1, next_start=1200, load=True),
                  a: dict(scheduled=0),
                  b: dict(scheduled=1, next_start=start_b, load=True)}
        raw_m, car_m = Model(t2_active_bins()).run(x, events)
        ofdm = T2Ofdm()
        self.dut = ofdm
        got = []

        async def tick(ctx):
            if ctx.get(ofdm.strobe_out):
                got.append(ctx.get(ofdm.re_out) | ctx.get(ofdm.im_out) << 16)
            await ctx.tick()

        async def bench(ctx):
            for k, v in regs.items():
                ctx.set(getattr(ofdm, k), v)
            ctx.set(ofdm.enable, 1)
            await ctx.tick()
            for n, (re, im) in enumerate(x):
                if n in (1000, b):
                    ctx.set(ofdm.scheduled, 1)
                    ctx.set(ofdm.next_start, events[n]['next_start'])
                    ctx.set(ofdm.load, 1)
                    await tick(ctx)
                    ctx.set(ofdm.load, 0)
                    await tick(ctx)
                if n == a:
                    ctx.set(ofdm.scheduled, 0)
                    await tick(ctx)
                ctx.set(ofdm.re_in, re)
                ctx.set(ofdm.im_in, im)
                ctx.set(ofdm.strobe_in, 1)
                await tick(ctx)
                ctx.set(ofdm.strobe_in, 0)
                for _ in range(GAP - 1):
                    await tick(ctx)
            for _ in range(200):
                await tick(ctx)
            self.assertEqual(ctx.get(ofdm.overflow), 0)

        self.simulate(bench)
        raw_h = [w for w in got if not w & (1 << 16)]
        car_h = [w for w in got if w & (1 << 16)]
        self.assertEqual(raw_h, raw_m)
        car_m = whole(car_m)
        self.assertGreaterEqual(len(car_m), 1706)
        n = min(len(car_h), len(car_m))
        bad = [i for i in range(n) if car_h[i] != car_m[i]]
        self.assertEqual(bad, [], f'first carrier mismatch at {bad[:4]} (of {n})')
        self.assertEqual(len(car_h), len(car_m))


if __name__ == '__main__':
    unittest.main()
