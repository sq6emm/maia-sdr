#
# SPDX-License-Identifier: MIT
#

"""T2Eq against its model (bit-exact), and the model against the cells sent:
synthetic 2K symbols (QPSK cells, PP2 scattered pilots with their signs)
through a frequency-selective channel with a common phase and a timing
slope on top; G the channel's inverse as trxd loads it. A data symbol, the
frame closing symbol (pilots every dx) and a P2 symbol (passed through),
with input gaps and output back-pressure."""

import random
import unittest

import numpy as np

from maia_hdl.t2eq import (T2Eq, Model, CARRIERS, Z_UNIT, prbs_bits, pn_bits,
                           carrier_of_position)
from .amaranth_sim import AmaranthSim

DX, DY = 6, 2
GSHIFT = 15
S = 3000.0


def pack(re, im):
    return (re & 0xFFFE) | (((im & 0xFFFE) | 1) << 16)


def pack_header(payload):
    re = ((payload & 0x7FFF) << 1) | 1
    im = (((payload >> 15) & 0x7FFF) << 1) | 1
    return re | im << 16


def make_symbol(j, rng, fc=False):
    prbs, pn = prbs_bits(), pn_bits()
    x = (rng.choice([-1, 1], CARRIERS) + 1j * rng.choice([-1, 1], CARRIERS)) / np.sqrt(2)
    d, k0 = (DX, 0) if fc else (DX * DY, DX * (j % DY))
    for k in range(k0, CARRIERS, d):
        x[k] = -4 / 3 if prbs[k] ^ pn[j] else 4 / 3
    k = np.arange(CARRIERS)
    h = 0.9 * np.exp(1j * (0.4 + 2 * np.pi * 7.3 * k / 2048)) * (1 + 0.3 * np.cos(2 * np.pi * k / 300))
    extra = np.exp(1j * (0.9 + 2 * np.pi * 0.8 * k / 2048))   # common phase + slope
    y = h * x * extra * S + (rng.normal(size=CARRIERS) + 1j * rng.normal(size=CARRIERS)) * 20
    g = Z_UNIT * 2**GSHIFT / (S * h)
    gq = [(int(round(v.real)), int(round(v.imag))) for v in g]
    pos_k = carrier_of_position()
    car = []
    for pos in range(CARRIERS):
        v = y[pos_k[pos]]
        car.append((int(round(v.real)) & ~1, int(round(v.imag)) & ~1))
    return x, gq, car


class TestT2Eq(AmaranthSim):
    def regs(self):
        return dict(enable=1, p2=8, dx=DX, dy=DY, fc_j=197,
                    rec_d=round(65536 / (DX * DY)), rec_fc=round(65536 / DX),
                    gshift=GSHIFT)

    def test_model_cells(self):
        rng = np.random.default_rng(1)
        md = Model()
        for j, fc in [(10, False), (11, False), (197, True)]:
            x, g, car = make_symbol(j, rng, fc)
            _, cells = md.symbol(j, [(a, b) for a, b in car], self.regs(), g)
            got = np.array([a + 1j * b for a, b in cells]) / 20
            err = np.abs(got - x)
            mer = -10 * np.log10(np.mean(err**2) / np.mean(np.abs(x)**2))
            print(f'symbol {j}: MER {mer:.1f} dB')
            self.assertGreater(mer, 24)

    def test_hdl(self):
        rng = np.random.default_rng(2)
        md = Model()
        regs = self.regs()
        syms = [(10, False), (3, False), (197, True)]
        built = [make_symbol(j, rng, fc) for j, fc in syms]
        g = built[0][1]          # one channel for all (the same h)
        words_in, want = [], []
        for (j, fc), (x, _, car) in zip(syms, built):
            payload = j | (12345 << 8)
            words_in.append(pack_header(payload))
            words_in += [pack(a, b) for a, b in car]
            if j >= regs['p2']:
                want.append(pack_header(payload) | (1 << 31))
                w, _ = md.symbol(j, car, regs, g)
                want += w
            else:
                want.append(pack_header(payload) & 0x7FFFFFFF)
                want += [pack(a, b) for a, b in car]
        self.dut = dut = T2Eq()
        got = []
        random.seed(5)

        async def bench(ctx):
            for name, v in [('enable', 1), ('p2', regs['p2']), ('dx', DX), ('dy', DY),
                            ('fc_j', 197), ('rec_d', regs['rec_d']),
                            ('rec_fc', regs['rec_fc']), ('gshift', GSHIFT), ('gbank', 1)]:
                ctx.set(getattr(dut, name), v)
            # G into bank 1
            ctx.set(dut.g_wbank, 1)
            for k, (a, b) in enumerate(g):
                ctx.set(dut.g_waddr, k)
                ctx.set(dut.g_wdata, (a & 0xFFFF) | (b & 0xFFFF) << 16)
                ctx.set(dut.g_we, 1)
                await ctx.tick()
            ctx.set(dut.g_we, 0)
            # the front end's pace: a word every 54 cycles (a sample at
            # 1.845 MS/s), into a 32-word FIFO that must never fill
            at, cycles, fifo, worst = 0, 0, [], 0
            while len(got) < len(want) and cycles < 800_000:
                if cycles % 54 == 0 and at < len(words_in):
                    fifo.append(words_in[at])
                    at += 1
                    worst = max(worst, len(fifo))
                    self.assertLessEqual(len(fifo), 32, f'FIFO overflow at word {at}')
                ctx.set(dut.i_rdy, bool(fifo))
                ctx.set(dut.i_data, fifo[0] if fifo else 0)
                ctx.set(dut.o_rdy, random.random() < 0.5)
                await ctx.delay(1e-9)
                if ctx.get(dut.o_en):
                    got.append(ctx.get(dut.o_data))
                took = bool(fifo) and ctx.get(dut.i_en)
                await ctx.tick()
                if took:
                    fifo.pop(0)
                cycles += 1
            self.worst = worst
            self.cycles = cycles

        self.simulate(bench)
        print(f'{self.cycles} cycles, {len(got)} of {len(want)} words, FIFO at most {self.worst}')
        self.assertEqual(len(got), len(want))
        bad = [i for i in range(len(want)) if got[i] != want[i]]
        self.assertEqual(bad, [], f'first mismatches at {bad[:4]}: got {[hex(got[i]) for i in bad[:2]]} want {[hex(want[i]) for i in bad[:2]]}')


if __name__ == '__main__':
    unittest.main()
