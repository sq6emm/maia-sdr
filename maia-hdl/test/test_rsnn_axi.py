#
# SPDX-License-Identifier: MIT
#

"""RsnnAxi (front end + temporal layers) against trxd's streaming network
(src/rsnn.rs `Stream`, fixed point), bit for bit: the last temporal layer's
outputs a frame, through the AXI4-Lite window, the temporal weights served
from a modelled DDR (AXI3 slave with random delays). Vectors from `cargo
test rsnn_front_vectors -- --ignored` (RSNN_FRONT_SMALL=1: c2 4, c1 8,
dilations 1 and 2)."""

import json
import os
import random
import unittest

from maia_hdl.rsnn_axi import RsnnAxi, ID
from maia_hdl.rsnn_front import NB
from .amaranth_sim import AmaranthSim

VEC = os.environ.get('RSNN_FRONT_VEC') or os.path.join(os.path.dirname(__file__), 'vectors',
                                                     'rsnn_front_small.json')
W_BASE = 0x1620_0000


class TestRsnnAxi(AmaranthSim):
    def test_vectors(self):
        with open(VEC) as fh:
            v = json.load(fh)
        c2, c1 = v['c2'], v['c1']
        rows, want = v['rows'], v['t_out']
        tw, tb, dils = v['tw'], v['tb'], v['dils']
        self.dut = dut = RsnnAxi()
        a = dut.temporal.axi
        # DDR: the temporal weights as 64-bit words (4 a word, low first)
        tw4 = tw + [0] * (-len(tw) % 4)
        ddr = {}
        for n in range(len(tw4) // 4):
            ddr[W_BASE + 8 * n] = sum((tw4[4 * n + j] & 0xFFFF) << (16 * j) for j in range(4))
        words = len(tw4) // 4
        got = []
        stats = {'bursts': 0}
        random.seed(3)

        async def ddr_slave(ctx):
            queue = []
            while True:
                ctx.set(a.arready, random.random() < 0.7)
                await ctx.delay(1e-9)
                if ctx.get(a.arvalid) and ctx.get(a.arready):
                    queue.append((ctx.get(a.araddr), ctx.get(a.arlen) + 1))
                    stats['bursts'] += 1
                if queue and random.random() < 0.8:
                    addr, n = queue[0]
                    ctx.set(a.rvalid, 1)
                    ctx.set(a.rdata, ddr.get(addr, 0))
                    ctx.set(a.rlast, n == 1)
                    await ctx.delay(1e-9)
                    if ctx.get(a.rready):
                        queue[0] = (addr + 8, n - 1)
                        if n == 1:
                            queue.pop(0)
                else:
                    ctx.set(a.rvalid, 0)
                await ctx.tick()

        async def write(ctx, addr, data):
            ctx.set(dut.s_axi_awaddr, addr)
            ctx.set(dut.s_axi_awvalid, 1)
            ctx.set(dut.s_axi_wdata, data & 0xFFFFFFFF)
            ctx.set(dut.s_axi_wstrb, 0xF)
            ctx.set(dut.s_axi_wvalid, 1)
            ctx.set(dut.s_axi_bready, 1)
            while not ctx.get(dut.s_axi_awready):
                await ctx.tick()
            await ctx.tick()
            ctx.set(dut.s_axi_awvalid, 0)
            ctx.set(dut.s_axi_wvalid, 0)
            while not ctx.get(dut.s_axi_bvalid):
                await ctx.tick()
            await ctx.tick()

        async def read(ctx, addr):
            ctx.set(dut.s_axi_araddr, addr)
            ctx.set(dut.s_axi_arvalid, 1)
            ctx.set(dut.s_axi_rready, 1)
            while not ctx.get(dut.s_axi_arready):
                await ctx.tick()
            await ctx.tick()
            ctx.set(dut.s_axi_arvalid, 0)
            while not ctx.get(dut.s_axi_rvalid):
                await ctx.tick()
            r = ctx.get(dut.s_axi_rdata)
            await ctx.tick()
            return r

        async def bench(ctx):
            self.assertEqual(await read(ctx, 0xFF08), ID)
            cfg = (c2 << 8) | (c1 << 16)
            await write(ctx, 0xFF00, cfg)
            w, b = v['w'], v['b']
            ww = w + [0] * (len(w) % 2)
            for k in range(0, len(ww), 2):
                await write(ctx, 2 * k, (ww[k] & 0xFFFF) | (ww[k + 1] & 0xFFFF) << 16)
            for k, x in enumerate(b):
                await write(ctx, 0x9000 + 4 * k, x)
            base = 0
            for l, d in enumerate(dils):
                await write(ctx, 0xFF20 + 4 * l, d | base << 7 | (4 * d + 1) << 17)
                for o in range(c1):
                    await write(ctx, 0x9800 + 4 * (64 * l + o), tb[l * c1 + o])
                base += 4 * d + 1
            # every layer slot's register (0..7) writes and reads back
            for l in range(8):
                await write(ctx, 0xFF20 + 4 * l, (l + 1) * 0x0123457 & 0x7FFFFFF)
            for l in range(8):
                self.assertEqual(await read(ctx, 0xFF20 + 4 * l), (l + 1) * 0x0123457 & 0x7FFFFFF, f'layer {l}')
            base = 0
            for l, d in enumerate(dils):
                await write(ctx, 0xFF20 + 4 * l, d | base << 7 | (4 * d + 1) << 17)
                base += 4 * d + 1
            await write(ctx, 0xFF14, W_BASE)
            await write(ctx, 0xFF18, words)
            await write(ctx, 0xFF10, len(dils))
            await write(ctx, 0xFF00, cfg | 2)
            while (await read(ctx, 0xFF04)) & 1:
                pass
            worst = 0
            for n, row in enumerate(rows):
                r = row + [0] * (64 - NB)
                for k in range(32):
                    await write(ctx, 0x9400 + 4 * k, (r[2 * k] & 0xFFFF) | (r[2 * k + 1] & 0xFFFF) << 16)
                await write(ctx, 0xFF00, cfg | 1)
                cyc = 0
                while True:
                    st = await read(ctx, 0xFF04)
                    cyc += 1
                    if not st & 1:
                        break
                worst = max(worst, cyc)
                if st & 4:
                    out = [await read(ctx, 0x9600 + 4 * o) for o in range(c1)]
                    got.append([x - (1 << 32) if x >> 31 else x for x in out])
            stats['frames'] = await read(ctx, 0xFF0C)

        from amaranth.sim import Simulator
        sim = Simulator(dut)
        sim.add_clock(10e-9)
        sim.add_testbench(ddr_slave, background=True)
        sim.add_testbench(bench)
        sim.run()
        print(f"{len(got)} frames out (want {len(want)}), {stats['bursts']} bursts")
        self.assertEqual(len(got), len(want))
        self.assertEqual(stats['frames'], len(want))
        for t in range(len(want)):
            self.assertEqual(got[t], want[t], f'frame {t}')


if __name__ == '__main__':
    unittest.main()
