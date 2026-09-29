#
# SPDX-License-Identifier: MIT
#

"""RsnnFront against trxd's fixed-point front (src/trxd/src/rsnn.rs
`Net::front_q`), bit for bit: vectors from `cargo test rsnn_front_vectors
-- --ignored` (a small random network, c2 4, c1 8: the 161K network's 450K
cycles a frame are too slow to simulate; RSNN_FRONT_VEC=... RSNN_FRONT_SMALL=1)."""

import json
import os
import unittest

from maia_hdl.rsnn_front import RsnnFront, RsnnFrontAxi, NB, ID
from .amaranth_sim import AmaranthSim

VEC = os.environ.get('RSNN_FRONT_VEC') or os.path.join(os.path.dirname(__file__), 'vectors',
                                                     'rsnn_front_small.json')
# RSNN_FRONT_ROWS: the first rows only (rsnn_front.json, the 161K network)
ROWS = int(os.environ.get('RSNN_FRONT_ROWS', '0'))


class TestRsnnFront(AmaranthSim):
    def test_vectors(self):
        with open(VEC) as fh:
            v = json.load(fh)
        c2, c1 = v['c2'], v['c1']
        w, b, rows, want = v['w'], v['b'], v['rows'], v['h']
        if ROWS:
            rows, want = rows[:ROWS], want[:ROWS - 3]
        self.dut = dut = RsnnFront()
        got = []
        stats = {}

        async def bench(ctx):
            ctx.set(dut.c2, c2)
            ctx.set(dut.c1, c1)
            ww = w + [0] * (len(w) % 2)
            for k in range(0, len(ww), 2):
                ctx.set(dut.w_waddr, k // 2)
                ctx.set(dut.w_wdata, (ww[k] & 0xFFFF) | (ww[k + 1] & 0xFFFF) << 16)
                ctx.set(dut.w_we, 1)
                await ctx.tick()
            ctx.set(dut.w_we, 0)
            for k, x in enumerate(b):
                ctx.set(dut.b_waddr, k)
                ctx.set(dut.b_wdata, x & 0xFFFFFFFF)
                ctx.set(dut.b_we, 1)
                await ctx.tick()
            ctx.set(dut.b_we, 0)
            ctx.set(dut.reset, 1)
            await ctx.tick()
            ctx.set(dut.reset, 0)
            await ctx.tick()
            while ctx.get(dut.busy):
                await ctx.tick()
            worst = 0
            for n, row in enumerate(rows):
                r = row + [0] * (64 - NB)
                for k in range(32):
                    ctx.set(dut.x_waddr, k)
                    ctx.set(dut.x_wdata, (r[2 * k] & 0xFFFF) | (r[2 * k + 1] & 0xFFFF) << 16)
                    ctx.set(dut.x_we, 1)
                    await ctx.tick()
                ctx.set(dut.x_we, 0)
                ctx.set(dut.go, 1)
                await ctx.tick()
                ctx.set(dut.go, 0)
                cyc = 0
                await ctx.tick()
                while ctx.get(dut.busy):
                    await ctx.tick()
                    cyc += 1
                worst = max(worst, cyc)
                if n >= 3:
                    self.assertTrue(ctx.get(dut.valid))
                    h = []
                    for o in range(c1):
                        ctx.set(dut.h_raddr, o)
                        await ctx.tick()
                        h.append(ctx.get(dut.h_rdata))
                    got.append(h)
                else:
                    self.assertFalse(ctx.get(dut.valid))
            stats['cycles'] = worst

        self.simulate(bench)
        print(f'{stats["cycles"]} cycles a frame at most; {len(got)} frames')
        n = min(len(got), len(want))
        self.assertGreaterEqual(n, len(rows) - 3)
        for t in range(n):
            self.assertEqual(got[t], want[t], f'frame {t}')

    def test_axi(self):
        """The same through the AXI window (6 rows: 3 frames out)."""
        with open(os.path.join(os.path.dirname(__file__), 'vectors', 'rsnn_front_small.json')) as fh:
            v = json.load(fh)
        c2, c1 = v['c2'], v['c1']
        w, b, rows, want = v['w'], v['b'], v['rows'][:6], v['h'][:3]
        self.dut = dut = RsnnFrontAxi()
        got = []

        async def write(ctx, addr, data):
            ctx.set(dut.s_axi_awaddr, addr)
            ctx.set(dut.s_axi_awvalid, 1)
            ctx.set(dut.s_axi_wdata, data)
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
            v = ctx.get(dut.s_axi_rdata)
            await ctx.tick()
            return v

        async def bench(ctx):
            self.assertEqual(await read(ctx, 0xFF08), ID)
            cfg = (c2 << 8) | (c1 << 16)
            await write(ctx, 0xFF00, cfg)
            ww = w + [0] * (len(w) % 2)
            for k in range(0, len(ww), 2):
                await write(ctx, 2 * k, (ww[k] & 0xFFFF) | (ww[k + 1] & 0xFFFF) << 16)
            for k, x in enumerate(b):
                await write(ctx, 0x9000 + 4 * k, x & 0xFFFFFFFF)
            await write(ctx, 0xFF00, cfg | 2)
            while (await read(ctx, 0xFF04)) & 1:
                pass
            for n, row in enumerate(rows):
                r = row + [0] * (64 - NB)
                for k in range(32):
                    await write(ctx, 0x9400 + 4 * k, (r[2 * k] & 0xFFFF) | (r[2 * k + 1] & 0xFFFF) << 16)
                await write(ctx, 0xFF00, cfg | 1)
                while True:
                    st = await read(ctx, 0xFF04)
                    if not st & 1:
                        break
                self.assertEqual(st >> 1 & 1, int(n >= 3))
                if n >= 3:
                    got.append([await read(ctx, 0x9500 + 4 * o) for o in range(c1)])

        self.simulate(bench)
        self.assertEqual(got, want)


if __name__ == '__main__':
    unittest.main()
