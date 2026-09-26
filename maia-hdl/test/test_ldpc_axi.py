#
# SPDX-License-Identifier: MIT
#

"""LdpcAxi: the AXI4-Lite window (id, RAM, control/status) around the
decoder; decodes an all-zero frame (a codeword: one iteration)."""

import unittest

from maia_hdl.ldpc_axi import LdpcAxi, ID
from .amaranth_sim import AmaranthSim


class TestLdpcAxi(AmaranthSim):
    def test_window(self):
        self.dut = dut = LdpcAxi()

        async def write(ctx, addr, data, strb=0xF):
            ctx.set(dut.s_axi_awaddr, addr)
            ctx.set(dut.s_axi_awvalid, 1)
            ctx.set(dut.s_axi_wdata, data)
            ctx.set(dut.s_axi_wstrb, strb)
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
            await write(ctx, 0x1230, 0x11223344)
            self.assertEqual(await read(ctx, 0x1230), 0x11223344)
            await write(ctx, 0x1230, 0x0000AA00, strb=0b0010)
            self.assertEqual(await read(ctx, 0x1230), 0x1122AA44)
            # all-zero LLRs: a codeword, one iteration
            await write(ctx, 0x1230, 0)
            await write(ctx, 0xFF00, 1 | (0 << 1) | (5 << 8))
            self.assertEqual(await read(ctx, 0xFF04) & 1, 1)   # busy
            while (await read(ctx, 0xFF04)) & 1:
                pass
            st = await read(ctx, 0xFF04)
            self.assertEqual((st >> 1) & 1, 1)       # converged
            self.assertEqual((st >> 8) & 0x3F, 1)    # in one iteration

        self.simulate(bench)


if __name__ == '__main__':
    unittest.main()
