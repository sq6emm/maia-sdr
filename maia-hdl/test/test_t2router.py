#
# SPDX-License-Identifier: MIT
#

"""T2Router: equalized symbols' cells from the eq clock into DDR frame
buffers through a destination table (a modelled AXI3 slave with random
delays): every data cell where the table says, as trxd keeps cells (the
7-bit values doubled, I the low byte), P2 symbols and non-equalized
symbols left alone, the buffer complete; the next frame in the next
buffer."""

import random
import unittest

from amaranth.sim import Simulator

from maia_hdl.t2router import T2Router, ID, WORDS, SKIP

TABLE, FB, STRIDE = 0x1640_0000, 0x1660_0000, 0x10_0000
NSYM, J0 = 12, 2


def header(j, f, eq):
    p = j | f << 8 | eq << 29
    return 1 | (p & 0x7FFF) << 1 | 1 << 16 | ((p >> 15) & 0x7FFF) << 17


def word(c):
    # four 7-bit values at bits 1, 8, 17, 24; tags: bit 16 set
    return ((c[0] & 0x7F) << 1 | (c[1] & 0x7F) << 8 | 1 << 16
            | (c[2] & 0x7F) << 17 | (c[3] & 0x7F) << 24)


class TestT2Router(unittest.TestCase):
    def test_frames(self):
        rng = random.Random(4)
        dut = T2Router()
        a = dut.axi
        ddr = {}
        # table: each data symbol's carriers, some skipped (pilots), the rest
        # to distinct cell indices
        ncells = (NSYM - J0) * 2 * WORDS
        dests = list(range(ncells))
        rng.shuffle(dests)
        table = {}
        at = 0
        for j in range(NSYM):
            for k in range(2 * WORDS):
                if j < J0 or rng.random() < 0.1:
                    table[(j, k)] = SKIP
                else:
                    table[(j, k)] = dests[at]
                    at += 1
        for j in range(NSYM):
            for k in range(0, 2 * WORDS, 2):
                ddr[TABLE + 8192 * j + 4 * k] = table[(j, k)] | table[(j, k + 1)] << 32
        # two frames of words: P2 symbols raw (eq 0), then equalized symbols
        frames = []
        stream = []
        for f in (100, 200):
            cells = {}
            for j in range(NSYM):
                stream.append(header(j, f, 1 if j >= J0 else 0))
                for w in range(WORDS):
                    c = [rng.randint(-64, 63) for _ in range(4)]
                    stream.append(word(c))
                    cells[(j, 2 * w)] = (c[0], c[1])
                    cells[(j, 2 * w + 1)] = (c[2], c[3])
            frames.append(cells)
        res = {}

        async def eq_feed(ctx):
            # as the equalizer gives them: a symbol's words at full rate, then
            # nothing until the next symbol (1.25 ms on air; 8000 cycles here)
            for i, x in enumerate(stream):
                ctx.set(dut.eq_data, x)
                ctx.set(dut.eq_valid, 1)
                await ctx.tick('eq')
                if (i + 1) % (WORDS + 1) == 0:
                    ctx.set(dut.eq_valid, 0)
                    for _ in range(8000):
                        await ctx.tick('eq')
            ctx.set(dut.eq_valid, 0)
            res['fed'] = True

        async def ddr_slave(ctx):
            rq, wq, wd = [], [], []
            bresp = 0
            while True:
                ctx.set(a.arready, rng.random() < 0.7)
                ctx.set(a.awready, rng.random() < 0.7)
                ctx.set(a.wready, rng.random() < 0.7)
                await ctx.delay(1e-9)
                if ctx.get(a.arvalid) and ctx.get(a.arready):
                    rq.append([ctx.get(a.araddr), ctx.get(a.arlen) + 1])
                if ctx.get(a.awvalid) and ctx.get(a.awready):
                    wq.append(ctx.get(a.awaddr))
                if ctx.get(a.wvalid) and ctx.get(a.wready):
                    wd.append((ctx.get(a.wdata), ctx.get(a.wstrb)))
                while wq and wd:
                    addr = wq.pop(0)
                    d, s = wd.pop(0)
                    old = ddr.get(addr, 0)
                    for b in range(8):
                        if s >> b & 1:
                            old = (old & ~(0xFF << 8 * b)) | (d & (0xFF << 8 * b))
                    ddr[addr] = old
                    bresp += 1
                ctx.set(a.bvalid, bresp > 0)
                if rq and rng.random() < 0.85:
                    addr, n = rq[0]
                    ctx.set(a.rvalid, 1)
                    ctx.set(a.rdata, ddr.get(addr, SKIP | SKIP << 32))
                    ctx.set(a.rlast, n == 1)
                    await ctx.delay(1e-9)
                    if ctx.get(a.rready):
                        rq[0] = [addr + 8, n - 1]
                        if n == 1:
                            rq.pop(0)
                else:
                    ctx.set(a.rvalid, 0)
                await ctx.delay(1e-9)
                if bresp and ctx.get(a.bready):
                    bresp -= 1
                await ctx.tick()

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
            r = ctx.get(dut.s_axi_rdata)
            await ctx.tick()
            return r

        async def bench(ctx):
            self.assertEqual(await read(ctx, 0x3C), ID)
            for r, v in [(0x04, TABLE), (0x08, FB), (0x0C, STRIDE), (0x10, NSYM), (0x14, J0), (0x00, 1)]:
                await write(ctx, r, v)
            while not res.get('fed'):
                await ctx.tick()
            for _ in range(20000):
                await ctx.tick()
            res['bufs'] = [await read(ctx, 0x20 + 4 * b) for b in range(4)]
            res['started'] = await read(ctx, 0x30)
            res['symbols'] = await read(ctx, 0x34)
            res['overflows'] = await read(ctx, 0x38)

        sim = Simulator(dut)
        sim.add_clock(10e-9)
        sim.add_clock(16e-9, domain='eq')
        sim.add_testbench(ddr_slave, background=True)
        sim.add_testbench(eq_feed)
        sim.add_testbench(bench)
        sim.run()
        print(res['bufs'], res['started'], res['symbols'], res['overflows'])
        self.assertEqual(res['overflows'], 0)
        self.assertEqual(res['started'], 2)
        self.assertEqual(res['symbols'], 2 * (NSYM - J0))
        # frame 100 in buffer 1, 200 in buffer 2, both complete
        self.assertEqual(res['bufs'][1], 1 << 31 | 100)
        self.assertEqual(res['bufs'][2], 1 << 31 | 200)

        def cell_at(b, d):
            addr = FB + STRIDE * b + 2 * d
            v = ddr.get(addr & ~7, None)
            if v is None:
                return None
            v = (v >> (8 * (addr & 7))) & 0xFFFF
            to8 = lambda x: x - 256 if x >= 128 else x
            return (to8(v & 0xFF), to8(v >> 8))

        for fi, b in ((0, 1), (1, 2)):
            bad = 0
            for (j, k), d in table.items():
                if d == SKIP:
                    continue
                c = frames[fi][(j, k)]
                if cell_at(b, d) != (2 * c[0], 2 * c[1]):
                    if bad < 6:
                        print('frame', fi, 'j', j, 'k', k, 'dest', d, 'got', cell_at(b, d), 'want', (2 * c[0], 2 * c[1]))
                    bad += 1
            self.assertEqual(bad, 0, f'frame {fi}: {bad} cells wrong')


if __name__ == '__main__':
    unittest.main()
