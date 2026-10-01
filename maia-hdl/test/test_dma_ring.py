#
# SPDX-License-Identifier: MIT
#

"""DmaStreamWrite in ring mode: a continuous stream into a ring buffer that
software follows with committed_address (the DATV DDC output ring)."""

import random
import unittest

from amaranth import *

from maia_hdl.dma import DmaStreamWrite
from .amaranth_sim import AmaranthSim


class TestDmaRing(AmaranthSim):
    def test_ring(self):
        start, bursts = 0x0001_0000, 5
        burst_bytes = 16 * 8  # 16 beats of 64 bits
        end = start + bursts * burst_bytes
        self.dut = DmaStreamWrite(start, end, ring=True)
        axi = self.dut.axi
        words = bursts * 16
        laps = 3.5
        nwords = int(words * laps)
        mem = {}
        aw_seen = []
        # (committed address, words written so far) at each change
        committed = []
        rnd = random.Random(1)

        async def bench(ctx):
            # The stream source and an AXI3 subordinate in one testbench (so
            # both see each cycle's inputs): the memory takes addresses at
            # once, data with a random wready, and answers each burst a few
            # cycles after its last beat.
            await ctx.tick()
            ctx.set(self.dut.start, 1)
            await ctx.tick()
            ctx.set(self.dut.start, 0)
            n = 0
            pending_addr = []
            beat = 0
            responses = []  # cycles until each B
            written = 0
            last_committed = None
            for _ in range(nwords * 4 + 400):
                wready = rnd.random() < 0.7
                ctx.set(axi.awready, 1)
                ctx.set(axi.wready, int(wready))
                bvalid = bool(responses) and responses[0] <= 0
                ctx.set(axi.bvalid, int(bvalid))
                ctx.set(self.dut.stream_data, n)
                ctx.set(self.dut.stream_valid, int(n < nwords))
                # What transfers at this edge is decided before it.
                if n < nwords and ctx.get(self.dut.stream_ready):
                    n += 1
                aw = ctx.get(axi.awvalid) and ctx.get(axi.awaddr)
                w = wready and ctx.get(axi.wvalid)
                wdata, wlast = ctx.get(axi.wdata), ctx.get(axi.wlast)
                b = bvalid and ctx.get(axi.bready)
                await ctx.tick()
                if aw:
                    aw_seen.append(aw)
                    pending_addr.append(aw)
                if w:
                    a = pending_addr[0] + 8 * beat
                    mem[a] = wdata
                    written += 1
                    beat += 1
                    if wlast:
                        self.assertEqual(beat, 16)
                        beat = 0
                        pending_addr.pop(0)
                        responses.append(rnd.randint(1, 6))
                if b:
                    responses.pop(0)
                responses = [r - 1 for r in responses]
                c = ctx.get(self.dut.committed_address)
                if c != last_committed:
                    committed.append((c, written))
                    last_committed = c

        self.simulate(bench)

        # Bursts go round the ring in order.
        expect = [start + burst_bytes * (i % bursts)
                  for i in range(len(aw_seen))]
        self.assertEqual(aw_seen, expect)
        self.assertGreaterEqual(len(aw_seen), int(laps * bursts))
        # The ring holds the newest lap: word n at slot n mod words.
        top = max(mem.values())
        for a, v in mem.items():
            self.assertEqual((a - start) // 8, v % words)
            self.assertGreater(v, top - words)
        # committed_address steps one burst at a time, wraps, and never
        # runs ahead of the data: at each step every earlier burst is written.
        prev = start
        steps = 0
        for c, written in committed[1:]:
            nxt = start if prev + burst_bytes == end else prev + burst_bytes
            self.assertEqual(c, nxt)
            steps += 1
            self.assertGreaterEqual(written, 16 * steps)
            prev = c
        self.assertGreaterEqual(steps, int(laps * bursts) - 1)

    def test_stop_without_data(self):
        """A stop while a burst's address is out and the stream has gone
        quiet: the burst is completed (zeros), finished comes, the wrap
        counter counted the laps; a stop while stopped gives finished too;
        a new start begins clean (no write response of the old run)."""
        start, bursts = 0x0001_0000, 5
        end = start + bursts * 16 * 8
        self.dut = dut = DmaStreamWrite(start, end, ring=True)
        axi = dut.axi
        nwords = 2 * bursts * 16 + 24     # two laps and a half-filled burst

        async def bench(ctx):
            pending = []
            beat = 0
            responses = []
            beats = 0
            fins = 0
            n = 0

            async def cycle(stream=True):
                nonlocal beat, responses, beats, fins, n
                ctx.set(axi.awready, 1)
                ctx.set(axi.wready, 1)
                bvalid = bool(responses) and responses[0] <= 0
                ctx.set(axi.bvalid, int(bvalid))
                ctx.set(dut.stream_data, n + 1)
                valid = stream and n < nwords
                ctx.set(dut.stream_valid, int(valid))
                if valid and ctx.get(dut.stream_ready):
                    n += 1
                aw = ctx.get(axi.awvalid) and ctx.get(axi.awaddr)
                w = ctx.get(axi.wvalid)
                wlast = ctx.get(axi.wlast)
                fins += ctx.get(dut.finished)
                await ctx.tick()
                if aw:
                    pending.append(aw)
                if w:
                    beats += 1
                    beat += 1
                    if wlast:
                        self.assertEqual(beat, 16)
                        beat = 0
                        pending.pop(0)
                        responses.append(3)
                if bvalid:
                    responses.pop(0)
                responses = [r - 1 for r in responses]

            await ctx.tick()
            ctx.set(dut.start, 1)
            await ctx.tick()
            ctx.set(dut.start, 0)
            for _ in range(nwords * 2 + 100):
                await cycle()
            # stream quiet, a burst half done: stop
            self.assertGreaterEqual(len(pending), 1)
            self.assertEqual(ctx.get(dut.wraps), 2)
            ctx.set(dut.stop, 1)
            await cycle(False)
            ctx.set(dut.stop, 0)
            for _ in range(50):
                await cycle(False)
            self.assertEqual(pending, [])
            self.assertEqual(beats % 16, 0)
            self.assertEqual(fins, 1)
            # stop while stopped: finished again
            ctx.set(dut.stop, 1)
            await cycle(False)
            ctx.set(dut.stop, 0)
            for _ in range(10):
                await cycle(False)
            self.assertEqual(fins, 2)
            # a new start: committed at the start, not ahead of the data
            ctx.set(dut.start, 1)
            await cycle(False)
            ctx.set(dut.start, 0)
            for _ in range(50):
                await cycle(False)
            self.assertEqual(ctx.get(dut.committed_address), start)
            self.assertEqual(ctx.get(dut.wraps), 0)

        self.simulate(bench)


if __name__ == '__main__':
    unittest.main()
