#
# SPDX-License-Identifier: MIT
#

"""LdpcAxi with the DDR engine (ldpc_dma.py, "LDP5"): a noisy frame of
vectors/ldpc_long.json loaded from a modelled DDR (AXI3 slave, random
delays), decoded, the decisions written back packed; they must be the
signs of trxd's model's posteriors (bit j of word k: RAM variable 32 k + j,
RAM variable 4 w + b: byte b of word w)."""

import json
import os
import random
import unittest

import numpy as np
from amaranth.sim import Simulator

from maia_hdl.ldpc_axi import LdpcAxi, ID5
from maia_hdl.ldpc_dec import N
from maia_hdl.ldpc_dec4 import cpu_layout
from maia_hdl.ldpc_dma import bb_model

VECTORS = os.path.join(os.path.dirname(__file__), 'vectors', 'ldpc_long.json')
IN, OUT = 0x1630_0000, 0x1631_0000
OUT_WORDS = 1024


def llr_model(cells, rot, kq, c14, s14):
    """trxd dvbt2/stream.rs: the QPSK LLRs of a block's cells."""
    def q(z):
        v = ((z >> 7) * kq + (1 << 16)) >> 17
        return max(-31, min(31, v))
    n = len(cells)
    out = []
    for j in range(n):
        i_, q_ = cells[j]
        if rot:
            d = cells[(j + 1) % n][1]
            zr, zi = i_ * c14 - d * s14, i_ * s14 + d * c14
        else:
            zr, zi = i_ * 16384, q_ * 16384
        out += [q(zr), q(zi)]
    return out


def llr16_model(cells, rot, kq, c14, s14, a14, rate):
    """trxd dvbt2/stream.rs + BitInterleaver::deinterleave_llr: a 16QAM
    block's LLRs in codeword order."""
    def q(z):
        v = ((z >> 7) * kq + (1 << 16)) >> 17
        return max(-31, min(31, v))
    n = len(cells)
    cl = []
    for j in range(n):
        i_, q_ = cells[j]
        if rot:
            d = cells[(j + 1) % n][1]
            zr, zi = i_ * c14 - d * s14, i_ * s14 + d * c14
        else:
            zr, zi = i_ * 16384, q_ * 16384
        cl += [q(zr), q(zi), q(abs(zr) - a14), q(abs(zi) - a14)]
    nbch, qq = {0: (32400, 90), 1: (48600, 45)}[rate]
    u = list(range(N))
    for t in range(qq):
        for s_ in range(360):
            u[nbch + 360 * t + s_] = nbch + qq * s_ + t
    rows = N // 8
    v = [0] * N
    index = 0
    for col, tw in enumerate([0, 0, 2, 4, 4, 5, 7, 7]):
        off = tw
        for _ in range(rows):
            v[off + rows * col] = u[index]
            index += 1
            off = 0 if off + 1 == rows else off + 1
    lookup = [v[rows * col + j] for j in range(rows) for col in range(8)]
    out = [0] * N
    for d in range(N // 8):
        for e, m in enumerate([7, 1, 4, 2, 5, 3, 6, 0]):
            out[lookup[8 * d + e]] = cl[8 * d + m]
    return out


def psk8_model(cells, kq):
    """trxd dvbs2/s2cells.rs: DVB-S2 8PSK cells -> codeword-order LLRs."""
    def q(z):
        y = ((z >> 7) * kq + (1 << 16)) >> 17
        return max(-31, min(31, y))
    phase = [1, 0, 4, 5, 2, 7, 3, 6]
    rows = len(cells)
    out = [0] * (3 * rows)
    for j, (i_, q_) in enumerate(cells):
        c = [i_ * 16384, (i_ + q_) * 11585, q_ * 16384, (q_ - i_) * 11585,
             -i_ * 16384, -(i_ + q_) * 11585, -q_ * 16384, (i_ - q_) * 11585]
        for b in range(3):
            mask = 4 >> b
            z0 = max(c[phase[v]] for v in range(8) if not v & mask)
            z1 = max(c[phase[v]] for v in range(8) if v & mask)
            out[b * rows + j] = q(z0 - z1)
    return out


def ddr_words(words, base):
    ddr = {}
    for n in range(0, len(words), 2):
        hi = int(words[n + 1]) & 0xFFFFFFFF if n + 1 < len(words) else 0
        ddr[base + 4 * n] = (int(words[n]) & 0xFFFFFFFF) | hi << 32
    return ddr


class TestLdpcDma(unittest.TestCase):
    def test_frame(self):
        with open(VECTORS) as fh:
            case = json.load(fh)[0]
        llr = np.array(case['llr'], dtype=np.int64)
        wd, by = cpu_layout(case['rate'])
        words = np.zeros(N // 4, dtype=np.int64)
        for v in range(N):
            words[wd[v]] |= int(llr[v] & 0xFF) << (8 * by[v])
        self.check_decode(case, ddr_words(words, IN), N // 4, [])

    def test_cells_decode(self):
        """Cells straight in: kq 1024, no rotation: each LLR is the cell's
        I or Q, so cells made of the vector's LLRs decode as it does."""
        with open(VECTORS) as fh:
            case = json.load(fh)[0]
        llr = [max(-31, min(31, x)) for x in case['llr']]
        words = [(llr[2 * j] & 0xFF) | (llr[2 * j + 1] & 0xFF) << 8
                 | (llr[2 * j + 2] & 0xFF) << 16 | (llr[2 * j + 3] & 0xFF) << 24
                 for j in range(0, N // 2, 2)]
        self.check_decode(case, ddr_words(words, IN), len(words),
                          [(0xFF20, 1), (0xFF24, 1024), (0xFF28, 0)])

    def test_cells_llr(self):
        """Rotated cells, a real kq: the RAM against trxd's formula and the
        decoder's layout (load only)."""
        rate = 0
        rng = np.random.default_rng(3)
        cells = [(int(a), int(b)) for a, b in rng.integers(-127, 128, (N // 2, 2))]
        kq, c14, s14 = 2931, 14330, -7943        # 29 degrees back
        llr = llr_model(cells, True, kq, c14, s14)
        wd, by = cpu_layout(rate)
        want = np.zeros(N // 4, dtype=np.int64)
        for v in range(N):
            want[wd[v]] |= (llr[v] & 0xFF) << (8 * by[v])
        words = [(cells[j][0] & 0xFF) | (cells[j][1] & 0xFF) << 8
                 | (cells[j + 1][0] & 0xFF) << 16 | (cells[j + 1][1] & 0xFF) << 24
                 for j in range(0, N // 2, 2)]
        got = self.run_dut(ddr_words(words, IN), len(words), rate, 0,
                           [(0xFF20, 1 | 2 | 4), (0xFF24, kq), (0xFF28, (c14 & 0xFFFF) | (s14 & 0xFFFF) << 16)],
                           read_ram=True)
        bad = np.nonzero(np.array(got['ram']) != want)[0]
        self.assertEqual(len(bad), 0, f'{len(bad)} RAM words differ, first at {bad[:5]}: '
                         f'{[hex(got["ram"][i]) for i in bad[:2]]} want {[hex(want[i]) for i in bad[:2]]}')

    def test_cells_16qam(self):
        """16QAM: four LLRs a cell and the bit deinterleaver, against trxd's
        (load only), both rates."""
        for rate in (0, 1):
            with self.subTest(rate=rate):
                rng = np.random.default_rng(5 + rate)
                cells = [(int(a), int(b)) for a, b in rng.integers(-127, 128, (N // 4, 2))]
                kq, c14, s14 = 1811, 15685, -4739  # 16.8 degrees back
                a14 = round(2 * 40 / 10 ** 0.5 * 16384)
                llr = llr16_model(cells, True, kq, c14, s14, a14, rate)
                wd, by = cpu_layout(rate)
                want = np.zeros(N // 4, dtype=np.int64)
                for v in range(N):
                    want[wd[v]] |= (llr[v] & 0xFF) << (8 * by[v])
                words = [(cells[j][0] & 0xFF) | (cells[j][1] & 0xFF) << 8
                         | (cells[j + 1][0] & 0xFF) << 16 | (cells[j + 1][1] & 0xFF) << 24
                         for j in range(0, N // 4, 2)]
                got = self.run_dut(ddr_words(words, IN), len(words), rate, 0,
                                   [(0xFF20, 1 | 2 | 4 | 8), (0xFF24, kq),
                                    (0xFF28, (c14 & 0xFFFF) | (s14 & 0xFFFF) << 16), (0xFF2C, a14)],
                                   read_ram=True)
                bad = np.nonzero(np.array(got['ram']) != want)[0]
                self.assertEqual(len(bad), 0, f'{len(bad)} RAM words differ, first at {bad[:5]}: '
                                 f'{[hex(got["ram"][i]) for i in bad[:2]]} want {[hex(want[i]) for i in bad[:2]]}')

    def test_cells_psk8(self):
        """DVB-S2 8PSK 3/4: three LLRs a cell (max-log) through the 3-column
        deinterleaver, against trxd's model (load only); full-scale and
        -128 cells included."""
        rate = 1
        rng = np.random.default_rng(11)
        cells = [(int(a), int(b)) for a, b in rng.integers(-128, 128, (N // 3, 2))]
        cells[:4] = [(127, 127), (-128, -128), (-128, 127), (0, 0)]
        kq = 4093
        llr = psk8_model(cells, kq)
        wd, by = cpu_layout(rate)
        want = np.zeros(N // 4, dtype=np.int64)
        for v in range(N):
            want[wd[v]] |= (llr[v] & 0xFF) << (8 * by[v])
        words = [(cells[j][0] & 0xFF) | (cells[j][1] & 0xFF) << 8
                 | (cells[j + 1][0] & 0xFF) << 16 | (cells[j + 1][1] & 0xFF) << 24
                 for j in range(0, N // 3, 2)]
        got = self.run_dut(ddr_words(words, IN), len(words), rate, 0,
                           [(0xFF20, 1 | 4 | 16), (0xFF24, kq)], read_ram=True)
        bad = np.nonzero(np.array(got['ram']) != want)[0]
        self.assertEqual(len(bad), 0, f'{len(bad)} RAM words differ, first at {bad[:5]}: '
                         f'{[hex(got["ram"][i]) for i in bad[:2]]} want {[hex(want[i]) for i in bad[:2]]}')

    def ring_case(self, psk8, pilots, seed):
        """DVB-S2 long frames from the receive ring (s2front.py): a frame
        wrapping at the ring's end, a lead, the segment table; the RAM
        against trxd's model (s2ring.rs, here s2front.model_cells) and the
        cell LLRs (load only)."""
        from maia_hdl.s2front import model_cells, GROUP, PILOT
        rng = np.random.default_rng(seed)
        ring_start, ring_words = 0x1610_0000, 0x10000          # 256 KiB
        ring_end = ring_start + 4 * ring_words
        n_cells = N // 3 if psk8 else N // 2
        groups = -(-n_cells // GROUP)
        nsym = n_cells + (groups - 1) * PILOT if pilots else n_cells
        lead = int(rng.integers(0, 32))
        first = ring_words - 8192 + lead                        # wraps
        words = [0] * ring_words
        # symbols of about 4000 (and a few at full scale)
        amp = rng.normal(0, 4000, (ring_words, 2)).round().astype(int)
        amp[first % ring_words: first % ring_words + 4] = [[32767, 32767], [-32768, -32768], [-32768, 32767], [0, 0]]
        for k in range(ring_words):
            re, im = (max(-32768, min(32767, int(v))) for v in amp[k])
            words[k] = (re & 0xFFFF) | (im & 0xFFFF) << 16
        frame = [words[(first + j) % ring_words] for j in range(nsym)]
        segs = [(int(rng.integers(0, 1 << 32)), int(rng.integers(-(1 << 22), 1 << 22))) for _ in range(groups)]
        gain = 10170
        cells = model_cells(frame, n_cells, pilots, segs, gain)
        kq = 4093 if psk8 else 2931
        llr = psk8_model(cells, kq) if psk8 else llr_model(cells, False, kq, 0, 0)
        rate = 1 if psk8 else 0
        wd, by = cpu_layout(rate)
        want = np.zeros(N // 4, dtype=np.int64)
        for v in range(N):
            want[wd[v]] |= (llr[v] & 0xFF) << (8 * by[v])
        in_addr = ring_start + 4 * (first - lead)
        regs = [(0xFF10, in_addr), (0xFF20, 1 | 4 | (16 if psk8 else 0) | 32), (0xFF24, kq),
                (0xFF34, ring_start), (0xFF38, ring_end),
                (0xFF3C, gain | lead << 24 | int(pilots) << 31)]
        for i, (a_, b_) in enumerate(segs):
            regs += [(0xFF40, a_), (0xFF44, b_ & 0xFFFFFFFF), (0xFF48, i)]
        got = self.run_dut(ddr_words(words, ring_start), lead + nsym, rate, 0, regs, read_ram=True)
        bad = np.nonzero(np.array(got['ram']) != want)[0]
        self.assertEqual(len(bad), 0, f'{len(bad)} RAM words differ, first at {bad[:5]}: '
                         f'{[hex(got["ram"][i]) for i in bad[:2]]} want {[hex(want[i]) for i in bad[:2]]}')

    def test_ring_psk8(self):
        self.ring_case(True, True, 21)

    def test_ring_qpsk(self):
        self.ring_case(False, True, 22)

    def test_bb(self):
        """BBFRAME out (0xFF20 bit 6): the decisions packed MSB first, the
        first Kbch descrambled, and the BCH remainder of the first Nbch, as
        the model makes them from the same decisions; every vector."""
        with open(VECTORS) as fh:
            cases = json.load(fh)
        for case in cases:
            llr = np.array(case['llr'], dtype=np.int64)
            wd, by = cpu_layout(case['rate'])
            words = np.zeros(N // 4, dtype=np.int64)
            for v in range(N):
                words[wd[v]] |= int(llr[v] & 0xFF) << (8 * by[v])
            nbch = 48600 if case['rate'] else 32400
            out_words = -(-nbch // 1024) * 32
            ddr = ddr_words(words, IN)
            res = self.run_dut(ddr, N // 4, case['rate'], case['max_iter'], [(0xFF20, 1 << 6)],
                               out_words=out_words, read_rem=True)
            self.assertEqual(res['iterations'], case['iterations'])
            post = np.array(case['post'], dtype=np.int64)
            dec = np.zeros(N, dtype=np.int64)
            for v in range(N):
                dec[4 * wd[v] + by[v]] = post[v] < 0
            want, rem = bb_model(dec, nbch, out_words)
            got = []
            for k in range(0, out_words, 2):
                beat = ddr.get(OUT + 4 * k)
                self.assertIsNotNone(beat, f'no beat at word {k}')
                got += [beat & 0xFFFFFFFF, beat >> 32]
            bad = [k for k in range(out_words) if got[k] != want[k]]
            self.assertEqual(bad, [], f'rate {case["rate"]}: {len(bad)} words differ')
            self.assertEqual(res['rem'], rem, f'rate {case["rate"]}: BCH remainder')
            self.assertEqual(res['zero'], int(rem == 0))
            print(f'rate {case["rate"]}: {out_words} words, remainder {"zero" if rem == 0 else "non-zero"}')

    def check_decode(self, case, ddr, in_words, regs):
        res = self.run_dut(ddr, in_words, case['rate'], case['max_iter'], regs)
        print(f"{res['iterations']} iterations, converged {res['converged']}, {res['polls']} status polls")
        self.assertEqual(res['iterations'], case['iterations'])
        self.assertEqual(bool(res['converged']), case['converged'])
        wd, by = cpu_layout(case['rate'])
        post = np.array(case['post'], dtype=np.int64)
        ram_sign = np.zeros(N, dtype=np.int64)
        for v in range(N):
            ram_sign[4 * wd[v] + by[v]] = post[v] < 0
        got = np.zeros(OUT_WORDS * 32, dtype=np.int64)
        for k in range(0, OUT_WORDS, 2):
            beat = ddr.get(OUT + 4 * k, None)
            self.assertIsNotNone(beat, f'no decision beat at word {k}')
            for half in range(2):
                wv = (beat >> (32 * half)) & 0xFFFFFFFF
                for j in range(32):
                    got[32 * (k + half) + j] = (wv >> j) & 1
        bad = np.nonzero(got != ram_sign[:OUT_WORDS * 32])[0]
        self.assertEqual(len(bad), 0, f'{len(bad)} decisions differ, first at {bad[:5]}')

    def run_dut(self, ddr, in_words, rate, max_iter, regs, read_ram=False, out_words=OUT_WORDS,
                read_rem=False):
        dut = LdpcAxi(lanes=4, dma=True)
        a = dut.dma.axi
        random.seed(7)
        res = {}

        async def ddr_slave(ctx):
            rq, wq, wdata = [], [], []
            while True:
                ctx.set(a.arready, random.random() < 0.7)
                ctx.set(a.awready, random.random() < 0.7)
                ctx.set(a.wready, random.random() < 0.8)
                await ctx.delay(1e-9)
                if ctx.get(a.arvalid) and ctx.get(a.arready):
                    rq.append([ctx.get(a.araddr), ctx.get(a.arlen) + 1])
                if ctx.get(a.awvalid) and ctx.get(a.awready):
                    wq.append([ctx.get(a.awaddr), 0])
                if ctx.get(a.wvalid) and ctx.get(a.wready):
                    wdata.append((ctx.get(a.wdata), ctx.get(a.wlast)))
                # writes: data beats land at the burst's address
                while wq and wdata:
                    addr, n = wq[0]
                    d, last = wdata.pop(0)
                    ddr[addr + 8 * n] = d
                    wq[0][1] += 1
                    if last:
                        wq.pop(0)
                        res['bresp'] = res.get('bresp', 0) + 1
                ctx.set(a.bvalid, res.get('bresp', 0) > 0)
                if rq and random.random() < 0.85:
                    addr, n = rq[0]
                    ctx.set(a.rvalid, 1)
                    ctx.set(a.rdata, ddr.get(addr, 0))
                    ctx.set(a.rlast, n == 1)
                    await ctx.delay(1e-9)
                    if ctx.get(a.rready):
                        rq[0] = [addr + 8, n - 1]
                        if n == 1:
                            rq.pop(0)
                else:
                    ctx.set(a.rvalid, 0)
                await ctx.delay(1e-9)
                if res.get('bresp', 0) and ctx.get(a.bready):
                    res['bresp'] -= 1
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
            self.assertEqual(await read(ctx, 0xFF08), ID5)
            await write(ctx, 0xFF10, IN)
            await write(ctx, 0xFF14, OUT)
            await write(ctx, 0xFF18, in_words)
            await write(ctx, 0xFF1C, out_words)
            for r, v in regs:
                await write(ctx, r, v)
            await write(ctx, 0xFF00, 1 | 4 | rate << 1 | max_iter << 8)
            polls = 0
            while (st := await read(ctx, 0xFF04)) & 1:
                polls += 1
            res['iterations'] = (st >> 8) & 0x3F
            res['converged'] = (st >> 1) & 1
            res['polls'] = polls
            res['zero'] = (st >> 3) & 1
            if read_rem:
                rem = 0
                for n in range(6):
                    rem |= (await read(ctx, 0xFF4C + 4 * n)) << (32 * n)
                res['rem'] = rem
            if read_ram:
                res['ram'] = [await read(ctx, 4 * w) for w in range(N // 4)]

        sim = Simulator(dut)
        sim.add_clock(10e-9)
        sim.add_testbench(ddr_slave, background=True)
        sim.add_testbench(bench)
        sim.run()
        return res


if __name__ == '__main__':
    unittest.main()
