#
# SPDX-License-Identifier: MIT
#

"""LdpcDecoder against trxd's bit-exact model (tezuka_fw_simple
src/trxd/src/dvbs2/ldpc_fpga.rs): every posterior after the decode, the
iterations used and the convergence flag. Vectors: vectors/ldpc_long.json,
from `LDPC_VECTORS=<file> cargo test ldpc_vectors -- --ignored`."""

import json
import os
import unittest

import numpy as np

from maia_hdl.dvbs2_tables import TABLES
from maia_hdl.ldpc_dec import LdpcDecoder, N
from .amaranth_sim import AmaranthSim

VECTORS = os.path.join(os.path.dirname(__file__), 'vectors', 'ldpc_long.json')


class TestLdpcDecoder(AmaranthSim):
    def run_case(self, case):
        self.dut = dut = LdpcDecoder(TABLES)
        llr = np.array(case['llr'], dtype=np.int64)
        got = np.zeros(N, dtype=np.int64)
        result = {}

        async def bench(ctx):
            # load: 4 LLRs per word, byte v % 4 of word v // 4
            b = (llr & 0xFF).reshape(-1, 4)
            words = b[:, 0] | (b[:, 1] << 8) | (b[:, 2] << 16) | (b[:, 3] << 24)
            ctx.set(dut.cpu_we, 0xF)
            for a, w in enumerate(words):
                ctx.set(dut.cpu_addr, a)
                ctx.set(dut.cpu_wdata, int(w))
                await ctx.tick()
            ctx.set(dut.cpu_we, 0)
            ctx.set(dut.rate, case['rate'])
            ctx.set(dut.max_iter, case['max_iter'])
            ctx.set(dut.start, 1)
            await ctx.tick()
            ctx.set(dut.start, 0)
            await ctx.tick()
            cycles = 0
            while ctx.get(dut.busy):
                await ctx.tick()
                cycles += 1
            result['cycles'] = cycles
            result['iterations'] = ctx.get(dut.iterations)
            result['converged'] = ctx.get(dut.converged)
            # read back
            for a in range(N // 4):
                ctx.set(dut.cpu_addr, a)
                ctx.set(dut.cpu_re, 1)
                await ctx.tick()
                w = ctx.get(dut.cpu_rdata)
                for i in range(4):
                    v = (w >> (8 * i)) & 0xFF
                    got[4 * a + i] = v - 256 if v >= 128 else v
            ctx.set(dut.cpu_re, 0)

        self.simulate(bench)
        print(f"rate {case['rate']}: {result['cycles']} cycles, "
              f"{result['iterations']} iterations, converged {result['converged']}")
        self.assertEqual(result['iterations'], case['iterations'])
        self.assertEqual(bool(result['converged']), case['converged'])
        exp = np.array(case['post'], dtype=np.int64)
        bad = np.nonzero(got != exp)[0]
        self.assertEqual(len(bad), 0, f'{len(bad)} posteriors differ, first at {bad[:5]}: '
                         f'got {got[bad[:5]]} expected {exp[bad[:5]]}')

    def test_cases(self):
        with open(VECTORS) as f:
            cases = json.load(f)
        which = os.environ.get('LDPC_CASE')
        for i, case in enumerate(cases):
            if which is not None and int(which) != i:
                continue
            with self.subTest(case=i):
                self.run_case(case)


if __name__ == '__main__':
    unittest.main()
