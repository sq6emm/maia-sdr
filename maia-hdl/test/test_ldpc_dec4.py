#
# SPDX-License-Identifier: MIT
#

"""LdpcDecoder4 (four lanes, parity in its own layout) against trxd's bit-exact model (tezuka_fw_simple
src/trxd/src/dvbs2/ldpc_fpga.rs): every posterior after the decode, the
iterations used and the convergence flag. Vectors: vectors/ldpc_long.json,
from `LDPC_VECTORS=<file> cargo test ldpc_vectors -- --ignored`."""

import json
import os
import unittest

import numpy as np

from maia_hdl.dvbs2_tables import TABLES
from maia_hdl.ldpc_dec import N
from maia_hdl.ldpc_dec4 import LdpcDecoder4, cpu_layout
from .amaranth_sim import AmaranthSim

VECTORS = os.path.join(os.path.dirname(__file__), 'vectors', 'ldpc_long.json')


class TestLdpcDecoder4(AmaranthSim):
    def run_case(self, case):
        self.dut = dut = LdpcDecoder4(TABLES)
        llr = np.array(case['llr'], dtype=np.int64)
        got = np.zeros(N, dtype=np.int64)
        result = {}

        async def bench(ctx):
            # load: variable v to (word, byte) of cpu_layout
            wd, by = cpu_layout(case['rate'])
            words = np.zeros(N // 4, dtype=np.int64)
            for v in range(N):
                words[wd[v]] |= int(llr[v] & 0xFF) << (8 * by[v])
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
            limit = int(os.environ.get('LDPC_MAXCYC', '2000000'))
            while ctx.get(dut.busy) and cycles < limit:
                await ctx.tick()
                cycles += 1
                if cycles % 20000 == 0 and os.environ.get('LDPC_TRACE'):
                    print('cycle', cycles, {k: ctx.get(v) for k, v in dut.dbg.items()}, flush=True)
            if cycles >= limit:
                raise AssertionError(f'decoder still busy after {cycles} cycles')
            result['cycles'] = cycles
            result['iterations'] = ctx.get(dut.iterations)
            result['converged'] = ctx.get(dut.converged)
            # read back
            back = np.zeros(N // 4, dtype=np.int64)
            for a in range(N // 4):
                ctx.set(dut.cpu_addr, a)
                ctx.set(dut.cpu_re, 1)
                await ctx.tick()
                back[a] = ctx.get(dut.cpu_rdata)
            for v in range(N):
                x = (int(back[wd[v]]) >> (8 * by[v])) & 0xFF
                got[v] = x - 256 if x >= 128 else x
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
