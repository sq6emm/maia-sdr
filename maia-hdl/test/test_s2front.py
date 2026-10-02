#
# SPDX-License-Identifier: MIT
#

"""s2front.py's model against trxd's (dvbs2/s2ring.rs, vectors/s2ring.json:
S2RING_VECTORS=<file> cargo test --release s2ring_vectors -- --ignored):
the PL scrambling sequence and the cells, bit for bit."""

import json
import os
import unittest

from maia_hdl.s2front import model_cells, pl_scrambling

VECTORS = os.path.join(os.path.dirname(__file__), 'vectors', 's2ring.json')


class TestS2FrontModel(unittest.TestCase):
    def test_against_trxd(self):
        with open(VECTORS) as fh:
            v = json.load(fh)
        self.assertEqual(pl_scrambling(64), v['scramble'])
        got = model_cells(v['words'], v['n_cells'], v['pilots'], [tuple(s) for s in v['segs']], v['gain'])
        want = [tuple(c) for c in v['cells']]
        bad = [i for i, (a, b) in enumerate(zip(got, want)) if a != b]
        self.assertEqual(len(got), len(want))
        self.assertEqual(bad, [], f'first differences at {bad[:5]}: {[got[i] for i in bad[:3]]} vs {[want[i] for i in bad[:3]]}')


if __name__ == '__main__':
    unittest.main()
