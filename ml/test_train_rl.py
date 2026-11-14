"""Checks for the RL trainer plumbing.

    python ml/test_train_rl.py
"""

import os
import sys
import unittest

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from train_policy import LAYERS, init_net  # noqa: E402
from train_rl import DEFAULT_HEADER, load_header, pack, unpack  # noqa: E402


class PackTests(unittest.TestCase):
    def test_round_trip_keeps_weights_separate(self):
        rng = np.random.default_rng(3)
        Ws, bs = init_net(rng)
        got_Ws, got_bs = unpack(pack(Ws, bs))
        self.assertEqual(len(got_Ws), len(LAYERS) - 1)
        for want, got in zip(Ws + bs, got_Ws + got_bs):
            np.testing.assert_array_equal(want, got)

    def test_scrambled_layout_would_change_outputs(self):
        rng = np.random.default_rng(4)
        Ws, bs = init_net(rng)
        theta = pack(Ws, bs)
        split = sum(w.size for w in Ws)
        bad = np.concatenate([theta[split:], theta[:split]])
        bad_Ws, bad_bs = unpack(bad)
        x = rng.normal(size=(1, LAYERS[0])).astype(np.float32)
        a = np.tanh(x @ Ws[0].T + bs[0].T)
        b = np.tanh(x @ bad_Ws[0].T + bad_bs[0].T)
        self.assertGreater(float(np.abs(a - b).max()), 0.1)


class HeaderTests(unittest.TestCase):
    def test_generated_header_loads(self):
        Ws, bs = load_header(DEFAULT_HEADER)
        for i, (w, b) in enumerate(zip(Ws, bs)):
            self.assertEqual(w.shape, (LAYERS[i + 1], LAYERS[i]))
            self.assertEqual(b.shape, (LAYERS[i + 1], 1))
            self.assertTrue(np.isfinite(w).all())
            self.assertTrue(np.isfinite(b).all())


if __name__ == "__main__":
    unittest.main(verbosity=2)
