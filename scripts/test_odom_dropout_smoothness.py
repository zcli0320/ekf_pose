#!/usr/bin/env python3

import math
import os
import sys
import unittest


SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

from evaluate_odom_dropout_window import compute_smoothness_stats


class OdomDropoutSmoothnessTest(unittest.TestCase):
    def test_constant_velocity_path_has_low_step_and_jerk(self):
        path = [(float(i), float(i), 0.0, 0.0) for i in range(6)]

        stats = compute_smoothness_stats(path, 0.0, 5.0)

        self.assertEqual(stats["smoothness_count"], 5)
        self.assertAlmostEqual(stats["step_p95_m"], 1.0)
        self.assertAlmostEqual(stats["step_max_m"], 1.0)
        self.assertAlmostEqual(stats["velocity_delta_p95_mps"], 0.0)
        self.assertAlmostEqual(stats["jerk_p95_mps3"], 0.0)

    def test_single_position_spike_is_visible_in_smoothness_stats(self):
        path = [
            (0.0, 0.0, 0.0, 0.0),
            (1.0, 1.0, 0.0, 0.0),
            (2.0, 6.0, 0.0, 0.0),
            (3.0, 3.0, 0.0, 0.0),
            (4.0, 4.0, 0.0, 0.0),
        ]

        stats = compute_smoothness_stats(path, 0.0, 4.0)

        self.assertGreater(stats["step_max_m"], 4.9)
        self.assertGreater(stats["velocity_delta_p95_mps"], 3.9)
        self.assertTrue(math.isfinite(stats["jerk_p95_mps3"]))


if __name__ == "__main__":
    unittest.main()
