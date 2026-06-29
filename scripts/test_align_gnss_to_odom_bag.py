#!/usr/bin/env python3
import math
import os
import sys
import unittest

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "dataset_tools"))

from align_gnss_to_odom_bag import align_positions_sliding, fit_rigid_2d, smooth_aligned_positions


def transform_xy(point, yaw, translation):
    c = math.cos(yaw)
    s = math.sin(yaw)
    return np.array(
        [
            c * point[0] - s * point[1] + translation[0],
            s * point[0] + c * point[1] + translation[1],
            point[2] + translation[2],
        ],
        dtype=float,
    )


class AlignGnssToOdomBagTest(unittest.TestCase):
    def test_smooth_alignment_limits_step_discontinuity(self):
        aligned = [
            (0.0, np.array([0.0, 0.0, 0.0], dtype=float)),
            (1.0, np.array([1.0, 0.0, 0.0], dtype=float)),
            (2.0, np.array([5.0, 0.0, 0.0], dtype=float)),
        ]
        odom = [
            (0.0, np.array([0.0, 0.0, 0.0], dtype=float)),
            (1.0, np.array([1.0, 0.0, 0.0], dtype=float)),
            (2.0, np.array([2.0, 0.0, 0.0], dtype=float)),
        ]

        smoothed = smooth_aligned_positions(aligned, odom, max_step_correction=0.2)
        previous_step = smoothed[1][1][:2] - smoothed[0][1][:2]
        current_step = smoothed[2][1][:2] - smoothed[1][1][:2]
        odom_step = odom[2][1][:2] - odom[1][1][:2]

        self.assertLessEqual(np.linalg.norm(current_step - odom_step), 0.2 + 1.0e-9)
        self.assertGreater(np.linalg.norm(aligned[2][1][:2] - aligned[1][1][:2]), 3.0)
        self.assertAlmostEqual(np.linalg.norm(previous_step), 1.0)

    def test_smooth_alignment_limits_z_step_discontinuity(self):
        aligned = [
            (0.0, np.array([0.0, 0.0, 0.0], dtype=float)),
            (1.0, np.array([1.0, 0.0, 0.1], dtype=float)),
            (2.0, np.array([2.0, 0.0, 3.0], dtype=float)),
        ]
        odom = [
            (0.0, np.array([0.0, 0.0, 0.0], dtype=float)),
            (1.0, np.array([1.0, 0.0, 0.1], dtype=float)),
            (2.0, np.array([2.0, 0.0, 0.2], dtype=float)),
        ]

        smoothed = smooth_aligned_positions(
            aligned,
            odom,
            max_step_correction=0.2,
            max_z_step_correction=0.08,
        )
        current_z_step = smoothed[2][1][2] - smoothed[1][1][2]
        odom_z_step = odom[2][1][2] - odom[1][1][2]

        self.assertLessEqual(abs(current_z_step - odom_z_step), 0.08 + 1.0e-9)
        self.assertGreater(abs(aligned[2][1][2] - aligned[1][1][2]), 2.0)

    def test_sliding_alignment_handles_time_varying_yaw(self):
        gnss = []
        odom = []
        translation = np.array([4.0, -2.0, 1.5], dtype=float)
        for i in range(120):
            stamp = float(i)
            raw = np.array([0.8 * i, 6.0 * math.sin(i / 15.0), 0.2 * math.cos(i / 10.0)], dtype=float)
            yaw = math.radians(-25.0 if i < 60 else 18.0)
            gnss.append((stamp, raw))
            odom.append((stamp, transform_xy(raw, yaw, translation)))

        _, _, global_errors = fit_rigid_2d([g[1] for g in gnss], [o[1] for o in odom])
        aligned, stats = align_positions_sliding(gnss, odom, window_s=25.0, min_pairs=8)
        aligned_errors = [
            float(np.linalg.norm(aligned_pos[:2] - odom_pos[:2]))
            for (_, aligned_pos), (_, odom_pos) in zip(aligned, odom)
        ]

        self.assertGreater(np.percentile(global_errors, 95), 8.0)
        self.assertLess(np.percentile(aligned_errors, 95), 0.8)
        self.assertEqual(stats["mode"], "sliding")
        self.assertEqual(stats["count"], len(gnss))


if __name__ == "__main__":
    unittest.main()
