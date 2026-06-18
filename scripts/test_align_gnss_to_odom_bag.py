#!/usr/bin/env python3
import math
import os
import sys
import unittest

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "dataset_tools"))

from align_gnss_to_odom_bag import align_positions_sliding, fit_rigid_2d


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
