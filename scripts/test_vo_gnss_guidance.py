#!/usr/bin/env python3
import math
import unittest

from vo_gnss_imu_guidance import PoseSample, Similarity2DGuidanceCore, fit_similarity_2d, quat_multiply, yaw_quat


def rotate_xy(point, yaw):
    c = math.cos(yaw)
    s = math.sin(yaw)
    return (c * point[0] - s * point[1], s * point[0] + c * point[1])


def make_reference_from_raw(raw_position, scale, yaw, translation):
    xy = rotate_xy(raw_position, yaw)
    return (scale * xy[0] + translation[0], scale * xy[1] + translation[1], 0.0)


class VoGnssGuidanceTest(unittest.TestCase):
    def test_similarity_fit_recovers_scale_yaw_translation(self):
        true_scale = 2.5
        true_yaw = math.radians(35.0)
        true_translation = (12.0, -7.0)
        raw = [(i * 0.8, 0.2 * math.sin(i * 0.1)) for i in range(20)]
        ref = [make_reference_from_raw((p[0], p[1], 0.0), true_scale, true_yaw, true_translation)[:2] for p in raw]

        scale, yaw, translation, residuals = fit_similarity_2d(raw, ref)

        self.assertAlmostEqual(scale, true_scale, places=9)
        self.assertAlmostEqual(yaw, true_yaw, places=9)
        self.assertAlmostEqual(translation[0], true_translation[0], places=9)
        self.assertAlmostEqual(translation[1], true_translation[1], places=9)
        self.assertLess(max(residuals), 1.0e-9)

    def test_horizontal_uniform_fast_motion_becomes_ready(self):
        true_scale = 3.0
        true_yaw = math.radians(-20.0)
        true_translation = (30.0, 15.0, 2.0)
        core = Similarity2DGuidanceCore(min_pairs=6, ready_frames=3, min_motion=8.0,
                                        min_speed=5.0, max_residual=0.05,
                                        require_imu=True, max_pair_dt=0.02)
        dt = 0.2
        ref_speed = 6.0
        raw_speed = ref_speed / true_scale
        for i in range(20):
            stamp = i * dt
            core.add_imu(stamp)
            raw_position = (raw_speed * stamp, 0.0, 0.0)
            core.add_odom(PoseSample(stamp, raw_position))
            ref_position = make_reference_from_raw(raw_position, true_scale, true_yaw, true_translation[:2])
            ref_position = (ref_position[0], ref_position[1], true_translation[2])
            core.add_reference(stamp, ref_position)

        self.assertTrue(core.ready)
        self.assertAlmostEqual(core.transform.scale, true_scale, places=6)
        self.assertAlmostEqual(core.transform.yaw, true_yaw, places=6)
        self.assertLess(core.transform.residual_max, 1.0e-9)
        guided = core.transform_sample(PoseSample(4.0, (raw_speed * 4.0, 0.0, 0.0)))
        expected = make_reference_from_raw((raw_speed * 4.0, 0.0, 0.0), true_scale, true_yaw, true_translation[:2])
        expected = (expected[0], expected[1], true_translation[2])
        self.assertAlmostEqual(guided.position[0], expected[0], places=6)
        self.assertAlmostEqual(guided.position[1], expected[1], places=6)
        self.assertAlmostEqual(guided.position[2], expected[2], places=6)

    def test_slow_motion_is_not_accepted(self):
        core = Similarity2DGuidanceCore(min_pairs=6, ready_frames=2, min_motion=2.0,
                                        min_speed=5.0, require_imu=False)
        for i in range(12):
            stamp = i * 0.5
            raw_position = (stamp, 0.0, 0.0)
            core.add_odom(PoseSample(stamp, raw_position))
            core.add_reference(stamp, raw_position)

        self.assertFalse(core.ready)
        self.assertEqual(core.rejected_reason, "insufficient_horizontal_motion")

    def test_vertical_motion_is_rejected(self):
        core = Similarity2DGuidanceCore(min_pairs=6, ready_frames=2, min_motion=5.0,
                                        min_speed=5.0, max_vertical_motion=1.0,
                                        require_imu=False)
        for i in range(12):
            stamp = i * 0.2
            raw_position = (6.0 * stamp, 0.0, 0.6 * i)
            core.add_odom(PoseSample(stamp, raw_position))
            core.add_reference(stamp, raw_position)

        self.assertFalse(core.ready)
        self.assertEqual(core.rejected_reason, "not_horizontal_motion")

    def test_transform_rotates_orientation_and_velocity(self):
        core = Similarity2DGuidanceCore(min_pairs=3, ready_frames=1, min_motion=1.0,
                                        min_speed=1.0, require_imu=False)
        core.transform = type("T", (), {
            "scale": 2.0,
            "yaw": math.pi / 2.0,
            "translation": (1.0, 2.0, 3.0),
            "residual_max": 0.0,
        })()
        sample = PoseSample(1.0, (1.0, 0.0, 0.0), yaw_quat(0.0), (1.0, 0.0, 0.0), (0.1, 0.0, 0.2))
        guided = core.transform_sample(sample)

        self.assertAlmostEqual(guided.position[0], 1.0, places=6)
        self.assertAlmostEqual(guided.position[1], 4.0, places=6)
        self.assertAlmostEqual(guided.position[2], 3.0, places=6)
        self.assertAlmostEqual(guided.linear_velocity[0], 0.0, places=6)
        self.assertAlmostEqual(guided.linear_velocity[1], 2.0, places=6)
        expected_q = quat_multiply(yaw_quat(math.pi / 2.0), yaw_quat(0.0))
        for a, b in zip(guided.orientation, expected_q):
            self.assertAlmostEqual(a, b, places=6)


if __name__ == "__main__":
    unittest.main()
