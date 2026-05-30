#!/usr/bin/env python3
import math
import unittest

from vio_gnss_imu_guidance import (
    PoseSample,
    VioGnssImuGuidanceCore,
    fit_yaw_translation_2d,
    quat_multiply,
    yaw_quat,
)


def rotate_xy(point, yaw):
    c = math.cos(yaw)
    s = math.sin(yaw)
    return (c * point[0] - s * point[1], s * point[0] + c * point[1])


def make_reference_from_vio(vio_position, yaw, translation):
    xy = rotate_xy(vio_position, yaw)
    return (xy[0] + translation[0], xy[1] + translation[1], vio_position[2] + translation[2])


class VioGnssGuidanceTest(unittest.TestCase):
    def test_yaw_translation_fit_keeps_vio_scale(self):
        true_yaw = math.radians(28.0)
        true_translation = (12.0, -4.0)
        vio = [(i * 0.7, 0.2 * math.sin(i * 0.2)) for i in range(20)]
        ref = [make_reference_from_vio((p[0], p[1], 0.0), true_yaw, (true_translation[0], true_translation[1], 0.0))[:2]
               for p in vio]

        yaw, translation, residuals, scale_estimate = fit_yaw_translation_2d(vio, ref)

        self.assertAlmostEqual(yaw, true_yaw, places=9)
        self.assertAlmostEqual(translation[0], true_translation[0], places=9)
        self.assertAlmostEqual(translation[1], true_translation[1], places=9)
        self.assertAlmostEqual(scale_estimate, 1.0, places=9)
        self.assertLess(max(residuals), 1.0e-9)

    def test_metric_vio_becomes_ready_with_short_motion(self):
        true_yaw = math.radians(-15.0)
        true_translation = (20.0, 7.0, 1.5)
        core = VioGnssImuGuidanceCore(min_pairs=5, ready_frames=2, min_motion=1.5,
                                      min_speed=0.3, max_residual=0.05,
                                      require_imu=True, max_pair_dt=0.03)
        for i in range(12):
            stamp = i * 0.2
            core.add_imu(stamp)
            vio_position = (0.6 * i, 0.05 * math.sin(i), 0.1)
            core.add_odom(PoseSample(stamp, vio_position))
            core.add_reference(stamp, make_reference_from_vio(vio_position, true_yaw, true_translation))

        self.assertTrue(core.ready)
        self.assertEqual(core.state, core.READY)
        self.assertAlmostEqual(core.transform.yaw, true_yaw, places=6)
        self.assertLess(core.transform.residual_max, 1.0e-9)
        guided = core.transform_sample(PoseSample(2.4, (7.2, 0.0, 0.1)))
        expected = make_reference_from_vio((7.2, 0.0, 0.1), true_yaw, true_translation)
        for a, b in zip(guided.position, expected):
            self.assertAlmostEqual(a, b, places=6)

    def test_scale_mismatch_is_rejected(self):
        core = VioGnssImuGuidanceCore(min_pairs=5, ready_frames=1, min_motion=1.0,
                                      min_speed=0.1, max_scale_error=0.2, require_imu=False)
        for i in range(10):
            stamp = i * 0.2
            vio_position = (float(i), 0.0, 0.0)
            core.add_odom(PoseSample(stamp, vio_position))
            core.add_reference(stamp, (2.0 * vio_position[0], 0.0, 0.0))

        self.assertFalse(core.ready)
        self.assertEqual(core.rejected_reason, "vio_scale_inconsistent")

    def test_lost_then_recovering_returns_ready_after_stable_window(self):
        core = VioGnssImuGuidanceCore(min_pairs=4, ready_frames=1, recovery_frames=3,
                                      min_motion=1.0, min_speed=0.1, lost_timeout=0.5,
                                      require_imu=False, reference_sample_interval=0.0)
        for i in range(6):
            stamp = i * 0.2
            p = (float(i), 0.0, 0.0)
            core.add_odom(PoseSample(stamp, p))
            core.add_reference(stamp, p)
        self.assertTrue(core.ready)

        core.mark_time(2.0)
        self.assertEqual(core.state, core.LOST)

        for i in range(6, 12):
            stamp = i * 0.2
            p = (float(i), 0.0, 0.0)
            core.add_odom(PoseSample(stamp, p))
            core.add_reference(stamp, p)

        self.assertEqual(core.state, core.READY)
        self.assertTrue(core.ready)

    def test_vio_pose_jump_enters_recovering(self):
        core = VioGnssImuGuidanceCore(min_pairs=4, ready_frames=1, recovery_frames=3,
                                      min_motion=1.0, min_speed=0.1, reset_distance=1.0,
                                      require_imu=False)
        for i in range(6):
            stamp = i * 0.2
            p = (float(i), 0.0, 0.0)
            core.add_odom(PoseSample(stamp, p))
            core.add_reference(stamp, p)
        self.assertTrue(core.ready)

        core.add_odom(PoseSample(1.4, (20.0, 0.0, 0.0)))

        self.assertEqual(core.state, core.RECOVERING)
        self.assertEqual(core.rejected_reason, "vio_reset_detected")

    def test_transform_rotates_orientation_and_velocity_without_scaling(self):
        core = VioGnssImuGuidanceCore(min_pairs=3, ready_frames=1, min_motion=1.0,
                                      min_speed=0.1, require_imu=False)
        core.transform = type("T", (), {
            "yaw": math.pi / 2.0,
            "translation": (1.0, 2.0, 3.0),
            "residual_max": 0.0,
            "yaw_observable": True,
        })()
        core.state = core.READY
        sample = PoseSample(1.0, (1.0, 0.0, 0.5), yaw_quat(0.0), (1.0, 0.0, 0.2), (0.1, 0.0, 0.2))
        guided = core.transform_sample(sample)

        self.assertAlmostEqual(guided.position[0], 1.0, places=6)
        self.assertAlmostEqual(guided.position[1], 3.0, places=6)
        self.assertAlmostEqual(guided.position[2], 3.5, places=6)
        self.assertAlmostEqual(guided.linear_velocity[0], 0.0, places=6)
        self.assertAlmostEqual(guided.linear_velocity[1], 1.0, places=6)
        expected_q = quat_multiply(yaw_quat(math.pi / 2.0), yaw_quat(0.0))
        for a, b in zip(guided.orientation, expected_q):
            self.assertAlmostEqual(a, b, places=6)

    def test_translation_only_ready_when_motion_is_too_small(self):
        core = VioGnssImuGuidanceCore(min_pairs=4, ready_frames=2, min_motion=10.0,
                                      min_speed=5.0, translation_only_max_residual=0.1,
                                      require_imu=False, allow_initial_translation_only=True)
        translation = (3.0, -2.0, 1.0)
        for i in range(8):
            stamp = i * 0.2
            p = (0.05 * i, 0.0, 0.1)
            core.add_odom(PoseSample(stamp, p))
            core.add_reference(stamp, (p[0] + translation[0], p[1] + translation[1], p[2] + translation[2]))

        self.assertTrue(core.ready)
        self.assertFalse(core.transform.yaw_observable)
        self.assertGreater(core.observation_scale(), 1.0)

    def test_initial_translation_only_disabled_by_default(self):
        core = VioGnssImuGuidanceCore(min_pairs=4, ready_frames=1, min_motion=10.0,
                                      min_speed=5.0, translation_only_max_residual=0.1,
                                      require_imu=False)
        for i in range(6):
            stamp = i * 0.2
            p = (0.05 * i, 0.0, 0.1)
            core.add_odom(PoseSample(stamp, p))
            core.add_reference(stamp, (p[0] + 3.0, p[1] - 2.0, p[2] + 1.0))

        self.assertFalse(core.ready)
        self.assertEqual(core.rejected_reason, "insufficient_motion")


if __name__ == "__main__":
    unittest.main()
