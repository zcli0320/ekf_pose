#!/usr/bin/env python3

import os
import unittest
import xml.etree.ElementTree as ET


REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
LAUNCH_PATH = os.path.join(REPO_ROOT, "launch", "ekf_lidar.launch")
NODE_PATH = os.path.join(REPO_ROOT, "src", "ekf_node_vio_timesync_with_acc_pub.cpp")


def launch_root():
    return ET.parse(LAUNCH_PATH).getroot()


def launch_arg_default(name):
    for element in launch_root().iter("arg"):
        if element.attrib.get("name") == name:
            return element.attrib["default"]
    raise AssertionError("missing launch arg: {}".format(name))


class EkfSmoothingConfigTest(unittest.TestCase):
    def test_launch_exposes_smoothing_parameters(self):
        self.assertEqual(launch_arg_default("gnss_velocity_window_size"), "2")
        self.assertEqual(launch_arg_default("gnss_velocity_smoothing_alpha"), "1.0")
        self.assertEqual(launch_arg_default("odom_recovery_frames"), "45")
        self.assertEqual(launch_arg_default("odom_recovery_min_scale"), "1.0")
        self.assertEqual(launch_arg_default("gyro_bias_rw_cov"), "0.0")
        self.assertEqual(launch_arg_default("acc_bias_rw_cov"), "0.0")
        self.assertEqual(launch_arg_default("enable_output_motion_smoothing"), "true")
        self.assertEqual(launch_arg_default("output_smoothing_natural_freq"), "3.0")
        self.assertEqual(launch_arg_default("output_smoothing_damping_ratio"), "1.0")
        self.assertEqual(launch_arg_default("output_smoothing_max_accel"), "50.0")
        self.assertEqual(launch_arg_default("output_smoothing_max_correction_speed"), "20.0")
        self.assertEqual(launch_arg_default("output_smoothing_normal_natural_freq"), "3.5")
        self.assertEqual(launch_arg_default("output_smoothing_normal_max_accel"), "50.0")
        self.assertEqual(launch_arg_default("output_smoothing_normal_max_correction_speed"), "20.0")
        self.assertEqual(launch_arg_default("output_smoothing_release_error"), "0.005")
        self.assertEqual(launch_arg_default("output_smoothing_recovery_duration"), "0.8")

    def test_node_uses_windowed_gnss_velocity_and_recovery_ramp(self):
        with open(NODE_PATH, encoding="utf-8") as source:
            code = source.read()

        self.assertIn("estimate_gnss_window_velocity", code)
        self.assertIn("gnss_velocity_smoothing_alpha", code)
        self.assertIn("odom_recovery_scale_for_current_frame", code)
        self.assertIn("gyro_bias_rw_cov", code)
        self.assertIn("acc_bias_rw_cov", code)
        self.assertIn("kErrorGyroBiasOffset", code)
        self.assertIn("kErrorAccelBiasOffset", code)

    def test_node_uses_consistent_output_motion_smoother(self):
        with open(NODE_PATH, encoding="utf-8") as source:
            code = source.read()

        self.assertIn("smooth_output_motion", code)
        self.assertIn("output_smoothing_natural_freq", code)
        self.assertIn("output_smoothing_damping_ratio", code)
        self.assertIn("second_order_position_correction_velocity", code)
        self.assertIn("output_smoothing_normal_natural_freq", code)
        self.assertIn("output_smoothing_low_latency_mode_active", code)
        self.assertIn("output_smoothing_release_error", code)
        self.assertIn("output_smoothing_recovery_duration", code)
        self.assertIn("output_smoothing_recovery_output_active", code)
        self.assertIn("output_smoothing_max_accel", code)
        self.assertIn("output_smoothing_max_correction_speed", code)
        self.assertIn("output_filter_velocity", code)
        self.assertIn("odom_fusion.twist.twist.linear.x = output_velocity", code)


if __name__ == "__main__":
    unittest.main()
