#!/usr/bin/env python3

import os
import re
import unittest
import xml.etree.ElementTree as ET


REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
LAUNCH_PATH = os.path.join(REPO_ROOT, "launch", "ekf_lidar.launch")
NODE_PATH = os.path.join(REPO_ROOT, "src", "ekf_node_vio_timesync_with_acc_pub.cpp")


def launch_arg_default(name):
    tree = ET.parse(LAUNCH_PATH)
    for element in tree.getroot().iter("arg"):
        if element.attrib.get("name") == name:
            return element.attrib["default"]
    raise AssertionError("missing launch arg: {}".format(name))


def cpp_default(name):
    with open(NODE_PATH, encoding="utf-8") as source:
        text = source.read()
    pattern = r"(?:bool|int|double)\s+{}\s*=\s*([^;]+);".format(re.escape(name))
    match = re.search(pattern, text)
    if not match:
        raise AssertionError("missing C++ default: {}".format(name))
    return match.group(1).strip()


class OdomRecoveryConfigTest(unittest.TestCase):
    def test_launch_exposes_odom_recovery_guard(self):
        self.assertEqual(launch_arg_default("enable_odom_recovery_guard"), "true")
        self.assertGreaterEqual(int(launch_arg_default("odom_recovery_frames")), 20)
        self.assertGreaterEqual(float(launch_arg_default("odom_recovery_scale")), 1000.0)

    def test_cpp_defaults_match_recovery_policy(self):
        self.assertEqual(cpp_default("enable_odom_recovery_guard"), "true")
        self.assertGreaterEqual(int(cpp_default("odom_recovery_frames")), 20)
        self.assertGreaterEqual(float(cpp_default("odom_recovery_scale")), 1000.0)

    def test_recovery_guard_skips_jump_realign_after_lost(self):
        with open(NODE_PATH, encoding="utf-8") as source:
            text = source.read()
        self.assertIn("odom_lost_before_update", text)
        self.assertIn("odom_recovery_frames_remaining", text)
        self.assertRegex(text, r"if \(odom_step > odom_jump_threshold && !odom_lost_before_update\)")
        self.assertIn("odom_recovery_active()", text)


if __name__ == "__main__":
    unittest.main()
