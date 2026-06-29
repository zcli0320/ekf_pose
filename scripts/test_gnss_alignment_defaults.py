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
    pattern = r"(?:int|double)\s+{}\s*=\s*([^;]+);".format(re.escape(name))
    match = re.search(pattern, text)
    if not match:
        raise AssertionError("missing C++ default: {}".format(name))
    return match.group(1).strip()


class GnssAlignmentDefaultsTest(unittest.TestCase):
    def test_launch_defaults_wait_for_observable_yaw_motion(self):
        self.assertGreaterEqual(float(launch_arg_default("gnss_alignment_min_motion")), 20.0)
        self.assertGreaterEqual(int(launch_arg_default("gnss_alignment_max_samples")), 120)
        self.assertGreaterEqual(float(launch_arg_default("gnss_alignment_max_residual")), 3.0)

    def test_cpp_defaults_match_conservative_alignment_window(self):
        self.assertGreaterEqual(float(cpp_default("gnss_alignment_min_motion")), 20.0)
        self.assertGreaterEqual(int(cpp_default("gnss_alignment_max_samples")), 120)
        self.assertGreaterEqual(float(cpp_default("gnss_alignment_max_residual")), 3.0)


if __name__ == "__main__":
    unittest.main()
