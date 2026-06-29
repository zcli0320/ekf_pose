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


class GnssAlignmentRefinementConfigTest(unittest.TestCase):
    def test_launch_enables_guarded_online_refinement(self):
        self.assertEqual(launch_arg_default("enable_gnss_alignment_refinement"), "true")
        self.assertGreaterEqual(float(launch_arg_default("gnss_alignment_refinement_min_motion")), 12.0)
        self.assertGreaterEqual(float(launch_arg_default("gnss_alignment_refinement_max_residual")), 2.0)
        self.assertLessEqual(float(launch_arg_default("gnss_alignment_refinement_gain")), 0.35)

    def test_cpp_defaults_match_launch_refinement_policy(self):
        self.assertEqual(cpp_default("enable_gnss_alignment_refinement"), "true")
        self.assertGreaterEqual(float(cpp_default("gnss_alignment_refinement_min_motion")), 12.0)
        self.assertGreaterEqual(float(cpp_default("gnss_alignment_refinement_max_residual")), 2.0)
        self.assertLessEqual(float(cpp_default("gnss_alignment_refinement_gain")), 0.35)


if __name__ == "__main__":
    unittest.main()
