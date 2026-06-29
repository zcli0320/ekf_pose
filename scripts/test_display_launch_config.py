#!/usr/bin/env python3

import os
import unittest
import xml.etree.ElementTree as ET


REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
LAUNCH_PATH = os.path.join(REPO_ROOT, "launch", "ekf_lidar.launch")
CMAKE_PATH = os.path.join(REPO_ROOT, "CMakeLists.txt")
PACKAGE_PATH = os.path.join(REPO_ROOT, "package.xml")
RVIZ_PATH = os.path.join(REPO_ROOT, "launch", "data2_aligned_gnss_display.rviz")


def launch_root():
    return ET.parse(LAUNCH_PATH).getroot()


def launch_arg_default(name):
    for element in launch_root().iter("arg"):
        if element.attrib.get("name") == name:
            return element.attrib["default"]
    raise AssertionError("missing launch arg: {}".format(name))


class DisplayLaunchConfigTest(unittest.TestCase):
    def test_launch_can_start_display_frame_publisher(self):
        self.assertEqual(launch_arg_default("start_display_frames"), "true")

        nodes = [
            element for element in launch_root().iter("node")
            if element.attrib.get("type") == "publish_display_frames.py"
        ]
        self.assertEqual(len(nodes), 1)
        node = nodes[0]
        self.assertEqual(node.attrib.get("pkg"), "ekf")
        self.assertEqual(node.attrib.get("if"), "$(arg start_display_frames)")

        params = {element.attrib["name"]: element.attrib["value"] for element in node.iter("param")}
        self.assertEqual(params["world_frame"], "odom")
        self.assertEqual(params["odom_topic"], "$(arg odom_primary_topic)")
        self.assertEqual(params["ekf_topic"], "/ekf/ekf_odom")
        self.assertEqual(params["gnss_path_topic"], "/ekf/gnss_path")
        self.assertEqual(params["axis_marker_topic"], "/ekf/display_frame_axes")
        self.assertEqual(params["odom_axis_length"], "3.0")
        self.assertEqual(params["ekf_axis_length"], "4.5")
        self.assertEqual(params["gnss_axis_length"], "2.0")

    def test_launch_can_start_review_path_cache(self):
        self.assertEqual(launch_arg_default("start_review_paths"), "true")

        nodes = [
            element for element in launch_root().iter("node")
            if element.attrib.get("type") == "persist_review_paths.py"
        ]
        self.assertEqual(len(nodes), 1)
        node = nodes[0]
        self.assertEqual(node.attrib.get("pkg"), "ekf")
        self.assertEqual(node.attrib.get("if"), "$(arg start_review_paths)")

        params = {element.attrib["name"]: element.attrib["value"] for element in node.iter("param")}
        self.assertEqual(params["input_path_topic"], "/ekf/input_path")
        self.assertEqual(params["ekf_path_topic"], "/ekf/ekf_path")
        self.assertEqual(params["gnss_path_topic"], "/ekf/gnss_path")
        self.assertEqual(params["review_input_path_topic"], "/ekf/review/input_path")
        self.assertEqual(params["review_ekf_path_topic"], "/ekf/review/ekf_path")
        self.assertEqual(params["review_gnss_path_topic"], "/ekf/review/gnss_path")
        self.assertEqual(params["restamp_to_now"], "true")

    def test_display_launch_uses_dense_paths(self):
        self.assertEqual(launch_arg_default("path_publish_stride"), "1")

        params = {
            element.attrib["name"]: element.attrib["value"]
            for element in launch_root().iter("param")
            if "name" in element.attrib and "value" in element.attrib
        }
        self.assertEqual(params["path_publish_stride"], "$(arg path_publish_stride)")

    def test_data2_rviz_uses_thick_paths_and_frame_axis_markers(self):
        with open(RVIZ_PATH, encoding="utf-8") as source:
            rviz_text = source.read()

        self.assertEqual(rviz_text.count("Line Style: Billboards"), 3)
        self.assertEqual(rviz_text.count("Line Width: 0.05999999865889549"), 3)
        self.assertIn("Topic: /ekf/review/input_path", rviz_text)
        self.assertIn("Topic: /ekf/review/ekf_path", rviz_text)
        self.assertIn("Topic: /ekf/review/gnss_path", rviz_text)
        self.assertIn("Class: rviz/MarkerArray", rviz_text)
        self.assertIn("Marker Topic: /ekf/display_frame_axes", rviz_text)

    def test_tf2_ros_runtime_dependency_is_declared(self):
        with open(CMAKE_PATH, encoding="utf-8") as source:
            cmake_text = source.read()
        with open(PACKAGE_PATH, encoding="utf-8") as source:
            package_text = source.read()

        self.assertIn("tf2_ros", cmake_text)
        self.assertIn("<run_depend>tf2_ros</run_depend>", package_text)


if __name__ == "__main__":
    unittest.main()
