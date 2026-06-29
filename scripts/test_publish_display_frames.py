#!/usr/bin/env python3

import os
import sys
import unittest

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from rospy import Time
from visualization_msgs.msg import Marker

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

from publish_display_frames import last_path_pose, make_axis_markers, make_transform


class DisplayFramePublisherTest(unittest.TestCase):
    def test_make_transform_copies_pose_and_frame_ids(self):
        pose = PoseStamped()
        pose.pose.position.x = 1.0
        pose.pose.position.y = -2.0
        pose.pose.position.z = 3.5
        pose.pose.orientation.x = 0.1
        pose.pose.orientation.y = 0.2
        pose.pose.orientation.z = 0.3
        pose.pose.orientation.w = 0.9

        transform = make_transform(Time.from_sec(12.5), "odom", "ekf_base_link", pose.pose)

        self.assertEqual(transform.header.frame_id, "odom")
        self.assertEqual(transform.child_frame_id, "ekf_base_link")
        self.assertEqual(transform.header.stamp, Time.from_sec(12.5))
        self.assertEqual(transform.transform.translation.x, 1.0)
        self.assertEqual(transform.transform.translation.y, -2.0)
        self.assertEqual(transform.transform.translation.z, 3.5)
        self.assertEqual(transform.transform.rotation.x, 0.1)
        self.assertEqual(transform.transform.rotation.y, 0.2)
        self.assertEqual(transform.transform.rotation.z, 0.3)
        self.assertEqual(transform.transform.rotation.w, 0.9)

    def test_last_path_pose_returns_latest_pose(self):
        path = Path()
        first = PoseStamped()
        first.header.stamp = Time.from_sec(1.0)
        first.pose.position.x = 1.0
        second = PoseStamped()
        second.header.stamp = Time.from_sec(2.0)
        second.pose.position.x = 2.0
        path.poses = [first, second]

        stamp, pose = last_path_pose(path)

        self.assertEqual(stamp, Time.from_sec(2.0))
        self.assertEqual(pose.position.x, 2.0)

    def test_last_path_pose_handles_empty_path(self):
        self.assertIsNone(last_path_pose(Path()))

    def test_make_axis_markers_uses_distinct_frame_lengths(self):
        markers = make_axis_markers(
            Time.from_sec(3.0),
            [
                ("odom_input_frame", 3.0),
                ("ekf_base_link", 4.5),
                ("aligned_gnss_frame", 2.0),
            ],
            shaft_diameter=0.08,
            head_diameter=0.24,
            head_length=0.36,
        ).markers

        self.assertEqual(len(markers), 9)
        self.assertEqual({marker.header.frame_id for marker in markers},
                         {"odom_input_frame", "ekf_base_link", "aligned_gnss_frame"})
        x_axes = [marker for marker in markers if marker.ns.endswith("_x")]
        lengths = {marker.header.frame_id: marker.points[1].x for marker in x_axes}
        self.assertEqual(lengths["odom_input_frame"], 3.0)
        self.assertEqual(lengths["ekf_base_link"], 4.5)
        self.assertEqual(lengths["aligned_gnss_frame"], 2.0)
        self.assertTrue(all(marker.type == Marker.ARROW for marker in markers))
        self.assertTrue(all(marker.frame_locked for marker in markers))


if __name__ == "__main__":
    unittest.main()
