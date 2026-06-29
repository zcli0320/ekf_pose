#!/usr/bin/env python3
"""Publish RViz-only TF frames and frame-axis markers for odom, EKF, and GNSS."""

import rospy
import tf2_ros
from geometry_msgs.msg import Point, TransformStamped
from nav_msgs.msg import Odometry, Path
from visualization_msgs.msg import Marker, MarkerArray


def make_transform(stamp, parent_frame, child_frame, pose):
    transform = TransformStamped()
    transform.header.stamp = stamp
    transform.header.frame_id = parent_frame
    transform.child_frame_id = child_frame
    transform.transform.translation.x = pose.position.x
    transform.transform.translation.y = pose.position.y
    transform.transform.translation.z = pose.position.z
    transform.transform.rotation = pose.orientation
    return transform


def last_path_pose(path_msg):
    if not path_msg.poses:
        return None
    pose_stamped = path_msg.poses[-1]
    stamp = pose_stamped.header.stamp
    if stamp == rospy.Time(0):
        stamp = path_msg.header.stamp
    return stamp, pose_stamped.pose


def _point(x, y, z):
    point = Point()
    point.x = x
    point.y = y
    point.z = z
    return point


def make_axis_markers(stamp, frame_lengths, shaft_diameter=0.08,
                      head_diameter=0.24, head_length=0.36):
    markers = MarkerArray()
    axes = [
        ("x", (1.0, 0.0, 0.0), (1.0, 0.05, 0.05)),
        ("y", (0.0, 1.0, 0.0), (0.05, 1.0, 0.05)),
        ("z", (0.0, 0.0, 1.0), (0.15, 0.35, 1.0)),
    ]
    marker_id = 0
    for frame, length in frame_lengths:
        for axis_name, direction, color in axes:
            marker = Marker()
            marker.header.stamp = stamp
            marker.header.frame_id = frame
            marker.ns = "{}_{}".format(frame, axis_name)
            marker.id = marker_id
            marker.type = Marker.ARROW
            marker.action = Marker.ADD
            marker.frame_locked = True
            marker.points = [
                _point(0.0, 0.0, 0.0),
                _point(direction[0] * length, direction[1] * length, direction[2] * length),
            ]
            marker.scale.x = shaft_diameter
            marker.scale.y = head_diameter
            marker.scale.z = head_length
            marker.color.r = color[0]
            marker.color.g = color[1]
            marker.color.b = color[2]
            marker.color.a = 1.0
            markers.markers.append(marker)
            marker_id += 1
    return markers


class DisplayFramePublisher:
    def __init__(self):
        self.world_frame = rospy.get_param("~world_frame", "odom")
        self.odom_frame = rospy.get_param("~odom_frame", "odom_input_frame")
        self.ekf_frame = rospy.get_param("~ekf_frame", "ekf_base_link")
        self.gnss_frame = rospy.get_param("~gnss_frame", "aligned_gnss_frame")
        self.odom_axis_length = rospy.get_param("~odom_axis_length", 3.0)
        self.ekf_axis_length = rospy.get_param("~ekf_axis_length", 4.5)
        self.gnss_axis_length = rospy.get_param("~gnss_axis_length", 2.0)
        self.axis_shaft_diameter = rospy.get_param("~axis_shaft_diameter", 0.08)
        self.axis_head_diameter = rospy.get_param("~axis_head_diameter", 0.24)
        self.axis_head_length = rospy.get_param("~axis_head_length", 0.36)
        odom_topic = rospy.get_param("~odom_topic", "/mavros/odometry/out")
        ekf_topic = rospy.get_param("~ekf_topic", "/ekf/ekf_odom")
        gnss_path_topic = rospy.get_param("~gnss_path_topic", "/ekf/gnss_path")
        axis_marker_topic = rospy.get_param("~axis_marker_topic", "/ekf/display_frame_axes")

        self.broadcaster = tf2_ros.TransformBroadcaster()
        self.axis_pub = rospy.Publisher(axis_marker_topic, MarkerArray, queue_size=1, latch=True)
        self.subscribers = [
            rospy.Subscriber(odom_topic, Odometry, self.odom_callback, queue_size=20),
            rospy.Subscriber(ekf_topic, Odometry, self.ekf_callback, queue_size=20),
            rospy.Subscriber(gnss_path_topic, Path, self.gnss_path_callback, queue_size=5),
        ]
        self.axis_timer = rospy.Timer(rospy.Duration(1.0), self.publish_axis_markers)
        self.publish_axis_markers(None)

        rospy.loginfo(
            "Display TF frames: %s -> {%s, %s, %s}; topics odom=%s ekf=%s gnss_path=%s axes=%s",
            self.world_frame,
            self.odom_frame,
            self.ekf_frame,
            self.gnss_frame,
            odom_topic,
            ekf_topic,
            gnss_path_topic,
            axis_marker_topic,
        )

    def publish_axis_markers(self, _event):
        markers = make_axis_markers(
            rospy.Time(0),
            [
                (self.odom_frame, self.odom_axis_length),
                (self.ekf_frame, self.ekf_axis_length),
                (self.gnss_frame, self.gnss_axis_length),
            ],
            self.axis_shaft_diameter,
            self.axis_head_diameter,
            self.axis_head_length,
        )
        self.axis_pub.publish(markers)

    def odom_callback(self, msg):
        stamp = msg.header.stamp
        if stamp == rospy.Time(0):
            stamp = rospy.Time.now()
        self.broadcaster.sendTransform(
            make_transform(stamp, self.world_frame, self.odom_frame, msg.pose.pose)
        )

    def ekf_callback(self, msg):
        stamp = msg.header.stamp
        if stamp == rospy.Time(0):
            stamp = rospy.Time.now()
        self.broadcaster.sendTransform(
            make_transform(stamp, self.world_frame, self.ekf_frame, msg.pose.pose)
        )

    def gnss_path_callback(self, msg):
        latest = last_path_pose(msg)
        if latest is None:
            return
        stamp, pose = latest
        if stamp == rospy.Time(0):
            stamp = rospy.Time.now()
        self.broadcaster.sendTransform(
            make_transform(stamp, self.world_frame, self.gnss_frame, pose)
        )


def main():
    rospy.init_node("publish_display_frames")
    DisplayFramePublisher()
    rospy.spin()


if __name__ == "__main__":
    main()
