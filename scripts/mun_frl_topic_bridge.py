#!/usr/bin/env python3
import copy

import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, NavSatFix


class MunFrlTopicBridge:
    def __init__(self):
        self.imu_in = rospy.get_param("~imu_in", "/imu/data")
        self.fix_in = rospy.get_param("~fix_in", "/fix")
        self.odom_in = rospy.get_param("~odom_in", "")

        self.imu_out = rospy.get_param("~imu_out", "/mavros/imu/data")
        self.fix_out = rospy.get_param("~fix_out", "/mavros/global_position/global")
        self.odom_out = rospy.get_param("~odom_out", "/mavros/odometry/in")

        self.imu_frame_id = rospy.get_param("~imu_frame_id", "imu_link")
        self.gnss_frame_id = rospy.get_param("~gnss_frame_id", "gps_link")
        self.odom_frame_id = rospy.get_param("~odom_frame_id", "map")
        self.odom_child_frame_id = rospy.get_param("~odom_child_frame_id", "base_link")

        self.override_gnss_covariance = rospy.get_param("~override_gnss_covariance", False)
        self.gnss_cov_xy = rospy.get_param("~gnss_cov_xy", 0.4356)
        self.gnss_cov_z = rospy.get_param("~gnss_cov_z", 1.7424)

        self.imu_pub = rospy.Publisher(self.imu_out, Imu, queue_size=200)
        self.fix_pub = rospy.Publisher(self.fix_out, NavSatFix, queue_size=20)
        self.odom_pub = None
        if self.odom_in:
            self.odom_pub = rospy.Publisher(self.odom_out, Odometry, queue_size=50)

        rospy.Subscriber(self.imu_in, Imu, self.imu_callback, queue_size=400)
        rospy.Subscriber(self.fix_in, NavSatFix, self.fix_callback, queue_size=50)
        if self.odom_in:
            rospy.Subscriber(self.odom_in, Odometry, self.odom_callback, queue_size=100)

        rospy.loginfo("MUN-FRL bridge: %s -> %s", self.imu_in, self.imu_out)
        rospy.loginfo("MUN-FRL bridge: %s -> %s", self.fix_in, self.fix_out)
        if self.odom_in:
            rospy.loginfo("MUN-FRL bridge: %s -> %s", self.odom_in, self.odom_out)
        else:
            rospy.logwarn("MUN-FRL bridge: no odom_in set; run VIO/LiDAR odometry and pass its topic when available")

    def imu_callback(self, msg):
        out = copy.deepcopy(msg)
        out.header.frame_id = self.imu_frame_id
        self.imu_pub.publish(out)

    def fix_callback(self, msg):
        out = copy.deepcopy(msg)
        out.header.frame_id = self.gnss_frame_id
        if self.override_gnss_covariance or out.position_covariance_type == NavSatFix.COVARIANCE_TYPE_UNKNOWN:
            out.position_covariance = [0.0] * 9
            out.position_covariance[0] = self.gnss_cov_xy
            out.position_covariance[4] = self.gnss_cov_xy
            out.position_covariance[8] = self.gnss_cov_z
            out.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
        self.fix_pub.publish(out)

    def odom_callback(self, msg):
        out = copy.deepcopy(msg)
        out.header.frame_id = self.odom_frame_id
        out.child_frame_id = self.odom_child_frame_id
        self.odom_pub.publish(out)


def main():
    rospy.init_node("mun_frl_topic_bridge")
    MunFrlTopicBridge()
    rospy.spin()


if __name__ == "__main__":
    main()
