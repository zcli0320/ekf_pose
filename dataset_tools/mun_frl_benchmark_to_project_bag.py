#!/usr/bin/env python3
import argparse
import os

import rosbag
from sensor_msgs.msg import NavSatFix


def normalize_imu(msg, frame_id):
    msg.header.frame_id = frame_id
    return msg


def normalize_odom(msg, frame_id, child_frame_id):
    msg.header.frame_id = frame_id
    msg.child_frame_id = child_frame_id
    return msg


def normalize_fix(msg, frame_id, cov_xy, cov_z):
    msg.header.frame_id = frame_id
    if msg.position_covariance_type == NavSatFix.COVARIANCE_TYPE_UNKNOWN:
        msg.position_covariance[0] = cov_xy
        msg.position_covariance[4] = cov_xy
        msg.position_covariance[8] = cov_z
        msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
    return msg


def main():
    parser = argparse.ArgumentParser(
        description="Convert a MUN-FRL benchmarking bag to this EKF project's topic contract."
    )
    parser.add_argument("--input-bag", required=True)
    parser.add_argument("--output-bag", required=True)
    parser.add_argument("--imu-topic", default="/imu/data")
    parser.add_argument("--odom-topic", default="/Odometry")
    parser.add_argument("--gnss-topic", default="/fix")
    parser.add_argument("--imu-frame-id", default="imu_link")
    parser.add_argument("--odom-frame-id", default="map")
    parser.add_argument("--odom-child-frame-id", default="base_link")
    parser.add_argument("--gnss-frame-id", default="gps_link")
    parser.add_argument("--gnss-cov-xy", type=float, default=0.5184)
    parser.add_argument("--gnss-cov-z", type=float, default=2.0736)
    args = parser.parse_args()

    output_dir = os.path.dirname(os.path.abspath(args.output_bag))
    if output_dir:
        os.makedirs(output_dir, exist_ok=True)

    topic_map = {
        args.imu_topic: "/mavros/imu/data",
        args.odom_topic: "/mavros/odometry/in",
        args.gnss_topic: "/mavros/global_position/global",
    }
    written = {topic: 0 for topic in topic_map.values()}

    with rosbag.Bag(args.output_bag, "w") as out_bag:
        with rosbag.Bag(args.input_bag) as in_bag:
            for topic, msg, _ in in_bag.read_messages(topics=list(topic_map.keys())):
                out_topic = topic_map[topic]
                if topic == args.imu_topic:
                    msg = normalize_imu(msg, args.imu_frame_id)
                elif topic == args.odom_topic:
                    msg = normalize_odom(msg, args.odom_frame_id, args.odom_child_frame_id)
                elif topic == args.gnss_topic:
                    msg = normalize_fix(msg, args.gnss_frame_id, args.gnss_cov_xy, args.gnss_cov_z)
                out_bag.write(out_topic, msg, msg.header.stamp)
                written[out_topic] += 1

    print("input_bag={}".format(args.input_bag))
    print("output_bag={}".format(args.output_bag))
    for topic, count in sorted(written.items()):
        print("{} {}".format(topic, count))


if __name__ == "__main__":
    main()
