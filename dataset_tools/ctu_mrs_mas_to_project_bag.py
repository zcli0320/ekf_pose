#!/usr/bin/env python3
import argparse
import math
import os
from bisect import bisect_left

import rosbag
import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, NavSatFix


def nearest(seq, stamp, max_dt):
    if not seq:
        return None
    times = [item[0] for item in seq]
    index = bisect_left(times, stamp)
    candidates = []
    if index < len(seq):
        candidates.append(seq[index])
    if index > 0:
        candidates.append(seq[index - 1])
    if not candidates:
        return None
    best = min(candidates, key=lambda item: abs(item[0] - stamp))
    if abs(best[0] - stamp) > max_dt:
        return None
    return best


def collect_positions(bag_path, topic):
    positions = []
    with rosbag.Bag(bag_path) as bag:
        for _, msg, _ in bag.read_messages(topics=[topic]):
            p = msg.pose.pose.position
            positions.append((msg.header.stamp.to_sec(), (p.x, p.y, p.z)))
    return positions


def estimate_yaw_translation(source_positions, target_positions, max_dt):
    pairs = []
    for stamp, source in source_positions:
        target = nearest(target_positions, stamp, max_dt)
        if target is not None:
            pairs.append((source, target[1]))
    if len(pairs) < 10:
        raise RuntimeError("Need at least 10 paired samples for alignment")

    source_center = [sum(pair[0][i] for pair in pairs) / len(pairs) for i in range(3)]
    target_center = [sum(pair[1][i] for pair in pairs) / len(pairs) for i in range(3)]
    sin_term = 0.0
    cos_term = 0.0
    for source, target in pairs:
        sx = source[0] - source_center[0]
        sy = source[1] - source_center[1]
        tx = target[0] - target_center[0]
        ty = target[1] - target_center[1]
        sin_term += sx * ty - sy * tx
        cos_term += sx * tx + sy * ty
    yaw = math.atan2(sin_term, cos_term)
    c = math.cos(yaw)
    s = math.sin(yaw)
    tx = target_center[0] - (c * source_center[0] - s * source_center[1])
    ty = target_center[1] - (s * source_center[0] + c * source_center[1])
    tz = target_center[2] - source_center[2]
    return yaw, (tx, ty, tz), len(pairs)


def transform_position(position, yaw, translation):
    c = math.cos(yaw)
    s = math.sin(yaw)
    x, y, z = position
    return (c * x - s * y + translation[0], s * x + c * y + translation[1], z + translation[2])


def remap_imu(msg):
    out = Imu()
    out.header = msg.header
    out.header.frame_id = "imu_link"
    out.orientation = msg.orientation
    out.orientation_covariance = msg.orientation_covariance
    out.angular_velocity = msg.angular_velocity
    out.angular_velocity_covariance = msg.angular_velocity_covariance
    out.linear_acceleration = msg.linear_acceleration
    out.linear_acceleration_covariance = msg.linear_acceleration_covariance
    return out


def remap_odom(msg, start_time=None, end_time=None, drift=(0.0, 0.0, 0.0)):
    msg.header.frame_id = "map"
    msg.child_frame_id = "base_link"
    if start_time is not None and end_time is not None and end_time > start_time:
        ratio = (msg.header.stamp.to_sec() - start_time) / (end_time - start_time)
        ratio = max(0.0, min(1.0, ratio))
        msg.pose.pose.position.x += drift[0] * ratio
        msg.pose.pose.position.y += drift[1] * ratio
        msg.pose.pose.position.z += drift[2] * ratio
    return msg


def aligned_ground_truth(msg, yaw, translation):
    out = Odometry()
    out.header = msg.header
    out.header.frame_id = "map"
    out.child_frame_id = "base_link"
    out.pose = msg.pose
    p = msg.pose.pose.position
    x, y, z = transform_position((p.x, p.y, p.z), yaw, translation)
    out.pose.pose.position.x = x
    out.pose.pose.position.y = y
    out.pose.pose.position.z = z
    out.twist = msg.twist
    return out


def rtk_raw_to_navsat(msg, covariance_xy, covariance_z):
    out = NavSatFix()
    out.header = msg.header
    out.header.frame_id = "gps_link"
    out.status = msg.status
    out.latitude = msg.gps.latitude
    out.longitude = msg.gps.longitude
    out.altitude = msg.gps.altitude
    out.position_covariance[0] = covariance_xy
    out.position_covariance[4] = covariance_xy
    out.position_covariance[8] = covariance_z
    out.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
    return out


def main():
    parser = argparse.ArgumentParser(description="Convert CTU MRS MAS processed bag to this EKF project's topic contract.")
    parser.add_argument("--input-bag", required=True)
    parser.add_argument("--output-bag", required=True)
    parser.add_argument("--imu-topic", default="/pixhawk_imu")
    parser.add_argument("--odom-topic", default="/gps_fused_odom")
    parser.add_argument("--rtk-topic", default="/rtk_raw")
    parser.add_argument("--ground-truth-topic", default="/rtk_fused_odom")
    parser.add_argument("--max-align-dt", type=float, default=0.08)
    parser.add_argument("--rtk-cov-xy", type=float, default=0.04)
    parser.add_argument("--rtk-cov-z", type=float, default=0.09)
    parser.add_argument("--odom-drift-x-final", type=float, default=0.0)
    parser.add_argument("--odom-drift-y-final", type=float, default=0.0)
    parser.add_argument("--odom-drift-z-final", type=float, default=0.0)
    args = parser.parse_args()

    odom_positions = collect_positions(args.input_bag, args.odom_topic)
    gt_positions = collect_positions(args.input_bag, args.ground_truth_topic)
    yaw, translation, pairs = estimate_yaw_translation(gt_positions, odom_positions, args.max_align_dt)
    odom_start = odom_positions[0][0] if odom_positions else None
    odom_end = odom_positions[-1][0] if odom_positions else None
    odom_drift = (args.odom_drift_x_final, args.odom_drift_y_final, args.odom_drift_z_final)

    output_dir = os.path.dirname(os.path.abspath(args.output_bag))
    if output_dir:
        os.makedirs(output_dir, exist_ok=True)

    topics = [args.imu_topic, args.odom_topic, args.rtk_topic, args.ground_truth_topic]
    written = {
        "/mavros/imu/data": 0,
        "/mavros/odometry/in": 0,
        "/mavros/global_position/global": 0,
        "/ground_truth/odom": 0,
    }
    with rosbag.Bag(args.output_bag, "w") as out_bag:
        with rosbag.Bag(args.input_bag) as in_bag:
            for topic, msg, _ in in_bag.read_messages(topics=topics):
                if topic == args.imu_topic:
                    out = remap_imu(msg)
                    out_bag.write("/mavros/imu/data", out, out.header.stamp)
                    written["/mavros/imu/data"] += 1
                elif topic == args.odom_topic:
                    out = remap_odom(msg, odom_start, odom_end, odom_drift)
                    out_bag.write("/mavros/odometry/in", out, out.header.stamp)
                    written["/mavros/odometry/in"] += 1
                elif topic == args.rtk_topic:
                    out = rtk_raw_to_navsat(msg, args.rtk_cov_xy, args.rtk_cov_z)
                    out_bag.write("/mavros/global_position/global", out, out.header.stamp)
                    written["/mavros/global_position/global"] += 1
                elif topic == args.ground_truth_topic:
                    out = aligned_ground_truth(msg, yaw, translation)
                    out_bag.write("/ground_truth/odom", out, out.header.stamp)
                    written["/ground_truth/odom"] += 1

    print("input_bag={}".format(args.input_bag))
    print("output_bag={}".format(args.output_bag))
    print("alignment_yaw_rad={:.6f}".format(yaw))
    print("alignment_translation={:.6f},{:.6f},{:.6f}".format(*translation))
    print("alignment_pairs={}".format(pairs))
    print("odom_drift_final={:.6f},{:.6f},{:.6f}".format(*odom_drift))
    for topic, count in sorted(written.items()):
        print("{} {}".format(topic, count))


if __name__ == "__main__":
    main()
