#!/usr/bin/env python3
import argparse
import copy
import math
import os
from bisect import bisect_left

import rosbag
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
    if len(pairs) < 5:
        raise RuntimeError("Need at least 5 paired samples for alignment")

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


def estimate_translation_only(source_positions, target_positions, max_dt):
    for stamp, source in source_positions:
        target = nearest(target_positions, stamp, max_dt)
        if target is None:
            continue
        translation = (
            target[1][0] - source[0],
            target[1][1] - source[1],
            target[1][2] - source[2],
        )
        return 0.0, translation, 1
    raise RuntimeError("Need at least one paired sample for translation alignment")


def horizontal_motion_span(positions):
    if len(positions) < 2:
        return 0.0
    x0, y0, _ = positions[0][1]
    max_span = 0.0
    for _, position in positions:
        dx = position[0] - x0
        dy = position[1] - y0
        max_span = max(max_span, math.sqrt(dx * dx + dy * dy))
    return max_span


def transform_position(position, yaw, translation):
    c = math.cos(yaw)
    s = math.sin(yaw)
    x, y, z = position
    return (c * x - s * y + translation[0], s * x + c * y + translation[1], z + translation[2])


def remap_imu(msg):
    out = Imu()
    out.header = copy.deepcopy(msg.header)
    out.header.frame_id = "imu_link"
    out.orientation = copy.deepcopy(msg.orientation)
    out.orientation_covariance = list(msg.orientation_covariance)
    out.angular_velocity = copy.deepcopy(msg.angular_velocity)
    out.angular_velocity_covariance = list(msg.angular_velocity_covariance)
    out.linear_acceleration = copy.deepcopy(msg.linear_acceleration)
    out.linear_acceleration_covariance = list(msg.linear_acceleration_covariance)
    return out


def remap_odom(msg, start_time=None, end_time=None, drift=(0.0, 0.0, 0.0), cov_floor=0.0):
    out = copy.deepcopy(msg)
    out.header.frame_id = "map"
    out.child_frame_id = "base_link"
    if start_time is not None and end_time is not None and end_time > start_time:
        ratio = (out.header.stamp.to_sec() - start_time) / (end_time - start_time)
        ratio = max(0.0, min(1.0, ratio))
        out.pose.pose.position.x += drift[0] * ratio
        out.pose.pose.position.y += drift[1] * ratio
        out.pose.pose.position.z += drift[2] * ratio
    if cov_floor > 0.0:
        covariance = list(out.pose.covariance)
        for index in (0, 7, 14):
            covariance[index] = max(covariance[index], cov_floor)
        out.pose.covariance = covariance
    return out


def aligned_ground_truth(msg, yaw, translation):
    out = copy.deepcopy(msg)
    out.header.frame_id = "map"
    out.child_frame_id = "base_link"
    p = out.pose.pose.position
    x, y, z = transform_position((p.x, p.y, p.z), yaw, translation)
    out.pose.pose.position.x = x
    out.pose.pose.position.y = y
    out.pose.pose.position.z = z
    return out


def remap_navsat(msg, cov_xy=None, cov_z=None):
    out = copy.deepcopy(msg)
    out.header.frame_id = "gps_link"
    covariance = list(out.position_covariance)
    if cov_xy is not None:
        covariance[0] = cov_xy
        covariance[4] = cov_xy
    if cov_z is not None:
        covariance[8] = cov_z
    out.position_covariance = covariance
    if cov_xy is not None or cov_z is not None:
        out.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
    return out


def in_time_window(stamp, start_time, end_time):
    if start_time is not None and stamp < start_time:
        return False
    if end_time is not None and stamp > end_time:
        return False
    return True


def main():
    parser = argparse.ArgumentParser(description="Convert KARI drone bag to this EKF project's topic contract.")
    parser.add_argument("--input-bag", required=True)
    parser.add_argument("--output-bag", required=True)
    parser.add_argument("--imu-topic", default="/camera/imu")
    parser.add_argument("--odom-topic", default="/firefly_sbx/vio/odom")
    parser.add_argument("--gnss-topic", default="/mavros/global_position/global")
    parser.add_argument("--ground-truth-topic", default="/mavros/global_position/local")
    parser.add_argument("--max-align-dt", type=float, default=0.25)
    parser.add_argument("--alignment-mode", choices=["auto", "yaw", "translation"], default="auto")
    parser.add_argument("--min-yaw-motion", type=float, default=2.0)
    parser.add_argument("--crop-to-odom", action="store_true")
    parser.add_argument("--gnss-cov-xy", type=float)
    parser.add_argument("--gnss-cov-z", type=float)
    parser.add_argument("--odom-cov-floor", type=float, default=0.0025)
    parser.add_argument("--odom-drift-x-final", type=float, default=0.0)
    parser.add_argument("--odom-drift-y-final", type=float, default=0.0)
    parser.add_argument("--odom-drift-z-final", type=float, default=0.0)
    args = parser.parse_args()

    odom_positions = collect_positions(args.input_bag, args.odom_topic)
    gt_positions = collect_positions(args.input_bag, args.ground_truth_topic)
    if not odom_positions:
        raise SystemExit("no odom samples found on {}".format(args.odom_topic))
    if not gt_positions:
        raise SystemExit("no ground truth samples found on {}".format(args.ground_truth_topic))

    odom_motion = horizontal_motion_span(odom_positions)
    gt_motion = horizontal_motion_span(gt_positions)
    alignment_mode = args.alignment_mode
    if alignment_mode == "auto":
        if odom_motion >= args.min_yaw_motion and gt_motion >= args.min_yaw_motion:
            alignment_mode = "yaw"
        else:
            alignment_mode = "translation"
    if alignment_mode == "yaw":
        yaw, translation, pairs = estimate_yaw_translation(gt_positions, odom_positions, args.max_align_dt)
    else:
        yaw, translation, pairs = estimate_translation_only(gt_positions, odom_positions, args.max_align_dt)
    odom_start = odom_positions[0][0]
    odom_end = odom_positions[-1][0]
    crop_start = odom_start if args.crop_to_odom else None
    crop_end = odom_end if args.crop_to_odom else None
    odom_drift = (args.odom_drift_x_final, args.odom_drift_y_final, args.odom_drift_z_final)

    output_dir = os.path.dirname(os.path.abspath(args.output_bag))
    if output_dir:
        os.makedirs(output_dir, exist_ok=True)

    topics = [args.imu_topic, args.odom_topic, args.gnss_topic, args.ground_truth_topic]
    written = {
        "/mavros/imu/data": 0,
        "/mavros/odometry/in": 0,
        "/mavros/global_position/global": 0,
        "/ground_truth/odom": 0,
    }
    with rosbag.Bag(args.output_bag, "w") as out_bag:
        with rosbag.Bag(args.input_bag) as in_bag:
            for topic, msg, _ in in_bag.read_messages(topics=topics):
                stamp = msg.header.stamp.to_sec()
                if not in_time_window(stamp, crop_start, crop_end):
                    continue
                if topic == args.imu_topic:
                    out = remap_imu(msg)
                    out_bag.write("/mavros/imu/data", out, out.header.stamp)
                    written["/mavros/imu/data"] += 1
                elif topic == args.odom_topic:
                    out = remap_odom(msg, odom_start, odom_end, odom_drift, args.odom_cov_floor)
                    out_bag.write("/mavros/odometry/in", out, out.header.stamp)
                    written["/mavros/odometry/in"] += 1
                elif topic == args.gnss_topic:
                    out = remap_navsat(msg, args.gnss_cov_xy, args.gnss_cov_z)
                    out_bag.write("/mavros/global_position/global", out, out.header.stamp)
                    written["/mavros/global_position/global"] += 1
                elif topic == args.ground_truth_topic:
                    out = aligned_ground_truth(msg, yaw, translation)
                    out_bag.write("/ground_truth/odom", out, out.header.stamp)
                    written["/ground_truth/odom"] += 1

    print("input_bag={}".format(args.input_bag))
    print("output_bag={}".format(args.output_bag))
    print("odom_topic={}".format(args.odom_topic))
    print("alignment_mode={}".format(alignment_mode))
    print("alignment_yaw_rad={:.6f}".format(yaw))
    print("alignment_translation={:.6f},{:.6f},{:.6f}".format(*translation))
    print("alignment_pairs={}".format(pairs))
    print("horizontal_motion_odom={:.6f}".format(odom_motion))
    print("horizontal_motion_ground_truth={:.6f}".format(gt_motion))
    print("crop_to_odom={} start={:.6f} end={:.6f}".format(args.crop_to_odom, odom_start, odom_end))
    print("odom_drift_final={:.6f},{:.6f},{:.6f}".format(*odom_drift))
    for topic, count in sorted(written.items()):
        print("{} {}".format(topic, count))


if __name__ == "__main__":
    main()
