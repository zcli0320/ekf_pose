#!/usr/bin/env python3
import argparse
import copy
import math
import os

import rosbag
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix


EARTH_RADIUS = 6378137.0


def in_window(rel_time, start, duration):
    if start is None or duration <= 0.0:
        return False
    return start <= rel_time <= start + duration


def offset_navsat(msg, east_m, north_m, up_m):
    out = copy.deepcopy(msg)
    lat_rad = math.radians(out.latitude)
    cos_lat = max(1.0e-6, math.cos(lat_rad))
    out.latitude += math.degrees(north_m / EARTH_RADIUS)
    out.longitude += math.degrees(east_m / (EARTH_RADIUS * cos_lat))
    out.altitude += up_m
    return out


def scale_navsat_covariance(msg, scale):
    out = copy.deepcopy(msg)
    if out.position_covariance_type != NavSatFix.COVARIANCE_TYPE_UNKNOWN:
        out.position_covariance = [value * scale for value in out.position_covariance]
    return out


def offset_odom(msg, x, y, z):
    out = copy.deepcopy(msg)
    out.pose.pose.position.x += x
    out.pose.pose.position.y += y
    out.pose.pose.position.z += z
    return out


def drift_odom(msg, rel_time, start, duration, x, y, z):
    if not in_window(rel_time, start, duration):
        return msg, False
    ratio = (rel_time - start) / max(1.0e-6, duration)
    return offset_odom(msg, ratio * x, ratio * y, ratio * z), True


def make_ground_truth(msg, topic):
    out = copy.deepcopy(msg)
    out.header.frame_id = msg.header.frame_id or "map"
    out.child_frame_id = msg.child_frame_id or "base_link"
    return topic, out


def main():
    parser = argparse.ArgumentParser(description="Inject repeatable GNSS/odom anomalies into a project-format rosbag.")
    parser.add_argument("--input-bag", required=True)
    parser.add_argument("--output-bag", required=True)
    parser.add_argument("--gnss-topic", default="/mavros/global_position/global")
    parser.add_argument("--odom-topic", default="/mavros/odometry/in")
    parser.add_argument("--gnss-drop-start", type=float)
    parser.add_argument("--gnss-drop-duration", type=float, default=0.0)
    parser.add_argument("--gnss-jump-start", type=float)
    parser.add_argument("--gnss-jump-duration", type=float, default=0.0)
    parser.add_argument("--gnss-jump-east", type=float, default=0.0)
    parser.add_argument("--gnss-jump-north", type=float, default=0.0)
    parser.add_argument("--gnss-jump-up", type=float, default=0.0)
    parser.add_argument("--gnss-cov-scale-start", type=float)
    parser.add_argument("--gnss-cov-scale-duration", type=float, default=0.0)
    parser.add_argument("--gnss-cov-scale", type=float, default=1.0)
    parser.add_argument("--odom-drop-start", type=float)
    parser.add_argument("--odom-drop-duration", type=float, default=0.0)
    parser.add_argument("--odom-jump-start", type=float)
    parser.add_argument("--odom-jump-duration", type=float, default=0.0)
    parser.add_argument("--odom-jump-x", type=float, default=0.0)
    parser.add_argument("--odom-jump-y", type=float, default=0.0)
    parser.add_argument("--odom-jump-z", type=float, default=0.0)
    parser.add_argument("--odom-drift-start", type=float)
    parser.add_argument("--odom-drift-duration", type=float, default=0.0)
    parser.add_argument("--odom-drift-x", type=float, default=0.0)
    parser.add_argument("--odom-drift-y", type=float, default=0.0)
    parser.add_argument("--odom-drift-z", type=float, default=0.0)
    parser.add_argument("--ground-truth-from-odom", action="store_true")
    parser.add_argument("--ground-truth-topic", default="/ground_truth/odom")
    args = parser.parse_args()

    if not os.path.exists(args.input_bag):
        raise SystemExit("input bag not found: {}".format(args.input_bag))
    output_dir = os.path.dirname(os.path.abspath(args.output_bag))
    if output_dir:
        os.makedirs(output_dir, exist_ok=True)

    first_time = None
    counts = {
        "gnss_dropped": 0,
        "gnss_jumped": 0,
        "gnss_cov_scaled": 0,
        "odom_dropped": 0,
        "odom_jumped": 0,
        "odom_drifted": 0,
        "ground_truth_written": 0,
    }

    with rosbag.Bag(args.input_bag, "r") as src, rosbag.Bag(args.output_bag, "w") as dst:
        for topic, msg, stamp in src.read_messages():
            if first_time is None:
                first_time = stamp.to_sec()
            rel_time = stamp.to_sec() - first_time

            if topic == args.gnss_topic:
                if in_window(rel_time, args.gnss_drop_start, args.gnss_drop_duration):
                    counts["gnss_dropped"] += 1
                    continue
                out = msg
                if in_window(rel_time, args.gnss_jump_start, args.gnss_jump_duration):
                    out = offset_navsat(out, args.gnss_jump_east, args.gnss_jump_north, args.gnss_jump_up)
                    counts["gnss_jumped"] += 1
                if in_window(rel_time, args.gnss_cov_scale_start, args.gnss_cov_scale_duration):
                    out = scale_navsat_covariance(out, args.gnss_cov_scale)
                    counts["gnss_cov_scaled"] += 1
                dst.write(topic, out, stamp)
                continue

            if topic == args.odom_topic:
                if args.ground_truth_from_odom:
                    gt_topic, gt_msg = make_ground_truth(msg, args.ground_truth_topic)
                    dst.write(gt_topic, gt_msg, stamp)
                    counts["ground_truth_written"] += 1
                if in_window(rel_time, args.odom_drop_start, args.odom_drop_duration):
                    counts["odom_dropped"] += 1
                    continue
                out = msg
                if in_window(rel_time, args.odom_jump_start, args.odom_jump_duration):
                    out = offset_odom(out, args.odom_jump_x, args.odom_jump_y, args.odom_jump_z)
                    counts["odom_jumped"] += 1
                out, drifted = drift_odom(out,
                                          rel_time,
                                          args.odom_drift_start,
                                          args.odom_drift_duration,
                                          args.odom_drift_x,
                                          args.odom_drift_y,
                                          args.odom_drift_z)
                if drifted:
                    counts["odom_drifted"] += 1
                dst.write(topic, out, stamp)
                continue

            dst.write(topic, msg, stamp)

    print("wrote {}".format(args.output_bag))
    for key in sorted(counts):
        print("{}={}".format(key, counts[key]))


if __name__ == "__main__":
    main()
