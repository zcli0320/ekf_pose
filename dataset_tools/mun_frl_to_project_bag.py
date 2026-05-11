#!/usr/bin/env python3
import argparse
import csv
import math
import os
from bisect import bisect_left
from datetime import datetime, timezone

import rosbag
import rospy
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix
from tf.transformations import quaternion_from_euler


EARTH_RADIUS = 6378137.0


def navsat_to_enu(lat_deg, lon_deg, alt, origin):
    lat = math.radians(lat_deg)
    lon = math.radians(lon_deg)
    lat0, lon0, alt0, cos_lat0 = origin
    x = (lon - lon0) * cos_lat0 * EARTH_RADIUS
    y = (lat - lat0) * EARTH_RADIUS
    z = alt - alt0
    return (x, y, z)


def parse_ppk(path):
    rows = []
    with open(path, newline="") as ppk_file:
        for raw in ppk_file:
            line = raw.strip()
            if not line or line.startswith("%"):
                continue
            parts = line.split()
            if len(parts) < 15:
                continue
            stamp = datetime.strptime(parts[0] + " " + parts[1], "%Y/%m/%d %H:%M:%S.%f")
            epoch = stamp.replace(tzinfo=timezone.utc).timestamp()
            rows.append(
                {
                    "epoch": epoch,
                    "lat": float(parts[2]),
                    "lon": float(parts[3]),
                    "alt": float(parts[4]),
                    "quality": int(parts[5]),
                    "sdn": float(parts[7]),
                    "sde": float(parts[8]),
                    "sdu": float(parts[9]),
                }
            )
    return rows


def collect_odom_positions(bag_path, odom_topic, max_samples):
    positions = []
    with rosbag.Bag(bag_path) as bag:
        for _, msg, _ in bag.read_messages(topics=[odom_topic]):
            p = msg.pose.pose.position
            positions.append((msg.header.stamp.to_sec(), (p.x, p.y, p.z)))
            if max_samples and len(positions) >= max_samples:
                break
    return positions


def collect_fix_positions(bag_path, fix_topic):
    fixes = []
    with rosbag.Bag(bag_path) as bag:
        for _, msg, _ in bag.read_messages(topics=[fix_topic]):
            if msg.status.status < 0:
                continue
            if not all(math.isfinite(v) for v in (msg.latitude, msg.longitude, msg.altitude)):
                continue
            fixes.append((msg.header.stamp.to_sec(), msg.latitude, msg.longitude, msg.altitude))
    return fixes


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


def estimate_time_offset(fixes, ppk_rows):
    if not fixes or not ppk_rows:
        raise RuntimeError("Cannot estimate time offset without both raw fix and PPK samples")
    ppk_origin = (math.radians(ppk_rows[0]["lat"]), math.radians(ppk_rows[0]["lon"]), ppk_rows[0]["alt"], math.cos(math.radians(ppk_rows[0]["lat"])))
    ppk_enu = [(row["epoch"], navsat_to_enu(row["lat"], row["lon"], row["alt"], ppk_origin)) for row in ppk_rows]
    ppk_times = [item[0] for item in ppk_enu]
    first_fix = fixes[0]
    raw_origin = (math.radians(first_fix[1]), math.radians(first_fix[2]), first_fix[3], math.cos(math.radians(first_fix[1])))
    raw_enu = [(stamp, navsat_to_enu(lat, lon, alt, raw_origin)) for stamp, lat, lon, alt in fixes[:200]]

    candidate = raw_enu[0][0] - ppk_enu[0][0]
    best_offset = candidate
    best_error = float("inf")
    for delta in [candidate + step * 0.2 for step in range(-25, 26)]:
        errors = []
        for raw_stamp, raw_pos in raw_enu:
            ppk_stamp = raw_stamp - delta
            idx = bisect_left(ppk_times, ppk_stamp)
            if idx <= 0 or idx >= len(ppk_enu):
                continue
            ppk_pos = ppk_enu[idx][1]
            errors.append(math.hypot(raw_pos[0] - ppk_pos[0], raw_pos[1] - ppk_pos[1]))
        if errors:
            mean_error = sum(errors) / len(errors)
            if mean_error < best_error:
                best_error = mean_error
                best_offset = delta
    return best_offset


def estimate_yaw_translation(ppk_positions, odom_positions, max_dt):
    pairs = []
    for stamp, ppk_pos in ppk_positions:
        odom = nearest(odom_positions, stamp, max_dt)
        if odom is not None:
            pairs.append((ppk_pos, odom[1]))
    if len(pairs) < 5:
        raise RuntimeError("Need at least 5 PPK/odom pairs for frame alignment")

    ppk_center = [sum(pair[0][i] for pair in pairs) / len(pairs) for i in range(3)]
    odom_center = [sum(pair[1][i] for pair in pairs) / len(pairs) for i in range(3)]
    sin_term = 0.0
    cos_term = 0.0
    for ppk, odom in pairs:
        px = ppk[0] - ppk_center[0]
        py = ppk[1] - ppk_center[1]
        ox = odom[0] - odom_center[0]
        oy = odom[1] - odom_center[1]
        sin_term += px * oy - py * ox
        cos_term += px * ox + py * oy
    yaw = math.atan2(sin_term, cos_term)
    c = math.cos(yaw)
    s = math.sin(yaw)
    tx = odom_center[0] - (c * ppk_center[0] - s * ppk_center[1])
    ty = odom_center[1] - (s * ppk_center[0] + c * ppk_center[1])
    tz = odom_center[2] - ppk_center[2]
    return yaw, (tx, ty, tz), len(pairs)


def transform_position(position, yaw, translation):
    c = math.cos(yaw)
    s = math.sin(yaw)
    x, y, z = position
    return (c * x - s * y + translation[0], s * x + c * y + translation[1], z + translation[2])


def yaw_quaternion(yaw):
    q = quaternion_from_euler(0.0, 0.0, yaw)
    return Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])


def make_ground_truth_msg(stamp, position, yaw):
    msg = Odometry()
    msg.header.stamp = rospy.Time.from_sec(stamp)
    msg.header.frame_id = "map"
    msg.child_frame_id = "base_link"
    msg.pose.pose.position.x = position[0]
    msg.pose.pose.position.y = position[1]
    msg.pose.pose.position.z = position[2]
    msg.pose.pose.orientation = yaw_quaternion(yaw)
    msg.pose.covariance[0] = 0.01
    msg.pose.covariance[7] = 0.01
    msg.pose.covariance[14] = 0.04
    msg.pose.covariance[21] = 0.25
    msg.pose.covariance[28] = 0.25
    msg.pose.covariance[35] = 0.25
    return msg


def remap_odom(msg):
    msg.header.frame_id = "map"
    msg.child_frame_id = "base_link"
    return msg


def remap_fix(msg):
    msg.header.frame_id = "gps_link"
    if msg.position_covariance_type == NavSatFix.COVARIANCE_TYPE_UNKNOWN:
        msg.position_covariance[0] = 4.0
        msg.position_covariance[4] = 4.0
        msg.position_covariance[8] = 9.0
        msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
    return msg


def main():
    parser = argparse.ArgumentParser(description="Convert MUN-FRL Lighthouse sample to this EKF project's topic contract.")
    parser.add_argument("--input-bag", required=True)
    parser.add_argument("--ppk", required=True)
    parser.add_argument("--output-bag", required=True)
    parser.add_argument("--odom-topic", default="/Odometry")
    parser.add_argument("--fix-topic", default="/fix")
    parser.add_argument("--imu-topic", default="/imu/data")
    parser.add_argument("--max-align-dt", type=float, default=0.12)
    args = parser.parse_args()

    ppk_rows = parse_ppk(args.ppk)
    fixes = collect_fix_positions(args.input_bag, args.fix_topic)
    odom_positions = collect_odom_positions(args.input_bag, args.odom_topic, max_samples=0)
    time_offset = estimate_time_offset(fixes, ppk_rows)

    ppk_origin = (
        math.radians(ppk_rows[0]["lat"]),
        math.radians(ppk_rows[0]["lon"]),
        ppk_rows[0]["alt"],
        math.cos(math.radians(ppk_rows[0]["lat"])),
    )
    ppk_positions = []
    for row in ppk_rows:
        if row["quality"] > 2:
            continue
        stamp = row["epoch"] + time_offset
        ppk_positions.append((stamp, navsat_to_enu(row["lat"], row["lon"], row["alt"], ppk_origin)))

    yaw, translation, align_pairs = estimate_yaw_translation(ppk_positions, odom_positions, args.max_align_dt)
    transformed_ppk = []
    previous_position = None
    previous_stamp = None
    for stamp, position in ppk_positions:
        transformed = transform_position(position, yaw, translation)
        heading = yaw
        if previous_position is not None and previous_stamp is not None:
            dx = transformed[0] - previous_position[0]
            dy = transformed[1] - previous_position[1]
            if math.hypot(dx, dy) > 0.05:
                heading = math.atan2(dy, dx)
        transformed_ppk.append((stamp, transformed, heading))
        previous_stamp = stamp
        previous_position = transformed

    output_dir = os.path.dirname(os.path.abspath(args.output_bag))
    if output_dir:
        os.makedirs(output_dir, exist_ok=True)

    written = {"/mavros/imu/data": 0, "/mavros/odometry/in": 0, "/mavros/global_position/global": 0, "/ground_truth/odom": 0}
    with rosbag.Bag(args.output_bag, "w") as out_bag:
        for stamp, position, heading in transformed_ppk:
            msg = make_ground_truth_msg(stamp, position, heading)
            out_bag.write("/ground_truth/odom", msg, msg.header.stamp)
            written["/ground_truth/odom"] += 1

        with rosbag.Bag(args.input_bag) as in_bag:
            for topic, msg, _ in in_bag.read_messages(topics=[args.imu_topic, args.odom_topic, args.fix_topic]):
                if topic == args.imu_topic:
                    out_topic = "/mavros/imu/data"
                    msg.header.frame_id = "imu_link"
                elif topic == args.odom_topic:
                    out_topic = "/mavros/odometry/in"
                    msg = remap_odom(msg)
                else:
                    out_topic = "/mavros/global_position/global"
                    msg = remap_fix(msg)
                out_bag.write(out_topic, msg, msg.header.stamp)
                written[out_topic] += 1

    print("input_bag={}".format(args.input_bag))
    print("ppk={}".format(args.ppk))
    print("output_bag={}".format(args.output_bag))
    print("time_offset_seconds={:.6f}".format(time_offset))
    print("alignment_yaw_rad={:.6f}".format(yaw))
    print("alignment_translation={:.6f},{:.6f},{:.6f}".format(*translation))
    print("alignment_pairs={}".format(align_pairs))
    for topic, count in sorted(written.items()):
        print("{} {}".format(topic, count))


if __name__ == "__main__":
    main()
