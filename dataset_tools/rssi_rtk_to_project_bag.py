#!/usr/bin/env python3
import argparse
import math
import os
from bisect import bisect_left

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
    return (
        (lon - lon0) * cos_lat0 * EARTH_RADIUS,
        (lat - lat0) * EARTH_RADIUS,
        alt - alt0,
    )


def read_rtk(path):
    rows = []
    with open(path) as rtk_file:
        for raw in rtk_file:
            parts = raw.strip().split(",")
            if len(parts) < 4:
                continue
            rows.append(
                {
                    "lat": float(parts[0]),
                    "lon": float(parts[1]),
                    "alt": float(parts[2]),
                    "stamp": float(parts[3]),
                }
            )
    return rows


def collect_odom_positions(bag_path, odom_topic):
    positions = []
    with rosbag.Bag(bag_path) as bag:
        for _, msg, _ in bag.read_messages(topics=[odom_topic]):
            p = msg.pose.pose.position
            positions.append((msg.header.stamp.to_sec(), (p.x, p.y, p.z)))
    return positions


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


def estimate_yaw_translation(rtk_positions, odom_positions, max_dt):
    pairs = []
    for stamp, rtk_pos in rtk_positions:
        odom = nearest(odom_positions, stamp, max_dt)
        if odom is not None:
            pairs.append((rtk_pos, odom[1]))
    if len(pairs) < 5:
        raise RuntimeError("Need at least 5 RTK/odom pairs for frame alignment")

    rtk_center = [sum(pair[0][i] for pair in pairs) / len(pairs) for i in range(3)]
    odom_center = [sum(pair[1][i] for pair in pairs) / len(pairs) for i in range(3)]
    sin_term = 0.0
    cos_term = 0.0
    for rtk, odom in pairs:
        rx = rtk[0] - rtk_center[0]
        ry = rtk[1] - rtk_center[1]
        ox = odom[0] - odom_center[0]
        oy = odom[1] - odom_center[1]
        sin_term += rx * oy - ry * ox
        cos_term += rx * ox + ry * oy
    yaw = math.atan2(sin_term, cos_term)
    c = math.cos(yaw)
    s = math.sin(yaw)
    tx = odom_center[0] - (c * rtk_center[0] - s * rtk_center[1])
    ty = odom_center[1] - (s * rtk_center[0] + c * rtk_center[1])
    tz = odom_center[2] - rtk_center[2]
    return yaw, (tx, ty, tz), len(pairs)


def transform_position(position, yaw, translation):
    c = math.cos(yaw)
    s = math.sin(yaw)
    x, y, z = position
    return (c * x - s * y + translation[0], s * x + c * y + translation[1], z + translation[2])


def make_ground_truth_msg(stamp, position, heading):
    q = quaternion_from_euler(0.0, 0.0, heading)
    msg = Odometry()
    msg.header.stamp = rospy.Time.from_sec(stamp)
    msg.header.frame_id = "map"
    msg.child_frame_id = "base_link"
    msg.pose.pose.position.x = position[0]
    msg.pose.pose.position.y = position[1]
    msg.pose.pose.position.z = position[2]
    msg.pose.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
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
    parser = argparse.ArgumentParser(description="Convert RSSI/RTK public localization bag to this EKF project's topic contract.")
    parser.add_argument("--input-bag", required=True)
    parser.add_argument("--rtk-log", required=True)
    parser.add_argument("--output-bag", required=True)
    parser.add_argument("--odom-topic", default="/RosAria/odom")
    parser.add_argument("--imu-topic", default="/imu/data")
    parser.add_argument("--fix-topic", default="/gps/fix")
    parser.add_argument("--max-align-dt", type=float, default=0.15)
    args = parser.parse_args()

    rtk_rows = read_rtk(args.rtk_log)
    if not rtk_rows:
        raise RuntimeError("No RTK rows found: {}".format(args.rtk_log))
    origin = (
        math.radians(rtk_rows[0]["lat"]),
        math.radians(rtk_rows[0]["lon"]),
        rtk_rows[0]["alt"],
        math.cos(math.radians(rtk_rows[0]["lat"])),
    )
    rtk_positions = [
        (row["stamp"], navsat_to_enu(row["lat"], row["lon"], row["alt"], origin))
        for row in rtk_rows
    ]
    odom_positions = collect_odom_positions(args.input_bag, args.odom_topic)
    yaw, translation, align_pairs = estimate_yaw_translation(rtk_positions, odom_positions, args.max_align_dt)

    transformed_rtk = []
    previous_position = None
    for stamp, position in rtk_positions:
        transformed = transform_position(position, yaw, translation)
        heading = yaw
        if previous_position is not None:
            dx = transformed[0] - previous_position[0]
            dy = transformed[1] - previous_position[1]
            if math.hypot(dx, dy) > 0.02:
                heading = math.atan2(dy, dx)
        transformed_rtk.append((stamp, transformed, heading))
        previous_position = transformed

    output_dir = os.path.dirname(os.path.abspath(args.output_bag))
    if output_dir:
        os.makedirs(output_dir, exist_ok=True)

    written = {"/mavros/imu/data": 0, "/mavros/odometry/in": 0, "/mavros/global_position/global": 0, "/ground_truth/odom": 0}
    with rosbag.Bag(args.output_bag, "w") as out_bag:
        for stamp, position, heading in transformed_rtk:
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
    print("rtk_log={}".format(args.rtk_log))
    print("output_bag={}".format(args.output_bag))
    print("alignment_yaw_rad={:.6f}".format(yaw))
    print("alignment_translation={:.6f},{:.6f},{:.6f}".format(*translation))
    print("alignment_pairs={}".format(align_pairs))
    for topic, count in sorted(written.items()):
        print("{} {}".format(topic, count))


if __name__ == "__main__":
    main()
