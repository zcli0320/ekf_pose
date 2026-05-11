#!/usr/bin/env python3
import argparse
import copy
import math
import os
from bisect import bisect_left

import rosbag
import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix
from tf.transformations import quaternion_from_euler


EARTH_RADIUS = 6378137.0


def navsat_to_enu(msg, origin):
    lat = math.radians(msg.latitude)
    lon = math.radians(msg.longitude)
    lat0, lon0, alt0, cos_lat0 = origin
    x = (lon - lon0) * cos_lat0 * EARTH_RADIUS
    y = (lat - lat0) * EARTH_RADIUS
    z = msg.altitude - alt0
    return x, y, z


def collect_fixes(bag_path, fix_topic):
    fixes = []
    origin = None
    with rosbag.Bag(bag_path) as bag:
        for _, msg, _ in bag.read_messages(topics=[fix_topic]):
            if msg.status.status < 0:
                continue
            if not all(math.isfinite(v) for v in (msg.latitude, msg.longitude, msg.altitude)):
                continue
            if origin is None:
                lat0 = math.radians(msg.latitude)
                lon0 = math.radians(msg.longitude)
                origin = (lat0, lon0, msg.altitude, math.cos(lat0))
            fixes.append((msg.header.stamp.to_sec(),) + navsat_to_enu(msg, origin))
    if len(fixes) < 2:
        raise RuntimeError("Need at least two valid GNSS fixes")
    return fixes


def interpolate_position(samples, stamp):
    times = [item[0] for item in samples]
    index = bisect_left(times, stamp)
    if index <= 0:
        return samples[0][1:4]
    if index >= len(samples):
        return samples[-1][1:4]
    before = samples[index - 1]
    after = samples[index]
    dt = after[0] - before[0]
    if dt <= 0.0:
        return before[1:4]
    ratio = (stamp - before[0]) / dt
    return tuple(before[i] + ratio * (after[i] - before[i]) for i in range(1, 4))


def heading_from_window(samples, stamp, window, fallback_yaw):
    p0 = interpolate_position(samples, stamp - window)
    p1 = interpolate_position(samples, stamp + window)
    dx = p1[0] - p0[0]
    dy = p1[1] - p0[1]
    if math.hypot(dx, dy) < 0.25:
        return fallback_yaw
    return math.atan2(dy, dx)


def make_odom(stamp, position, yaw, frame_id, child_frame_id, cov_xy, cov_z):
    msg = Odometry()
    msg.header.stamp = rospy.Time.from_sec(stamp)
    msg.header.frame_id = frame_id
    msg.child_frame_id = child_frame_id
    msg.pose.pose.position.x = position[0]
    msg.pose.pose.position.y = position[1]
    msg.pose.pose.position.z = position[2]
    q = quaternion_from_euler(0.0, 0.0, yaw)
    msg.pose.pose.orientation.x = q[0]
    msg.pose.pose.orientation.y = q[1]
    msg.pose.pose.orientation.z = q[2]
    msg.pose.pose.orientation.w = q[3]
    msg.pose.covariance[0] = cov_xy
    msg.pose.covariance[7] = cov_xy
    msg.pose.covariance[14] = cov_z
    msg.pose.covariance[21] = 0.25
    msg.pose.covariance[28] = 0.25
    msg.pose.covariance[35] = 0.25
    return msg


def main():
    parser = argparse.ArgumentParser(
        description="Generate a project bag with a GNSS-smoothed local odometry stream."
    )
    parser.add_argument("--input-bag", required=True)
    parser.add_argument("--output-bag", required=True)
    parser.add_argument("--imu-topic", default="/mavros/imu/data")
    parser.add_argument("--fix-topic", default="/mavros/global_position/global")
    parser.add_argument("--odom-topic", default="/mavros/odometry/in")
    parser.add_argument("--raw-odom-topic", default="/raw/floam_odom")
    parser.add_argument("--output-odom-topic", default="/mavros/odometry/in")
    parser.add_argument("--diagnostic-odom-topic", default="/gnss_smoothed/odom")
    parser.add_argument("--frame-id", default="map")
    parser.add_argument("--child-frame-id", default="base_link")
    parser.add_argument("--heading-window", type=float, default=1.0)
    parser.add_argument("--cov-xy", type=float, default=1.0)
    parser.add_argument("--cov-z", type=float, default=4.0)
    args = parser.parse_args()

    fixes = collect_fixes(args.input_bag, args.fix_topic)
    output_dir = os.path.dirname(os.path.abspath(args.output_bag))
    if output_dir:
        os.makedirs(output_dir, exist_ok=True)

    written = {
        args.imu_topic: 0,
        args.fix_topic: 0,
        args.output_odom_topic: 0,
        args.diagnostic_odom_topic: 0,
        args.raw_odom_topic: 0,
    }
    last_yaw = 0.0
    with rosbag.Bag(args.output_bag, "w") as out_bag:
        with rosbag.Bag(args.input_bag) as in_bag:
            topics = [args.imu_topic, args.fix_topic, args.odom_topic]
            for topic, msg, bag_time in in_bag.read_messages(topics=topics):
                if topic == args.imu_topic:
                    out = copy.deepcopy(msg)
                    out_bag.write(args.imu_topic, out, out.header.stamp)
                    written[args.imu_topic] += 1
                elif topic == args.fix_topic:
                    out = copy.deepcopy(msg)
                    out_bag.write(args.fix_topic, out, out.header.stamp)
                    written[args.fix_topic] += 1
                elif topic == args.odom_topic:
                    stamp = msg.header.stamp.to_sec()
                    position = interpolate_position(fixes, stamp)
                    last_yaw = heading_from_window(fixes, stamp, args.heading_window, last_yaw)
                    odom = make_odom(
                        stamp,
                        position,
                        last_yaw,
                        args.frame_id,
                        args.child_frame_id,
                        args.cov_xy,
                        args.cov_z,
                    )
                    out_bag.write(args.output_odom_topic, odom, odom.header.stamp)
                    out_bag.write(args.diagnostic_odom_topic, odom, odom.header.stamp)
                    written[args.output_odom_topic] += 1
                    written[args.diagnostic_odom_topic] += 1

                    raw = copy.deepcopy(msg)
                    raw.header.frame_id = args.frame_id
                    raw.child_frame_id = args.child_frame_id
                    out_bag.write(args.raw_odom_topic, raw, raw.header.stamp or bag_time)
                    written[args.raw_odom_topic] += 1

    print("input_bag={}".format(args.input_bag))
    print("output_bag={}".format(args.output_bag))
    for topic, count in sorted(written.items()):
        print("{} {}".format(topic, count))


if __name__ == "__main__":
    main()
