#!/usr/bin/env python3
"""Publish globally aligned odom and GNSS paths from a bag for RViz checks."""

import argparse
import bisect
import math
import statistics

import numpy as np
import rosbag
import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path


EARTH_RADIUS_M = 6378137.0


def navsat_to_local_enu(msg, origin):
    lat = math.radians(msg.latitude)
    lon = math.radians(msg.longitude)
    if origin is None:
        origin = (lat, lon, msg.altitude, math.cos(lat))
    lat0, lon0, alt0, cos_lat0 = origin
    enu = np.array(
        [
            (lon - lon0) * cos_lat0 * EARTH_RADIUS_M,
            (lat - lat0) * EARTH_RADIUS_M,
            msg.altitude - alt0,
        ],
        dtype=float,
    )
    return enu, origin


def read_bag_pairs(bag_path, odom_topic, gnss_topic, max_dt):
    odom = []
    gnss = []
    origin = None
    with rosbag.Bag(bag_path) as bag:
        for topic, msg, _ in bag.read_messages(topics=[odom_topic, gnss_topic]):
            if topic == odom_topic:
                p = msg.pose.pose.position
                q = msg.pose.pose.orientation
                odom.append(
                    (
                        msg.header.stamp.to_sec(),
                        np.array([p.x, p.y, p.z], dtype=float),
                        (q.x, q.y, q.z, q.w),
                    )
                )
            elif topic == gnss_topic:
                if not all(math.isfinite(v) for v in (msg.latitude, msg.longitude, msg.altitude)):
                    continue
                enu, origin = navsat_to_local_enu(msg, origin)
                gnss.append((msg.header.stamp.to_sec(), enu))

    odom_times = [item[0] for item in odom]

    def nearest_odom(stamp):
        idx = bisect.bisect_left(odom_times, stamp)
        candidates = []
        if idx < len(odom):
            candidates.append(odom[idx])
        if idx > 0:
            candidates.append(odom[idx - 1])
        if not candidates:
            return None
        best = min(candidates, key=lambda item: abs(item[0] - stamp))
        if abs(best[0] - stamp) > max_dt:
            return None
        return best

    pairs = []
    for stamp, enu in gnss:
        odom_sample = nearest_odom(stamp)
        if odom_sample is not None:
            pairs.append((stamp, enu, odom_sample[1]))
    return odom, gnss, pairs


def fit_yaw_translation(pairs):
    gnss_xy = np.array([pair[1][:2] for pair in pairs])
    odom_xy = np.array([pair[2][:2] for pair in pairs])
    gnss_center = gnss_xy.mean(axis=0)
    odom_center = odom_xy.mean(axis=0)
    gnss_centered = gnss_xy - gnss_center
    odom_centered = odom_xy - odom_center
    sin_term = np.sum(gnss_centered[:, 0] * odom_centered[:, 1] - gnss_centered[:, 1] * odom_centered[:, 0])
    cos_term = np.sum(gnss_centered[:, 0] * odom_centered[:, 0] + gnss_centered[:, 1] * odom_centered[:, 1])
    yaw = math.atan2(sin_term, cos_term)
    rot = np.array([[math.cos(yaw), -math.sin(yaw)], [math.sin(yaw), math.cos(yaw)]])
    offset_xy = odom_center - rot @ gnss_center
    offset_z = statistics.mean(pair[2][2] - pair[1][2] for pair in pairs)
    return yaw, offset_xy, offset_z


def transform_gnss(enu, yaw, offset_xy, offset_z):
    rot = np.array([[math.cos(yaw), -math.sin(yaw)], [math.sin(yaw), math.cos(yaw)]])
    xy = rot @ enu[:2] + offset_xy
    return np.array([xy[0], xy[1], enu[2] + offset_z], dtype=float)


def make_path(frame_id, points, orientation=None):
    path = Path()
    path.header.frame_id = frame_id
    path.header.stamp = rospy.Time.now()
    for stamp, point in points:
        pose = PoseStamped()
        pose.header.frame_id = frame_id
        pose.header.stamp = rospy.Time.from_sec(stamp)
        pose.pose.position.x = float(point[0])
        pose.pose.position.y = float(point[1])
        pose.pose.position.z = float(point[2])
        if orientation is None:
            pose.pose.orientation.w = 1.0
        else:
            qx, qy, qz, qw = orientation
            pose.pose.orientation.x = qx
            pose.pose.orientation.y = qy
            pose.pose.orientation.z = qz
            pose.pose.orientation.w = qw
        path.poses.append(pose)
    return path


def error_stats(pairs, yaw, offset_xy):
    errors = []
    for _, enu, odom_pos in pairs:
        aligned = transform_gnss(enu, yaw, offset_xy, 0.0)
        errors.append(float(np.linalg.norm(aligned[:2] - odom_pos[:2])))
    errors_sorted = sorted(errors)

    def pct(q):
        return errors_sorted[min(len(errors_sorted) - 1, int(round((len(errors_sorted) - 1) * q)))]

    return statistics.mean(errors), pct(0.50), pct(0.95), max(errors)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("bag")
    parser.add_argument("--odom-topic", default="/mavros/odometry/out")
    parser.add_argument("--gnss-topic", default="/mavros/global_position/raw/fix")
    parser.add_argument("--frame-id", default="odom")
    parser.add_argument("--input-path-topic", default="/ekf/input_path")
    parser.add_argument("--gnss-path-topic", default="/ekf/gnss_path")
    parser.add_argument("--ekf-path-topic", default="/ekf/ekf_path")
    parser.add_argument("--max-dt", type=float, default=0.2)
    parser.add_argument("--rate", type=float, default=1.0)
    args = parser.parse_args()

    odom, gnss, pairs = read_bag_pairs(args.bag, args.odom_topic, args.gnss_topic, args.max_dt)
    if len(pairs) < 5:
        raise RuntimeError("not enough synchronized GNSS/odom pairs")

    yaw, offset_xy, offset_z = fit_yaw_translation(pairs)
    mean, p50, p95, err_max = error_stats(pairs, yaw, offset_xy)
    rospy.init_node("publish_bag_aligned_gnss_paths", anonymous=True)
    rospy.loginfo(
        "Global GNSS alignment: pairs=%d yaw=%.3f rad %.2f deg offset=(%.3f, %.3f, %.3f) "
        "xy_error mean=%.3f p50=%.3f p95=%.3f max=%.3f",
        len(pairs),
        yaw,
        math.degrees(yaw),
        offset_xy[0],
        offset_xy[1],
        offset_z,
        mean,
        p50,
        p95,
        err_max,
    )

    odom_points = [(stamp, position) for stamp, position, _ in odom]
    aligned_gnss_points = [(stamp, transform_gnss(enu, yaw, offset_xy, offset_z)) for stamp, enu in gnss]
    odom_path = make_path(args.frame_id, odom_points)
    gnss_path = make_path(args.frame_id, aligned_gnss_points)
    empty_ekf_path = make_path(args.frame_id, [])

    odom_pub = rospy.Publisher(args.input_path_topic, Path, queue_size=1, latch=True)
    gnss_pub = rospy.Publisher(args.gnss_path_topic, Path, queue_size=1, latch=True)
    ekf_pub = rospy.Publisher(args.ekf_path_topic, Path, queue_size=1, latch=True)
    rate = rospy.Rate(args.rate)
    while not rospy.is_shutdown():
        now = rospy.Time.now()
        odom_path.header.stamp = now
        gnss_path.header.stamp = now
        empty_ekf_path.header.stamp = now
        odom_pub.publish(odom_path)
        gnss_pub.publish(gnss_path)
        ekf_pub.publish(empty_ekf_path)
        rate.sleep()


if __name__ == "__main__":
    main()
