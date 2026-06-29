#!/usr/bin/env python3
import argparse
import copy
import math
import os
from bisect import bisect_left

import numpy as np
import rosbag
import rospy
from sensor_msgs.msg import NavSatFix


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


def local_enu_to_navsat(position, origin):
    lat0, lon0, alt0, cos_lat0 = origin
    lat = lat0 + position[1] / EARTH_RADIUS_M
    lon = lon0 + position[0] / (cos_lat0 * EARTH_RADIUS_M)
    return math.degrees(lat), math.degrees(lon), alt0 + position[2]


def interpolate_samples(samples, stamp):
    times = [item[0] for item in samples]
    index = bisect_left(times, stamp)
    if index <= 0:
        return samples[0][1]
    if index >= len(samples):
        return samples[-1][1]
    t0, p0 = samples[index - 1]
    t1, p1 = samples[index]
    dt = t1 - t0
    if dt <= 0.0:
        return p0
    ratio = (stamp - t0) / dt
    return p0 * (1.0 - ratio) + p1 * ratio


def fit_rigid_2d(source_points, target_points):
    source = np.asarray(source_points, dtype=float)
    target = np.asarray(target_points, dtype=float)
    if len(source) != len(target) or len(source) < 2:
        raise ValueError("source and target need at least two paired points")

    source_xy = source[:, :2]
    target_xy = target[:, :2]
    source_center = source_xy.mean(axis=0)
    target_center = target_xy.mean(axis=0)
    source_centered = source_xy - source_center
    target_centered = target_xy - target_center

    sin_term = np.sum(
        source_centered[:, 0] * target_centered[:, 1]
        - source_centered[:, 1] * target_centered[:, 0]
    )
    cos_term = np.sum(
        source_centered[:, 0] * target_centered[:, 0]
        + source_centered[:, 1] * target_centered[:, 1]
    )
    yaw = math.atan2(sin_term, cos_term)
    rot = np.array([[math.cos(yaw), -math.sin(yaw)], [math.sin(yaw), math.cos(yaw)]])
    translation_xy = target_center - rot @ source_center
    source_z = source[:, 2] if source.shape[1] > 2 else np.zeros(len(source))
    target_z = target[:, 2] if target.shape[1] > 2 else np.zeros(len(target))
    translation_z = float(np.mean(target_z - source_z))
    translation = np.array([translation_xy[0], translation_xy[1], translation_z], dtype=float)

    transformed_xy = source_xy @ rot.T + translation_xy
    errors = np.linalg.norm(transformed_xy - target_xy, axis=1)
    return yaw, translation, errors


def transform_position(position, yaw, translation):
    c = math.cos(yaw)
    s = math.sin(yaw)
    return np.array(
        [
            c * position[0] - s * position[1] + translation[0],
            s * position[0] + c * position[1] + translation[1],
            position[2] + translation[2],
        ],
        dtype=float,
    )


def smooth_aligned_positions(aligned, odom_samples, max_step_correction=None,
                             max_z_step_correction=None):
    xy_enabled = max_step_correction is not None and max_step_correction > 0.0
    z_enabled = max_z_step_correction is not None and max_z_step_correction > 0.0
    if (not xy_enabled and not z_enabled) or len(aligned) < 2:
        return [(stamp, position.copy()) for stamp, position in aligned]

    smoothed = []
    previous_position = None
    previous_odom = None
    max_step_correction = float(max_step_correction or 0.0)
    max_z_step_correction = float(max_z_step_correction or 0.0)
    for stamp, raw_position in aligned:
        odom_position = interpolate_samples(odom_samples, stamp)
        if previous_position is None:
            position = raw_position.copy()
        else:
            predicted = previous_position + (odom_position - previous_odom)
            correction = raw_position - predicted
            correction_xy_norm = float(np.linalg.norm(correction[:2]))
            if xy_enabled and correction_xy_norm > max_step_correction:
                correction[:2] *= max_step_correction / correction_xy_norm
            if z_enabled:
                correction[2] = max(-max_z_step_correction,
                                    min(max_z_step_correction, correction[2]))
            position = predicted + correction
        smoothed.append((stamp, position))
        previous_position = position
        previous_odom = odom_position
    return smoothed


def _nearest_sample_index(samples, stamp):
    times = [item[0] for item in samples]
    index = bisect_left(times, stamp)
    if index <= 0:
        return 0
    if index >= len(times):
        return len(times) - 1
    if abs(times[index] - stamp) < abs(times[index - 1] - stamp):
        return index
    return index - 1


def _window_indices(samples, stamp, window_s, min_pairs):
    half_window = window_s * 0.5
    indices = [i for i, (sample_stamp, _) in enumerate(samples) if abs(sample_stamp - stamp) <= half_window]
    if len(indices) >= min_pairs:
        return indices

    center = _nearest_sample_index(samples, stamp)
    left = max(0, center - min_pairs // 2)
    right = min(len(samples), left + min_pairs)
    left = max(0, right - min_pairs)
    return list(range(left, right))


def _candidate_window_indices(samples, stamp, window_s, min_pairs):
    center = _nearest_sample_index(samples, stamp)
    candidates = [_window_indices(samples, stamp, window_s, min_pairs)]

    trailing_left = max(0, center - min_pairs + 1)
    candidates.append(list(range(trailing_left, center + 1)))

    leading_right = min(len(samples), center + min_pairs)
    candidates.append(list(range(center, leading_right)))

    centered_left = max(0, center - min_pairs // 2)
    centered_right = min(len(samples), centered_left + min_pairs)
    centered_left = max(0, centered_right - min_pairs)
    candidates.append(list(range(centered_left, centered_right)))

    unique = []
    seen = set()
    for candidate in candidates:
        key = tuple(candidate)
        if len(candidate) >= 2 and key not in seen:
            unique.append(candidate)
            seen.add(key)
    return unique


def align_positions_sliding(gnss_samples, odom_samples, window_s=30.0, min_pairs=20,
                            max_step_correction=None, max_z_step_correction=None):
    if len(gnss_samples) < min_pairs:
        raise ValueError("not enough GNSS samples for sliding alignment")
    if len(odom_samples) < 2:
        raise ValueError("not enough odom samples for interpolation")

    paired = []
    for stamp, gnss_position in gnss_samples:
        paired.append((stamp, gnss_position, interpolate_samples(odom_samples, stamp)))

    paired_for_fit = [(stamp, gnss, odom) for stamp, gnss, odom in paired]
    aligned = []
    residuals = []
    yaws = []
    for stamp, gnss_position, _ in paired:
        candidates = _candidate_window_indices(
            [(p[0], p[1]) for p in paired_for_fit], stamp, window_s, min_pairs
        )
        if not candidates:
            raise ValueError("not enough paired samples in local window")

        best = None
        for indices in candidates:
            source = [paired_for_fit[i][1] for i in indices]
            target = [paired_for_fit[i][2] for i in indices]
            yaw, translation, errors = fit_rigid_2d(source, target)
            score = float(np.mean(errors))
            if best is None or score < best[0]:
                best = (score, yaw, translation, errors)

        score, yaw, translation, errors = best
        aligned_position = transform_position(gnss_position, yaw, translation)
        aligned.append((stamp, aligned_position))
        residuals.append(score)
        yaws.append(yaw)

    aligned = smooth_aligned_positions(
        aligned,
        odom_samples,
        max_step_correction,
        max_z_step_correction,
    )

    aligned_errors = []
    step_errors = []
    previous_aligned = None
    previous_odom = None
    for stamp, aligned_position in aligned:
        odom_position = interpolate_samples(odom_samples, stamp)
        aligned_errors.append(float(np.linalg.norm(aligned_position[:2] - odom_position[:2])))
        if previous_aligned is not None:
            aligned_step = aligned_position[:2] - previous_aligned[:2]
            odom_step = odom_position[:2] - previous_odom[:2]
            step_errors.append(float(np.linalg.norm(aligned_step - odom_step)))
        previous_aligned = aligned_position
        previous_odom = odom_position

    stats = {
        "mode": "sliding",
        "count": len(aligned),
        "window_s": float(window_s),
        "min_pairs": int(min_pairs),
        "max_step_correction": float(max_step_correction or 0.0),
        "max_z_step_correction": float(max_z_step_correction or 0.0),
        "residual_mean": float(np.mean(residuals)),
        "residual_p95": float(np.percentile(residuals, 95)),
        "aligned_error_mean": float(np.mean(aligned_errors)),
        "aligned_error_p95": float(np.percentile(aligned_errors, 95)),
        "aligned_error_max": float(np.max(aligned_errors)),
        "step_error_p95": float(np.percentile(step_errors, 95)) if step_errors else 0.0,
        "step_error_max": float(np.max(step_errors)) if step_errors else 0.0,
        "yaw_min_deg": float(math.degrees(min(yaws))),
        "yaw_max_deg": float(math.degrees(max(yaws))),
    }
    return aligned, stats


def read_samples(bag_path, gnss_topic, odom_topic):
    gnss_messages = []
    gnss_samples = []
    odom_samples = []
    origin = None
    with rosbag.Bag(bag_path) as bag:
        for topic, msg, _ in bag.read_messages(topics=[gnss_topic, odom_topic]):
            if topic == gnss_topic:
                if msg.status.status < 0:
                    continue
                if not all(math.isfinite(v) for v in (msg.latitude, msg.longitude, msg.altitude)):
                    continue
                position, origin = navsat_to_local_enu(msg, origin)
                stamp = msg.header.stamp.to_sec()
                gnss_messages.append((stamp, copy.deepcopy(msg)))
                gnss_samples.append((stamp, position))
            elif topic == odom_topic:
                p = msg.pose.pose.position
                odom_samples.append((msg.header.stamp.to_sec(), np.array([p.x, p.y, p.z], dtype=float)))

    if origin is None:
        raise RuntimeError("no valid GNSS messages found")
    if len(gnss_samples) < 2 or len(odom_samples) < 2:
        raise RuntimeError("need both GNSS and odom samples")
    return gnss_messages, gnss_samples, odom_samples, origin


def make_aligned_fix(source_msg, local_position, output_origin, frame_id, covariance_xy, covariance_z):
    msg = copy.deepcopy(source_msg)
    msg.header.frame_id = frame_id
    lat, lon, alt = local_enu_to_navsat(local_position, output_origin)
    msg.latitude = lat
    msg.longitude = lon
    msg.altitude = alt
    covariance = list(msg.position_covariance)
    if covariance_xy is not None:
        covariance[0] = covariance_xy
        covariance[4] = covariance_xy
    if covariance_z is not None:
        covariance[8] = covariance_z
    msg.position_covariance = covariance
    msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
    return msg


def write_output_bag(input_bag, output_bag, aligned_by_stamp, gnss_messages, output_topic,
                     frame_id, covariance_xy, covariance_z):
    os.makedirs(os.path.dirname(os.path.abspath(output_bag)), exist_ok=True)
    first_stamp = min(aligned_by_stamp)
    first_position = aligned_by_stamp[first_stamp]
    first_source = min(gnss_messages, key=lambda item: abs(item[0] - first_stamp))[1]
    output_origin = (
        math.radians(first_source.latitude),
        math.radians(first_source.longitude),
        first_position[2],
        math.cos(math.radians(first_source.latitude)),
    )
    source_by_stamp = {stamp: msg for stamp, msg in gnss_messages}
    written_stamps = set()
    written = 0
    with rosbag.Bag(output_bag, "w") as out_bag:
        with rosbag.Bag(input_bag) as in_bag:
            for topic, msg, bag_time in in_bag.read_messages():
                out_bag.write(topic, msg, bag_time)
                stamp = msg.header.stamp.to_sec() if hasattr(msg, "header") else None
                if topic in () or stamp is None:
                    continue
                if stamp in aligned_by_stamp and stamp in source_by_stamp and stamp not in written_stamps:
                    fix = make_aligned_fix(
                        source_by_stamp[stamp],
                        aligned_by_stamp[stamp],
                        output_origin,
                        frame_id,
                        covariance_xy,
                        covariance_z,
                    )
                    out_bag.write(output_topic, fix, fix.header.stamp)
                    written_stamps.add(stamp)
                    written += 1
    return written


def main():
    parser = argparse.ArgumentParser(
        description="Add an odom-compatible, locally aligned NavSatFix topic to a bag."
    )
    parser.add_argument("--input-bag", required=True)
    parser.add_argument("--output-bag", required=True)
    parser.add_argument("--gnss-topic", default="/mavros/global_position/raw/fix")
    parser.add_argument("--odom-topic", default="/mavros/odometry/out")
    parser.add_argument("--output-topic", default="/ekf/aligned_gnss/fix")
    parser.add_argument("--frame-id", default="odom")
    parser.add_argument("--window-s", type=float, default=30.0)
    parser.add_argument("--min-pairs", type=int, default=20)
    parser.add_argument(
        "--max-step-correction",
        type=float,
        default=0.10,
        help="Maximum horizontal correction per GNSS sample after odom-guided continuity prediction. "
             "Set <=0 to disable.",
    )
    parser.add_argument(
        "--max-z-step-correction",
        type=float,
        default=0.08,
        help="Maximum vertical correction per GNSS sample after odom-guided continuity prediction. "
             "Set <=0 to disable.",
    )
    parser.add_argument("--covariance-xy", type=float, default=4.0)
    parser.add_argument("--covariance-z", type=float, default=9.0)
    args = parser.parse_args()

    gnss_messages, gnss_samples, odom_samples, _ = read_samples(
        args.input_bag, args.gnss_topic, args.odom_topic
    )
    aligned, stats = align_positions_sliding(
        gnss_samples,
        odom_samples,
        window_s=args.window_s,
        min_pairs=args.min_pairs,
        max_step_correction=args.max_step_correction,
        max_z_step_correction=args.max_z_step_correction,
    )
    aligned_by_stamp = {stamp: position for stamp, position in aligned}
    written = write_output_bag(
        args.input_bag,
        args.output_bag,
        aligned_by_stamp,
        gnss_messages,
        args.output_topic,
        args.frame_id,
        args.covariance_xy,
        args.covariance_z,
    )
    print("input_bag={}".format(args.input_bag))
    print("output_bag={}".format(args.output_bag))
    print("output_topic={}".format(args.output_topic))
    print("written_aligned_fixes={}".format(written))
    for key in sorted(stats):
        print("{}={}".format(key, stats[key]))


if __name__ == "__main__":
    main()
