#!/usr/bin/env python3
import argparse
import copy
import os

import rosbag


def header_stamp(msg, fallback_stamp):
    if hasattr(msg, "header") and msg.header.stamp.to_sec() > 0.0:
        return msg.header.stamp.to_sec()
    return fallback_stamp.to_sec()


def make_ground_truth(msg):
    out = copy.deepcopy(msg)
    if not out.header.frame_id:
        out.header.frame_id = "odom"
    if hasattr(out, "child_frame_id") and not out.child_frame_id:
        out.child_frame_id = "base_link"
    return out


def first_topic_header_time(bag_path, topic_name):
    with rosbag.Bag(bag_path, "r") as bag:
        for topic, msg, stamp in bag.read_messages(topics=[topic_name]):
            return header_stamp(msg, stamp)
    raise RuntimeError("topic not found in bag: {}".format(topic_name))


def max_header_gap(bag_path, topic_name):
    previous = None
    max_gap = 0.0
    count = 0
    with rosbag.Bag(bag_path, "r") as bag:
        for topic, msg, stamp in bag.read_messages(topics=[topic_name]):
            current = header_stamp(msg, stamp)
            if previous is not None:
                max_gap = max(max_gap, current - previous)
            previous = current
            count += 1
    return count, max_gap


def main():
    parser = argparse.ArgumentParser(description="Create a header-time odom-dropout validation bag.")
    parser.add_argument("--input-bag", required=True)
    parser.add_argument("--output-bag", required=True)
    parser.add_argument("--odom-topic", default="/mavros/odometry/out")
    parser.add_argument("--ground-truth-topic", default="/ground_truth/odom")
    parser.add_argument("--window-start", type=float, default=0.0)
    parser.add_argument("--window-duration", type=float, default=40.0)
    parser.add_argument("--odom-drop-start", type=float, default=8.0)
    parser.add_argument("--odom-drop-duration", type=float, default=24.0)
    args = parser.parse_args()

    if not os.path.exists(args.input_bag):
        raise SystemExit("input bag not found: {}".format(args.input_bag))
    if args.window_duration <= 0.0:
        raise SystemExit("--window-duration must be positive")
    if args.odom_drop_duration <= 0.0:
        raise SystemExit("--odom-drop-duration must be positive")

    output_dir = os.path.dirname(os.path.abspath(args.output_bag))
    if output_dir:
        os.makedirs(output_dir, exist_ok=True)

    origin = first_topic_header_time(args.input_bag, args.odom_topic) + args.window_start
    window_end = args.window_duration
    drop_end = args.odom_drop_start + args.odom_drop_duration
    counts = {
        "read": 0,
        "written": 0,
        "window_skipped": 0,
        "odom_kept": 0,
        "odom_dropped": 0,
        "ground_truth_written": 0,
    }
    per_topic = {}

    with rosbag.Bag(args.input_bag, "r") as src, rosbag.Bag(args.output_bag, "w") as dst:
        for topic, msg, stamp in src.read_messages():
            counts["read"] += 1
            msg_time = header_stamp(msg, stamp)
            rel_time = msg_time - origin
            if rel_time < 0.0 or rel_time > window_end:
                counts["window_skipped"] += 1
                continue

            write_stamp = msg.header.stamp if hasattr(msg, "header") and msg.header.stamp.to_sec() > 0.0 else stamp

            if topic == args.odom_topic:
                dst.write(args.ground_truth_topic, make_ground_truth(msg), write_stamp)
                counts["ground_truth_written"] += 1
                per_topic[args.ground_truth_topic] = per_topic.get(args.ground_truth_topic, 0) + 1
                if args.odom_drop_start <= rel_time <= drop_end:
                    counts["odom_dropped"] += 1
                    continue
                counts["odom_kept"] += 1

            dst.write(topic, msg, write_stamp)
            counts["written"] += 1
            per_topic[topic] = per_topic.get(topic, 0) + 1

    odom_count, odom_max_gap = max_header_gap(args.output_bag, args.odom_topic)
    print("wrote {}".format(args.output_bag))
    print("origin_header_time={:.6f}".format(origin))
    print("window={:.3f}-{:.3f}s".format(0.0, window_end))
    print("odom_dropout={:.3f}-{:.3f}s by odom header.stamp".format(args.odom_drop_start, drop_end))
    for key in sorted(counts):
        print("{}={}".format(key, counts[key]))
    print("kept_odom_count={}".format(odom_count))
    print("kept_odom_max_header_gap={:.6f}s".format(odom_max_gap))
    for topic in sorted(per_topic):
        print("topic_count {}={}".format(topic, per_topic[topic]))


if __name__ == "__main__":
    main()
