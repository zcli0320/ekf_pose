#!/usr/bin/env python3
import argparse
import bisect
import csv
import json
import math
import os
import signal
import subprocess
import sys
import time

import matplotlib.pyplot as plt
import rosbag
import rospy
from nav_msgs.msg import Odometry


def terminate_process(process, timeout=5.0):
    if process.poll() is not None:
        return
    process.send_signal(signal.SIGINT)
    try:
        process.wait(timeout=timeout)
    except subprocess.TimeoutExpired:
        process.terminate()
        try:
            process.wait(timeout=timeout)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait()


def wait_for_master(env, timeout=15.0):
    deadline = time.time() + timeout
    while time.time() < deadline:
        try:
            subprocess.check_call(
                ["rostopic", "list"],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                env=env,
            )
            return
        except subprocess.CalledProcessError:
            time.sleep(0.2)
    raise RuntimeError("ROS master did not become available")


def percentile(values, fraction):
    if not values:
        return float("nan")
    ordered = sorted(values)
    index = min(len(ordered) - 1, int(round((len(ordered) - 1) * fraction)))
    return ordered[index]


def summarize(errors):
    if not errors:
        return {
            "count": 0,
            "mean_m": float("nan"),
            "mse_m2": float("nan"),
            "rmse_m": float("nan"),
            "p95_m": float("nan"),
            "max_m": float("nan"),
        }
    mse = sum(value * value for value in errors) / len(errors)
    return {
        "count": len(errors),
        "mean_m": sum(errors) / len(errors),
        "mse_m2": mse,
        "rmse_m": math.sqrt(mse),
        "p95_m": percentile(errors, 0.95),
        "max_m": max(errors),
    }


def compute_smoothness_stats(path, start_abs, end_abs):
    samples = [item for item in sorted(path) if start_abs <= item[0] <= end_abs]
    steps = []
    velocities = []
    accelerations = []
    for previous, current in zip(samples, samples[1:]):
        dt = current[0] - previous[0]
        if dt <= 1.0e-9:
            continue
        dx = current[1] - previous[1]
        dy = current[2] - previous[2]
        dz = current[3] - previous[3]
        step = math.sqrt(dx * dx + dy * dy + dz * dz)
        steps.append(step)
        velocities.append((current[0], dx / dt, dy / dt, dz / dt))

    velocity_deltas = []
    for previous, current in zip(velocities, velocities[1:]):
        dvx = current[1] - previous[1]
        dvy = current[2] - previous[2]
        dvz = current[3] - previous[3]
        velocity_deltas.append(math.sqrt(dvx * dvx + dvy * dvy + dvz * dvz))
        dt = current[0] - previous[0]
        if dt > 1.0e-9:
            accelerations.append((current[0], dvx / dt, dvy / dt, dvz / dt))

    jerks = []
    for previous, current in zip(accelerations, accelerations[1:]):
        dt = current[0] - previous[0]
        if dt <= 1.0e-9:
            continue
        dax = current[1] - previous[1]
        day = current[2] - previous[2]
        daz = current[3] - previous[3]
        jerks.append(math.sqrt(dax * dax + day * day + daz * daz) / dt)

    return {
        "smoothness_count": len(steps),
        "step_mean_m": sum(steps) / len(steps) if steps else float("nan"),
        "step_p95_m": percentile(steps, 0.95),
        "step_max_m": max(steps) if steps else float("nan"),
        "velocity_delta_p95_mps": percentile(velocity_deltas, 0.95),
        "velocity_delta_max_mps": max(velocity_deltas) if velocity_deltas else float("nan"),
        "jerk_p95_mps3": percentile(jerks, 0.95),
        "jerk_max_mps3": max(jerks) if jerks else float("nan"),
    }


def finite_or_none(value):
    return None if isinstance(value, float) and math.isnan(value) else value


class WindowCollector:
    def __init__(self, ekf_topic, reference_topic):
        self.ekf = []
        self.reference = []
        self.ekf_sub = rospy.Subscriber(ekf_topic, Odometry, self.ekf_cb, queue_size=1000)
        self.reference_sub = rospy.Subscriber(reference_topic, Odometry, self.reference_cb, queue_size=1000)

    def ekf_cb(self, msg):
        self.ekf.append(self.pose_tuple(msg))

    def reference_cb(self, msg):
        self.reference.append(self.pose_tuple(msg))

    @staticmethod
    def pose_tuple(msg):
        p = msg.pose.pose.position
        return (msg.header.stamp.to_sec(), p.x, p.y, p.z)


def nearest_by_time(seq, times, stamp, max_dt):
    if not seq:
        return None
    idx = bisect.bisect_left(times, stamp)
    candidates = []
    if idx < len(seq):
        candidates.append(seq[idx])
    if idx > 0:
        candidates.append(seq[idx - 1])
    best = min(candidates, key=lambda item: abs(item[0] - stamp))
    if abs(best[0] - stamp) > max_dt:
        return None
    return best


def distance(a, b):
    return math.sqrt((a[1] - b[1]) ** 2 + (a[2] - b[2]) ** 2 + (a[3] - b[3]) ** 2)


def compute_window_errors(ekf, reference, start_abs, end_abs, max_dt):
    reference = sorted(reference)
    reference_times = [item[0] for item in reference]
    pairs = []
    errors = []
    for estimate in sorted(ekf):
        if estimate[0] < start_abs or estimate[0] > end_abs:
            continue
        ref = nearest_by_time(reference, reference_times, estimate[0], max_dt)
        if ref is None:
            continue
        err = distance(estimate, ref)
        pairs.append(
            {
                "time_s": estimate[0] - start_abs,
                "bag_time_s": estimate[0],
                "reference_time_s": ref[0],
                "error_m": err,
            }
        )
        errors.append(err)
    return pairs, summarize(errors)


def write_outputs(output_dir, label, window_start, window_end, pairs, stats, smoothness_stats, metadata):
    os.makedirs(output_dir, exist_ok=True)
    csv_path = os.path.join(output_dir, "{}_ekf_vs_hidden_odom_window_errors.csv".format(label))
    json_path = os.path.join(output_dir, "{}_ekf_vs_hidden_odom_window_stats.json".format(label))
    png_path = os.path.join(output_dir, "{}_ekf_vs_hidden_odom_window_stats.png".format(label))

    with open(csv_path, "w", newline="", encoding="utf-8") as output:
        writer = csv.DictWriter(output, fieldnames=["time_s", "bag_time_s", "reference_time_s", "error_m"])
        writer.writeheader()
        for row in pairs:
            writer.writerow(row)

    report = {
        "label": label,
        "window_start_s": window_start,
        "window_end_s": window_end,
        "stats": {key: finite_or_none(value) for key, value in stats.items()},
        "smoothness": {key: finite_or_none(value) for key, value in smoothness_stats.items()},
        "metadata": metadata,
        "csv": csv_path,
        "figure": png_path,
    }
    with open(json_path, "w", encoding="utf-8") as output:
        json.dump(report, output, indent=2, sort_keys=True)

    times = [row["time_s"] for row in pairs]
    errors = [row["error_m"] for row in pairs]
    p95 = stats["p95_m"]
    mse = stats["mse_m2"]

    fig, axes = plt.subplots(1, 2, figsize=(10.5, 3.8), constrained_layout=True)
    axes[0].plot(times, errors, color="#2F6B9A", linewidth=1.3)
    if math.isfinite(p95):
        axes[0].axhline(p95, color="#C44E52", linestyle="--", linewidth=1.1, label="P95 = {:.3f} m".format(p95))
        axes[0].legend(frameon=False, fontsize=9)
    axes[0].set_title("Dropout Window Error")
    axes[0].set_xlabel("Time since dropout start (s)")
    axes[0].set_ylabel("EKF vs hidden odom error (m)")
    axes[0].grid(alpha=0.25)

    step_p95 = smoothness_stats["step_p95_m"]
    bars = axes[1].bar(["P95 (m)", "MSE (m^2)", "Step P95 (m)"], [p95, mse, step_p95], color=["#4C78A8", "#F58518", "#54A24B"], width=0.52)
    axes[1].set_title("Window Statistics")
    axes[1].grid(axis="y", alpha=0.25)
    ymax = max([value for value in [p95, mse, step_p95] if math.isfinite(value)] + [1.0])
    for bar, value in zip(bars, [p95, mse, step_p95]):
        if math.isfinite(value):
            axes[1].text(
                bar.get_x() + bar.get_width() / 2.0,
                value + ymax * 0.03,
                "{:.4f}".format(value),
                ha="center",
                va="bottom",
                fontsize=9,
            )
    for axis in axes:
        axis.spines["top"].set_visible(False)
        axis.spines["right"].set_visible(False)
    fig.suptitle("EKF Compared with Hidden Odom During {:.1f}-{:.1f}s Odom Dropout".format(window_start, window_end))
    fig.savefig(png_path, dpi=220)
    plt.close(fig)
    return csv_path, json_path, png_path


def main():
    parser = argparse.ArgumentParser(description="Evaluate EKF against hidden odom during an odom-dropout window.")
    parser.add_argument("--bag", required=True)
    parser.add_argument("--output-dir", default="results/odom_ablation_40s/window_stats")
    parser.add_argument("--label", default="dropout_8_32")
    parser.add_argument("--window-start", type=float, default=8.0)
    parser.add_argument("--window-end", type=float, default=32.0)
    parser.add_argument("--max-dt", type=float, default=0.04)
    parser.add_argument("--ros-port", type=int, default=11339)
    parser.add_argument("--ekf-topic", default="/ekf/ekf_odom")
    parser.add_argument("--reference-topic", default="/ground_truth/odom")
    parser.add_argument("--play-rate", default="3.0")
    parser.add_argument("--launch-arg", action="append", default=[])
    args = parser.parse_args()

    if not os.path.exists(args.bag):
        raise SystemExit("bag not found: {}".format(args.bag))
    if args.window_end <= args.window_start:
        raise SystemExit("--window-end must be greater than --window-start")

    with rosbag.Bag(args.bag) as bag:
        bag_start = bag.get_start_time()

    env = os.environ.copy()
    env["ROS_MASTER_URI"] = "http://localhost:{}".format(args.ros_port)
    env["ROS_HOME"] = os.path.join("/tmp", "{}_ros_home".format(args.label))
    os.environ.update(env)

    roscore = subprocess.Popen(["roscore", "-p", str(args.ros_port)], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, env=env)
    launch = None
    try:
        wait_for_master(env)
        subprocess.check_call(["rosparam", "set", "/use_sim_time", "true"], env=env)
        rospy.init_node("odom_dropout_window_eval", anonymous=True)
        collector = WindowCollector(args.ekf_topic, args.reference_topic)

        launch_args = [
            "roslaunch",
            "ekf",
            "ekf_lidar.launch",
            "start_rviz:=false",
            "start_display_frames:=false",
        ]
        launch_args.extend(args.launch_arg)
        launch = subprocess.Popen(
            launch_args,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            universal_newlines=True,
            preexec_fn=os.setsid,
            env=env,
        )
        time.sleep(2.0)
        play = subprocess.Popen(["rosbag", "play", "--clock", "--quiet", "-r", args.play_rate, args.bag], env=env)
        play.wait()
        time.sleep(1.0)

        launch_output = ""
        if launch is not None:
            terminate_process(launch)
            launch_output, _ = launch.communicate(timeout=5.0)

        pairs, stats = compute_window_errors(
            collector.ekf,
            collector.reference,
            bag_start + args.window_start,
            bag_start + args.window_end,
            args.max_dt,
        )
        metadata = {
            "bag": args.bag,
            "bag_start_s": bag_start,
            "ekf_topic": args.ekf_topic,
            "reference_topic": args.reference_topic,
            "max_dt_s": args.max_dt,
            "ekf_samples": len(collector.ekf),
            "reference_samples": len(collector.reference),
            "reset_count": launch_output.count("Resetting EKF"),
            "odom_lost_count": launch_output.count("Odom observation health=LOST"),
            "odom_weak_count": launch_output.count("Odom observation health=WEAK"),
            "gnss_velocity_update_count": launch_output.count("GNSS velocity pseudo-update active"),
        }
        smoothness_stats = compute_smoothness_stats(
            collector.ekf,
            bag_start + args.window_start,
            bag_start + args.window_end,
        )
        csv_path, json_path, png_path = write_outputs(
            args.output_dir,
            args.label,
            args.window_start,
            args.window_end,
            pairs,
            stats,
            smoothness_stats,
            metadata,
        )
        print("count={count} p95={p95_m:.6f} mse={mse_m2:.6f} rmse={rmse_m:.6f} max={max_m:.6f} step_p95={step_p95_m:.6f} vdelta_p95={velocity_delta_p95_mps:.6f} jerk_p95={jerk_p95_mps3:.6f}".format(
            step_p95_m=smoothness_stats["step_p95_m"],
            velocity_delta_p95_mps=smoothness_stats["velocity_delta_p95_mps"],
            jerk_p95_mps3=smoothness_stats["jerk_p95_mps3"],
            **stats))
        print("csv={}".format(csv_path))
        print("json={}".format(json_path))
        print("figure={}".format(png_path))
    finally:
        if launch is not None:
            terminate_process(launch)
        terminate_process(roscore)


if __name__ == "__main__":
    sys.exit(main())
