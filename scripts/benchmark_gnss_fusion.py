#!/usr/bin/env python3
import argparse
import json
import math
import os
import signal
import subprocess
import sys
import time

import rospy

from evaluate_gnss_fusion import FusionEvaluator


DEFAULT_TRIALS = [
    {
        "name": "no_gnss",
        "use_gnss": "false",
    },
    {
        "name": "gnss_very_conservative",
        "use_gnss": "true",
        "gnss_min_interval": "1.0",
        "gnss_min_cov_xy": "100.0",
        "gnss_min_cov_z": "144.0",
    },
    {
        "name": "gnss_conservative",
        "use_gnss": "true",
        "gnss_min_interval": "1.0",
        "gnss_min_cov_xy": "64.0",
        "gnss_min_cov_z": "100.0",
    },
    {
        "name": "gnss_medium",
        "use_gnss": "true",
        "gnss_min_interval": "0.75",
        "gnss_min_cov_xy": "36.0",
        "gnss_min_cov_z": "64.0",
    },
    {
        "name": "gnss_balanced",
        "use_gnss": "true",
        "gnss_min_interval": "0.5",
        "gnss_min_cov_xy": "16.0",
        "gnss_min_cov_z": "25.0",
    },
]


def wait_for_master(timeout=15.0):
    deadline = time.time() + timeout
    while time.time() < deadline:
        try:
            subprocess.check_call(
                ["rostopic", "list"],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )
            return
        except subprocess.CalledProcessError:
            time.sleep(0.2)
    raise RuntimeError("ROS master did not become available")


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


def serialize_metrics(metrics):
    clean = {}
    for key, value in metrics.items():
        if isinstance(value, dict):
            clean[key] = serialize_metrics(value)
        elif isinstance(value, float) and math.isnan(value):
            clean[key] = None
        else:
            clean[key] = value
    return clean


def score(metrics):
    odom = metrics["ekf_vs_odom"]
    gnss = metrics["ekf_vs_aligned_gnss"]
    if odom["count"] == 0:
        return float("inf")
    value = odom["p95"] + 0.5 * metrics["ekf_step_p95"]
    if gnss["count"] > 0 and not math.isnan(gnss["p95"]):
        value += 0.35 * gnss["p95"]
    return value


def run_trial(bag_path, trial, evaluator):
    evaluator.reset()
    launch_args = ["roslaunch", "ekf", "ekf_lidar.launch"]
    for key, value in trial.items():
        if key == "name":
            continue
        launch_args.append("{}:={}".format(key, value))

    launch = subprocess.Popen(
        launch_args,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        universal_newlines=True,
        preexec_fn=os.setsid,
    )
    time.sleep(2.0)
    play = subprocess.Popen(["rosbag", "play", "--clock", "--quiet", bag_path])
    play.wait()
    time.sleep(1.0)
    terminate_process(launch)
    launch_output, _ = launch.communicate(timeout=5.0)

    metrics = evaluator.snapshot_metrics()
    metrics["reset_count"] = launch_output.count("Resetting EKF")
    metrics["gnss_reject_count"] = launch_output.count("Rejecting GNSS update")
    metrics["score"] = score(metrics)
    return metrics


def main():
    parser = argparse.ArgumentParser(description="Run repeatable EKF/GNSS fusion benchmark trials.")
    parser.add_argument("bag", help="Input rosbag path")
    parser.add_argument("--output", default="/tmp/ekf_fusion_benchmark.json", help="JSON result path")
    args = parser.parse_args()

    if not os.path.exists(args.bag):
        raise SystemExit("bag not found: {}".format(args.bag))

    roscore = subprocess.Popen(["roscore"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    try:
        wait_for_master()
        subprocess.check_call(["rosparam", "set", "/use_sim_time", "true"])
        rospy.init_node("ekf_fusion_benchmark", anonymous=True)
        evaluator = FusionEvaluator()

        results = []
        for trial in DEFAULT_TRIALS:
            print("running {}".format(trial["name"]))
            metrics = run_trial(args.bag, trial, evaluator)
            result = {"trial": trial, "metrics": serialize_metrics(metrics)}
            results.append(result)
            print(json.dumps(result, indent=2, sort_keys=True))

        best = min(results, key=lambda item: item["metrics"]["score"])
        report = {"bag": args.bag, "best": best["trial"]["name"], "results": results}
        with open(args.output, "w") as output_file:
            json.dump(report, output_file, indent=2, sort_keys=True)
        print("best={} output={}".format(report["best"], args.output))
    finally:
        terminate_process(roscore)


if __name__ == "__main__":
    sys.exit(main())
