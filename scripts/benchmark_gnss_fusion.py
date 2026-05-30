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
    {
        "name": "gnss_trusted",
        "use_gnss": "true",
        "gnss_min_interval": "0.2",
        "gnss_min_cov_xy": "1.0",
        "gnss_min_cov_z": "4.0",
        "gnss_cov_scale": "0.5",
        "gnss_alignment_min_samples": "20",
        "gnss_alignment_max_samples": "40",
        "gnss_alignment_min_motion": "3.0",
    },
    {
        "name": "gnss_health_gate",
        "use_gnss": "true",
        "gnss_min_interval": "1.0",
        "gnss_min_cov_xy": "64.0",
        "gnss_min_cov_z": "100.0",
        "enable_gnss_mahalanobis_gate": "true",
        "enable_gnss_motion_consistency": "true",
        "gnss_motion_consistency_threshold": "1.0",
        "gnss_motion_consistency_reject_threshold": "3.0",
        "gnss_alignment_min_samples": "5",
        "gnss_alignment_max_samples": "30",
        "gnss_alignment_min_motion": "1.0",
    },
    {
        "name": "gnss_no_anomaly_handling",
        "use_gnss": "true",
        "gnss_min_interval": "0.2",
        "gnss_min_cov_xy": "0.04",
        "gnss_min_cov_z": "0.09",
        "enable_gnss_mahalanobis_gate": "false",
        "enable_gnss_motion_consistency": "false",
        "enable_gnss_health_score": "false",
        "enable_gnss_nis_state_machine": "false",
        "enable_odom_gnss_consistency_health": "false",
        "gnss_alignment_min_samples": "12",
        "gnss_alignment_max_samples": "40",
        "gnss_alignment_min_motion": "2.0",
    },
    {
        "name": "gnss_direct_reject",
        "use_gnss": "true",
        "gnss_min_interval": "0.2",
        "gnss_min_cov_xy": "0.04",
        "gnss_min_cov_z": "0.09",
        "enable_gnss_mahalanobis_gate": "true",
        "enable_gnss_nis_state_machine": "false",
        "enable_gnss_motion_consistency": "false",
        "enable_gnss_health_score": "false",
        "gnss_mahalanobis_weak_threshold": "7.815",
        "gnss_mahalanobis_reject_threshold": "16.266",
        "gnss_alignment_min_samples": "12",
        "gnss_alignment_max_samples": "40",
        "gnss_alignment_min_motion": "2.0",
    },
    {
        "name": "gnss_adaptive_nis_window",
        "use_gnss": "true",
        "gnss_min_interval": "0.2",
        "gnss_min_cov_xy": "0.04",
        "gnss_min_cov_z": "0.09",
        "enable_gnss_mahalanobis_gate": "true",
        "enable_gnss_nis_state_machine": "true",
        "enable_gnss_motion_consistency": "true",
        "enable_gnss_health_score": "true",
        "gnss_health_window_size": "5",
        "gnss_degraded_count_threshold": "3",
        "gnss_severe_count_threshold": "3",
        "gnss_recover_count_threshold": "3",
        "gnss_isolation_time": "1.0",
        "gnss_r_degraded_scale": "5.0",
        "gnss_r_severe_scale": "20.0",
        "gnss_alignment_min_samples": "12",
        "gnss_alignment_max_samples": "40",
        "gnss_alignment_min_motion": "2.0",
    },
    {
        "name": "gnss_drift_no_anomaly_handling",
        "use_gnss": "true",
        "position_cov": "1.0",
        "gnss_min_interval": "0.2",
        "gnss_min_cov_xy": "0.04",
        "gnss_min_cov_z": "0.09",
        "enable_gnss_mahalanobis_gate": "false",
        "enable_gnss_motion_consistency": "false",
        "enable_gnss_health_score": "false",
        "enable_gnss_nis_state_machine": "false",
        "enable_odom_gnss_consistency_health": "true",
        "odom_gnss_consistency_threshold": "0.4",
        "odom_gnss_consistency_poor_threshold": "1.5",
        "odom_weak_health_threshold": "0.6",
        "gnss_healthy_odom_weak_scale": "0.1",
        "gnss_alignment_min_samples": "12",
        "gnss_alignment_max_samples": "40",
        "gnss_alignment_min_motion": "2.0",
    },
    {
        "name": "gnss_drift_direct_reject",
        "use_gnss": "true",
        "position_cov": "1.0",
        "gnss_min_interval": "0.2",
        "gnss_min_cov_xy": "0.04",
        "gnss_min_cov_z": "0.09",
        "enable_gnss_mahalanobis_gate": "true",
        "enable_gnss_nis_state_machine": "false",
        "enable_gnss_motion_consistency": "false",
        "enable_gnss_health_score": "false",
        "enable_odom_gnss_consistency_health": "true",
        "odom_gnss_consistency_threshold": "0.4",
        "odom_gnss_consistency_poor_threshold": "1.5",
        "odom_weak_health_threshold": "0.6",
        "gnss_healthy_odom_weak_scale": "0.1",
        "gnss_mahalanobis_weak_threshold": "7.815",
        "gnss_mahalanobis_reject_threshold": "16.266",
        "gnss_alignment_min_samples": "12",
        "gnss_alignment_max_samples": "40",
        "gnss_alignment_min_motion": "2.0",
    },
    {
        "name": "gnss_drift_adaptive_nis_window",
        "use_gnss": "true",
        "position_cov": "1.0",
        "gnss_min_interval": "0.2",
        "gnss_min_cov_xy": "0.04",
        "gnss_min_cov_z": "0.09",
        "enable_gnss_mahalanobis_gate": "true",
        "enable_gnss_nis_state_machine": "true",
        "enable_gnss_motion_consistency": "true",
        "enable_gnss_health_score": "true",
        "enable_odom_gnss_consistency_health": "true",
        "odom_gnss_consistency_threshold": "0.4",
        "odom_gnss_consistency_poor_threshold": "1.5",
        "odom_weak_health_threshold": "0.6",
        "gnss_healthy_odom_weak_scale": "0.1",
        "gnss_health_window_size": "5",
        "gnss_degraded_count_threshold": "3",
        "gnss_severe_count_threshold": "3",
        "gnss_recover_count_threshold": "3",
        "gnss_isolation_time": "1.0",
        "gnss_r_degraded_scale": "5.0",
        "gnss_r_severe_scale": "20.0",
        "gnss_alignment_min_samples": "12",
        "gnss_alignment_max_samples": "40",
        "gnss_alignment_min_motion": "2.0",
    },
    {
        "name": "gnss_drift_correction",
        "use_gnss": "true",
        "position_cov": "1.0",
        "gnss_min_interval": "0.2",
        "gnss_min_cov_xy": "0.04",
        "gnss_min_cov_z": "0.09",
        "gnss_cov_scale": "1.0",
        "enable_gnss_mahalanobis_gate": "true",
        "enable_gnss_nis_state_machine": "true",
        "enable_gnss_motion_consistency": "true",
        "enable_odom_gnss_consistency_health": "true",
        "odom_gnss_consistency_threshold": "0.4",
        "odom_gnss_consistency_poor_threshold": "1.5",
        "odom_weak_health_threshold": "0.6",
        "gnss_healthy_odom_weak_scale": "0.1",
        "gnss_alignment_min_samples": "12",
        "gnss_alignment_max_samples": "40",
        "gnss_alignment_min_motion": "2.0",
    },
    {
        "name": "gnss_trusted_odom_loose",
        "use_gnss": "true",
        "position_cov": "0.05",
        "gnss_min_interval": "0.2",
        "gnss_min_cov_xy": "1.0",
        "gnss_min_cov_z": "4.0",
        "gnss_cov_scale": "0.5",
        "gnss_alignment_min_samples": "20",
        "gnss_alignment_max_samples": "40",
        "gnss_alignment_min_motion": "3.0",
    },
    {
        "name": "gnss_strong_odom_loose",
        "use_gnss": "true",
        "position_cov": "0.10",
        "gnss_min_interval": "0.2",
        "gnss_min_cov_xy": "0.25",
        "gnss_min_cov_z": "1.0",
        "gnss_cov_scale": "0.25",
        "gnss_alignment_min_samples": "20",
        "gnss_alignment_max_samples": "40",
        "gnss_alignment_min_motion": "3.0",
    },
    {
        "name": "imu_gnss_degraded",
        "use_gnss": "true",
        "position_cov": "0.10",
        "odom_loss_timeout": "1.0",
        "gnss_min_interval": "0.2",
        "gnss_min_cov_xy": "0.25",
        "gnss_min_cov_z": "1.0",
        "gnss_cov_scale": "0.25",
        "gnss_healthy_odom_weak_scale": "0.1",
        "enable_gnss_velocity_when_odom_lost": "true",
        "gnss_velocity_cov": "1.0",
        "gnss_alignment_min_samples": "3",
        "gnss_alignment_max_samples": "30",
        "gnss_alignment_min_motion": "0.01",
        "gnss_alignment_max_residual": "2.0",
    },
    {
        "name": "imu_gnss_cold_start",
        "use_gnss": "true",
        "enable_gnss_cold_start": "true",
        "position_cov": "0.10",
        "odom_loss_timeout": "1.0",
        "gnss_min_interval": "0.2",
        "gnss_min_cov_xy": "0.25",
        "gnss_min_cov_z": "1.0",
        "gnss_cov_scale": "0.25",
        "gnss_healthy_odom_weak_scale": "0.1",
        "enable_gnss_velocity_when_odom_lost": "true",
        "gnss_velocity_cov": "1.0",
        "gnss_alignment_min_samples": "3",
        "gnss_alignment_max_samples": "30",
        "gnss_alignment_min_motion": "0.01",
        "gnss_alignment_max_residual": "2.0",
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
    ground_truth = metrics.get("ekf_vs_ground_truth", {"count": 0})
    if ground_truth["count"] > 0 and not math.isnan(ground_truth["p95"]):
        value = ground_truth["p95"] + 0.5 * metrics["ekf_step_p95"]
        odom_ground_truth = metrics.get("odom_vs_ground_truth", {"count": 0})
        if odom_ground_truth["count"] > 0 and not math.isnan(odom_ground_truth["p95"]):
            value += 0.25 * max(0.0, ground_truth["p95"] - odom_ground_truth["p95"])
        return value

    odom = metrics["ekf_vs_odom"]
    if odom["count"] == 0:
        return float("inf")
    value = odom["p95"] + 0.5 * metrics["ekf_step_p95"]
    rigid_gnss = metrics.get("ekf_vs_rigid_gnss", {"count": 0, "p95": float("nan")})
    if rigid_gnss["count"] > 0 and not math.isnan(rigid_gnss["p95"]) and rigid_gnss["p95"] < 5.0:
        gnss = metrics.get("ekf_vs_node_gnss_path", metrics["ekf_vs_aligned_gnss"])
        if gnss["count"] == 0:
            gnss = rigid_gnss
        if gnss["count"] > 0 and not math.isnan(gnss["p95"]):
            value += 0.35 * gnss["p95"]
    return value


def run_trial(bag_path, trial, evaluator, extra_launch_args, extra_play_args, launch_file):
    evaluator.reset()
    launch_args = ["roslaunch", "ekf", launch_file, "start_rviz:=false"]
    for key, value in trial.items():
        if key == "name":
            continue
        launch_args.append("{}:={}".format(key, value))
    launch_args.extend(extra_launch_args)

    launch = subprocess.Popen(
        launch_args,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        universal_newlines=True,
        preexec_fn=os.setsid,
    )
    time.sleep(2.0)
    play = subprocess.Popen(["rosbag", "play", "--clock", "--quiet"] + extra_play_args + [bag_path])
    play.wait()
    time.sleep(1.0)
    terminate_process(launch)
    launch_output, _ = launch.communicate(timeout=5.0)

    metrics = evaluator.snapshot_metrics()
    metrics["reset_count"] = launch_output.count("Resetting EKF")
    metrics["odom_realign_count"] = launch_output.count("Realigned odom frame")
    metrics["odom_weak_count"] = launch_output.count("Odom observation health=WEAK")
    metrics["odom_lost_count"] = launch_output.count("Odom observation health=LOST")
    metrics["gnss_reject_count"] = launch_output.count("Rejecting GNSS update")
    metrics["gnss_weak_count"] = launch_output.count("GNSS observation health=WEAK")
    metrics["gnss_motion_inconsistent_count"] = launch_output.count("motion inconsistency")
    metrics["gnss_nis_monitor_count"] = launch_output.count("GNSS NIS monitor")
    metrics["gnss_nis_isolated_count"] = launch_output.count("NIS state=ISOLATED")
    metrics["gnss_velocity_update_count"] = launch_output.count("GNSS velocity pseudo-update active")
    metrics["gnss_cold_start_count"] = launch_output.count("GNSS cold start initialized EKF")
    metrics["gnss_yaw_alignment_count"] = launch_output.count("Initialized GNSS yaw alignment")
    metrics["vio_ready_count"] = launch_output.count("VIO guidance ready")
    metrics["vio_waiting_count"] = launch_output.count("VIO guidance waiting")
    metrics["vio_lost_count"] = launch_output.count("vio_lost")
    metrics["vio_reset_detected_count"] = launch_output.count("vio_reset_detected")
    metrics["score"] = score(metrics)
    return metrics


def main():
    parser = argparse.ArgumentParser(description="Run repeatable EKF/GNSS fusion benchmark trials.")
    parser.add_argument("bag", help="Input rosbag path")
    parser.add_argument("--output", default="/tmp/ekf_fusion_benchmark.json", help="JSON result path")
    parser.add_argument("--ekf-topic", default="/ekf/ekf_odom", help="EKF topic used by evaluator")
    parser.add_argument("--odom-topic", default="/mavros/odometry/in", help="Odom reference topic used by evaluator")
    parser.add_argument("--gnss-topic", default="/mavros/global_position/global", help="GNSS topic used by evaluator")
    parser.add_argument("--ground-truth-topic", default="/ground_truth/odom", help="Ground truth odometry topic used by evaluator")
    parser.add_argument("--aligned-gnss-path-topic", default="/ekf/gnss_path", help="Aligned GNSS path topic published by EKF node")
    parser.add_argument("--launch-file", default="ekf_lidar.launch", help="Launch file in the ekf package to benchmark")
    parser.add_argument("--single-trial-name", default=None, help="Run one launch-default trial with this name")
    parser.add_argument("--anomaly-end-time", type=float, default=-1.0, help="Bag-relative anomaly end time for recovery metric")
    parser.add_argument("--recovery-error-threshold", type=float, default=0.5, help="Error threshold in meters for recovery metric")
    parser.add_argument("--recovery-hold-samples", type=int, default=5, help="Consecutive samples required for recovery metric")
    parser.add_argument("--launch-arg", action="append", default=[], help="Extra roslaunch arg, e.g. use_gnss:=false")
    parser.add_argument("--play-arg", action="append", default=[], help="Extra rosbag play arg/remap")
    parser.add_argument("--trial-name", action="append", default=[], help="Run only matching trial name. May be repeated.")
    args = parser.parse_args()

    if not os.path.exists(args.bag):
        raise SystemExit("bag not found: {}".format(args.bag))

    roscore = subprocess.Popen(["roscore"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    try:
        wait_for_master()
        subprocess.check_call(["rosparam", "set", "/use_sim_time", "true"])
        rospy.init_node("ekf_fusion_benchmark", anonymous=True)
        rospy.set_param("~ekf_topic", args.ekf_topic)
        rospy.set_param("~odom_topic", args.odom_topic)
        rospy.set_param("~gnss_topic", args.gnss_topic)
        rospy.set_param("~ground_truth_topic", args.ground_truth_topic)
        rospy.set_param("~aligned_gnss_path_topic", args.aligned_gnss_path_topic)
        rospy.set_param("~anomaly_end_time", args.anomaly_end_time)
        rospy.set_param("~recovery_error_threshold", args.recovery_error_threshold)
        rospy.set_param("~recovery_hold_samples", args.recovery_hold_samples)
        evaluator = FusionEvaluator()

        results = []
        selected_trials = DEFAULT_TRIALS
        if args.single_trial_name:
            selected_trials = [{"name": args.single_trial_name}]
        elif args.trial_name:
            selected = set(args.trial_name)
            selected_trials = [trial for trial in DEFAULT_TRIALS if trial["name"] in selected]
            if not selected_trials:
                raise RuntimeError("No trials matched --trial-name {}".format(args.trial_name))

        for trial in selected_trials:
            print("running {}".format(trial["name"]))
            metrics = run_trial(args.bag, trial, evaluator, args.launch_arg, args.play_arg, args.launch_file)
            result = {"trial": trial, "metrics": serialize_metrics(metrics)}
            results.append(result)
            print(json.dumps(result, indent=2, sort_keys=True))

        best = min(results, key=lambda item: item["metrics"]["score"])
        report = {
            "bag": args.bag,
            "best": best["trial"]["name"],
            "evaluator_topics": {
                "ekf": args.ekf_topic,
                "odom": args.odom_topic,
                "gnss": args.gnss_topic,
                "ground_truth": args.ground_truth_topic,
                "aligned_gnss_path": args.aligned_gnss_path_topic,
            },
            "extra_launch_args": args.launch_arg,
            "extra_play_args": args.play_arg,
            "launch_file": args.launch_file,
            "results": results,
        }
        output_dir = os.path.dirname(os.path.abspath(args.output))
        if output_dir:
            os.makedirs(output_dir, exist_ok=True)
        with open(args.output, "w") as output_file:
            json.dump(report, output_file, indent=2, sort_keys=True)
        print("best={} output={}".format(report["best"], args.output))
    finally:
        terminate_process(roscore)


if __name__ == "__main__":
    sys.exit(main())
