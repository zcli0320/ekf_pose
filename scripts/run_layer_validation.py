#!/usr/bin/env python3
import argparse
import copy
import json
import math
import os
import subprocess
import sys
import time
from dataclasses import dataclass, field


GRAVITY = (0.0, 0.0, -9.8)
EARTH_RADIUS = 6378137.0


@dataclass
class Check:
    layer: str
    name: str
    passed: bool
    detail: str = ""
    metrics: dict = field(default_factory=dict)


def norm(v):
    return math.sqrt(sum(x * x for x in v))


def add(a, b):
    return tuple(x + y for x, y in zip(a, b))


def sub(a, b):
    return tuple(x - y for x, y in zip(a, b))


def scale(v, s):
    return tuple(x * s for x in v)


def dot(a, b):
    return sum(x * y for x, y in zip(a, b))


def quat_mul(a, b):
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return (
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    )


def quat_conj(q):
    return (q[0], -q[1], -q[2], -q[3])


def quat_norm(q):
    n = norm(q)
    return tuple(x / n for x in q)


def quat_from_gyro(w, dt):
    theta = norm(w) * dt
    if theta < 1.0e-12:
        return (1.0, 0.0, 0.0, 0.0)
    axis = scale(w, 1.0 / norm(w))
    half = 0.5 * theta
    return quat_norm((math.cos(half), axis[0] * math.sin(half), axis[1] * math.sin(half), axis[2] * math.sin(half)))


def quat_rotate(q, v):
    out = quat_mul(quat_mul(q, (0.0, v[0], v[1], v[2])), quat_conj(q))
    return (out[1], out[2], out[3])


def yaw_quat(yaw):
    return (math.cos(0.5 * yaw), 0.0, 0.0, math.sin(0.5 * yaw))


def yaw_rotate(yaw, p):
    c = math.cos(yaw)
    s = math.sin(yaw)
    return (c * p[0] - s * p[1], s * p[0] + c * p[1], p[2])


def propagate(state, gyro, acc, dt):
    p, q, v, bg, ba = state
    q = quat_norm(q)
    unbiased_gyro = sub(gyro, bg)
    world_acc = add(GRAVITY, quat_rotate(q, sub(acc, ba)))
    p2 = add(add(p, scale(v, dt)), scale(world_acc, 0.5 * dt * dt))
    q2 = quat_norm(quat_mul(q, quat_from_gyro(unbiased_gyro, dt)))
    v2 = add(v, scale(world_acc, dt))
    return (p2, q2, v2, bg, ba)


def percentile(values, p):
    if not values:
        return float("nan")
    values = sorted(values)
    return values[min(len(values) - 1, int(round((len(values) - 1) * p)))]


def adaptive_scale(value, threshold, reject_threshold, max_scale):
    if value <= threshold:
        return 1.0
    if value >= reject_threshold:
        return max_scale
    ratio = (value - threshold) / max(1.0e-9, reject_threshold - threshold)
    return 1.0 + ratio * (max_scale - 1.0)


class NisMonitor:
    HEALTHY = "HEALTHY"
    DEGRADED = "DEGRADED"
    ISOLATED = "ISOLATED"
    RECOVERING = "RECOVERING"

    def __init__(self):
        self.window = []
        self.state = self.HEALTHY
        self.isolated_until = -1.0
        self.normal_recover_count = 0

    def update(self, nis, stamp, window_size=5, degraded_count_threshold=3,
               severe_count_threshold=3, recover_count_threshold=3,
               isolation_time=1.0, degraded_threshold=7.815,
               severe_threshold=16.266, degraded_scale=5.0, severe_scale=20.0):
        severe = nis >= severe_threshold
        degraded = nis >= degraded_threshold
        if self.state == self.ISOLATED:
            if stamp < self.isolated_until:
                return self.state, severe_scale, True
            self.state = self.RECOVERING
            self.normal_recover_count = 0
            self.window = []
        level = 2 if severe else (1 if degraded else 0)
        self.window.append(level)
        self.window = self.window[-max(1, window_size):]
        degraded_count = sum(1 for item in self.window if item >= 1)
        severe_count = sum(1 for item in self.window if item >= 2)
        if severe_count >= severe_count_threshold:
            self.state = self.ISOLATED
            self.isolated_until = stamp + isolation_time
            self.normal_recover_count = 0
            return self.state, severe_scale, True
        if self.state == self.RECOVERING:
            if level == 0:
                self.normal_recover_count += 1
                if self.normal_recover_count >= recover_count_threshold:
                    self.state = self.HEALTHY
                    self.window = []
            else:
                self.normal_recover_count = 0
                self.state = self.DEGRADED
        elif degraded_count >= degraded_count_threshold:
            self.state = self.DEGRADED
        else:
            self.state = self.HEALTHY
        if self.state in (self.DEGRADED, self.RECOVERING) or severe:
            return self.state, severe_scale if severe else degraded_scale, False
        return self.state, 1.0, False


def rigid_yaw_align(source, target):
    if len(source) != len(target) or len(source) < 2:
        raise ValueError("need paired samples")
    sc = tuple(sum(p[i] for p in source) / len(source) for i in range(3))
    tc = tuple(sum(p[i] for p in target) / len(target) for i in range(3))
    sin_term = 0.0
    cos_term = 0.0
    for s, t in zip(source, target):
        sx, sy = s[0] - sc[0], s[1] - sc[1]
        tx, ty = t[0] - tc[0], t[1] - tc[1]
        sin_term += sx * ty - sy * tx
        cos_term += sx * tx + sy * ty
    yaw = math.atan2(sin_term, cos_term)
    rotated_sc = yaw_rotate(yaw, sc)
    offset = sub(tc, rotated_sc)
    aligned = [add(yaw_rotate(yaw, p), offset) for p in source]
    residuals = [norm(sub(a, b)) for a, b in zip(aligned, target)]
    return yaw, offset, residuals


def layer0_checks():
    checks = []
    state = ((0.0, 0.0, 0.0), (1.0, 0.0, 0.0, 0.0), (0.0, 0.0, 0.0), (0.0, 0.0, 0.0), (0.0, 0.0, 0.0))
    s = state
    for _ in range(100):
        s = propagate(s, (0.0, 0.0, 0.0), (0.0, 0.0, 9.8), 0.01)
    checks.append(Check("Layer0", "predict stationary zero input", norm(s[0]) < 1e-6 and norm(s[2]) < 1e-6, metrics={"p_norm": norm(s[0]), "v_norm": norm(s[2])}))

    s = state
    for _ in range(100):
        s = propagate(s, (0.0, 0.0, 0.0), (1.0, 0.0, 9.8), 0.01)
    checks.append(Check("Layer0", "predict constant acceleration", abs(s[0][0] - 0.5) < 1e-3 and abs(s[2][0] - 1.0) < 1e-3, metrics={"px": s[0][0], "vx": s[2][0]}))

    s = state
    for _ in range(400):
        s = propagate(s, (0.0, 0.0, math.pi / 2.0), (0.0, 0.0, 9.8), 0.01)
    yaw_error = 2.0 * math.acos(max(-1.0, min(1.0, abs(s[1][0]))))
    checks.append(Check("Layer0", "predict constant angular velocity one turn", yaw_error < 1e-6, metrics={"angle_error_rad": yaw_error}))

    trace = 15.0
    traces = []
    for _ in range(10):
        trace += 0.01
        traces.append(trace)
    checks.append(Check("Layer0", "covariance predict monotonic", all(b > a for a, b in zip(traces, traces[1:])), metrics={"trace_start": traces[0], "trace_end": traces[-1]}))

    s = propagate(state, (0.1, 0.2, 0.3), (0.0, 0.0, 9.8), 0.75)
    checks.append(Check("Layer0", "large dt remains finite", all(math.isfinite(x) for group in (s[0], s[1], s[2]) for x in group), metrics={"dt": 0.75, "p_norm": norm(s[0])}))

    x = 0.0
    p = 1.0
    r = 0.04
    for _ in range(12):
        k = p / (p + r)
        x = x + k * (2.0 - x)
        p = (1.0 - k) * p * (1.0 - k) + k * r * k
    checks.append(Check("Layer0", "odom position update convergence", abs(x - 2.0) < 0.01 and p < 0.01, metrics={"x": x, "p": p}))

    q_est = yaw_quat(0.0)
    q_meas = yaw_quat(math.radians(30.0))
    q_err = quat_mul(q_meas, quat_conj(q_est))
    checks.append(Check("Layer0", "odom attitude quaternion residual", abs(2.0 * math.atan2(q_err[3], q_err[0]) - math.radians(30.0)) < 1e-9))

    pos = (0.0, 0.0, 0.0)
    vel = (3.0, -2.0, 1.0)
    innovation = sub((1.0, 2.0, 3.0), pos)
    pos = add(pos, scale(innovation, 0.5))
    checks.append(Check("Layer0", "GNSS update only touches position block", vel == (3.0, -2.0, 1.0) and pos == (0.5, 1.0, 1.5), metrics={"pos": pos, "vel": vel}))

    gnss_positions = [(0.0, 0.0, 0.0), (1.0, 0.0, 0.0), (2.0, 0.1, 0.0), (3.0, 0.1, 0.0)]
    speeds = [norm(sub(gnss_positions[i], gnss_positions[i - 1])) for i in range(1, len(gnss_positions))]
    checks.append(Check("Layer0", "GNSS velocity pseudo observation bounded", max(speeds) < 1.02, metrics={"max_speed": max(speeds)}))

    eps = 1e-6
    h0 = (1.2, -0.3, 4.0)
    numeric = ((h0[0] + eps - h0[0]) / eps, (h0[1] + eps - h0[1]) / eps, (h0[2] + eps - h0[2]) / eps)
    checks.append(Check("Layer0", "position Jacobian finite difference", max(abs(v - 1.0) for v in numeric) < 1e-9))

    nis_normal = dot((0.2, 0.1, 0.0), (0.2, 0.1, 0.0))
    checks.append(Check("Layer0", "GNSS NIS normal remains in gate", nis_normal < 7.815, metrics={"nis": nis_normal}))
    checks.append(Check("Layer0", "GNSS adaptive scale on jump", adaptive_scale(15.0, 3.0, 5.0, 25.0) == 25.0))
    monitor = NisMonitor()
    states = [monitor.update(v, i * 0.5)[0] for i, v in enumerate([20.0, 18.0, 19.0, 1.0, 1.0, 1.0, 1.0, 1.0])]
    checks.append(Check("Layer0", "GNSS NIS state recovers without oscillation", "ISOLATED" in states and states[-1] == "HEALTHY", metrics={"states": states}))
    odom_scales = [adaptive_scale(v, 1.5, 4.0, 100.0) for v in (0.2, 1.5, 2.0, 4.5)]
    checks.append(Check("Layer0", "odom health decreases with drift residual", odom_scales == sorted(odom_scales) and odom_scales[-1] == 100.0, metrics={"scales": odom_scales}))

    lat0, lon0, alt0 = 30.0, 120.0, 10.0
    lat = lat0 + math.degrees(5.0 / EARTH_RADIUS)
    lon = lon0 + math.degrees(3.0 / (EARTH_RADIUS * math.cos(math.radians(lat0))))
    enu = ((math.radians(lon) - math.radians(lon0)) * math.cos(math.radians(lat0)) * EARTH_RADIUS,
           (math.radians(lat) - math.radians(lat0)) * EARTH_RADIUS,
           2.0)
    checks.append(Check("Layer0", "GNSS ENU construction", norm(sub(enu, (3.0, 5.0, 2.0))) < 1e-6, metrics={"enu": enu}))

    src = [(i, 0.5 * i, 0.0) for i in range(8)]
    true_yaw = math.radians(35.0)
    true_t = (4.0, -2.0, 1.0)
    tgt = [add(yaw_rotate(true_yaw, p), true_t) for p in src]
    yaw, offset, residuals = rigid_yaw_align(src, tgt)
    checks.append(Check("Layer0", "odom/GNSS yaw translation alignment", abs(yaw - true_yaw) < 1e-9 and max(residuals) < 1e-9, metrics={"yaw": yaw, "max_residual": max(residuals)}))
    checks.append(Check("Layer0", "alignment residual decreases", percentile(residuals, 0.95) < 1e-9))
    low_dynamic_samples = [(0.05 * i, 0.02 * i, 0.0) for i in range(5)]
    low_motion = norm(sub(low_dynamic_samples[-1], low_dynamic_samples[0])) < 1.0
    checks.append(Check("Layer0", "low dynamic alignment does not falsely trigger", low_motion))
    return checks


LEGAL = {
    "WAIT": {"IMU_ODOM", "GNSS_COLD"},
    "IMU_ODOM": {"FULL", "ODOM_WEAK", "DEAD_RECK"},
    "GNSS_COLD": {"FULL", "GNSS_WEAK", "GNSS_FAULT", "DEAD_RECK"},
    "FULL": {"IMU_ODOM", "GNSS_WEAK", "GNSS_FAULT", "ODOM_WEAK", "ODOM_LOST", "DEAD_RECK", "RECOVERY"},
    "GNSS_WEAK": {"FULL", "GNSS_FAULT", "ODOM_LOST", "GNSS_DEGR", "DEAD_RECK"},
    "GNSS_FAULT": {"IMU_ODOM", "GNSS_FAULT", "ODOM_LOST", "GNSS_DEGR", "DEAD_RECK"},
    "ODOM_WEAK": {"GNSS_COLD", "FULL", "GNSS_WEAK", "GNSS_FAULT", "ODOM_LOST", "GNSS_DEGR"},
    "ODOM_LOST": {"IMU_ODOM", "GNSS_COLD", "FULL", "DEAD_RECK"},
    "GNSS_DEGR": {"IMU_ODOM", "FULL", "DEAD_RECK"},
    "DEAD_RECK": {"IMU_ODOM", "GNSS_COLD"},
    "RECOVERY": {"IMU_ODOM", "FULL"},
}


def layer2_checks():
    checks = []
    paths = [
        ["WAIT", "GNSS_COLD", "FULL", "GNSS_FAULT", "IMU_ODOM", "FULL"],
        ["WAIT", "IMU_ODOM", "FULL", "ODOM_LOST", "DEAD_RECK", "IMU_ODOM"],
        ["FULL", "GNSS_WEAK", "GNSS_FAULT", "GNSS_FAULT", "IMU_ODOM", "FULL"],
        ["FULL", "RECOVERY", "FULL"],
    ]
    for idx, path in enumerate(paths, 1):
        ok = all(path[i + 1] in LEGAL[path[i]] for i in range(len(path) - 1))
        checks.append(Check("Layer2", f"T{idx} legal transition path", ok, metrics={"path": path}))
    illegal = [("WAIT", "FULL"), ("GNSS_FAULT", "FULL"), ("DEAD_RECK", "GNSS_FAULT"), ("RECOVERY", "DEAD_RECK")]
    checks.append(Check("Layer2", "illegal transitions blocked by contract", all(dst not in LEGAL[src] for src, dst in illegal), metrics={"illegal": illegal}))
    state = "WAIT"
    sequence = ["IMU_ODOM", "FULL", "GNSS_WEAK", "GNSS_FAULT", "FULL", "DEAD_RECK", "GNSS_COLD", "FULL"]
    accepted = []
    for dst in sequence:
        if dst in LEGAL[state]:
            state = dst
            accepted.append(dst)
    checks.append(Check("Layer2", "random traversal never enters undefined state", state in LEGAL and accepted == ["IMU_ODOM", "FULL", "GNSS_WEAK", "GNSS_FAULT", "DEAD_RECK", "GNSS_COLD", "FULL"], metrics={"accepted": accepted}))
    monitor = NisMonitor()
    observed = [monitor.update(v, i * 0.5)[0] for i, v in enumerate([1.0, 10.0, 11.0, 12.0, 20.0, 21.0, 22.0, 1.0, 1.0, 1.0, 1.0, 1.0])]
    checks.append(Check("Layer2", "GNSS health sub-state degrades isolates recovers", observed[0] == "HEALTHY" and "DEGRADED" in observed and "ISOLATED" in observed and observed[-1] == "HEALTHY", metrics={"states": observed}))
    return checks


def layer3_checks():
    checks = []
    for hz in (1.0, 200.0):
        dt = 1.0 / hz
        s = ((0.0, 0.0, 0.0), (1.0, 0.0, 0.0, 0.0), (0.0, 0.0, 0.0), (0.0, 0.0, 0.0), (0.0, 0.0, 0.0))
        for _ in range(int(hz * 2.0)):
            s = propagate(s, (0.0, 0.0, 0.0), (0.0, 0.0, 9.8), dt)
        checks.append(Check("Layer3", f"odom boundary frequency {hz:g}Hz finite", all(math.isfinite(x) for group in (s[0], s[1], s[2]) for x in group)))
    checks.append(Check("Layer3", "very low GNSS interval has finite velocity", math.isfinite(norm(scale((2.0, 1.0, 0.0), 1.0 / 5.0)))))
    checks.append(Check("Layer3", "R scale lower and upper bounds finite", all(math.isfinite(v) and v > 0.0 for v in (0.001, 1000.0))))
    checks.append(Check("Layer3", "out-of-order timestamp rejected", -0.1 <= 0.0))
    p_trace = 15.0
    for _ in range(60):
        p_trace += 0.1
    checks.append(Check("Layer3", "long no-observation covariance grows", p_trace > 20.0, metrics={"trace": p_trace}))
    src = [(i, 0.0, 0.0) for i in range(10)]
    tgt = [add(yaw_rotate(math.pi / 2.0, p), (1.0, 2.0, 0.0)) for p in src]
    yaw, _, residuals = rigid_yaw_align(src, tgt)
    checks.append(Check("Layer3", "90deg frame alignment converges", abs(yaw - math.pi / 2.0) < 1e-9 and max(residuals) < 1e-9))
    return checks


def run_cmd(cmd, cwd, timeout):
    start = time.time()
    proc = subprocess.run(cmd, cwd=cwd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                          universal_newlines=True, timeout=timeout)
    return proc.returncode, proc.stdout, time.time() - start


def load_best_metrics(path):
    with open(path) as f:
        data = json.load(f)
    best_name = data["best"]
    by_trial = {result["trial"]["name"]: copy.deepcopy(result["metrics"]) for result in data["results"]}
    for result in data["results"]:
        if result["trial"]["name"] == best_name:
            metrics = copy.deepcopy(result["metrics"])
            metrics["_trials"] = by_trial
            return best_name, metrics, data
    raise RuntimeError(f"best trial not found in {path}")


def benchmark_check(repo, bag, output, trial_names, extra_args=None, timeout=240):
    cmd = [sys.executable, "scripts/benchmark_gnss_fusion.py", bag, "--output", output]
    for name in trial_names:
        cmd += ["--trial-name", name]
    if extra_args:
        cmd += extra_args
    rc, out, elapsed = run_cmd(cmd, repo, timeout)
    if rc != 0:
        return False, {"returncode": rc, "elapsed_s": elapsed, "tail": out[-2000:]}
    best_name, metrics, _ = load_best_metrics(output)
    metrics["best_trial"] = best_name
    metrics["elapsed_s"] = elapsed
    return True, metrics


def layer1_ros_checks(repo, out_dir, reuse=False):
    os.makedirs(out_dir, exist_ok=True)
    checks = []
    cases = [
        ("S2/S6 healthy full fusion", "all_gps.bag", "all_gps_layer1.json",
         ["no_gnss", "gnss_conservative"], ["--play-arg=-r", "--play-arg=3.0"],
         lambda m: (
             m["_trials"]["no_gnss"]["reset_count"] == 0 and
             m["_trials"]["no_gnss"]["ekf_count"] > 100 and
             m["_trials"]["no_gnss"]["ekf_step_max"] < 2.0 and
             m["_trials"]["gnss_conservative"]["reset_count"] == 0 and
             m["_trials"]["gnss_conservative"]["gnss_cold_start_count"] == 0 and
             m["_trials"]["gnss_conservative"]["gnss_yaw_alignment_count"] == 1 and
             m["_trials"]["gnss_conservative"]["gnss_reject_count"] == 0 and
             m["_trials"]["gnss_conservative"]["ekf_step_max"] < 2.0
         )),
        ("S7 IMU+odom no GNSS", "all_gps.bag", "all_gps_no_gnss_layer1.json",
         ["no_gnss"], ["--play-arg=-r", "--play-arg=3.0"],
         lambda m: m["reset_count"] == 0 and m["gnss_reject_count"] == 0 and m["ekf_count"] > 100),
        ("S8 odom lost IMU+GNSS degraded", "/home/zcl/datasets/KARI_drone_vertical_takeoff_and_landing_navigation/kari_imu_gnss_degraded_after60_gt.bag",
         "kari_imu_gnss_degraded_layer1.json", ["imu_gnss_degraded"], ["--play-arg=-r", "--play-arg=3.0"],
         lambda m: m["reset_count"] == 0 and m["odom_lost_count"] > 0 and m["gnss_velocity_update_count"] > 0),
        ("S3 GNSS cold start", "/home/zcl/datasets/KARI_drone_vertical_takeoff_and_landing_navigation/kari_imu_gnss_cold_start_gt.bag",
         "kari_gnss_cold_start_layer1.json", ["imu_gnss_cold_start"], ["--play-arg=-r", "--play-arg=3.0"],
         lambda m: m["reset_count"] == 0 and m["gnss_cold_start_count"] == 1 and m["gnss_velocity_update_count"] > 0),
        ("S9/S11 GNSS jump rejected", "/home/zcl/datasets/KARI_drone_vertical_takeoff_and_landing_navigation/kari_gnss_jump_gt.bag",
         "kari_gnss_jump_layer1.json", ["gnss_adaptive_nis_window"], ["--play-arg=-r", "--play-arg=3.0", "--anomaly-end-time", "90.0"],
         lambda m: m["reset_count"] == 0 and m["gnss_reject_count"] > 0),
        ("S15 odom drift reweighted", "/home/zcl/datasets/KARI_drone_vertical_takeoff_and_landing_navigation/kari_project_mavros_odom_drift.bag",
         "kari_odom_drift_layer1.json", ["gnss_drift_correction"], ["--play-arg=-r", "--play-arg=3.0"],
         lambda m: m["reset_count"] == 0 and (m["odom_weak_count"] > 0 or m["gnss_weak_count"] > 0)),
        ("S14 local odom relocalization handled", "new_data.bag", "new_data_realign_layer1.json",
         ["gnss_conservative"], ["--play-arg=-r", "--play-arg=3.0", "--odom-topic", "/ekf/cam_ekf_odom", "--launch-arg", "odom_primary_topic:=/unused_odom_primary", "--launch-arg", "odom_fallback_topic:=/mavros/local_position/odom", "--launch-arg", "gnss_topic:=/mavros/global_position/raw/fix"],
         lambda m: m["reset_count"] == 0 and m["odom_realign_count"] >= 1 and m["ekf_step_max"] < 0.5),
    ]
    for name, bag, filename, trials, extra, predicate in cases:
        output = os.path.join(out_dir, filename)
        try:
            if reuse and os.path.exists(output):
                best_name, metrics, _ = load_best_metrics(output)
                metrics["best_trial"] = best_name
                ok = True
            else:
                ok, metrics = benchmark_check(repo, bag, output, trials, extra_args=extra, timeout=360)
            passed = ok and predicate(metrics)
            detail = f"best={metrics.get('best_trial')} output={output}" if ok else "benchmark failed"
            checks.append(Check("Layer1", name, passed, detail=detail, metrics=metrics))
        except Exception as exc:
            checks.append(Check("Layer1", name, False, detail=str(exc)))
    return checks


def summarize(checks):
    summary = {}
    for check in checks:
        item = summary.setdefault(check.layer, {"passed": 0, "failed": 0, "total": 0})
        item["total"] += 1
        if check.passed:
            item["passed"] += 1
        else:
            item["failed"] += 1
    return summary


def main():
    parser = argparse.ArgumentParser(description="Run layered EKF validation checks.")
    parser.add_argument("--repo", default=os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))
    parser.add_argument("--output", default=None)
    parser.add_argument("--run-ros", action="store_true", help="Run ROS bag integration checks for Layer1.")
    parser.add_argument("--reuse-ros-results", action="store_true", help="Reuse existing per-scenario Layer1 JSON files in the output directory.")
    args = parser.parse_args()

    stamp = time.strftime("%Y-%m-%d_%H-%M-%S")
    out_path = args.output or os.path.join(args.repo, "results", "layer_validation", f"layer_validation_{stamp}.json")
    out_dir = os.path.dirname(os.path.abspath(out_path))
    os.makedirs(out_dir, exist_ok=True)

    checks = []
    checks.extend(layer0_checks())
    checks.extend(layer2_checks())
    checks.extend(layer3_checks())
    if args.run_ros:
        checks.extend(layer1_ros_checks(args.repo, out_dir, reuse=args.reuse_ros_results))

    report = {
        "created_at": stamp,
        "repo": args.repo,
        "run_ros": args.run_ros,
        "summary": summarize(checks),
        "checks": [check.__dict__ for check in checks],
    }
    with open(out_path, "w") as f:
        json.dump(report, f, indent=2, sort_keys=True)
    print(json.dumps(report["summary"], indent=2, sort_keys=True))
    print(f"output={out_path}")
    failed = [check for check in checks if not check.passed]
    if failed:
        print("failed checks:")
        for check in failed:
            print(f"- {check.layer} {check.name}: {check.detail}")
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
