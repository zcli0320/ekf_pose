#!/usr/bin/env python3
import math
import signal
import threading

import rospy
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import NavSatFix


EARTH_RADIUS = 6378137.0


class FusionEvaluator:
    def __init__(self):
        self.lock = threading.Lock()
        self.ekf = []
        self.odom = []
        self.gnss = []
        self.node_gnss_path = []
        self.ground_truth = []
        self.gnss_origin = None
        self.gnss_alignment = None
        self.ekf_topic = rospy.get_param("~ekf_topic", "/ekf/ekf_odom")
        self.odom_topic = rospy.get_param("~odom_topic", "/mavros/odometry/in")
        self.gnss_topic = rospy.get_param("~gnss_topic", "/mavros/global_position/global")
        self.ground_truth_topic = rospy.get_param("~ground_truth_topic", "/ground_truth/odom")
        self.aligned_gnss_path_topic = rospy.get_param("~aligned_gnss_path_topic", "/ekf/gnss_path")
        self.max_odom_dt = rospy.get_param("~max_odom_dt", 0.04)
        self.max_gnss_dt = rospy.get_param("~max_gnss_dt", 0.10)
        self.max_ground_truth_dt = rospy.get_param("~max_ground_truth_dt", 0.10)
        self.anomaly_end_time = rospy.get_param("~anomaly_end_time", -1.0)
        self.recovery_error_threshold = rospy.get_param("~recovery_error_threshold", 0.5)
        self.recovery_hold_samples = rospy.get_param("~recovery_hold_samples", 5)

        rospy.Subscriber(self.ekf_topic, Odometry, self.ekf_cb, queue_size=200)
        rospy.Subscriber(self.odom_topic, Odometry, self.odom_cb, queue_size=200)
        rospy.Subscriber(self.gnss_topic, NavSatFix, self.gnss_cb, queue_size=200)
        rospy.Subscriber(self.ground_truth_topic, Odometry, self.ground_truth_cb, queue_size=200)
        rospy.Subscriber(self.aligned_gnss_path_topic, Path, self.node_gnss_path_cb, queue_size=20)

    def ekf_cb(self, msg):
        self._append_pose(self.ekf, msg.header.stamp.to_sec(), msg.pose.pose.position)

    def odom_cb(self, msg):
        self._append_pose(self.odom, msg.header.stamp.to_sec(), msg.pose.pose.position)

    def ground_truth_cb(self, msg):
        self._append_pose(self.ground_truth, msg.header.stamp.to_sec(), msg.pose.pose.position)

    def gnss_cb(self, msg):
        if msg.status.status < 0:
            return
        if not all(math.isfinite(v) for v in (msg.latitude, msg.longitude, msg.altitude)):
            return
        lat = math.radians(msg.latitude)
        lon = math.radians(msg.longitude)
        if self.gnss_origin is None:
            self.gnss_origin = (lat, lon, msg.altitude, math.cos(lat))
        lat0, lon0, alt0, cos_lat0 = self.gnss_origin
        x = (lon - lon0) * cos_lat0 * EARTH_RADIUS
        y = (lat - lat0) * EARTH_RADIUS
        z = msg.altitude - alt0
        with self.lock:
            self.gnss.append((msg.header.stamp.to_sec(), x, y, z))

    def node_gnss_path_cb(self, msg):
        if not msg.poses:
            return
        pose = msg.poses[-1]
        stamp = pose.header.stamp.to_sec()
        position = pose.pose.position
        with self.lock:
            if self.node_gnss_path and abs(self.node_gnss_path[-1][0] - stamp) < 1.0e-9:
                self.node_gnss_path[-1] = (stamp, position.x, position.y, position.z)
            else:
                self.node_gnss_path.append((stamp, position.x, position.y, position.z))

    def _append_pose(self, seq, stamp, position):
        with self.lock:
            seq.append((stamp, position.x, position.y, position.z))

    @staticmethod
    def nearest(seq, stamp, max_dt=0.04):
        if not seq:
            return None
        best = min(seq, key=lambda item: abs(item[0] - stamp))
        if abs(best[0] - stamp) > max_dt:
            return None
        return best

    @staticmethod
    def percentile(values, p):
        if not values:
            return float("nan")
        values = sorted(values)
        index = min(len(values) - 1, int(round((len(values) - 1) * p)))
        return values[index]

    @staticmethod
    def distance(a, b):
        return math.sqrt((a[1] - b[1]) ** 2 + (a[2] - b[2]) ** 2 + (a[3] - b[3]) ** 2)

    @staticmethod
    def step_stats(seq):
        steps = [FusionEvaluator.distance(seq[i - 1], seq[i]) for i in range(1, len(seq))]
        if not steps:
            return 0.0, 0.0
        return max(steps), FusionEvaluator.percentile(steps, 0.95)

    def paired_error(self, reference, estimate, max_dt):
        errors = []
        for est in estimate:
            ref = self.nearest(reference, est[0], max_dt)
            if ref is not None:
                errors.append(self.distance(ref, est))
        return errors

    def recovery_time(self, reference, estimate, max_dt):
        if self.anomaly_end_time < 0.0 or not reference or not estimate:
            return float("nan")
        anomaly_end = self.anomaly_end_time
        if anomaly_end < estimate[0][0]:
            anomaly_end = estimate[0][0] + anomaly_end
        hold = max(1, int(self.recovery_hold_samples))
        streak = 0
        first_good_time = None
        for est in estimate:
            if est[0] < anomaly_end:
                continue
            ref = self.nearest(reference, est[0], max_dt)
            if ref is None:
                continue
            if self.distance(ref, est) <= self.recovery_error_threshold:
                if streak == 0:
                    first_good_time = est[0]
                streak += 1
                if streak >= hold:
                    return max(0.0, first_good_time - anomaly_end)
            else:
                streak = 0
                first_good_time = None
        return float("nan")

    def aligned_gnss(self):
        if not self.gnss or not self.ekf:
            return []
        first_gnss = None
        first_ekf = None
        for gnss in self.gnss:
            ekf = self.nearest(self.ekf, gnss[0], max_dt=self.max_gnss_dt)
            if ekf is not None:
                first_gnss = gnss
                first_ekf = ekf
                break
        if first_gnss is None:
            return []
        offset = (
            0.0,
            first_ekf[1] - first_gnss[1],
            first_ekf[2] - first_gnss[2],
            first_ekf[3] - first_gnss[3],
        )
        return [(t, x + offset[1], y + offset[2], z + offset[3]) for t, x, y, z in self.gnss]

    def rigid_aligned_gnss(self):
        return self.rigid_align(self.gnss, self.ekf, self.max_gnss_dt)

    def rigid_aligned_gnss_to_ground_truth(self):
        return self.rigid_align(self.gnss, self.ground_truth, self.max_ground_truth_dt)

    def rigid_align(self, source, target, max_dt):
        pairs = []
        for item in source:
            ref = self.nearest(target, item[0], max_dt=max_dt)
            if ref is not None:
                pairs.append((item, ref))
        if len(pairs) < 5:
            return [], float("nan")

        source_center = [0.0, 0.0, 0.0]
        target_center = [0.0, 0.0, 0.0]
        for item, ref in pairs:
            for i in range(3):
                source_center[i] += item[i + 1]
                target_center[i] += ref[i + 1]
        count = float(len(pairs))
        source_center = [v / count for v in source_center]
        target_center = [v / count for v in target_center]

        sin_term = 0.0
        cos_term = 0.0
        for item, ref in pairs:
            sx = item[1] - source_center[0]
            sy = item[2] - source_center[1]
            tx = ref[1] - target_center[0]
            ty = ref[2] - target_center[1]
            sin_term += sx * ty - sy * tx
            cos_term += sx * tx + sy * ty
        yaw = math.atan2(sin_term, cos_term)
        c = math.cos(yaw)
        s = math.sin(yaw)
        tx = target_center[0] - (c * source_center[0] - s * source_center[1])
        ty = target_center[1] - (s * source_center[0] + c * source_center[1])
        tz = target_center[2] - source_center[2]
        aligned = []
        for t, x, y, z in source:
            aligned.append((t, c * x - s * y + tx, s * x + c * y + ty, z + tz))
        return aligned, yaw

    def report(self):
        with self.lock:
            ekf = list(self.ekf)
            odom = list(self.odom)
            gnss = list(self.gnss)
            node_gnss_path = list(self.node_gnss_path)
            ground_truth = list(self.ground_truth)
        metrics = self.compute_metrics(ekf, odom, gnss, node_gnss_path, ground_truth)
        self.ekf = ekf
        self.odom = odom
        self.gnss = gnss
        self.node_gnss_path = node_gnss_path
        self.ground_truth = ground_truth

        print("samples: ekf={} odom={} gnss={} ground_truth={}".format(
            metrics["ekf_count"],
            metrics["odom_count"],
            metrics["gnss_count"],
            metrics["ground_truth_count"],
        ))
        print(self.format_line("ekf_vs_odom", metrics["ekf_vs_odom"]))
        print(self.format_line("ekf_vs_ground_truth", metrics["ekf_vs_ground_truth"]))
        print(self.format_line("ekf_vs_aligned_gnss", metrics["ekf_vs_aligned_gnss"]))
        print("ekf_step: max={:.4f}m p95={:.4f}m".format(metrics["ekf_step_max"], metrics["ekf_step_p95"]))

    def reset(self):
        with self.lock:
            self.ekf = []
            self.odom = []
            self.gnss = []
            self.node_gnss_path = []
            self.ground_truth = []
            self.gnss_origin = None
            self.gnss_alignment = None

    def snapshot_metrics(self):
        with self.lock:
            ekf = list(self.ekf)
            odom = list(self.odom)
            gnss = list(self.gnss)
            node_gnss_path = list(self.node_gnss_path)
            ground_truth = list(self.ground_truth)
        return self.compute_metrics(ekf, odom, gnss, node_gnss_path, ground_truth)

    @staticmethod
    def summarize(values):
        if not values:
            return {"count": 0, "mean": float("nan"), "p95": float("nan"), "max": float("nan")}
        return {
            "count": len(values),
            "mean": sum(values) / len(values),
            "p95": FusionEvaluator.percentile(values, 0.95),
            "max": max(values),
        }

    @staticmethod
    def format_line(name, stats):
        if stats["count"] == 0:
            return "{}: count=0".format(name)
        return "{}: count={} mean={:.4f}m p95={:.4f}m max={:.4f}m".format(
            name,
            stats["count"],
            stats["mean"],
            stats["p95"],
            stats["max"],
        )

    def compute_metrics(self, ekf, odom, gnss, node_gnss_path=None, ground_truth=None):
        if node_gnss_path is None:
            node_gnss_path = []
        if ground_truth is None:
            ground_truth = []
        self.ekf = ekf
        self.odom = odom
        self.gnss = gnss
        self.node_gnss_path = node_gnss_path
        self.ground_truth = ground_truth
        odom_errors = self.paired_error(odom, ekf, self.max_odom_dt)
        ekf_ground_truth_errors = self.paired_error(ground_truth, ekf, self.max_ground_truth_dt)
        odom_ground_truth_errors = self.paired_error(ground_truth, odom, self.max_ground_truth_dt)
        gnss_aligned = self.aligned_gnss()
        gnss_errors = self.paired_error(gnss_aligned, ekf, self.max_gnss_dt)
        rigid_gnss, rigid_yaw = self.rigid_aligned_gnss()
        rigid_gnss_errors = self.paired_error(rigid_gnss, ekf, self.max_gnss_dt)
        rigid_gnss_ground_truth, rigid_ground_truth_yaw = self.rigid_aligned_gnss_to_ground_truth()
        gnss_ground_truth_errors = self.paired_error(rigid_gnss_ground_truth, ground_truth, self.max_ground_truth_dt)
        node_gnss_errors = self.paired_error(node_gnss_path, ekf, self.max_gnss_dt)
        max_step, p95_step = self.step_stats(ekf)
        recovery_reference = ground_truth if ground_truth else odom
        recovery_dt = self.max_ground_truth_dt if ground_truth else self.max_odom_dt
        return {
            "ekf_count": len(ekf),
            "odom_count": len(odom),
            "gnss_count": len(gnss),
            "node_gnss_path_count": len(node_gnss_path),
            "ground_truth_count": len(ground_truth),
            "ekf_vs_odom": self.summarize(odom_errors),
            "ekf_vs_ground_truth": self.summarize(ekf_ground_truth_errors),
            "odom_vs_ground_truth": self.summarize(odom_ground_truth_errors),
            "gps_vs_ground_truth": self.summarize(gnss_ground_truth_errors),
            "ekf_vs_aligned_gnss": self.summarize(gnss_errors),
            "ekf_vs_rigid_gnss": self.summarize(rigid_gnss_errors),
            "ekf_vs_node_gnss_path": self.summarize(node_gnss_errors),
            "gnss_rigid_yaw_rad": rigid_yaw,
            "gnss_ground_truth_yaw_rad": rigid_ground_truth_yaw,
            "ekf_step_max": max_step,
            "ekf_step_p95": p95_step,
            "recovery_time_s": self.recovery_time(recovery_reference, ekf, recovery_dt),
        }


if __name__ == "__main__":
    rospy.init_node("evaluate_gnss_fusion", anonymous=True)
    evaluator = FusionEvaluator()
    signal.signal(signal.SIGTERM, lambda signum, frame: rospy.signal_shutdown("timeout"))
    rospy.on_shutdown(evaluator.report)
    rospy.spin()
