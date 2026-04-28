#!/usr/bin/env python3
import math
import signal
import threading

import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix


EARTH_RADIUS = 6378137.0


class FusionEvaluator:
    def __init__(self):
        self.lock = threading.Lock()
        self.ekf = []
        self.odom = []
        self.gnss = []
        self.gnss_origin = None
        self.gnss_alignment = None

        rospy.Subscriber("/ekf/ekf_odom", Odometry, self.ekf_cb, queue_size=200)
        rospy.Subscriber("/mavros/odometry/in", Odometry, self.odom_cb, queue_size=200)
        rospy.Subscriber("/mavros/global_position/global", NavSatFix, self.gnss_cb, queue_size=200)

    def ekf_cb(self, msg):
        self._append_pose(self.ekf, msg.header.stamp.to_sec(), msg.pose.pose.position)

    def odom_cb(self, msg):
        self._append_pose(self.odom, msg.header.stamp.to_sec(), msg.pose.pose.position)

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

    def paired_error(self, reference, estimate, max_dt=0.04):
        errors = []
        for est in estimate:
            ref = self.nearest(reference, est[0], max_dt)
            if ref is not None:
                errors.append(self.distance(ref, est))
        return errors

    def aligned_gnss(self):
        if not self.gnss or not self.ekf:
            return []
        first_gnss = None
        first_ekf = None
        for gnss in self.gnss:
            ekf = self.nearest(self.ekf, gnss[0], max_dt=0.10)
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

    def report(self):
        with self.lock:
            ekf = list(self.ekf)
            odom = list(self.odom)
            gnss = list(self.gnss)
        self.ekf = ekf
        self.odom = odom
        self.gnss = gnss

        odom_errors = self.paired_error(odom, ekf)
        gnss_aligned = self.aligned_gnss()
        gnss_errors = self.paired_error(gnss_aligned, ekf, max_dt=0.10)
        max_step, p95_step = self.step_stats(ekf)

        def line(name, values):
            if not values:
                return "{}: count=0".format(name)
            return "{}: count={} mean={:.4f}m p95={:.4f}m max={:.4f}m".format(
                name,
                len(values),
                sum(values) / len(values),
                self.percentile(values, 0.95),
                max(values),
            )

        print("samples: ekf={} odom={} gnss={}".format(len(ekf), len(odom), len(gnss)))
        print(line("ekf_vs_odom", odom_errors))
        print(line("ekf_vs_aligned_gnss", gnss_errors))
        print("ekf_step: max={:.4f}m p95={:.4f}m".format(max_step, p95_step))


if __name__ == "__main__":
    rospy.init_node("evaluate_gnss_fusion", anonymous=True)
    evaluator = FusionEvaluator()
    signal.signal(signal.SIGTERM, lambda signum, frame: rospy.signal_shutdown("timeout"))
    rospy.on_shutdown(evaluator.report)
    rospy.spin()
