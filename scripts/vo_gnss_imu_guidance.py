#!/usr/bin/env python3
import copy
import json
import math
from collections import deque
from dataclasses import dataclass

import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, NavSatFix, NavSatStatus
from std_msgs.msg import String


EARTH_RADIUS_METERS = 6378137.0


@dataclass
class PoseSample:
    stamp: float
    position: tuple
    orientation: tuple = (1.0, 0.0, 0.0, 0.0)
    linear_velocity: tuple = (0.0, 0.0, 0.0)
    angular_velocity: tuple = (0.0, 0.0, 0.0)


@dataclass
class Transform2D:
    scale: float
    yaw: float
    translation: tuple
    residual_mean: float
    residual_max: float
    average_speed: float


def rotate_xy(point, yaw):
    c = math.cos(yaw)
    s = math.sin(yaw)
    return (c * point[0] - s * point[1], s * point[0] + c * point[1])


def quat_multiply(a, b):
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return (
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    )


def quat_normalize(q):
    n = math.sqrt(sum(v * v for v in q))
    if n <= 1.0e-12:
        return (1.0, 0.0, 0.0, 0.0)
    return tuple(v / n for v in q)


def yaw_quat(yaw):
    half = 0.5 * yaw
    return (math.cos(half), 0.0, 0.0, math.sin(half))


def fit_similarity_2d(source_points, target_points):
    if len(source_points) != len(target_points) or len(source_points) < 2:
        return None

    n = float(len(source_points))
    src_c = (
        sum(p[0] for p in source_points) / n,
        sum(p[1] for p in source_points) / n,
    )
    tgt_c = (
        sum(p[0] for p in target_points) / n,
        sum(p[1] for p in target_points) / n,
    )

    denom = 0.0
    cos_term = 0.0
    sin_term = 0.0
    for src, tgt in zip(source_points, target_points):
        sx = src[0] - src_c[0]
        sy = src[1] - src_c[1]
        tx = tgt[0] - tgt_c[0]
        ty = tgt[1] - tgt_c[1]
        denom += sx * sx + sy * sy
        cos_term += sx * tx + sy * ty
        sin_term += sx * ty - sy * tx

    if denom <= 1.0e-9:
        return None

    scale_cos = cos_term / denom
    scale_sin = sin_term / denom
    scale = math.hypot(scale_cos, scale_sin)
    yaw = math.atan2(scale_sin, scale_cos)
    src_rot = rotate_xy(src_c, yaw)
    translation = (
        tgt_c[0] - scale * src_rot[0],
        tgt_c[1] - scale * src_rot[1],
    )

    residuals = []
    for src, tgt in zip(source_points, target_points):
        r = rotate_xy(src, yaw)
        pred = (scale * r[0] + translation[0], scale * r[1] + translation[1])
        residuals.append(math.hypot(pred[0] - tgt[0], pred[1] - tgt[1]))

    return scale, yaw, translation, residuals


class Similarity2DGuidanceCore:
    def __init__(self,
                 min_pairs=8,
                 max_pairs=40,
                 max_pair_dt=0.08,
                 min_motion=10.0,
                 min_speed=5.0,
                 max_vertical_motion=2.0,
                 uniform_speed_max_cv=0.35,
                 min_scale=0.05,
                 max_scale=20.0,
                 max_residual=1.0,
                 scale_stability=0.05,
                 yaw_stability=0.05,
                 ready_frames=4,
                 imu_timeout=0.5,
                 require_imu=True):
        self.min_pairs = int(min_pairs)
        self.max_pairs = int(max_pairs)
        self.max_pair_dt = float(max_pair_dt)
        self.min_motion = float(min_motion)
        self.min_speed = float(min_speed)
        self.max_vertical_motion = float(max_vertical_motion)
        self.uniform_speed_max_cv = float(uniform_speed_max_cv)
        self.min_scale = float(min_scale)
        self.max_scale = float(max_scale)
        self.max_residual = float(max_residual)
        self.scale_stability = float(scale_stability)
        self.yaw_stability = float(yaw_stability)
        self.ready_frames = int(ready_frames)
        self.imu_timeout = float(imu_timeout)
        self.require_imu = bool(require_imu)

        self.odom_buffer = deque(maxlen=400)
        self.pairs = deque(maxlen=self.max_pairs)
        self.last_imu_stamp = None
        self.transform = None
        self.ready_count = 0
        self.rejected_reason = "waiting"

    @property
    def ready(self):
        return self.transform is not None and self.ready_count >= self.ready_frames

    def add_imu(self, stamp):
        self.last_imu_stamp = float(stamp)

    def add_odom(self, sample):
        self.odom_buffer.append(sample)

    def add_reference(self, stamp, reference_position):
        stamp = float(stamp)
        if self.require_imu and (self.last_imu_stamp is None or abs(stamp - self.last_imu_stamp) > self.imu_timeout):
            self.rejected_reason = "no_recent_imu"
            return False

        odom = self._nearest_odom(stamp)
        if odom is None:
            self.rejected_reason = "no_synced_vo"
            return False

        self.pairs.append((odom, PoseSample(stamp, reference_position)))
        return self._update_transform()

    def transform_sample(self, sample):
        if self.transform is None:
            return None
        t = self.transform
        r_xy = rotate_xy(sample.position, t.yaw)
        position = (
            t.scale * r_xy[0] + t.translation[0],
            t.scale * r_xy[1] + t.translation[1],
            t.scale * sample.position[2] + t.translation[2],
        )
        orientation = quat_normalize(quat_multiply(yaw_quat(t.yaw), sample.orientation))
        linear_xy = rotate_xy(sample.linear_velocity, t.yaw)
        angular_xy = rotate_xy(sample.angular_velocity, t.yaw)
        return PoseSample(
            sample.stamp,
            position,
            orientation,
            (t.scale * linear_xy[0], t.scale * linear_xy[1], t.scale * sample.linear_velocity[2]),
            (angular_xy[0], angular_xy[1], sample.angular_velocity[2]),
        )

    def status(self):
        data = {
            "ready": self.ready,
            "ready_count": self.ready_count,
            "pairs": len(self.pairs),
            "rejected_reason": self.rejected_reason,
        }
        if self.transform is not None:
            data.update({
                "scale": self.transform.scale,
                "yaw": self.transform.yaw,
                "translation_x": self.transform.translation[0],
                "translation_y": self.transform.translation[1],
                "translation_z": self.transform.translation[2],
                "residual_mean": self.transform.residual_mean,
                "residual_max": self.transform.residual_max,
                "average_speed": self.transform.average_speed,
            })
        return data

    def _nearest_odom(self, stamp):
        best = None
        best_dt = self.max_pair_dt
        for sample in self.odom_buffer:
            dt = abs(sample.stamp - stamp)
            if dt <= best_dt:
                best = sample
                best_dt = dt
        return best

    def _update_transform(self):
        if len(self.pairs) < self.min_pairs:
            self.rejected_reason = "not_enough_pairs"
            return False

        source = [(p[0].position[0], p[0].position[1]) for p in self.pairs]
        target = [(p[1].position[0], p[1].position[1]) for p in self.pairs]
        stamps = [p[1].stamp for p in self.pairs]
        z_values = [p[1].position[2] for p in self.pairs]

        motion = math.hypot(target[-1][0] - target[0][0], target[-1][1] - target[0][1])
        duration = max(1.0e-6, stamps[-1] - stamps[0])
        average_speed = motion / duration
        if motion < self.min_motion or average_speed < self.min_speed:
            self.rejected_reason = "insufficient_horizontal_motion"
            return False
        if max(z_values) - min(z_values) > self.max_vertical_motion:
            self.rejected_reason = "not_horizontal_motion"
            return False

        segment_speeds = []
        for i in range(1, len(target)):
            dt = stamps[i] - stamps[i - 1]
            if dt > 1.0e-6:
                segment_speeds.append(math.hypot(target[i][0] - target[i - 1][0],
                                                 target[i][1] - target[i - 1][1]) / dt)
        if len(segment_speeds) >= 3:
            mean_speed = sum(segment_speeds) / float(len(segment_speeds))
            variance = sum((v - mean_speed) ** 2 for v in segment_speeds) / float(len(segment_speeds))
            cv = math.sqrt(variance) / max(1.0e-6, mean_speed)
            if cv > self.uniform_speed_max_cv:
                self.rejected_reason = "speed_not_uniform"
                return False

        result = fit_similarity_2d(source, target)
        if result is None:
            self.rejected_reason = "fit_degenerate"
            return False

        scale, yaw, translation, residuals = result
        residual_mean = sum(residuals) / float(len(residuals))
        residual_max = max(residuals)
        if not (self.min_scale <= scale <= self.max_scale):
            self.rejected_reason = "scale_out_of_bounds"
            return False
        if residual_max > self.max_residual:
            self.rejected_reason = "residual_too_large"
            return False

        src_z_mean = sum(p[0].position[2] for p in self.pairs) / float(len(self.pairs))
        tgt_z_mean = sum(p[1].position[2] for p in self.pairs) / float(len(self.pairs))
        translation_3d = (translation[0], translation[1], tgt_z_mean - scale * src_z_mean)
        candidate = Transform2D(scale, yaw, translation_3d, residual_mean, residual_max, average_speed)
        if self.transform is None:
            self.ready_count = 1
        else:
            scale_delta = abs(candidate.scale - self.transform.scale) / max(1.0e-6, self.transform.scale)
            yaw_delta = abs(math.atan2(math.sin(candidate.yaw - self.transform.yaw),
                                       math.cos(candidate.yaw - self.transform.yaw)))
            if scale_delta <= self.scale_stability and yaw_delta <= self.yaw_stability:
                self.ready_count += 1
            else:
                self.ready_count = 1
        self.transform = candidate
        self.rejected_reason = "ready" if self.ready else "stabilizing"
        return self.ready


class VoGnssImuGuidanceNode:
    def __init__(self):
        self.raw_vo_topic = rospy.get_param("~raw_vo_topic", "/vo/odom")
        self.gnss_topic = rospy.get_param("~gnss_topic", "/mavros/global_position/global")
        self.imu_topic = rospy.get_param("~imu_topic", "/mavros/imu/data")
        self.guided_odom_topic = rospy.get_param("~guided_odom_topic", "/ekf/guided_vo_odom")
        self.status_topic = rospy.get_param("~status_topic", "/ekf/vo_guidance_status")
        self.world_frame_id = rospy.get_param("~world_frame_id", "map")
        self.child_frame_id = rospy.get_param("~child_frame_id", "base_link")
        self.publish_before_ready = rospy.get_param("~publish_before_ready", False)

        self.core = Similarity2DGuidanceCore(
            min_pairs=rospy.get_param("~min_pairs", 8),
            max_pairs=rospy.get_param("~max_pairs", 40),
            max_pair_dt=rospy.get_param("~max_pair_dt", 0.08),
            min_motion=rospy.get_param("~min_motion", 10.0),
            min_speed=rospy.get_param("~min_speed", 5.0),
            max_vertical_motion=rospy.get_param("~max_vertical_motion", 2.0),
            uniform_speed_max_cv=rospy.get_param("~uniform_speed_max_cv", 0.35),
            min_scale=rospy.get_param("~min_scale", 0.05),
            max_scale=rospy.get_param("~max_scale", 20.0),
            max_residual=rospy.get_param("~max_residual", 1.0),
            scale_stability=rospy.get_param("~scale_stability", 0.05),
            yaw_stability=rospy.get_param("~yaw_stability", 0.05),
            ready_frames=rospy.get_param("~ready_frames", 4),
            imu_timeout=rospy.get_param("~imu_timeout", 0.5),
            require_imu=rospy.get_param("~require_imu", True),
        )

        self.origin = None
        self.odom_pub = rospy.Publisher(self.guided_odom_topic, Odometry, queue_size=100)
        self.status_pub = rospy.Publisher(self.status_topic, String, queue_size=10)
        rospy.Subscriber(self.raw_vo_topic, Odometry, self.odom_callback, queue_size=200)
        rospy.Subscriber(self.gnss_topic, NavSatFix, self.gnss_callback, queue_size=50)
        rospy.Subscriber(self.imu_topic, Imu, self.imu_callback, queue_size=400)

        rospy.loginfo("VO guidance: raw VO %s + GNSS %s + IMU %s -> %s",
                      self.raw_vo_topic, self.gnss_topic, self.imu_topic, self.guided_odom_topic)

    def imu_callback(self, msg):
        self.core.add_imu(msg.header.stamp.to_sec())

    def odom_callback(self, msg):
        sample = PoseSample(
            msg.header.stamp.to_sec(),
            (msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z),
            (msg.pose.pose.orientation.w, msg.pose.pose.orientation.x,
             msg.pose.pose.orientation.y, msg.pose.pose.orientation.z),
            (msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z),
            (msg.twist.twist.angular.x, msg.twist.twist.angular.y, msg.twist.twist.angular.z),
        )
        self.core.add_odom(sample)
        if self.core.ready or self.publish_before_ready:
            guided = self.core.transform_sample(sample)
            if guided is not None:
                self.odom_pub.publish(self._make_odom(msg, guided))

    def gnss_callback(self, msg):
        if msg.status.status < NavSatStatus.STATUS_FIX:
            return
        enu = self._navsat_to_enu(msg)
        if enu is None:
            return
        self.core.add_reference(msg.header.stamp.to_sec(), enu)
        status = self.core.status()
        self.status_pub.publish(String(json.dumps(status, sort_keys=True)))
        if self.core.ready:
            rospy.loginfo_throttle(2.0,
                                   "VO guidance ready: scale %.4f yaw %.3f residual_max %.3f speed %.2f pairs %d",
                                   status["scale"],
                                   status["yaw"],
                                   status["residual_max"],
                                   status["average_speed"],
                                   status["pairs"])
        else:
            rospy.logwarn_throttle(2.0,
                                   "VO guidance waiting: reason=%s pairs=%d ready_count=%d",
                                   status["rejected_reason"],
                                   status["pairs"],
                                   status["ready_count"])

    def _make_odom(self, src, sample):
        out = copy.deepcopy(src)
        out.header.frame_id = self.world_frame_id
        out.child_frame_id = self.child_frame_id
        out.pose.pose.position.x = sample.position[0]
        out.pose.pose.position.y = sample.position[1]
        out.pose.pose.position.z = sample.position[2]
        out.pose.pose.orientation.w = sample.orientation[0]
        out.pose.pose.orientation.x = sample.orientation[1]
        out.pose.pose.orientation.y = sample.orientation[2]
        out.pose.pose.orientation.z = sample.orientation[3]
        out.twist.twist.linear.x = sample.linear_velocity[0]
        out.twist.twist.linear.y = sample.linear_velocity[1]
        out.twist.twist.linear.z = sample.linear_velocity[2]
        out.twist.twist.angular.x = sample.angular_velocity[0]
        out.twist.twist.angular.y = sample.angular_velocity[1]
        out.twist.twist.angular.z = sample.angular_velocity[2]
        if self.core.transform is not None:
            residual_var = max(0.01, self.core.transform.residual_max ** 2)
            covariance = list(out.pose.covariance)
            covariance[0] = max(covariance[0], residual_var)
            covariance[7] = max(covariance[7], residual_var)
            covariance[14] = max(covariance[14], 0.25)
            out.pose.covariance = covariance
        return out

    def _navsat_to_enu(self, msg):
        if not (math.isfinite(msg.latitude) and math.isfinite(msg.longitude) and math.isfinite(msg.altitude)):
            return None
        lat = math.radians(msg.latitude)
        lon = math.radians(msg.longitude)
        if self.origin is None:
            self.origin = (lat, lon, msg.altitude, math.cos(lat))
            rospy.loginfo("VO guidance GNSS origin: lat %.9f lon %.9f alt %.3f",
                          msg.latitude, msg.longitude, msg.altitude)
        lat0, lon0, alt0, cos_lat0 = self.origin
        return (
            (lon - lon0) * cos_lat0 * EARTH_RADIUS_METERS,
            (lat - lat0) * EARTH_RADIUS_METERS,
            msg.altitude - alt0,
        )


def main():
    rospy.init_node("vo_gnss_imu_guidance")
    VoGnssImuGuidanceNode()
    rospy.spin()


if __name__ == "__main__":
    main()
