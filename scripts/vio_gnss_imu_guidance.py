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
    pose_covariance: tuple = None


@dataclass
class YawTranslationTransform:
    yaw: float
    translation: tuple
    residual_mean: float
    residual_max: float
    average_speed: float
    scale_estimate: float
    yaw_observable: bool = True


def rotate_xy(point, yaw):
    c = math.cos(yaw)
    s = math.sin(yaw)
    return (c * point[0] - s * point[1], s * point[0] + c * point[1])


def yaw_quat(yaw):
    half = 0.5 * yaw
    return (math.cos(half), 0.0, 0.0, math.sin(half))


def yaw_from_quat(q):
    w, x, y, z = quat_normalize(q)
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


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


def fit_yaw_translation_2d(source_points, target_points):
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

    yaw = math.atan2(sin_term, cos_term)
    scale_estimate = math.hypot(cos_term, sin_term) / denom
    src_rot = rotate_xy(src_c, yaw)
    translation = (tgt_c[0] - src_rot[0], tgt_c[1] - src_rot[1])

    residuals = []
    for src, tgt in zip(source_points, target_points):
        r = rotate_xy(src, yaw)
        pred = (r[0] + translation[0], r[1] + translation[1])
        residuals.append(math.hypot(pred[0] - tgt[0], pred[1] - tgt[1]))

    return yaw, translation, residuals, scale_estimate


class VioGnssImuGuidanceCore:
    WAITING_VIO = "WAITING_VIO"
    ALIGNING = "ALIGNING"
    READY = "READY"
    LOST = "LOST"
    RECOVERING = "RECOVERING"

    def __init__(self,
                 min_pairs=5,
                 max_pairs=30,
                 max_pair_dt=0.08,
                 reference_sample_interval=0.2,
                 min_motion=2.0,
                 min_speed=0.3,
                 max_residual=1.0,
                 translation_only_max_residual=0.75,
                 max_scale_error=0.35,
                 yaw_stability=0.08,
                 translation_stability=0.5,
                 ready_frames=3,
                 recovery_frames=5,
                 lost_timeout=1.0,
                 reset_distance=2.0,
                 reset_yaw=0.7,
                 imu_timeout=0.5,
                 require_imu=True,
                 allow_translation_only=True,
                 allow_initial_translation_only=False,
                 translation_only_covariance_scale=25.0):
        self.min_pairs = int(min_pairs)
        self.max_pairs = int(max_pairs)
        self.max_pair_dt = float(max_pair_dt)
        self.reference_sample_interval = float(reference_sample_interval)
        self.min_motion = float(min_motion)
        self.min_speed = float(min_speed)
        self.max_residual = float(max_residual)
        self.translation_only_max_residual = float(translation_only_max_residual)
        self.max_scale_error = float(max_scale_error)
        self.yaw_stability = float(yaw_stability)
        self.translation_stability = float(translation_stability)
        self.ready_frames = int(ready_frames)
        self.recovery_frames = int(recovery_frames)
        self.lost_timeout = float(lost_timeout)
        self.reset_distance = float(reset_distance)
        self.reset_yaw = float(reset_yaw)
        self.imu_timeout = float(imu_timeout)
        self.require_imu = bool(require_imu)
        self.allow_translation_only = bool(allow_translation_only)
        self.allow_initial_translation_only = bool(allow_initial_translation_only)
        self.translation_only_covariance_scale = float(translation_only_covariance_scale)

        self.odom_buffer = deque(maxlen=500)
        self.pairs = deque(maxlen=self.max_pairs)
        self.transform = None
        self.state = self.WAITING_VIO
        self.ready_count = 0
        self.recovery_count = 0
        self.last_imu_stamp = None
        self.last_odom_stamp = None
        self.last_reference_pair_stamp = None
        self.last_guided_position = None
        self.last_guided_yaw = None
        self.rejected_reason = "waiting"
        self.last_recovery_scale = 1.0

    @property
    def ready(self):
        return self.state == self.READY and self.transform is not None

    def add_imu(self, stamp):
        self.last_imu_stamp = float(stamp)

    def add_odom(self, sample):
        self.last_odom_stamp = float(sample.stamp)
        if self.state == self.READY and self.transform is not None:
            guided = self.transform_sample(sample)
            if guided is not None and self.last_guided_position is not None:
                jump = math.sqrt(sum((guided.position[i] - self.last_guided_position[i]) ** 2
                                     for i in range(3)))
                guided_yaw = yaw_from_quat(guided.orientation)
                yaw_jump = 0.0
                if self.last_guided_yaw is not None:
                    yaw_jump = abs(math.atan2(math.sin(guided_yaw - self.last_guided_yaw),
                                              math.cos(guided_yaw - self.last_guided_yaw)))
                if jump > self.reset_distance or yaw_jump > self.reset_yaw:
                    self.state = self.RECOVERING
                    self.ready_count = 0
                    self.recovery_count = 0
                    self.pairs.clear()
                    self.rejected_reason = "vio_reset_detected"
            if guided is not None:
                self.last_guided_position = guided.position
                self.last_guided_yaw = yaw_from_quat(guided.orientation)
        self.odom_buffer.append(sample)
        if self.state == self.WAITING_VIO:
            self.state = self.ALIGNING
        elif self.state == self.LOST:
            self.state = self.RECOVERING
            self.ready_count = 0
            self.recovery_count = 0

    def mark_time(self, stamp):
        stamp = float(stamp)
        if self.last_odom_stamp is None:
            return
        if stamp - self.last_odom_stamp > self.lost_timeout and self.state != self.LOST:
            self.state = self.LOST
            self.ready_count = 0
            self.recovery_count = 0
            self.pairs.clear()
            self.rejected_reason = "vio_lost"

    def add_reference(self, stamp, reference_position):
        stamp = float(stamp)
        self.mark_time(stamp)
        if self.require_imu and (self.last_imu_stamp is None or abs(stamp - self.last_imu_stamp) > self.imu_timeout):
            self.rejected_reason = "no_recent_imu"
            return False

        odom = self._nearest_odom(stamp)
        if odom is None:
            self.rejected_reason = "no_synced_vio"
            return False

        if (self.last_reference_pair_stamp is not None and
                (stamp - self.last_reference_pair_stamp) < self.reference_sample_interval):
            return self.ready
        self.last_reference_pair_stamp = stamp
        self.pairs.append((odom, PoseSample(stamp, reference_position)))
        return self._update_transform()

    def transform_sample(self, sample):
        if self.transform is None:
            return None
        t = self.transform
        r_xy = rotate_xy(sample.position, t.yaw)
        position = (
            r_xy[0] + t.translation[0],
            r_xy[1] + t.translation[1],
            sample.position[2] + t.translation[2],
        )
        orientation = quat_normalize(quat_multiply(yaw_quat(t.yaw), sample.orientation))
        linear_xy = rotate_xy(sample.linear_velocity, t.yaw)
        angular_xy = rotate_xy(sample.angular_velocity, t.yaw)
        return PoseSample(
            sample.stamp,
            position,
            orientation,
            (linear_xy[0], linear_xy[1], sample.linear_velocity[2]),
            (angular_xy[0], angular_xy[1], sample.angular_velocity[2]),
            sample.pose_covariance,
        )

    def observation_scale(self):
        if self.state == self.RECOVERING:
            return self.last_recovery_scale
        if self.state == self.LOST:
            return 100.0
        if self.transform is not None and not self.transform.yaw_observable:
            return self.translation_only_covariance_scale
        return 1.0

    def status(self):
        data = {
            "ready": self.ready,
            "state": self.state,
            "ready_count": self.ready_count,
            "recovery_count": self.recovery_count,
            "pairs": len(self.pairs),
            "rejected_reason": self.rejected_reason,
            "observation_scale": self.observation_scale(),
        }
        if self.transform is not None:
            data.update({
                "yaw": self.transform.yaw,
                "translation_x": self.transform.translation[0],
                "translation_y": self.transform.translation[1],
                "translation_z": self.transform.translation[2],
                "residual_mean": self.transform.residual_mean,
                "residual_max": self.transform.residual_max,
                "average_speed": self.transform.average_speed,
                "scale_estimate": self.transform.scale_estimate,
                "yaw_observable": self.transform.yaw_observable,
            })
        return data

    def _fit_translation_only(self, source, target, average_speed):
        if not self.allow_translation_only:
            return None
        if self.transform is None and not self.allow_initial_translation_only:
            return None
        yaw = self.transform.yaw if self.transform is not None else 0.0
        rotated = [rotate_xy(src, yaw) for src in source]
        n = float(len(source))
        translation_xy = (
            sum(target[i][0] - rotated[i][0] for i in range(len(source))) / n,
            sum(target[i][1] - rotated[i][1] for i in range(len(source))) / n,
        )
        residuals = []
        for r, tgt in zip(rotated, target):
            pred = (r[0] + translation_xy[0], r[1] + translation_xy[1])
            residuals.append(math.hypot(pred[0] - tgt[0], pred[1] - tgt[1]))
        residual_mean = sum(residuals) / float(len(residuals))
        residual_max = max(residuals)
        if residual_max > self.translation_only_max_residual:
            return None
        return yaw, translation_xy, residual_mean, residual_max, 1.0, False

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

        motion = math.hypot(target[-1][0] - target[0][0], target[-1][1] - target[0][1])
        duration = max(1.0e-6, stamps[-1] - stamps[0])
        average_speed = motion / duration
        yaw_observable = True
        if motion < self.min_motion or average_speed < self.min_speed:
            translation_only = self._fit_translation_only(source, target, average_speed)
            if translation_only is None:
                self.rejected_reason = "insufficient_motion"
                return False
            yaw, translation_xy, residual_mean, residual_max, scale_estimate, yaw_observable = translation_only
        else:
            result = fit_yaw_translation_2d(source, target)
            if result is None:
                translation_only = self._fit_translation_only(source, target, average_speed)
                if translation_only is None:
                    self.rejected_reason = "fit_degenerate"
                    return False
                yaw, translation_xy, residual_mean, residual_max, scale_estimate, yaw_observable = translation_only
            else:
                yaw, translation_xy, residuals, scale_estimate = result
                residual_mean = sum(residuals) / float(len(residuals))
                residual_max = max(residuals)
                if abs(scale_estimate - 1.0) > self.max_scale_error:
                    translation_only = self._fit_translation_only(source, target, average_speed)
                    if translation_only is None:
                        self.rejected_reason = "vio_scale_inconsistent"
                        return False
                    yaw, translation_xy, residual_mean, residual_max, scale_estimate, yaw_observable = translation_only
                if residual_max > self.max_residual:
                    translation_only = self._fit_translation_only(source, target, average_speed)
                    if translation_only is None:
                        self.rejected_reason = "residual_too_large"
                        return False
                    yaw, translation_xy, residual_mean, residual_max, scale_estimate, yaw_observable = translation_only

        src_z_mean = sum(p[0].position[2] for p in self.pairs) / float(len(self.pairs))
        tgt_z_mean = sum(p[1].position[2] for p in self.pairs) / float(len(self.pairs))
        candidate = YawTranslationTransform(
            yaw,
            (translation_xy[0], translation_xy[1], tgt_z_mean - src_z_mean),
            residual_mean,
            residual_max,
            average_speed,
            scale_estimate,
            yaw_observable,
        )

        if self.transform is None:
            stable = True
        else:
            yaw_delta = abs(math.atan2(math.sin(candidate.yaw - self.transform.yaw),
                                       math.cos(candidate.yaw - self.transform.yaw)))
            translation_delta = math.sqrt(sum((candidate.translation[i] - self.transform.translation[i]) ** 2
                                              for i in range(3)))
            stable = yaw_delta <= self.yaw_stability and translation_delta <= self.translation_stability

        self.transform = candidate
        if stable:
            self.ready_count += 1
        else:
            self.ready_count = 1
            self.recovery_count = 0

        if self.state == self.RECOVERING:
            self.recovery_count += 1 if stable else 0
            remaining = max(0, self.recovery_frames - self.recovery_count)
            self.last_recovery_scale = 1.0 + remaining * 10.0
            if self.recovery_count >= self.recovery_frames:
                self.state = self.READY
                self.last_recovery_scale = 1.0
        elif self.ready_count >= self.ready_frames:
            self.state = self.READY
        else:
            self.state = self.ALIGNING

        if self.ready:
            self.rejected_reason = "ready" if self.transform.yaw_observable else "translation_ready"
        else:
            self.rejected_reason = "stabilizing"
        return self.ready


class VioGnssImuGuidanceNode:
    def __init__(self):
        self.raw_vio_topic = rospy.get_param("~raw_vio_topic", "/vio/odom")
        self.gnss_topic = rospy.get_param("~gnss_topic", "/mavros/global_position/global")
        self.imu_topic = rospy.get_param("~imu_topic", "/mavros/imu/data")
        self.guided_odom_topic = rospy.get_param("~guided_odom_topic", "/ekf/guided_vio_odom")
        self.status_topic = rospy.get_param("~status_topic", "/ekf/vio_guidance_status")
        self.world_frame_id = rospy.get_param("~world_frame_id", "map")
        self.child_frame_id = rospy.get_param("~child_frame_id", "base_link")
        self.publish_before_ready = rospy.get_param("~publish_before_ready", False)
        self.max_position_covariance = float(rospy.get_param("~max_position_covariance", 4.0))
        self.recovery_covariance_scale = float(rospy.get_param("~recovery_covariance_scale", 100.0))

        self.core = VioGnssImuGuidanceCore(
            min_pairs=rospy.get_param("~min_pairs", 5),
            max_pairs=rospy.get_param("~max_pairs", 30),
            max_pair_dt=rospy.get_param("~max_pair_dt", 0.08),
            reference_sample_interval=rospy.get_param("~reference_sample_interval", 0.2),
            min_motion=rospy.get_param("~min_motion", 2.0),
            min_speed=rospy.get_param("~min_speed", 0.3),
            max_residual=rospy.get_param("~max_residual", 1.0),
            translation_only_max_residual=rospy.get_param("~translation_only_max_residual", 0.75),
            max_scale_error=rospy.get_param("~max_scale_error", 0.35),
            yaw_stability=rospy.get_param("~yaw_stability", 0.08),
            translation_stability=rospy.get_param("~translation_stability", 0.5),
            ready_frames=rospy.get_param("~ready_frames", 3),
            recovery_frames=rospy.get_param("~recovery_frames", 5),
            lost_timeout=rospy.get_param("~lost_timeout", 1.0),
            reset_distance=rospy.get_param("~reset_distance", 2.0),
            reset_yaw=rospy.get_param("~reset_yaw", 0.7),
            imu_timeout=rospy.get_param("~imu_timeout", 0.5),
            require_imu=rospy.get_param("~require_imu", True),
            allow_translation_only=rospy.get_param("~allow_translation_only", True),
            allow_initial_translation_only=rospy.get_param("~allow_initial_translation_only", False),
            translation_only_covariance_scale=rospy.get_param("~translation_only_covariance_scale", 25.0),
        )

        self.origin = None
        self.odom_pub = rospy.Publisher(self.guided_odom_topic, Odometry, queue_size=100)
        self.status_pub = rospy.Publisher(self.status_topic, String, queue_size=10)
        rospy.Subscriber(self.raw_vio_topic, Odometry, self.odom_callback, queue_size=200)
        rospy.Subscriber(self.gnss_topic, NavSatFix, self.gnss_callback, queue_size=50)
        rospy.Subscriber(self.imu_topic, Imu, self.imu_callback, queue_size=400)

        rospy.loginfo("VIO guidance: raw VIO %s + GNSS %s + IMU %s -> %s",
                      self.raw_vio_topic, self.gnss_topic, self.imu_topic, self.guided_odom_topic)

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
            tuple(msg.pose.covariance),
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
                                   "VIO guidance ready: yaw %.3f residual_max %.3f scale_est %.3f state %s",
                                   status["yaw"],
                                   status["residual_max"],
                                   status["scale_estimate"],
                                   status["state"])
        else:
            rospy.logwarn_throttle(2.0,
                                   "VIO guidance waiting: state=%s reason=%s pairs=%d ready=%d recovery=%d",
                                   status["state"],
                                   status["rejected_reason"],
                                   status["pairs"],
                                   status["ready_count"],
                                   status["recovery_count"])

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

        covariance = list(out.pose.covariance)
        residual_var = 0.01
        if self.core.transform is not None:
            residual_var = max(0.01, self.core.transform.residual_max ** 2)
        scale = max(self.core.observation_scale(), self.recovery_covariance_scale if self.core.state == self.core.RECOVERING else 1.0)
        for index in (0, 7, 14):
            base = covariance[index] if covariance[index] > 0.0 else residual_var
            covariance[index] = min(self.max_position_covariance * scale, max(base, residual_var) * scale)
        for index in (21, 28, 35):
            base = covariance[index] if covariance[index] > 0.0 else 0.05
            covariance[index] = max(base, 0.05) * scale
        out.pose.covariance = covariance
        return out

    def _navsat_to_enu(self, msg):
        if not (math.isfinite(msg.latitude) and math.isfinite(msg.longitude) and math.isfinite(msg.altitude)):
            return None
        lat = math.radians(msg.latitude)
        lon = math.radians(msg.longitude)
        if self.origin is None:
            self.origin = (lat, lon, msg.altitude, math.cos(lat))
            rospy.loginfo("VIO guidance GNSS origin: lat %.9f lon %.9f alt %.3f",
                          msg.latitude, msg.longitude, msg.altitude)
        lat0, lon0, alt0, cos_lat0 = self.origin
        return (
            (lon - lon0) * cos_lat0 * EARTH_RADIUS_METERS,
            (lat - lat0) * EARTH_RADIUS_METERS,
            msg.altitude - alt0,
        )


def main():
    rospy.init_node("vio_gnss_imu_guidance")
    VioGnssImuGuidanceNode()
    rospy.spin()


if __name__ == "__main__":
    main()
