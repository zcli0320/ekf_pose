#include "ekf.h"

#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/MarkerArray.h>
#include <Eigen/Eigen>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <geometry_msgs/PoseStamped.h>
#include <algorithm>
#include <cmath>
#include <deque>
#include <iostream>
#include <limits>
#include <string>
#include <vector>
// #include <geometry_msgs/Accel.h>
#include "conversion.h"

using namespace std;
using namespace Eigen;

// Main data-flow overview for maintainers:
//   1. IMU callback runs at the highest rate. It rotates/scales raw IMU samples,
//      propagates the 16D nominal state X=[p,q,v,bg,ba], and propagates the 15D
//      error-state covariance P.
//   2. Odom callback is the primary local pose correction. It forms a 6D
//      residual [dp,dtheta], where dtheta is an SO(3) rotation-vector residual,
//      then applies the correction through boxplus().
//   3. GNSS callback is a conservative global position correction. NavSatFix is
//      converted to local ENU, aligned to the odom/EKF frame, checked by health
//      gates, and fused as a 3D position update or optional position+velocity
//      pseudo-update when odom is lost.
//   4. Time synchronization is handled by replay buffers. The node stores the
//      state/covariance before each IMU sample; when an odom measurement arrives
//      with an older stamp, the filter rolls back to the nearest cached IMU
//      state, updates, and replays the later IMU samples.
//
// Keep topic names, frame_id usage, and message types stable unless a launch
// remap is enough. Downstream reproduction scripts and RViz configs depend on
// the public ROS interface documented in docs/reproduction.md.
//
// Function-level maintenance notes and mathematical derivations are documented
// in docs/ekf_node_function_reference.md. Keep that file synchronized whenever
// adding, removing, or changing functions in this core node.

#define POS_DIFF_THRESHOLD (0.8f)

ros::Publisher odom_pub, ahead_odom_pub;
ros::Publisher cam_odom_pub;
ros::Publisher input_path_pub, ekf_path_pub, measurement_path_pub;
ros::Publisher ekf_segments_pub;
ros::Publisher ekf_arrows_pub;
ros::Publisher gnss_path_pub;
ros::Time imu_back_time = ros::Time(0), imu_front_time = ros::Time(0);

// EKF state layout. These offsets are part of the implementation contract.
//
//   Nominal state X_state is 16D:
//     p      X_state(0:2)    position in the current world/odom frame
//     q      X_state(3:6)    quaternion in w,x,y,z order
//     v      X_state(7:9)    velocity in the world/odom frame
//     bg     X_state(10:12)  gyro bias
//     ba     X_state(13:15)  accelerometer bias
//
//   Error state dx is 15D:
//     dx = [dp, dtheta, dv, dbg, dba]
//
//   StateCovariance is the 15x15 covariance of dx. It is intentionally not a
//   16x16 covariance over [p,q,v,bg,ba], because quaternion uncertainty is
//   represented by the 3D Lie-algebra perturbation dtheta. All pose updates must
//   therefore apply dx through boxplus(); direct X_state += dx would mix a 3D
//   attitude error with the 4D quaternion storage and break the unit-norm
//   constraint.
//
//   IMU input u is 6D [gyro, acc], and Qt is the corresponding process noise.
//   Odom residual is 6D [position residual, SO(3) rotation-vector residual].
//   GNSS residual is 3D position, optionally expanded to 6D position+velocity
//   when odom is lost. Z_measurement stores the raw 7D odom pose [p,q_wxyz]
//   before the residual is formed against g_model().
constexpr int kStatePositionOffset = 0;
constexpr int kStateQuaternionOffset = 3;
constexpr int kStateVelocityOffset = 7;
constexpr int kStateGyroBiasOffset = 10;
constexpr int kStateAccelBiasOffset = 13;

constexpr int kErrorPositionOffset = 0;
constexpr int kErrorRotationOffset = 3;
constexpr int kErrorVelocityOffset = 6;
constexpr int kErrorGyroBiasOffset = 9;
constexpr int kErrorAccelBiasOffset = 12;

constexpr int kResidualPositionOffset = 0;
constexpr int kResidualRotationOffset = 3;

size_t stateSize;
size_t errorstateSize;
size_t measurementSize;
size_t inputSize;
VectorXd X_state;
VectorXd Z_measurement;
MatrixXd StateCovariance;
MatrixXd Kt_kalmanGain;
// MatrixXd Ct_stateToMeasurement;                  // Ct
//  VectorXd innovation;                         // z - Hx

MatrixXd Qt;
MatrixXd Rt;
MatrixXd current_odom_Rt;
Vector3d u_gyro;
Vector3d u_acc;
Vector3d gravity(0., 0., -9.8); // need to estimate the bias 9.8099
Vector3d bg_0(0., 0., 0);       // need to estimate the bias
Vector3d ba_0(0., 0., 0);       // need to estimate the bias  0.1
Vector3d ng(0., 0., 0.);
Vector3d na(0., 0., 0.);
Vector3d nbg(0., 0., 0.);
Vector3d nba(0., 0., 0.);
Matrix3d rotation_imu;

Quaterniond q_last;
Vector3d bg_last;
Vector3d ba_last;
// Static transform and noise parameters loaded from launch/PX4_vio_drone.yaml
// and launch/ekf_lidar.launch. Smaller Q entries trust IMU propagation more;
// smaller R entries trust odom observations more.
double imu_trans_x = 0.0;
double imu_trans_y = 0.0;
double imu_trans_z = 0.0;
double gyro_cov = 0.01;
double acc_cov = 0.01;
// Rt odom covariance: smaller values make the filter trust odom more.
double position_cov = 0.1;
double q_rp_cov = 0.1;
double q_yaw_cov = 0.1;
double scale_g;
double dt = 0.005; // second
bool first_frame_imu = true;
bool first_frame_tag_odom = true;
bool ekf_initialized = false;

double time_now, time_last;
double time_odom_tag_now;
// double diff_time;

double cutoff_freq = 20;
double sample_freq = 120;
int publish_warmup_frames = 0;
bool enable_output_motion_smoothing = true;
double output_smoothing_natural_freq = 3.0;
double output_smoothing_damping_ratio = 1.0;
double output_smoothing_max_accel = 50.0;
double output_smoothing_max_correction_speed = 20.0;
double output_smoothing_normal_natural_freq = 3.5;
double output_smoothing_normal_max_accel = 50.0;
double output_smoothing_normal_max_correction_speed = 20.0;
double output_smoothing_release_error = 0.005;
double output_smoothing_recovery_duration = 0.8;

// Odom health management:
//   - odom_jump_threshold detects discontinuities in raw odom.
//   - innovation thresholds compare aligned odom against EKF prediction.
//   - adaptive covariance weakens suspicious odom instead of immediately
//     resetting the filter.
//   - odom_loss_timeout lets GNSS/IMU degraded mode take over when odom stops.
double odom_jump_threshold = 2.0;
double innovation_reject_threshold = 1.0;
double innovation_reset_threshold = 2.0;
bool odom_use_msg_covariance = false;
double odom_msg_min_position_cov = 0.01;
double odom_msg_min_orientation_cov = 0.01;
bool enable_odom_realign = true;
bool enable_adaptive_observation_covariance = true;
double odom_adaptive_threshold = 1.5;
double odom_adaptive_reject_threshold = 4.0;
double odom_adaptive_max_scale = 100.0;
double odom_loss_timeout = 1.0;
bool enable_odom_recovery_guard = true;
int odom_recovery_frames = 45;
double odom_recovery_scale = 1000.0;
double odom_recovery_min_scale = 1.0;
bool enable_gnss_velocity_when_odom_lost = false;
double gnss_velocity_min_dt = 0.05;
double gnss_velocity_max_dt = 1.5;
double gnss_velocity_cov = 1.0;
int gnss_velocity_window_size = 2;
double gnss_velocity_smoothing_alpha = 1.0;
double gyro_bias_rw_cov = 0.0;
double acc_bias_rw_cov = 0.0;
double gnss_adaptive_threshold = 3.0;
double gnss_adaptive_reject_threshold = 5.0;
double gnss_adaptive_max_scale = 25.0;
int odom_realign_settle_frames = 20;
double odom_source_switch_grace = 0.5;

// GNSS health management:
//   GNSS is intentionally conservative. Before fusion, each NavSatFix sample is
//   converted to ENU, aligned into the EKF frame, checked by NIS/Mahalanobis
//   gates, compared against odom motion when possible, and finally scaled by a
//   health score. The default behavior is to down-weight weak GNSS before
//   rejecting it, except when the observation is clearly inconsistent.
bool use_gnss = true;
bool enable_gnss_cold_start = true;
double gnss_cold_start_delay = 1.0;
bool gnss_update_only_when_odom_lost = false;
bool enable_gnss_position_snap_when_odom_lost = false;
bool gnss_use_msg_covariance = true;
double gnss_min_interval = 0.5;
double gnss_min_cov_xy = 4.0;
double gnss_min_cov_z = 9.0;
double gnss_cov_scale = 1.0;
double gnss_position_covariance_floor_xy = 0.0;
double gnss_position_covariance_floor_z = 0.0;
bool enable_gnss_mahalanobis_gate = true;
double gnss_mahalanobis_weak_threshold = 7.815;
double gnss_mahalanobis_reject_threshold = 16.266;
bool enable_gnss_motion_consistency = true;
double gnss_motion_consistency_min_motion = 0.5;
double gnss_motion_consistency_threshold = 2.0;
double gnss_motion_consistency_reject_threshold = 5.0;
double gnss_motion_consistency_max_scale = 10.0;
double gnss_healthy_odom_weak_scale = 1.0;
bool enable_gnss_health_score = true;
double gnss_good_covariance_xy = 4.0;
double gnss_poor_covariance_xy = 16.0;
double gnss_min_health_score = 0.2;
double gnss_health_trust_threshold = 0.75;
double odom_weak_health_threshold = 0.5;
double gnss_health_low_score_max_scale = 8.0;
bool enable_gnss_nis_state_machine = true;
int gnss_health_window_size = 5;
int gnss_degraded_count_threshold = 3;
int gnss_severe_count_threshold = 3;
int gnss_recover_count_threshold = 3;
double gnss_isolation_time = 1.0;
double gnss_nis_degraded_threshold = 7.815;
double gnss_nis_severe_threshold = 16.266;
double gnss_r_degraded_scale = 5.0;
double gnss_r_severe_scale = 20.0;
bool enable_odom_gnss_consistency_health = true;
double odom_gnss_consistency_threshold = 0.75;
double odom_gnss_consistency_poor_threshold = 2.5;
double odom_gnss_consistency_max_scale = 25.0;
double odom_gnss_consistency_timeout = 2.0;
bool enable_gnss_yaw_alignment = true;
bool gnss_require_yaw_alignment_before_update = true;
int gnss_alignment_min_samples = 5;
int gnss_alignment_max_samples = 120;
double gnss_alignment_min_motion = 20.0;
double gnss_alignment_sample_interval = 0.5;
double gnss_odom_sync_max_dt = 0.2;
double gnss_alignment_max_residual = 3.0;
bool enable_gnss_alignment_refinement = true;
int gnss_alignment_refinement_min_samples = 20;
int gnss_alignment_refinement_max_samples = 60;
double gnss_alignment_refinement_min_motion = 12.0;
double gnss_alignment_refinement_max_residual = 2.0;
double gnss_alignment_refinement_gain = 0.35;
double gnss_alignment_refinement_max_yaw_step = 0.35;
double gnss_alignment_refinement_max_translation_step = 5.0;
int gnss_min_status = 0;
int path_publish_stride = 5;
int path_max_points = 2000;
int arrow_publish_stride = 30;
int arrow_max_markers = 1000;
string world_frame_id = "odom";
string gnss_cold_start_frame_id = "odom";
double first_gnss_cold_start_candidate_time = -1.0;
bool last_accepted_gnss_position_initialized = false;
Vector3d last_accepted_gnss_position = Vector3d::Zero();
double last_accepted_gnss_position_time = -1.0;
bool last_accepted_gnss_velocity_initialized = false;
Vector3d last_accepted_gnss_velocity = Vector3d::Zero();
std::deque<std::pair<double, Vector3d>> accepted_gnss_history;
std::deque<nav_msgs::Odometry::ConstPtr> pending_odom_measurements;
const size_t max_pending_odom_measurements = 200;

/// @brief Forward declaration for odom-loss query used by IMU and GNSS paths.
bool odom_is_lost_at(double stamp_sec);
/// @brief Forward declaration for the odom processing pipeline.
void process_vioodom(const nav_msgs::Odometry::ConstPtr &msg);
/// @brief Forward declaration for draining future-dated odom measurements.
void drain_pending_odom_measurements();
/// @brief Forward declaration for output smoother reset used by state resets.
void reset_output_motion_smoother();

/// @brief Normalize the quaternion part of the nominal state X=[p,q,v,bg,ba].
void normalize_state_quaternion(VectorXd &state)
{
    Quaterniond q(state(3), state(4), state(5), state(6));
    if (!std::isfinite(q.w()) || !std::isfinite(q.x()) || !std::isfinite(q.y()) || !std::isfinite(q.z()) || q.norm() < 1.0e-12)
    {
        q = Quaterniond::Identity();
    }
    else
    {
        q.normalize();
    }
    state(3) = q.w();
    state(4) = q.x();
    state(5) = q.y();
    state(6) = q.z();
}

/// @brief Integrate an angular-rate sample into a small quaternion increment.
Quaterniond delta_quaternion_from_gyro(const Vector3d &omega, double dt)
{
    const Vector3d delta_theta = omega * dt;
    const double angle = delta_theta.norm();
    if (angle < 1.0e-12)
    {
        return Quaterniond::Identity();
    }
    return Quaterniond(AngleAxisd(angle, delta_theta / angle));
}

/// @brief Keep covariance numerically symmetric after EKF propagation or update.
void symmetrize_covariance(MatrixXd &covariance)
{
    covariance = 0.5 * (covariance + covariance.transpose());
}

/// @brief Add bias random-walk process noise to the 15D error covariance.
void apply_bias_random_walk_covariance(double propagation_dt)
{
    if (propagation_dt <= 0.0)
    {
        return;
    }
    StateCovariance.block<3, 3>(kErrorGyroBiasOffset, kErrorGyroBiasOffset) +=
        Matrix3d::Identity() * std::max(0.0, gyro_bias_rw_cov) * propagation_dt;
    StateCovariance.block<3, 3>(kErrorAccelBiasOffset, kErrorAccelBiasOffset) +=
        Matrix3d::Identity() * std::max(0.0, acc_bias_rw_cov) * propagation_dt;
    symmetrize_covariance(StateCovariance);
}

/// @brief Apply Joseph-form covariance update for a 15D error-state EKF.
void joseph_covariance_update(const MatrixXd &H, const MatrixXd &K, const MatrixXd &R)
{
    const MatrixXd I = MatrixXd::Identity(errorstateSize, errorstateSize);
    const MatrixXd IKH = I - K * H;
    StateCovariance = IKH * StateCovariance * IKH.transpose() + K * R * K.transpose();
    symmetrize_covariance(StateCovariance);
}

double offset_px, offset_py, offset_pz;

// Time-sync buffers store the nominal state/covariance before each IMU sample.
// Odom updates roll back to the nearest buffered sample, update, then replay IMU.
deque<pair<VectorXd, sensor_msgs::Imu>> sys_seq;
deque<MatrixXd> cov_seq;
double dt_0_rp; // the dt for the first frame in repropagation
/// @brief Cache the pre-propagation state, IMU sample, and covariance for time-sync replay.
void seq_keep(const sensor_msgs::Imu::ConstPtr &imu_msg)
{
    // 缓存的是“本帧 IMU 传播之前”的 X/P，以及这帧 IMU 测量本身。
    // 当较低频的 odom 延迟到达时，可以把队首移动到离 odom 时间最近的
    // IMU 帧，先用缓存的 X/P 做观测更新，再重放后续 IMU。这样比直接在
    // 最新状态上融合延迟 odom 更接近传感器真实时间顺序。
    static const size_t kMaxImuReplayBufferSize = 100;
    if (sys_seq.size() < kMaxImuReplayBufferSize)
    {
        sys_seq.push_back(make_pair(X_state, *imu_msg)); // X_state before propagation and imu at that time
        cov_seq.push_back(StateCovariance);
    }
    else
    {
        sys_seq.pop_front();
        sys_seq.push_back(make_pair(X_state, *imu_msg));
        cov_seq.pop_front();
        cov_seq.push_back(StateCovariance);
    }
    imu_front_time = sys_seq.front().second.header.stamp;
    imu_back_time = sys_seq.back().second.header.stamp;
    // ensure that the later frame time > the former one
}
// choose the coordinate frame imu for the measurement
/// @brief Find the IMU buffer frame closest to an odom timestamp before an update.
bool search_proper_frame(double odom_time)
{
    // 目标：选择与 odom_time 最近的缓存 IMU 帧，并丢弃更早的缓存。
    // 函数返回值只表示 odom_time 是否落在当前缓存时间范围内；即使返回
    // false，rightframe 仍会被夹到首帧或末帧，调用者可退化为 latest-state
    // update 或按边界帧处理。
    if (sys_seq.size() == 0)
    {
        ROS_ERROR("sys_seq.size() == 0. if appear this error, should check the code");
        return false;
    }
    if (sys_seq.size() == 1)
    {
        ROS_ERROR("sys_seq.size() == 1. at least two IMU samples are needed for interpolation/replay");
        return false;
    }

    size_t rightframe = sys_seq.size() - 1;
    bool find_proper_frame = false;
    for (size_t i = 1; i < sys_seq.size(); i++)
    {
        double time_before = odom_time - sys_seq[i - 1].second.header.stamp.toSec();
        double time_after = odom_time - sys_seq[i].second.header.stamp.toSec();
        if ((time_before >= 0) && (time_after < 0))
        {
            if (abs(time_before) > abs(time_after))
            {
                rightframe = i;
            }
            else
            {
                rightframe = i - 1;
            }

            if (rightframe != 0)
            {
                dt_0_rp = sys_seq[rightframe].second.header.stamp.toSec() - sys_seq[rightframe - 1].second.header.stamp.toSec();
            }
            else
            { // if rightframe is the first frame in the seq, set dt_0_rp as the next dt
                dt_0_rp = sys_seq[rightframe + 1].second.header.stamp.toSec() - sys_seq[rightframe].second.header.stamp.toSec();
            }

            find_proper_frame = true;
            break;
        }
    }
    if (!find_proper_frame)
    {
        if ((odom_time - sys_seq[0].second.header.stamp.toSec()) <= 0) // if odom time before the first frame, set first frame
        {
            rightframe = 0;
            // if rightframe is the first frame in the seq, set dt_0_rp as the next dt
            dt_0_rp = sys_seq[rightframe + 1].second.header.stamp.toSec() - sys_seq[rightframe].second.header.stamp.toSec();
        }
        if ((odom_time - sys_seq[sys_seq.size() - 1].second.header.stamp.toSec()) >= 0) // if odom time after the last frame, set last frame
        {
            rightframe = sys_seq.size() - 1;
            dt_0_rp = sys_seq[rightframe].second.header.stamp.toSec() - sys_seq[rightframe - 1].second.header.stamp.toSec();
        }
        // no process, set the latest one
    }

    // set the right frame as the first frame in the queue
    for (size_t i = 0; i < rightframe; i++)
    {
        sys_seq.pop_front();
        cov_seq.pop_front();
    }

    if (find_proper_frame)
    {
        return true;
    }
    else
    {
        return false;
    }
}
/// @brief Replay cached IMU samples after a delayed odom update to return to the latest time.
void re_propagate()
{
    // 这里的 sys_seq[0] 已经在 process_vioodom() 中被回退并完成了一次
    // odom 观测更新。后面的样本是“观测时间之后、当前 IMU 时间之前”的
    // IMU 输入。逐帧重放它们，可以把刚刚修正过的状态重新推进到最新时刻。
    //
    // 注意：
    //   1. 重放时使用每一帧缓存的 IMU 测量和相邻时间戳计算 dt。
    //   2. P 的传播仍然是误差状态协方差传播：P = F P F^T + V Q V^T。
    //   3. 如果 bag 或传输导致时间戳乱序，dt <= 0 的片段会被跳过，避免
    //      负时间传播污染状态。
    for (size_t i = 1; i < sys_seq.size(); i++)
    {
        // re-prediction for the rightframe
        dt = sys_seq[i].second.header.stamp.toSec() - sys_seq[i - 1].second.header.stamp.toSec();
        if (dt <= 0.0)
        {
            ROS_WARN_THROTTLE(1.0, "Skipping non-monotonic IMU repropagation step: dt %.6f s", dt);
            continue;
        }

        u_gyro(0) = sys_seq[i].second.angular_velocity.x;
        u_gyro(1) = sys_seq[i].second.angular_velocity.y;
        u_gyro(2) = sys_seq[i].second.angular_velocity.z;
        u_acc(0) = sys_seq[i].second.linear_acceleration.x;
        u_acc(1) = sys_seq[i].second.linear_acceleration.y;
        u_acc(2) = sys_seq[i].second.linear_acceleration.z;

        MatrixXd Ft;
        MatrixXd Vt;

        q_last.w() = sys_seq[i].first(3);
        q_last.x() = sys_seq[i].first(4);
        q_last.y() = sys_seq[i].first(5);
        q_last.z() = sys_seq[i].first(6);

        bg_last = sys_seq[i].first.segment<3>(10);
        ba_last = sys_seq[i].first.segment<3>(13);

        Ft = MatrixXd::Identity(errorstateSize, errorstateSize) + dt * diff_f_diff_x(q_last, u_gyro, u_acc, bg_last, ba_last);

        Vt = dt * diff_f_diff_n(q_last);

        X_state = propagate_nominal_state(X_state, u_gyro, u_acc, dt);

        StateCovariance = Ft * StateCovariance * Ft.transpose() + Vt * Qt * Vt.transpose();
        apply_bias_random_walk_covariance(dt);
    }
}
/// @brief IMU callback: rotate/scale-correct raw IMU, propagate nominal state and covariance.
void imu_callback(const sensor_msgs::Imu::ConstPtr &msg)
{
    // IMU 原始坐标轴和 EKF 使用的 body/world 约定可能不同，因此先应用
    // launch/PX4_vio_drone.yaml 中的 rotation_imu。scale_g 用于修正加速度
    // 量纲/重力标定偏差。完成这一步后，u_gyro/u_acc 才能进入预测模型。
    sensor_msgs::Imu::Ptr new_msg(new sensor_msgs::Imu(*msg));
    Eigen::Vector3d temp1, temp2;

    temp1[0] = new_msg->linear_acceleration.x;
    temp1[1] = new_msg->linear_acceleration.y;
    temp1[2] = new_msg->linear_acceleration.z;
    temp2 = rotation_imu * temp1;
    new_msg->linear_acceleration.x = scale_g * temp2[0];
    new_msg->linear_acceleration.y = scale_g * temp2[1];
    new_msg->linear_acceleration.z = scale_g * temp2[2];

    temp1[0] = new_msg->angular_velocity.x;
    temp1[1] = new_msg->angular_velocity.y;
    temp1[2] = new_msg->angular_velocity.z;
    temp2 = rotation_imu * temp1;
    new_msg->angular_velocity.x = temp2[0];
    new_msg->angular_velocity.y = temp2[1];
    new_msg->angular_velocity.z = temp2[2];

    if (!first_frame_tag_odom)
    { // get the initial pose and orientation in the first frame of measurement
        if (first_frame_imu)
        {
            first_frame_imu = false;
            time_now = new_msg->header.stamp.toSec();
            time_last = time_now;
            seq_keep(new_msg); // keep before propagation

            if (ekf_initialized) {
                system_pub(X_state, new_msg->header.stamp);
            }
            // cout << "first frame imu" << endl;
        }
        else
        {
            seq_keep(new_msg); // keep before propagation
            // cout << "\033[1;32m[ INFO] [IMU] [time-sync] [seq_keep] [OK] \033[0m" << endl;
            time_now = new_msg->header.stamp.toSec();
            dt = time_now - time_last;
            if (dt <= 0.0)
            {
                ROS_WARN_THROTTLE(1.0,
                                  "Ignoring out-of-order IMU sample: dt %.6f s at %.3f s",
                                  dt,
                                  new_msg->header.stamp.toSec());
                return;
            }
            if (dt > 1.0e-4)
            {
                sample_freq = 1.0 / dt;
            }

            MatrixXd Ft;
            MatrixXd Vt;

            u_gyro(0) = new_msg->angular_velocity.x;
            u_gyro(1) = new_msg->angular_velocity.y;
            u_gyro(2) = new_msg->angular_velocity.z;
            u_acc(0) = new_msg->linear_acceleration.x;
            u_acc(1) = new_msg->linear_acceleration.y;
            u_acc(2) = new_msg->linear_acceleration.z;

            q_last.w() = X_state(3);
            q_last.x() = X_state(4);
            q_last.y() = X_state(5);
            q_last.z() = X_state(6);

            bg_last = X_state.segment<3>(10);
            ba_last = X_state.segment<3>(13);

            // 离散化的一阶误差状态传播：
            //   dx_k ~= (I + dt * F) dx_{k-1} + dt * V n
            // 其中 F 来自 diff_f_diff_x()，V 来自 diff_f_diff_n()。
            Ft = MatrixXd::Identity(errorstateSize, errorstateSize) + dt * diff_f_diff_x(q_last, u_gyro, u_acc, bg_last, ba_last);

            Vt = dt * diff_f_diff_n(q_last);
            const bool odom_lost_for_prediction =
                odom_is_lost_at(time_now) && last_accepted_gnss_velocity_initialized;
            const Vector3d position_before_prediction = X_state.segment<3>(0);
            X_state = propagate_nominal_state(X_state, u_gyro, u_acc, dt);
            if (odom_lost_for_prediction)
            {
                // odom 丢失时，纯 IMU 积分会快速漂移。若 GNSS 已提供可信的
                // 近似速度，则用该速度约束位置/速度预测，保证退化模式下输出
                // 仍具有连续性。真正的 GNSS 观测更新仍在 gnss_fix_callback()
                // 中完成。
                X_state.segment<3>(0) =
                    position_before_prediction + last_accepted_gnss_velocity * dt;
                X_state.segment<3>(7) = last_accepted_gnss_velocity;
            }
           

            StateCovariance = Ft * StateCovariance * Ft.transpose() + Vt * Qt * Vt.transpose();
            apply_bias_random_walk_covariance(dt);

            time_last = time_now;

            Eigen::VectorXd X_state_ahead = propagate_nominal_state(X_state, u_gyro, u_acc, 0.01);

            if (ekf_initialized) {
                system_pub(X_state, new_msg->header.stamp);
                ahead_system_pub(X_state_ahead, new_msg->header.stamp);
            }
            drain_pending_odom_measurements();

            // cout << "[IMU] [time-sync] [OK]" << endl;

            // system_pub(X_state, ros::Time::now());
            // ahead_system_pub(X_state_ahead, ros::Time::now());
        }
    }
}

Matrix3d Rr_i;
Vector3d tr_i; //  rigid body in imu frame
nav_msgs::Path input_path_msg;
nav_msgs::Path ekf_path_msg;
nav_msgs::Path measurement_path_msg;
nav_msgs::Path gnss_path_msg;
bool output_filter_initialized = false;
Vector3d output_filter_state = Vector3d::Zero();
Vector3d output_filter_velocity = Vector3d::Zero();
double last_output_filter_time = -1.0;
double output_smoothing_recovery_until = -1.0;
int input_path_counter = 0;
int ekf_path_counter = 0;
int measurement_path_counter = 0;
int gnss_path_counter = 0;

std::string active_odom_source;
double active_odom_source_start_time = -1.0;
bool primary_odom_received = false;
double primary_odom_first_time = -1.0;
double fallback_odom_first_time = -1.0;
bool odom_measurement_position_initialized = false;
bool odom_ever_initialized = false;
Vector3d last_odom_measurement_position = Vector3d::Zero();
Quaterniond last_odom_measurement_orientation = Quaterniond::Identity();
std::deque<std::pair<double, Vector3d>> odom_position_history;
Matrix3d odom_alignment_R = Matrix3d::Identity();
Vector3d odom_alignment_t = Vector3d::Zero();
int odom_realign_count = 0;
int odom_realign_settle_count = 0;
int odom_recovery_frames_remaining = 0;
int odom_update_count = 0;
int odom_weak_count = 0;
int odom_lost_count = 0;
double last_odom_observation_scale = 1.0;
double last_odom_health_score = 1.0;
double last_odom_message_time = -1.0;
bool odom_loss_reported = false;
double last_odom_gnss_consistency_score = 1.0;
double last_odom_gnss_consistency_scale = 1.0;
double last_odom_gnss_consistency_time = -1.0;
int gnss_update_count = 0;
int gnss_weak_count = 0;
int gnss_reject_count = 0;
int gnss_motion_inconsistent_count = 0;
double last_gnss_health_score = 0.0;
bool gnss_origin_initialized = false;
bool gnss_alignment_initialized = false;
bool gnss_yaw_alignment_initialized = false;
double gnss_origin_lat_rad = 0.0;
double gnss_origin_lon_rad = 0.0;
double gnss_origin_alt = 0.0;
double gnss_origin_cos_lat = 1.0;
double last_gnss_update_time = -1.0;
double last_gnss_alignment_sample_time = -1.0;
Matrix3d gnss_alignment_R = Matrix3d::Identity();
Vector3d gnss_alignment_offset = Vector3d::Zero();
std::deque<std::pair<Vector3d, Vector3d>> gnss_alignment_pairs;
bool last_gnss_motion_reference_initialized = false;
Vector3d last_gnss_motion_reference = Vector3d::Zero();
Vector3d last_odom_motion_reference = Vector3d::Zero();
visualization_msgs::MarkerArray ekf_segment_markers;
visualization_msgs::MarkerArray ekf_arrow_markers;
size_t ekf_active_segment_index = 0;
int ekf_arrow_counter = 0;
int ekf_arrow_next_id = 0;

/// @brief Forward declaration for caching odom positions for GNSS alignment.
void record_odom_position_for_gnss_sync(const ros::Time &stamp, const Vector3d &position);
/// @brief Forward declaration for GNSS/odom timestamp lookup.
bool lookup_odom_position_for_gnss_sync(const ros::Time &stamp, Vector3d &position);

/// @brief Create one RViz line-strip marker used to display a continuous EKF segment.
visualization_msgs::Marker make_ekf_segment_marker(const std::string &frame_id, const ros::Time &stamp, int id)
{
    visualization_msgs::Marker marker;
    marker.header.frame_id = frame_id;
    marker.header.stamp = stamp;
    marker.ns = "ekf_segments";
    marker.id = id;
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.04;
    marker.color.r = 80.0 / 255.0;
    marker.color.g = 1.0;
    marker.color.b = 120.0 / 255.0;
    marker.color.a = 1.0;
    marker.lifetime = ros::Duration(0);
    return marker;
}

/// @brief Create one RViz arrow marker for visualizing current EKF pose heading.
visualization_msgs::Marker make_ekf_arrow_marker(const std::string &frame_id,
                                                 const ros::Time &stamp,
                                                 int id,
                                                 const Vector3d &position,
                                                 const Quaterniond &orientation)
{
    visualization_msgs::Marker marker;
    marker.header.frame_id = frame_id;
    marker.header.stamp = stamp;
    marker.ns = "ekf_pose_arrows";
    marker.id = id;
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = position.x();
    marker.pose.position.y = position.y();
    marker.pose.position.z = position.z();
    marker.pose.orientation.w = orientation.w();
    marker.pose.orientation.x = orientation.x();
    marker.pose.orientation.y = orientation.y();
    marker.pose.orientation.z = orientation.z();
    marker.scale.x = 0.55;
    marker.scale.y = 0.08;
    marker.scale.z = 0.08;
    marker.color.r = 80.0 / 255.0;
    marker.color.g = 1.0;
    marker.color.b = 120.0 / 255.0;
    marker.color.a = 0.75;
    marker.lifetime = ros::Duration(0);
    return marker;
}

/// @brief Append a downsampled EKF heading arrow to the marker array.
void append_ekf_arrow_marker(const std::string &frame_id,
                             const Vector3d &position,
                             const Quaterniond &orientation,
                             const ros::Time &stamp)
{
    ekf_arrow_counter++;
    if (ekf_arrow_counter % std::max(1, arrow_publish_stride) != 0)
    {
        return;
    }
    if (static_cast<int>(ekf_arrow_markers.markers.size()) >= arrow_max_markers)
    {
        return;
    }
    ekf_arrow_markers.markers.push_back(make_ekf_arrow_marker(frame_id,
                                                              stamp,
                                                              ekf_arrow_next_id++,
                                                              position,
                                                              orientation));
    ekf_arrows_pub.publish(ekf_arrow_markers);
}

/// @brief Ensure the EKF segment marker array has an active segment to append to.
void ensure_ekf_segment(const std::string &frame_id, const ros::Time &stamp)
{
    if (ekf_segment_markers.markers.empty())
    {
        ekf_segment_markers.markers.push_back(make_ekf_segment_marker(frame_id, stamp, 0));
        ekf_active_segment_index = 0;
    }
}

/// @brief Append the current fused position to the active EKF trajectory segment.
void append_pose_to_ekf_segments(const std::string &frame_id,
                                 const Vector3d &position,
                                 const ros::Time &stamp)
{
    ensure_ekf_segment(frame_id, stamp);
    visualization_msgs::Marker &marker = ekf_segment_markers.markers[ekf_active_segment_index];
    marker.header.stamp = stamp;
    geometry_msgs::Point point;
    point.x = position.x();
    point.y = position.y();
    point.z = position.z();
    marker.points.push_back(point);
    ekf_segments_pub.publish(ekf_segment_markers);
}

/// @brief Start a new EKF path segment after reset, relocalization, or reinitialization.
void start_new_ekf_segment(const std::string &frame_id, const ros::Time &stamp)
{
    ensure_ekf_segment(frame_id, stamp);
    if (ekf_segment_markers.markers[ekf_active_segment_index].points.empty())
    {
        ekf_segment_markers.markers[ekf_active_segment_index].header.stamp = stamp;
        ekf_segment_markers.markers[ekf_active_segment_index].header.frame_id = frame_id;
        return;
    }
    int next_id = static_cast<int>(ekf_segment_markers.markers.size());
    ekf_segment_markers.markers.push_back(make_ekf_segment_marker(frame_id, stamp, next_id));
    ekf_active_segment_index = ekf_segment_markers.markers.size() - 1;
    ekf_segments_pub.publish(ekf_segment_markers);
}

/// @brief Append a pose to a nav_msgs/Path with stride-based downsampling and length limit.
void append_pose_to_path(nav_msgs::Path &path_msg,
                         ros::Publisher &path_pub,
                         const std::string &frame_id,
                         const Vector3d &position,
                         const Quaterniond &orientation,
                         const ros::Time &stamp,
                         int &counter)
{
    counter++;
    if (counter % std::max(1, path_publish_stride) != 0)
    {
        return;
    }

    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header.stamp = stamp;
    pose_stamped.header.frame_id = frame_id;
    pose_stamped.pose.position.x = position.x();
    pose_stamped.pose.position.y = position.y();
    pose_stamped.pose.position.z = position.z();
    pose_stamped.pose.orientation.w = orientation.w();
    pose_stamped.pose.orientation.x = orientation.x();
    pose_stamped.pose.orientation.y = orientation.y();
    pose_stamped.pose.orientation.z = orientation.z();

    path_msg.header.stamp = stamp;
    path_msg.header.frame_id = frame_id;
    path_msg.poses.push_back(pose_stamped);
    if (static_cast<int>(path_msg.poses.size()) > path_max_points)
    {
        path_msg.poses.erase(path_msg.poses.begin(),
                             path_msg.poses.begin() + (path_msg.poses.size() - path_max_points));
    }
    path_pub.publish(path_msg);
}

/// @brief Convert incoming odometry pose from rigid-body frame to IMU-center pose.
VectorXd get_pose_from_VIOodom(const nav_msgs::Odometry::ConstPtr &msg)
{
    Matrix3d Rr_w; // rigid body in world
    Vector3d tr_w;
    Matrix3d Ri_w;
    Vector3d ti_w;
    Vector3d p_temp;
    p_temp(0) = msg->pose.pose.position.x;
    p_temp(1) = msg->pose.pose.position.y;
    p_temp(2) = msg->pose.pose.position.z;
    Quaterniond q;
    q.w() = msg->pose.pose.orientation.w;
    q.x() = msg->pose.pose.orientation.x;
    q.y() = msg->pose.pose.orientation.y;
    q.z() = msg->pose.pose.orientation.z;
    q.normalize();

    // Convert the reported rigid-body pose to the IMU-center pose using the
    // configured rigid-body-to-IMU extrinsic Rr_i/tr_i.
    Rr_w = q.toRotationMatrix();
    tr_w = p_temp;
    Ri_w = Rr_w * Rr_i.inverse();
    ti_w = tr_w - Ri_w * tr_i;
    Quaterniond q_wi = Quaterniond(Ri_w);

    VectorXd pose = VectorXd::Random(7);
    pose.segment<3>(0) = ti_w;
    pose.segment<4>(3) = Vector4d(q_wi.w(), q_wi.x(), q_wi.y(), q_wi.z());

    return pose;
}

/// @brief Extract yaw angle from a quaternion using the ZYX convention.
double yaw_from_quaternion(const Quaterniond &q_in)
{
    Quaterniond q = q_in.normalized();
    return std::atan2(2.0 * (q.w() * q.z() + q.x() * q.y()),
                      1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z()));
}

/// @brief Wrap an angle into [-pi, pi] for bounded yaw updates.
double normalize_yaw_angle(double angle)
{
    return std::atan2(std::sin(angle), std::cos(angle));
}

/// @brief Build a pure-Z yaw rotation matrix for 2D frame alignment.
Matrix3d yaw_rotation(double yaw)
{
    Matrix3d R = Matrix3d::Identity();
    const double c = std::cos(yaw);
    const double s = std::sin(yaw);
    R(0, 0) = c;
    R(0, 1) = -s;
    R(1, 0) = s;
    R(1, 1) = c;
    return R;
}

/// @brief Apply the current odom-frame yaw and translation alignment to an odom pose.
VectorXd apply_odom_alignment(const VectorXd &raw_pose)
{
    VectorXd aligned_pose = raw_pose;
    Quaterniond q_raw(raw_pose(3), raw_pose(4), raw_pose(5), raw_pose(6));
    q_raw.normalize();
    Quaterniond q_align(odom_alignment_R);
    Quaterniond q_aligned = (q_align * q_raw).normalized();
    aligned_pose.segment<3>(0) = odom_alignment_R * raw_pose.segment<3>(0) + odom_alignment_t;
    aligned_pose.segment<4>(3) = Vector4d(q_aligned.w(), q_aligned.x(), q_aligned.y(), q_aligned.z());
    return aligned_pose;
}

/// @brief Re-estimate odom-frame alignment after detecting a large odom jump.
void realign_odom_frame(const VectorXd &raw_pose, const ros::Time &stamp, double odom_step)
{
    if (!odom_measurement_position_initialized)
    {
        return;
    }
    Quaterniond q_raw(raw_pose(3), raw_pose(4), raw_pose(5), raw_pose(6));
    q_raw.normalize();
    const double yaw_delta = yaw_from_quaternion(last_odom_measurement_orientation) - yaw_from_quaternion(q_raw);
    odom_alignment_R = yaw_rotation(yaw_delta);
    odom_alignment_t = last_odom_measurement_position - odom_alignment_R * raw_pose.segment<3>(0);
    odom_realign_count++;
    odom_realign_settle_count = std::max(1, odom_realign_settle_frames);
    ROS_WARN("Realigned odom frame after %.3f m jump at %.3f s: yaw_delta %.3f rad, offset %.3f %.3f %.3f",
             odom_step,
             stamp.toSec(),
             yaw_delta,
             odom_alignment_t.x(),
             odom_alignment_t.y(),
             odom_alignment_t.z());
}

/// @brief Compute an adaptive observation covariance scale from innovation size.
double adaptive_observation_scale(double innovation_norm, double start_threshold, double reject_threshold, double max_scale)
{
    if (!enable_adaptive_observation_covariance || innovation_norm <= start_threshold)
    {
        return 1.0;
    }
    if (innovation_norm >= reject_threshold)
    {
        return max_scale;
    }
    const double ratio = (innovation_norm - start_threshold) / std::max(1.0e-6, reject_threshold - start_threshold);
    return 1.0 + ratio * ratio * (max_scale - 1.0);
}

/// @brief Smoothly map a scalar health metric into a bounded covariance scale.
double bounded_adaptive_scale(double value, double start_threshold, double reject_threshold, double max_scale)
{
    if (value <= start_threshold)
    {
        return 1.0;
    }
    if (value >= reject_threshold)
    {
        return max_scale;
    }
    const double ratio = (value - start_threshold) / std::max(1.0e-6, reject_threshold - start_threshold);
    return 1.0 + ratio * ratio * (max_scale - 1.0);
}

/// @brief Clamp a scalar score into the [0, 1] interval.
double clamp01(double value)
{
    return std::max(0.0, std::min(1.0, value));
}

/// @brief Clamp a vector magnitude while preserving its direction.
Vector3d clamp_vector_norm(const Vector3d &value, double max_norm)
{
    if (max_norm <= 0.0)
    {
        return value;
    }
    const double norm = value.norm();
    if (norm <= max_norm || norm <= 1.0e-12)
    {
        return value;
    }
    return value * (max_norm / norm);
}

/// @brief Convert a metric where lower is better into a descending health score.
double descending_score(double value, double good_value, double poor_value)
{
    if (value <= good_value)
    {
        return 1.0;
    }
    if (value >= poor_value)
    {
        return 0.0;
    }
    return 1.0 - (value - good_value) / std::max(1.0e-6, poor_value - good_value);
}

/// @brief Store accepted GNSS positions for windowed velocity estimation.
void push_accepted_gnss_sample(double stamp, const Vector3d &position)
{
    if (!accepted_gnss_history.empty() && stamp <= accepted_gnss_history.back().first)
    {
        accepted_gnss_history.clear();
    }
    accepted_gnss_history.push_back(std::make_pair(stamp, position));
    const size_t max_samples = static_cast<size_t>(std::max(2, gnss_velocity_window_size));
    while (accepted_gnss_history.size() > max_samples)
    {
        accepted_gnss_history.pop_front();
    }
}

/// @brief Estimate GNSS velocity by fitting a line over the recent accepted GNSS window.
bool estimate_gnss_window_velocity(double stamp, const Vector3d &position, Vector3d &velocity, double &window_dt)
{
    std::vector<std::pair<double, Vector3d>> samples;
    const size_t max_samples = static_cast<size_t>(std::max(2, gnss_velocity_window_size));
    for (auto it = accepted_gnss_history.rbegin(); it != accepted_gnss_history.rend() && samples.size() + 1 < max_samples; ++it)
    {
        const double age = stamp - it->first;
        if (age < -1.0e-9)
        {
            continue;
        }
        if (age > gnss_velocity_max_dt * std::max(1, gnss_velocity_window_size - 1))
        {
            break;
        }
        samples.push_back(*it);
    }
    samples.push_back(std::make_pair(stamp, position));
    if (samples.size() < 2)
    {
        return false;
    }

    double min_time = samples.front().first;
    double max_time = samples.front().first;
    double mean_time = 0.0;
    Vector3d mean_position = Vector3d::Zero();
    for (const auto &sample : samples)
    {
        min_time = std::min(min_time, sample.first);
        max_time = std::max(max_time, sample.first);
        mean_time += sample.first;
        mean_position += sample.second;
    }
    mean_time /= static_cast<double>(samples.size());
    mean_position /= static_cast<double>(samples.size());
    window_dt = max_time - min_time;
    if (window_dt < gnss_velocity_min_dt)
    {
        return false;
    }

    double denominator = 0.0;
    Vector3d numerator = Vector3d::Zero();
    for (const auto &sample : samples)
    {
        const double centered_time = sample.first - mean_time;
        denominator += centered_time * centered_time;
        numerator += centered_time * (sample.second - mean_position);
    }
    if (denominator <= 1.0e-9)
    {
        return false;
    }
    velocity = numerator / denominator;
    return true;
}

/// @brief Update the low-pass filtered GNSS velocity reference used during odom loss.
bool update_smoothed_gnss_velocity(double stamp, const Vector3d &position, Vector3d &velocity, double &window_dt)
{
    Vector3d raw_velocity = Vector3d::Zero();
    if (!estimate_gnss_window_velocity(stamp, position, raw_velocity, window_dt))
    {
        return false;
    }
    const double alpha = std::max(0.0, std::min(1.0, gnss_velocity_smoothing_alpha));
    if (last_accepted_gnss_velocity_initialized)
    {
        last_accepted_gnss_velocity = alpha * raw_velocity + (1.0 - alpha) * last_accepted_gnss_velocity;
    }
    else
    {
        last_accepted_gnss_velocity = raw_velocity;
        last_accepted_gnss_velocity_initialized = true;
    }
    velocity = last_accepted_gnss_velocity;
    return true;
}

struct SensorHealthMonitor
{
    // Sliding-window GNSS NIS state machine:
    //   HEALTHY    normal covariance.
    //   DEGRADED   NIS has been high often enough; inflate covariance.
    //   ISOLATED   severe NIS repeated; reject GNSS for a short isolation time.
    //   RECOVERING isolation ended; require several normal samples before
    //              returning to HEALTHY.
    enum State
    {
        HEALTHY = 0,
        DEGRADED = 1,
        ISOLATED = 2,
        RECOVERING = 3
    };

    struct Decision
    {
        State state;
        double scale;
        bool reject;
    };

    std::deque<int> window;
    State state = HEALTHY;
    double isolated_until = -1.0;
    int normal_recover_count = 0;

    /// @brief Reset the NIS-based health state machine to HEALTHY.
    void reset()
    {
        window.clear();
        state = HEALTHY;
        isolated_until = -1.0;
        normal_recover_count = 0;
    }

    /// @brief Return a printable name for the current health state.
    const char *state_name() const
    {
        switch (state)
        {
        case HEALTHY:
            return "HEALTHY";
        case DEGRADED:
            return "DEGRADED";
        case ISOLATED:
            return "ISOLATED";
        case RECOVERING:
            return "RECOVERING";
        default:
            return "UNKNOWN";
        }
    }

    /// @brief Update GNSS health state from NIS and return scale/reject decision.
    Decision update(double nis,
                    double stamp,
                    int window_size,
                    int degraded_count_threshold,
                    int severe_count_threshold,
                    int recover_count_threshold,
                    double isolation_time,
                    double degraded_threshold,
                    double severe_threshold,
                    double degraded_scale,
                    double severe_scale)
    {
        const bool severe = nis >= severe_threshold;
        const bool degraded = nis >= degraded_threshold;

        if (state == ISOLATED)
        {
            if (stamp < isolated_until)
            {
                return {state, severe_scale, true};
            }
            state = RECOVERING;
            normal_recover_count = 0;
            window.clear();
        }

        int level = 0;
        if (severe)
        {
            level = 2;
        }
        else if (degraded)
        {
            level = 1;
        }

        window.push_back(level);
        while (static_cast<int>(window.size()) > std::max(1, window_size))
        {
            window.pop_front();
        }

        int degraded_count = 0;
        int severe_count = 0;
        for (const int sample : window)
        {
            if (sample >= 1)
            {
                degraded_count++;
            }
            if (sample >= 2)
            {
                severe_count++;
            }
        }

        if (severe_count >= severe_count_threshold)
        {
            state = ISOLATED;
            isolated_until = stamp + isolation_time;
            normal_recover_count = 0;
            return {state, severe_scale, true};
        }

        if (state == RECOVERING)
        {
            if (level == 0)
            {
                normal_recover_count++;
                if (normal_recover_count >= recover_count_threshold)
                {
                    state = HEALTHY;
                    window.clear();
                }
            }
            else
            {
                normal_recover_count = 0;
                state = DEGRADED;
            }
        }
        else if (degraded_count >= degraded_count_threshold)
        {
            state = DEGRADED;
        }
        else
        {
            state = HEALTHY;
        }

        if (state == DEGRADED || state == RECOVERING || severe)
        {
            return {state, severe ? severe_scale : degraded_scale, false};
        }
        return {state, 1.0, false};
    }
};

SensorHealthMonitor gnss_nis_monitor;

/// @brief Whether odom has reappeared after being LOST and should be trusted slowly.
bool odom_recovery_active()
{
    return enable_odom_recovery_guard && odom_recovery_frames_remaining > 0;
}

/// @brief Whether the output smoother should still protect a recent odom recovery.
bool output_smoothing_recovery_output_active(double stamp_sec)
{
    return output_smoothing_recovery_duration > 0.0 &&
           output_smoothing_recovery_until > 0.0 &&
           stamp_sec <= output_smoothing_recovery_until;
}

/// @brief Return the current odom recovery covariance scale with smooth decay.
double odom_recovery_scale_for_current_frame()
{
    if (!odom_recovery_active())
    {
        return 1.0;
    }
    const int total = std::max(1, odom_recovery_frames);
    const double progress = static_cast<double>(odom_recovery_frames_remaining - 1) /
                            static_cast<double>(total);
    const double high = std::max(odom_recovery_min_scale, odom_recovery_scale);
    return odom_recovery_min_scale + progress * progress * (high - odom_recovery_min_scale);
}

/// @brief Start a short covariance-inflation window after odom recovers from LOST.
void start_odom_recovery_guard(const ros::Time &stamp, double raw_step)
{
    if (!enable_odom_recovery_guard)
    {
        return;
    }
    odom_recovery_frames_remaining =
        std::max(odom_recovery_frames_remaining, std::max(1, odom_recovery_frames));
    if (output_smoothing_recovery_duration > 0.0)
    {
        output_smoothing_recovery_until =
            std::max(output_smoothing_recovery_until,
                     stamp.toSec() + output_smoothing_recovery_duration);
    }
    ROS_WARN("Odom recovery guard active after LOST at %.3f s: raw_step %.3f m, guarded_frames=%d scale>=%.1f",
             stamp.toSec(),
             raw_step,
             odom_recovery_frames_remaining,
             odom_recovery_scale);
}

/// @brief Fuse covariance, NIS, motion, and status scores into one GNSS health score.
double gnss_health_score_from_factors(double covariance_score,
                                      double nis_score,
                                      double motion_score,
                                      double status_score)
{
    return clamp01(0.35 * covariance_score +
                   0.35 * nis_score +
                   0.20 * motion_score +
                   0.10 * status_score);
}

/// @brief Compute odom observation scale and update odom health counters.
double odom_health_scale(double innovation_norm, double stamp_sec)
{
    last_odom_message_time = stamp_sec;
    odom_loss_reported = false;
    double scale = adaptive_observation_scale(innovation_norm,
                                              odom_adaptive_threshold,
                                              odom_adaptive_reject_threshold,
                                              odom_adaptive_max_scale);
    if (enable_odom_gnss_consistency_health &&
        last_gnss_health_score >= gnss_health_trust_threshold &&
        last_odom_gnss_consistency_time > 0.0 &&
        std::abs(stamp_sec - last_odom_gnss_consistency_time) <= odom_gnss_consistency_timeout)
    {
        scale = std::max(scale, last_odom_gnss_consistency_scale);
    }
    if (odom_realign_settle_count > 0)
    {
        scale = std::max(scale, odom_adaptive_max_scale);
        odom_realign_settle_count--;
    }
    if (odom_recovery_active())
    {
        scale = std::max(scale, odom_recovery_scale_for_current_frame());
        odom_recovery_frames_remaining--;
    }
    odom_update_count++;
    if (scale > 1.0)
    {
        odom_weak_count++;
        ROS_WARN_THROTTLE(1.0,
                          "Odom observation health=WEAK innovation %.3f m scale %.2f gnss_score %.2f odom_gnss_score %.2f odom_gnss_scale %.2f updates=%d weak=%d realign=%d settle_left=%d recovery_left=%d",
                          innovation_norm,
                          scale,
                          last_gnss_health_score,
                          last_odom_gnss_consistency_score,
                          last_odom_gnss_consistency_scale,
                          odom_update_count,
                          odom_weak_count,
                          odom_realign_count,
                          odom_realign_settle_count,
                          odom_recovery_frames_remaining);
    }
    else
    {
        ROS_INFO_THROTTLE(5.0,
                          "Odom observation health=HEALTHY innovation %.3f m updates=%d weak=%d realign=%d",
                          innovation_norm,
                          odom_update_count,
                          odom_weak_count,
                          odom_realign_count);
    }
    last_odom_observation_scale = scale;
    last_odom_health_score = clamp01(1.0 / std::max(1.0, scale));
    return scale;
}

/// @brief Update odom-loss state based on timeout and last accepted odom timestamp.
bool update_odom_loss_health(double stamp_sec)
{
    if (odom_loss_timeout <= 0.0)
    {
        return false;
    }
    if (!odom_ever_initialized && ekf_initialized && !first_frame_tag_odom)
    {
        last_odom_health_score = 0.0;
        last_odom_observation_scale = odom_adaptive_max_scale;
        if (!odom_loss_reported)
        {
            odom_lost_count++;
            odom_loss_reported = true;
        }
        ROS_WARN_THROTTLE(1.0,
                          "Odom observation health=LOST no odom source initialized lost=%d",
                          odom_lost_count);
        return true;
    }
    if (last_odom_message_time <= 0.0)
    {
        return false;
    }
    const double age = stamp_sec - last_odom_message_time;
    if (age <= odom_loss_timeout)
    {
        return false;
    }

    last_odom_health_score = 0.0;
    last_odom_observation_scale = odom_adaptive_max_scale;
    if (!odom_loss_reported)
    {
        odom_lost_count++;
        odom_loss_reported = true;
    }
    ROS_WARN_THROTTLE(1.0,
                      "Odom observation health=LOST age %.3f s timeout %.3f s lost=%d",
                      age,
                      odom_loss_timeout,
                      odom_lost_count);
    return true;
}

/// @brief Query whether odom should be treated as lost at a given timestamp.
bool odom_is_lost_at(double stamp_sec)
{
    if (odom_loss_timeout <= 0.0)
    {
        return false;
    }
    if (!odom_ever_initialized && ekf_initialized && !first_frame_tag_odom)
    {
        return true;
    }
    return last_odom_message_time > 0.0 &&
           (stamp_sec - last_odom_message_time) > odom_loss_timeout;
}

/// @brief Hard-reset nominal state and covariance from an odom pose measurement.
void reset_filter_to_measurement(const VectorXd &odom_pose, const ros::Time &stamp, const char *reason)
{
    ROS_WARN("Resetting EKF state from odom measurement at %.3f s: %s", stamp.toSec(), reason);
    X_state.segment<3>(0) = odom_pose.segment<3>(0);
    X_state.segment<4>(3) = odom_pose.segment<4>(3);
    normalize_state_quaternion(X_state);
    X_state.segment<3>(7).setZero();
    StateCovariance = MatrixXd::Identity(errorstateSize, errorstateSize);
    sys_seq.clear();
    cov_seq.clear();
    imu_front_time = ros::Time(0);
    imu_back_time = ros::Time(0);
    last_odom_message_time = stamp.toSec();
    odom_ever_initialized = true;
    odom_loss_reported = false;
    last_gnss_motion_reference_initialized = false;
    last_accepted_gnss_position_initialized = false;
    last_accepted_gnss_velocity_initialized = false;
    accepted_gnss_history.clear();
    gnss_nis_monitor.reset();
    ekf_path_msg.poses.clear();
    ekf_path_counter = 0;
    reset_output_motion_smoother();
    start_new_ekf_segment(world_frame_id, stamp);
}

/// @brief Select primary or fallback odometry source with startup grace handling.
bool should_use_odom_source(const nav_msgs::Odometry::ConstPtr &msg,
                            const std::string &source_name,
                            bool is_primary)
{
    const double stamp = msg->header.stamp.toSec();
    if (is_primary)
    {
        primary_odom_received = true;
        if (primary_odom_first_time < 0.0)
        {
            primary_odom_first_time = stamp;
        }
    }
    else if (fallback_odom_first_time < 0.0)
    {
        fallback_odom_first_time = stamp;
    }

    if (active_odom_source.empty())
    {
        if (!is_primary && (stamp - fallback_odom_first_time) < odom_source_switch_grace)
        {
            return false;
        }
        active_odom_source = source_name;
        active_odom_source_start_time = stamp;
        ROS_INFO("Using odometry source: %s", active_odom_source.c_str());
        return true;
    }

    if (active_odom_source == source_name)
    {
        return true;
    }

    if (is_primary && (stamp - active_odom_source_start_time) <= odom_source_switch_grace)
    {
        ROS_WARN("Switching odometry source from %s to %s during startup grace window",
                 active_odom_source.c_str(),
                 source_name.c_str());
        active_odom_source = source_name;
        return true;
    }

    return false;
}

/// @brief Build the odom measurement covariance, optionally from nav_msgs/Odometry covariance.
MatrixXd odom_measurement_covariance_from_msg(const nav_msgs::Odometry::ConstPtr &msg)
{
    MatrixXd R = Rt;
    if (!odom_use_msg_covariance)
    {
        return R;
    }

    const boost::array<double, 36> &cov = msg->pose.covariance;
    const int indices[6] = {0, 7, 14, 21, 28, 35};
    bool has_covariance = false;
    for (int i = 0; i < 6; ++i)
    {
        const double value = cov[indices[i]];
        if (std::isfinite(value) && value > 1.0e-12)
        {
            has_covariance = true;
            break;
        }
    }
    if (!has_covariance)
    {
        return R;
    }

    R.setZero();
    R(0, 0) = std::max(odom_msg_min_position_cov, cov[0]);
    R(1, 1) = std::max(odom_msg_min_position_cov, cov[7]);
    R(2, 2) = std::max(odom_msg_min_position_cov, cov[14]);
    R(3, 3) = std::max(odom_msg_min_orientation_cov, cov[21]);
    R(4, 4) = std::max(odom_msg_min_orientation_cov, cov[28]);
    R(5, 5) = std::max(odom_msg_min_orientation_cov, cov[35]);
    return R;
}

struct OdomPoseResidual
{
    VectorXd innovation;
    double position_norm;
    double rotation_norm;
};

/// @brief Build the 6D odom residual used by both latest-state and replayed updates.
///
/// z_measurement and predicted_measurement are both stored as 7D pose vectors
/// [px, py, pz, qw, qx, qy, qz]. The Kalman residual, however, must be 6D:
///   - first 3 rows: direct position difference z_p - p
///   - last 3 rows: SO(3) logarithm of q_pred.inverse() * q_meas
///
/// The quaternion residual is expressed in the same 3D minimal attitude-error
/// coordinates as dx(dtheta). This keeps H = diff_g_diff_x() simple and makes
/// the later boxplus() correction mathematically consistent with the 15D error
/// covariance.
OdomPoseResidual build_odom_pose_residual(const VectorXd &z_measurement,
                                          const VectorXd &predicted_measurement)
{
    OdomPoseResidual residual;
    residual.innovation = VectorXd::Zero(measurementSize - 1);
    residual.innovation.segment<3>(kResidualPositionOffset) =
        z_measurement.segment<3>(kStatePositionOffset) -
        predicted_measurement.segment<3>(kStatePositionOffset);

    Quaterniond q_pred(predicted_measurement(kStateQuaternionOffset),
                       predicted_measurement(kStateQuaternionOffset + 1),
                       predicted_measurement(kStateQuaternionOffset + 2),
                       predicted_measurement(kStateQuaternionOffset + 3));
    q_pred.normalize();

    Quaterniond q_meas(z_measurement(kStateQuaternionOffset),
                       z_measurement(kStateQuaternionOffset + 1),
                       z_measurement(kStateQuaternionOffset + 2),
                       z_measurement(kStateQuaternionOffset + 3));
    q_meas.normalize();

    Quaterniond error_q = q_pred.inverse() * q_meas;
    error_q.normalize();
    residual.innovation.segment<3>(kResidualRotationOffset) =
        rotation_2_lie_algebra(error_q.toRotationMatrix());

    residual.position_norm =
        residual.innovation.segment<3>(kResidualPositionOffset).norm();
    residual.rotation_norm =
        residual.innovation.segment<3>(kResidualRotationOffset).norm();
    return residual;
}

/// @brief Apply an odom update directly to the latest EKF state without IMU replay.
///
/// This is not a dead path: process_vioodom() calls it when the odom stamp is
/// older than the cached IMU buffer front, or when the buffer does not contain
/// enough samples to replay to the exact odom time. In that case the filter uses
/// the latest nominal state X=[p,q,v,bg,ba] and the 15D error covariance
/// directly, avoiding an invalid time-synchronized rollback.
void update_lastest_state()
{
    MatrixXd Ct;
    MatrixXd Wt;
    // H maps the 15D error state dx into the 6D odom residual. W is currently
    // identity, but keeping it explicit makes the measurement equation
    // residual = H * dx + W * v easy to compare with standard EKF notation.
    Ct = diff_g_diff_x();
    Wt = diff_g_diff_v();

    VectorXd gg = g_model();
    const OdomPoseResidual residual = build_odom_pose_residual(Z_measurement, gg);
    const VectorXd &innovation = residual.innovation;
    const double pos_diff = residual.position_norm;
    if (pos_diff > POS_DIFF_THRESHOLD)
    {
        ROS_WARN_THROTTLE(5.0,
                          "Position diff too large between measurement and model prediction: threshold %.3f m, measured %.3f m",
                          POS_DIFF_THRESHOLD,
                          pos_diff);
        // return;
    }

    // Innovation health is converted into a covariance scale, not directly into
    // a state reset. Large residuals therefore reduce trust in this odom sample
    // while preserving the IMU-predicted state continuity.
    const double odom_scale = odom_health_scale(pos_diff, time_odom_tag_now);
    const MatrixXd R_odom = odom_scale * Wt * current_odom_Rt * Wt.transpose();
    Kt_kalmanGain = StateCovariance * Ct.transpose() * (Ct * StateCovariance * Ct.transpose() + R_odom).inverse();

    // The Kalman correction is a 15D error-state increment. boxplus() applies
    // position/velocity/bias additively and composes the 3D rotation increment
    // onto the stored unit quaternion.
    X_state = boxplus(X_state, Kt_kalmanGain * (innovation));

    joseph_covariance_update(Ct, Kt_kalmanGain, R_odom);
}
/// @brief Convert a rotation matrix residual into a 3D Lie-algebra error vector.
Vector3d rotation_2_lie_algebra(Matrix3d R)
{

    Eigen::Vector3d omega;
    // SO(3) logarithm:
    //   theta = acos((trace(R)-1)/2)
    //   omega = theta / (2 sin(theta)) * vee(R - R^T)
    //
    // Floating-point roundoff can push (trace(R)-1)/2 slightly outside
    // [-1, 1], so clamp before acos. Without the clamp, near-identity residuals
    // can become NaN and poison the Kalman update.
    const double cos_theta = std::max(-1.0, std::min(1.0, (R.trace() - 1.0) / 2.0));
    double theta = std::acos(cos_theta);

    if (theta < 1e-6)
    {
        omega << 0, 0, 0;
    }
    else
    {
        omega << R(2, 1) - R(1, 2),
            R(0, 2) - R(2, 0),
            R(1, 0) - R(0, 1);
        omega = omega * (theta / (2 * std::sin(theta)));
    }
    // Quaterniond q(R);
    // // 轴角
    // Eigen::AngleAxisd angle_axis(q);
    // omega = angle_axis.angle() * angle_axis.axis();
    return omega;
}

/// @brief Odom callback core: initialize, align, gate, time-sync update, and replay IMU.
///
/// The odom path is deliberately more trusted than GNSS for short-term motion,
/// but it still passes through jump detection and innovation-based health
/// scaling. The function has three main branches:
///   1. First odom sample initializes the EKF pose if GNSS has not already done
///      a cold start.
///   2. Future-dated odom samples are queued until the IMU buffer catches up.
///   3. Normal samples roll back to the nearest cached IMU state, apply the EKF
///      pose update, then replay the later IMU samples.
void process_vioodom(const nav_msgs::Odometry::ConstPtr &msg)
{ // assume that the odom_tag from camera is sychronized with the imus and without delay. !!!

    // current_odom_Rt 是本帧 odom 的 6x6 观测协方差。默认来自参数 Rt；
    // 如果 odom_use_msg_covariance=true 且消息携带有效 covariance，则使用
    // 消息中的对角项并施加最小方差下限。
    current_odom_Rt = odom_measurement_covariance_from_msg(msg);
    double buffertime_ms = (imu_front_time - msg->header.stamp).toSec() * 1000;
    if (buffertime_ms > 0)
    {
        ROS_WARN_THROTTLE(1.0,
                          "odom time %.2f ms older than IMU buffer front, falling back to latest-state update",
                          buffertime_ms);
    }

    // your code for update
    static Eigen::Vector3d last_pos(0, 0, 0);
    if (first_frame_tag_odom || !odom_ever_initialized)
    { // system begins in first odom frame
        // 初始化分两种情况：
        //   1. 常规启动：第一帧 odom 直接给 EKF 初始 p/q/v。
        //   2. GNSS cold start 已经初始化：第一帧 odom 不覆盖 EKF，而是估计
        //      odom frame 到当前 EKF/world frame 的 yaw+translation 对齐。
        // 这样可以保持 world_frame_id 和已有 GNSS 初始化结果一致。
        const bool initialize_filter_from_odom = first_frame_tag_odom;
        first_frame_tag_odom = false;
        time_odom_tag_now = msg->header.stamp.toSec();
        last_odom_message_time = time_odom_tag_now;
        odom_ever_initialized = true;
        odom_loss_reported = false;

        VectorXd raw_odom_pose = get_pose_from_VIOodom(msg);
        if (!initialize_filter_from_odom && ekf_initialized)
        {
            Quaterniond q_raw(raw_odom_pose(3), raw_odom_pose(4), raw_odom_pose(5), raw_odom_pose(6));
            q_raw.normalize();
            Quaterniond q_state(X_state(3), X_state(4), X_state(5), X_state(6));
            q_state.normalize();
            const double yaw_delta = yaw_from_quaternion(q_state) - yaw_from_quaternion(q_raw);
            odom_alignment_R = yaw_rotation(yaw_delta);
            odom_alignment_t = X_state.segment<3>(0) - odom_alignment_R * raw_odom_pose.segment<3>(0);
            odom_realign_count++;
            ROS_WARN("Initialized odom alignment after GNSS cold start at %.3f s: yaw_delta %.3f rad, offset %.3f %.3f %.3f",
                     msg->header.stamp.toSec(),
                     yaw_delta,
                     odom_alignment_t.x(),
                     odom_alignment_t.y(),
                     odom_alignment_t.z());
        }
        VectorXd odom_pose = apply_odom_alignment(raw_odom_pose);
        if (initialize_filter_from_odom || !ekf_initialized)
        {
            X_state.segment<3>(0) = odom_pose.segment<3>(0);
            X_state.segment<4>(3) = odom_pose.segment<4>(3);
            normalize_state_quaternion(X_state);
            X_state.segment<3>(7) << msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist.twist.linear.z;
            ekf_initialized = true;
            world_frame_id = msg->header.frame_id;
        }
        input_path_msg.header.frame_id = world_frame_id;
        measurement_path_msg.header.frame_id = world_frame_id;
        ekf_path_msg.header.frame_id = world_frame_id;
        ensure_ekf_segment(world_frame_id, msg->header.stamp);

        last_pos(0) = msg->pose.pose.position.x;
        last_pos(1) = msg->pose.pose.position.y;
        last_pos(2) = msg->pose.pose.position.z;

        last_odom_measurement_position = odom_pose.segment<3>(0);
        last_odom_measurement_orientation = Quaterniond(odom_pose(3), odom_pose(4), odom_pose(5), odom_pose(6)).normalized();
        odom_measurement_position_initialized = true;
        record_odom_position_for_gnss_sync(msg->header.stamp, last_odom_measurement_position);
        Quaterniond q_input_init;
        q_input_init.w() = msg->pose.pose.orientation.w;
        q_input_init.x() = msg->pose.pose.orientation.x;
        q_input_init.y() = msg->pose.pose.orientation.y;
        q_input_init.z() = msg->pose.pose.orientation.z;
        append_pose_to_path(input_path_msg, input_path_pub, msg->header.frame_id, last_pos, q_input_init, msg->header.stamp, input_path_counter);

        // cout << "last_pos: "<<last_pos.transpose()<<endl;

        // cout << "\033[1;33m"
        //  << "odom_tag init"
        //  << "\033[0m" << endl;

        // cout << X_state.segment<3>(0).transpose()<<endl;
    }
    else
    {
        // cout << "\033[1;33m"
        //  << "odom_tag update"
        //  << "\033[0m" << endl;
        if (sys_seq.empty() || msg->header.stamp > imu_back_time + ros::Duration(1.0e-4))
        {
            // odom 时间比当前 IMU 缓存末尾还新，说明 IMU 还没有推进到这帧
            // odom 的时间。先排队，等 imu_callback() 收到足够新的 IMU 后再
            // drain_pending_odom_measurements()，避免“未来观测”提前修正状态。
            pending_odom_measurements.push_back(msg);
            while (pending_odom_measurements.size() > max_pending_odom_measurements)
            {
                ROS_WARN_THROTTLE(1.0,
                                  "Dropping queued odom because pending queue exceeded %zu samples",
                                  max_pending_odom_measurements);
                pending_odom_measurements.pop_front();
            }
            const double wait_ms = sys_seq.empty()
                                       ? std::numeric_limits<double>::infinity()
                                       : (msg->header.stamp - imu_back_time).toSec() * 1000.0;
            ROS_WARN_THROTTLE(1.0,
                              "Queueing odom %.2f ms newer than IMU buffer back for time-sync update",
                              wait_ms);
            return;
        }
        const bool odom_lost_before_update = enable_odom_recovery_guard &&
                                             odom_is_lost_at(msg->header.stamp.toSec());
        double odom_step = (last_pos - Vector3d(msg->pose.pose.position.x,
                                                msg->pose.pose.position.y,
                                                msg->pose.pose.position.z))
                               .norm();
        if (odom_lost_before_update)
        {
            start_odom_recovery_guard(msg->header.stamp, odom_step);
        }
        if (odom_step > odom_jump_threshold && !odom_lost_before_update)
        {
            ROS_WARN("Detected odom jump %.3f m at %.3f s", odom_step, msg->header.stamp.toSec());
            VectorXd raw_odom_pose = get_pose_from_VIOodom(msg);
            if (enable_odom_realign && odom_measurement_position_initialized)
            {
                // 优先重新估计 odom frame 对齐，而不是立刻 reset EKF。这样可以
                // 处理 VIO/VO relocalization 导致的 frame 跳变，同时保留 IMU
                // 和 GNSS 已经积累的状态连续性。
                realign_odom_frame(raw_odom_pose, msg->header.stamp, odom_step);
            }
            else
            {
                // 如果无法对齐，只能把滤波器拉回当前 odom 观测。该路径会清空
                // IMU replay buffer，后续从新的参考状态继续。
                VectorXd odom_pose = apply_odom_alignment(raw_odom_pose);
                reset_filter_to_measurement(odom_pose, msg->header.stamp, "odom jump");
                last_pos(0) = msg->pose.pose.position.x;
                last_pos(1) = msg->pose.pose.position.y;
                last_pos(2) = msg->pose.pose.position.z;
                system_pub(X_state, msg->header.stamp);
                cam_system_pub(msg->header.stamp);
                return;
            }
        }

        last_pos(0) = msg->pose.pose.position.x;
        last_pos(1) = msg->pose.pose.position.y;
        last_pos(2) = msg->pose.pose.position.z;
        Quaterniond q_input;
        q_input.w() = msg->pose.pose.orientation.w;
        q_input.x() = msg->pose.pose.orientation.x;
        q_input.y() = msg->pose.pose.orientation.y;
        q_input.z() = msg->pose.pose.orientation.z;
        append_pose_to_path(input_path_msg, input_path_pub, msg->header.frame_id, last_pos, q_input, msg->header.stamp, input_path_counter);

        time_odom_tag_now = msg->header.stamp.toSec();
        //    double t = clock();

        VectorXd odom_pose = apply_odom_alignment(get_pose_from_VIOodom(msg));
        last_odom_measurement_position = odom_pose.segment<3>(0);
        last_odom_measurement_orientation = Quaterniond(odom_pose(3), odom_pose(4), odom_pose(5), odom_pose(6)).normalized();
        odom_measurement_position_initialized = true;
        record_odom_position_for_gnss_sync(msg->header.stamp, last_odom_measurement_position);

        Z_measurement.segment<3>(0) = odom_pose.segment<3>(0);
        Z_measurement.segment<4>(3) = odom_pose.segment<4>(3);


        // Time-sync update:
        // The measurement is applied at the closest cached IMU timestamp, not
        // blindly at the latest state. This avoids biasing the filter when
        // rosbag playback or sensor transport delivers odom after later IMU
        // samples.
        if (sys_seq.size() == 0 || buffertime_ms > 0)
        {
            update_lastest_state();
            cam_system_pub(msg->header.stamp);
            return;
        }
        // call back to the proper time
        if (sys_seq.size() == 1)
        {
            // ROS_ERROR("sys_seq.size() == 1");
            update_lastest_state();
            cam_system_pub(msg->header.stamp);
            return;
        }
        search_proper_frame(time_odom_tag_now);

        // cam_system_pub(msg->header.stamp);
        MatrixXd Ct;
        MatrixXd Wt;

        // 从缓存的边界状态开始做“局部重算”：
        //   sys_seq[0].first / cov_seq[0] 是 odom 时间附近的传播前 X/P；
        //   sys_seq[0].second 是该时刻对应的 IMU 输入；
        //   dt_0_rp 是这次局部预测使用的第一段 dt。
        // 先预测到 odom 可融合的时刻，再做观测更新，最后 re_propagate()
        // 重放剩余 IMU。
        dt = dt_0_rp;

        u_gyro(0) = sys_seq[0].second.angular_velocity.x;
        u_gyro(1) = sys_seq[0].second.angular_velocity.y;
        u_gyro(2) = sys_seq[0].second.angular_velocity.z;
        u_acc(0) = sys_seq[0].second.linear_acceleration.x;
        u_acc(1) = sys_seq[0].second.linear_acceleration.y;
        u_acc(2) = sys_seq[0].second.linear_acceleration.z;

        MatrixXd Ft;
        MatrixXd Vt;

        X_state = sys_seq[0].first;
        StateCovariance = cov_seq[0];


        q_last.w() = sys_seq[0].first(3);
        q_last.x() = sys_seq[0].first(4);
        q_last.y() = sys_seq[0].first(5);
        q_last.z() = sys_seq[0].first(6);

        bg_last = sys_seq[0].first.segment<3>(10);
        ba_last = sys_seq[0].first.segment<3>(13);

        Ft = MatrixXd::Identity(errorstateSize, errorstateSize) + dt * diff_f_diff_x(q_last, u_gyro, u_acc, bg_last, ba_last);

        // std::cout << "Ft" << std::endl
        //           << Ft << std::endl;

        Vt = dt * diff_f_diff_n(q_last);

        // std::cout << "Vt" << std::endl
        //   << Vt << std::endl;

        X_state = propagate_nominal_state(X_state, u_gyro, u_acc, dt);
        // std::cout << "X_state" << std::endl
        //   << X_state << std::endl;

        StateCovariance = Ft * StateCovariance * Ft.transpose() + Vt * Qt * Vt.transpose();
        apply_bias_random_walk_covariance(dt);
        // std::cout << "StateCovariance" << std::endl
        //           << StateCovariance << std::endl;

        // 在回退/局部预测后的状态上执行 odom 观测更新。Ct/H 的非零块只在
        // dp 和 dtheta 上，因为 odom 只直接观测 pose；v/bg/ba 通过 P 的
        // 交叉协方差被间接修正。
        Ct = diff_g_diff_x();
        // std::cout << "Ct" << std::endl
        //           << Ct << std::endl;
        Wt = diff_g_diff_v();

        // std::cout << "Wt" << std::endl
        //           << Wt << std::endl;

        VectorXd gg = g_model();
        const OdomPoseResidual residual = build_odom_pose_residual(Z_measurement, gg);
        const VectorXd &innovation = residual.innovation;
        const double pos_diff = residual.position_norm;
        if (pos_diff > innovation_reject_threshold)
        {
            ROS_WARN("Large innovation %.3f m at %.3f s", pos_diff, msg->header.stamp.toSec());
            if (pos_diff > innovation_reset_threshold)
            {
                if (!enable_adaptive_observation_covariance)
                {
                    reset_filter_to_measurement(odom_pose, msg->header.stamp, "large innovation");
                    system_pub(X_state, msg->header.stamp);
                    cam_system_pub(msg->header.stamp);
                    return;
                }
                ROS_WARN("Weakening odom update instead of reset: innovation %.3f m", pos_diff);
            }
        }

        // Adaptive odom R is the main protection against weak odom. The base
        // covariance can come from fixed parameters or nav_msgs/Odometry
        // covariance; the health scale then inflates it before the Kalman gain
        // is computed.
        const double odom_scale = odom_health_scale(pos_diff, msg->header.stamp.toSec());
        const MatrixXd R_odom_meas = odom_scale * Wt * current_odom_Rt * Wt.transpose();
        Kt_kalmanGain = StateCovariance * Ct.transpose() * (Ct * StateCovariance * Ct.transpose() + R_odom_meas).inverse();

        VectorXd dx(errorstateSize);
        dx = VectorXd::Zero(errorstateSize);
        dx += Kt_kalmanGain * (innovation);

        // Apply the 15D correction at the delayed IMU frame, then replay later
        // IMU samples so the published state remains at the current IMU time.
        X_state = boxplus(X_state, dx);
        joseph_covariance_update(Ct, Kt_kalmanGain, R_odom_meas);

        re_propagate();
        cam_system_pub(msg->header.stamp);
    }
}

/// @brief Process pending odom measurements once the IMU buffer has caught up.
void drain_pending_odom_measurements()
{
    while (!pending_odom_measurements.empty())
    {
        const nav_msgs::Odometry::ConstPtr &msg = pending_odom_measurements.front();
        if (sys_seq.empty() || msg->header.stamp > imu_back_time + ros::Duration(1.0e-4))
        {
            break;
        }
        nav_msgs::Odometry::ConstPtr ready_msg = msg;
        pending_odom_measurements.pop_front();
        ROS_INFO_THROTTLE(1.0,
                          "Processing queued odom after IMU buffer catch-up: delay %.2f ms pending=%zu",
                          (imu_back_time - ready_msg->header.stamp).toSec() * 1000.0,
                          pending_odom_measurements.size());
        process_vioodom(ready_msg);
    }
}

/// @brief Primary odom subscriber wrapper with source arbitration.
void vioodom_primary_callback(const nav_msgs::Odometry::ConstPtr &msg)
{
    if (should_use_odom_source(msg, "primary", true))
    {
        process_vioodom(msg);
    }
}

/// @brief Fallback odom subscriber wrapper used when the primary source is unavailable.
void vioodom_fallback_callback(const nav_msgs::Odometry::ConstPtr &msg)
{
    if (should_use_odom_source(msg, "fallback", false))
    {
        process_vioodom(msg);
    }
}

/// @brief Convert NavSatFix latitude, longitude, and altitude into a local ENU position.
bool navsat_to_local_enu(const sensor_msgs::NavSatFix::ConstPtr &msg, Vector3d &enu)
{
    if (!std::isfinite(msg->latitude) || !std::isfinite(msg->longitude) || !std::isfinite(msg->altitude))
    {
        return false;
    }

    const double lat_rad = msg->latitude * PI / 180.0;
    const double lon_rad = msg->longitude * PI / 180.0;
    if (!gnss_origin_initialized)
    {
        // 使用第一帧有效 GNSS 作为局部 ENU 原点。这是小范围无人机数据集
        // 常用的近似：后续经纬度差按地球半径投影到 East/North，海拔差作为
        // Up。若跨越很大地理范围，应改成更严格的 ECEF/ENU 转换。
        gnss_origin_initialized = true;
        gnss_origin_lat_rad = lat_rad;
        gnss_origin_lon_rad = lon_rad;
        gnss_origin_alt = msg->altitude;
        gnss_origin_cos_lat = std::cos(gnss_origin_lat_rad);
        ROS_INFO("Initialized GNSS origin: lat %.9f lon %.9f alt %.3f",
                 msg->latitude,
                 msg->longitude,
                 msg->altitude);
    }

    static const double kEarthRadiusMeters = 6378137.0;
    enu.x() = (lon_rad - gnss_origin_lon_rad) * gnss_origin_cos_lat * kEarthRadiusMeters;
    enu.y() = (lat_rad - gnss_origin_lat_rad) * kEarthRadiusMeters;
    enu.z() = msg->altitude - gnss_origin_alt;
    return true;
}

/// @brief Initialize the EKF from GNSS when odom has not yet provided an initial pose.
bool initialize_filter_from_gnss(const Vector3d &gnss_local, const ros::Time &stamp)
{
    if (!enable_gnss_cold_start)
    {
        return false;
    }

    // GNSS cold start 只初始化位置和单位姿态，速度置零。此时没有可靠 yaw，
    // 因此后续第一批 odom 会重新估计 odom 到 EKF frame 的 yaw/translation。
    first_frame_tag_odom = false;
    ekf_initialized = true;
    first_frame_imu = true;
    time_now = stamp.toSec();
    time_last = time_now;
    time_odom_tag_now = time_now;

    gnss_alignment_R = Matrix3d::Identity();
    gnss_alignment_offset = Vector3d::Zero();
    gnss_alignment_initialized = true;
    gnss_yaw_alignment_initialized = true;
    gnss_alignment_pairs.clear();

    X_state.segment<3>(0) = gnss_local;
    X_state.segment<4>(3) = Vector4d(1.0, 0.0, 0.0, 0.0);
    X_state.segment<3>(7).setZero();
    normalize_state_quaternion(X_state);

    if (!gnss_cold_start_frame_id.empty())
    {
        world_frame_id = gnss_cold_start_frame_id;
    }
    input_path_msg.header.frame_id = world_frame_id;
    measurement_path_msg.header.frame_id = world_frame_id;
    ekf_path_msg.header.frame_id = world_frame_id;
    gnss_path_msg.header.frame_id = world_frame_id;
    ensure_ekf_segment(world_frame_id, stamp);

    last_odom_health_score = 0.0;
    last_odom_observation_scale = odom_adaptive_max_scale;
    if (!odom_loss_reported)
    {
        odom_lost_count++;
        odom_loss_reported = true;
    }

    last_gnss_update_time = stamp.toSec();
    last_gnss_health_score = 1.0;
    last_accepted_gnss_position = gnss_local;
    last_accepted_gnss_position_time = stamp.toSec();
    last_accepted_gnss_position_initialized = true;
    last_accepted_gnss_velocity_initialized = false;
    accepted_gnss_history.clear();
    push_accepted_gnss_sample(stamp.toSec(), gnss_local);
    last_gnss_motion_reference_initialized = false;
    gnss_nis_monitor.reset();
    reset_output_motion_smoother();

    append_pose_to_path(gnss_path_msg,
                        gnss_path_pub,
                        world_frame_id,
                        gnss_local,
                        Quaterniond::Identity(),
                        stamp,
                        gnss_path_counter);
    system_pub(X_state, stamp);
    ROS_WARN("GNSS cold start initialized EKF at %.3f s in frame %s position %.3f %.3f %.3f",
             stamp.toSec(),
             world_frame_id.c_str(),
             gnss_local.x(),
             gnss_local.y(),
             gnss_local.z());
    return true;
}

/// @brief Store recent odom positions for later GNSS/odom timestamp matching.
void record_odom_position_for_gnss_sync(const ros::Time &stamp, const Vector3d &position)
{
    const double stamp_sec = stamp.toSec();
    if (!std::isfinite(stamp_sec))
    {
        return;
    }
    odom_position_history.emplace_back(stamp_sec, position);
    const double oldest_allowed = stamp_sec - 5.0;
    while (!odom_position_history.empty() && odom_position_history.front().first < oldest_allowed)
    {
        odom_position_history.pop_front();
    }
}

/// @brief Find the nearest odom position sample to a GNSS timestamp within tolerance.
bool lookup_odom_position_for_gnss_sync(const ros::Time &stamp, Vector3d &position)
{
    if (odom_position_history.empty())
    {
        return false;
    }

    const double stamp_sec = stamp.toSec();
    double best_dt = std::numeric_limits<double>::infinity();
    bool found = false;
    for (const auto &sample : odom_position_history)
    {
        const double dt = std::abs(sample.first - stamp_sec);
        if (dt < best_dt)
        {
            best_dt = dt;
            position = sample.second;
            found = true;
        }
    }
    return found && best_dt <= gnss_odom_sync_max_dt;
}

/// @brief Estimate GNSS-to-odom yaw and translation alignment from paired positions.
void update_gnss_alignment(const Vector3d &gnss_local, const Vector3d &odom_position, const ros::Time &stamp)
{
    if (!gnss_alignment_initialized)
    {
        // 平移先用单帧建立，保证 GNSS 位置能落入当前 EKF frame 的量级。
        // yaw 需要有足够水平运动后再估计，否则低速/静止窗口会让航向不可观。
        gnss_alignment_R = Matrix3d::Identity();
        gnss_alignment_offset = odom_position - gnss_local;
        gnss_alignment_initialized = true;
        gnss_path_msg.header.frame_id = world_frame_id;
        ROS_INFO("Initialized GNSS translation offset from odom measurement: %.3f %.3f %.3f",
                 gnss_alignment_offset.x(),
                 gnss_alignment_offset.y(),
                 gnss_alignment_offset.z());
    }

    const bool needs_initial_yaw =
        enable_gnss_yaw_alignment && !gnss_yaw_alignment_initialized;
    const bool can_refine_alignment =
        enable_gnss_alignment_refinement && gnss_yaw_alignment_initialized;
    if (!needs_initial_yaw && !can_refine_alignment)
    {
        return;
    }

    const double stamp_sec = stamp.toSec();
    if (last_gnss_alignment_sample_time > 0.0 &&
        (stamp_sec - last_gnss_alignment_sample_time) < gnss_alignment_sample_interval)
    {
        return;
    }
    last_gnss_alignment_sample_time = stamp_sec;
    gnss_alignment_pairs.emplace_back(gnss_local, odom_position);

    const int window_max_samples = can_refine_alignment
                                       ? std::max(gnss_alignment_refinement_min_samples,
                                                  std::min(gnss_alignment_max_samples,
                                                           gnss_alignment_refinement_max_samples))
                                       : gnss_alignment_max_samples;
    while (static_cast<int>(gnss_alignment_pairs.size()) > window_max_samples)
    {
        gnss_alignment_pairs.pop_front();
    }

    const int required_samples = can_refine_alignment
                                     ? std::max(gnss_alignment_min_samples,
                                                gnss_alignment_refinement_min_samples)
                                     : gnss_alignment_min_samples;
    if (static_cast<int>(gnss_alignment_pairs.size()) < required_samples)
    {
        return;
    }

    const double min_motion = can_refine_alignment
                                  ? gnss_alignment_refinement_min_motion
                                  : gnss_alignment_min_motion;
    const Vector3d gnss_motion = gnss_alignment_pairs.back().first - gnss_alignment_pairs.front().first;
    const Vector3d odom_motion = gnss_alignment_pairs.back().second - gnss_alignment_pairs.front().second;
    if (gnss_motion.head<2>().norm() < min_motion ||
        odom_motion.head<2>().norm() < min_motion)
    {
        return;
    }

    Vector3d gnss_center = Vector3d::Zero();
    Vector3d odom_center = Vector3d::Zero();
    for (const auto &pair : gnss_alignment_pairs)
    {
        gnss_center += pair.first;
        odom_center += pair.second;
    }
    gnss_center /= static_cast<double>(gnss_alignment_pairs.size());
    odom_center /= static_cast<double>(gnss_alignment_pairs.size());

    double sin_term = 0.0;
    double cos_term = 0.0;
    for (const auto &pair : gnss_alignment_pairs)
    {
        const Vector3d g = pair.first - gnss_center;
        const Vector3d o = pair.second - odom_center;
        sin_term += g.x() * o.y() - g.y() * o.x();
        cos_term += g.x() * o.x() + g.y() * o.y();
    }

    const double yaw_delta = std::atan2(sin_term, cos_term);
    const Matrix3d candidate_R = yaw_rotation(yaw_delta);
    const Vector3d candidate_offset = odom_center - candidate_R * gnss_center;
    // 使用 paired GNSS/odom 点集的 2D Procrustes-like 闭式解估计 yaw。
    // residual 检查是必要的：如果 GNSS 跳点或 odom 中途重定位，单纯的 yaw
    // 拟合可能给出数值结果，但该结果不应进入滤波器。
    double residual_sum = 0.0;
    double residual_max = 0.0;
    for (const auto &pair : gnss_alignment_pairs)
    {
        const Vector3d residual = candidate_R * pair.first + candidate_offset - pair.second;
        const double residual_norm = residual.head<2>().norm();
        residual_sum += residual_norm;
        residual_max = std::max(residual_max, residual_norm);
    }
    const double residual_mean = residual_sum / static_cast<double>(gnss_alignment_pairs.size());
    const double max_allowed_residual = can_refine_alignment
                                            ? gnss_alignment_refinement_max_residual
                                            : gnss_alignment_max_residual;
    if (residual_max > max_allowed_residual)
    {
        ROS_WARN_THROTTLE(1.0,
                          "Skipping GNSS %s alignment: residual mean %.3f max %.3f exceeds %.3f yaw_delta %.3f",
                          can_refine_alignment ? "refinement" : "yaw",
                          residual_mean,
                          residual_max,
                          max_allowed_residual,
                          yaw_delta);
        return;
    }

    if (!can_refine_alignment)
    {
        gnss_alignment_R = candidate_R;
        gnss_alignment_offset = candidate_offset;
        gnss_yaw_alignment_initialized = true;
        ROS_WARN("Initialized GNSS yaw alignment at %.3f s: samples=%zu yaw_delta=%.3f rad residual_mean=%.3f residual_max=%.3f offset %.3f %.3f %.3f",
                 stamp.toSec(),
                 gnss_alignment_pairs.size(),
                 yaw_delta,
                 residual_mean,
                 residual_max,
                 gnss_alignment_offset.x(),
                 gnss_alignment_offset.y(),
                 gnss_alignment_offset.z());
        return;
    }

    const double gain = clamp01(gnss_alignment_refinement_gain);
    if (gain <= 0.0)
    {
        return;
    }
    const double current_yaw = std::atan2(gnss_alignment_R(1, 0), gnss_alignment_R(0, 0));
    const double yaw_error = normalize_yaw_angle(yaw_delta - current_yaw);
    const double yaw_step_limit = std::max(0.0, gnss_alignment_refinement_max_yaw_step);
    const double limited_yaw_error =
        std::max(-yaw_step_limit, std::min(yaw_step_limit, yaw_error));
    const double refined_yaw = normalize_yaw_angle(current_yaw + gain * limited_yaw_error);

    Vector3d offset_delta = candidate_offset - gnss_alignment_offset;
    const double offset_delta_norm = offset_delta.norm();
    const double offset_step_limit =
        std::max(0.0, gnss_alignment_refinement_max_translation_step);
    if (offset_step_limit > 0.0 && offset_delta_norm > offset_step_limit)
    {
        offset_delta *= offset_step_limit / offset_delta_norm;
    }

    gnss_alignment_R = yaw_rotation(refined_yaw);
    gnss_alignment_offset += gain * offset_delta;
    ROS_INFO_THROTTLE(2.0,
                      "Refined GNSS alignment at %.3f s: samples=%zu candidate_yaw=%.3f current_yaw=%.3f refined_yaw=%.3f residual_mean=%.3f residual_max=%.3f offset %.3f %.3f %.3f",
                      stamp.toSec(),
                      gnss_alignment_pairs.size(),
                      yaw_delta,
                      current_yaw,
                      refined_yaw,
                      residual_mean,
                      residual_max,
                      gnss_alignment_offset.x(),
                      gnss_alignment_offset.y(),
                      gnss_alignment_offset.z());
}

/// @brief GNSS callback: ENU conversion, alignment, health gating, and position update.
///
/// GNSS update order:
///   1. Convert NavSatFix to local ENU using the first valid GNSS as origin.
///   2. Align ENU into the current EKF/odom frame with translation and optional
///      yaw estimated from synchronized GNSS/odom pairs.
///   3. Build R from NavSatFix covariance plus project-level floors.
///   4. Run NIS/Mahalanobis, motion consistency, health score, and odom/GNSS
///      consistency checks.
///   5. Apply a 3D position EKF update, or a 6D position+velocity pseudo-update
///      while odom is lost and GNSS velocity support is enabled.
void gnss_fix_callback(const sensor_msgs::NavSatFix::ConstPtr &msg)
{
    if (!use_gnss || msg->status.status < gnss_min_status)
    {
        return;
    }

    const double stamp = msg->header.stamp.toSec();
    if (last_gnss_update_time > 0.0 && (stamp - last_gnss_update_time) < gnss_min_interval)
    {
        return;
    }
    const bool odom_lost = update_odom_loss_health(stamp);

    Vector3d gnss_local;
    if (!navsat_to_local_enu(msg, gnss_local))
    {
        return;
    }
    if (first_frame_tag_odom)
    {
        if (first_gnss_cold_start_candidate_time < 0.0)
        {
            first_gnss_cold_start_candidate_time = stamp;
        }
        if (enable_gnss_cold_start &&
            (stamp - first_gnss_cold_start_candidate_time) < gnss_cold_start_delay)
        {
            ROS_INFO_THROTTLE(1.0,
                              "Waiting %.3f s before GNSS cold start to allow odom initialization",
                              gnss_cold_start_delay);
            return;
        }
        initialize_filter_from_gnss(gnss_local, msg->header.stamp);
        return;
    }

    Vector3d synced_odom_position = last_odom_measurement_position;
    const bool has_synced_odom_position = lookup_odom_position_for_gnss_sync(msg->header.stamp, synced_odom_position);
    if (!gnss_alignment_initialized || (enable_gnss_yaw_alignment && !gnss_yaw_alignment_initialized))
    {
        if (!has_synced_odom_position)
        {
            ROS_WARN_THROTTLE(1.0,
                              "Skipping GNSS alignment: no odom sample within %.3f s of GNSS stamp %.3f",
                              gnss_odom_sync_max_dt,
                              msg->header.stamp.toSec());
            return;
        }
        update_gnss_alignment(gnss_local, synced_odom_position, msg->header.stamp);
        if (gnss_require_yaw_alignment_before_update &&
            enable_gnss_yaw_alignment &&
            !gnss_yaw_alignment_initialized &&
            !odom_lost)
        {
            ROS_WARN_THROTTLE(2.0,
                              "Skipping GNSS update until yaw alignment is ready: samples=%zu/%d min_motion=%.3f max_residual=%.3f",
                              gnss_alignment_pairs.size(),
                              gnss_alignment_min_samples,
                              gnss_alignment_min_motion,
                              gnss_alignment_max_residual);
            return;
        }
        ROS_INFO_THROTTLE(5.0,
                          "Using GNSS translation alignment while waiting for yaw alignment: yaw_ready=%d",
                          static_cast<int>(gnss_yaw_alignment_initialized));
    }
    else if (enable_gnss_alignment_refinement &&
             gnss_yaw_alignment_initialized &&
             has_synced_odom_position)
    {
        update_gnss_alignment(gnss_local, synced_odom_position, msg->header.stamp);
    }

    const Vector3d z_gnss = gnss_alignment_R * gnss_local + gnss_alignment_offset;
    const Vector3d innovation = z_gnss - X_state.segment<3>(0);
    const double innovation_norm = innovation.norm();

    // GNSS 的标准观测模型只观测位置：
    //   z = p + v_gnss, H = [I_3, 0, 0, 0, 0]
    // 姿态、速度和 bias 不被直接观测，只能通过 StateCovariance 中的交叉项
    // 被 Kalman gain 间接修正。
    MatrixXd H = MatrixXd::Zero(3, errorstateSize);
    H.block<3, 3>(0, 0) = Matrix3d::Identity();

    Matrix3d R_base = Matrix3d::Zero();
    if (gnss_use_msg_covariance && msg->position_covariance_type != sensor_msgs::NavSatFix::COVARIANCE_TYPE_UNKNOWN)
    {
        // NavSatFix covariance 的布局是 row-major 3x3。当前只使用对角项，
        // 并用 gnss_min_cov_* 保证弱 covariance 不会让 GNSS 被过度信任。
        R_base(0, 0) = std::max(gnss_min_cov_xy, gnss_cov_scale * msg->position_covariance[0]);
        R_base(1, 1) = std::max(gnss_min_cov_xy, gnss_cov_scale * msg->position_covariance[4]);
        R_base(2, 2) = std::max(gnss_min_cov_z, gnss_cov_scale * msg->position_covariance[8]);
    }
    else
    {
        R_base(0, 0) = gnss_min_cov_xy;
        R_base(1, 1) = gnss_min_cov_xy;
        R_base(2, 2) = gnss_min_cov_z;
    }

    if (gnss_position_covariance_floor_xy > 0.0)
    {
        // 可选地提高 P 的位置方差下限。用途是防止长期 odom 约束后 P 过小，
        // 导致合理的 GNSS 创新也被 NIS 门控判为异常。
        StateCovariance(0, 0) = std::max(StateCovariance(0, 0), gnss_position_covariance_floor_xy);
        StateCovariance(1, 1) = std::max(StateCovariance(1, 1), gnss_position_covariance_floor_xy);
    }
    if (gnss_position_covariance_floor_z > 0.0)
    {
        StateCovariance(2, 2) = std::max(StateCovariance(2, 2), gnss_position_covariance_floor_z);
    }
    symmetrize_covariance(StateCovariance);

    if (gnss_update_only_when_odom_lost && !odom_lost && odom_ever_initialized)
    {
        // fallback-only 模式下，odom 健康时 GNSS 不进入 Kalman update，只刷新
        // 最近 GNSS 位置参考。odom 丢失后仍只使用 GNSS 位置观测，不使用
        // GNSS 位置差分得到的速度伪观测。
        push_accepted_gnss_sample(stamp, z_gnss);
        last_gnss_update_time = stamp;
        last_gnss_health_score = 1.0;
        last_accepted_gnss_position = z_gnss;
        last_accepted_gnss_position_time = stamp;
        last_accepted_gnss_position_initialized = true;
        append_pose_to_path(gnss_path_msg,
                            gnss_path_pub,
                            world_frame_id,
                            z_gnss,
                            Quaterniond(X_state(3), X_state(4), X_state(5), X_state(6)),
                            msg->header.stamp,
                            gnss_path_counter);
        ROS_INFO_THROTTLE(5.0,
                          "GNSS fallback-only mode: keeping GNSS reference without Kalman update while odom is healthy");
        return;
    }

    const Matrix3d S_gate = H * StateCovariance * H.transpose() + R_base;
    const double gnss_mahalanobis = innovation.transpose() * S_gate.ldlt().solve(innovation);
    SensorHealthMonitor::Decision gnss_nis_decision{SensorHealthMonitor::HEALTHY, 1.0, false};
    if (enable_gnss_mahalanobis_gate)
    {
        // NIS/Mahalanobis gate 使用 S = HPH^T + R 归一化创新。相比单纯位置差，
        // 它会同时考虑当前滤波器位置不确定度和 GNSS covariance。
        if (enable_gnss_nis_state_machine)
        {
            gnss_nis_decision = gnss_nis_monitor.update(gnss_mahalanobis,
                                                        stamp,
                                                        gnss_health_window_size,
                                                        gnss_degraded_count_threshold,
                                                        gnss_severe_count_threshold,
                                                        gnss_recover_count_threshold,
                                                        gnss_isolation_time,
                                                        gnss_nis_degraded_threshold,
                                                        gnss_nis_severe_threshold,
                                                        gnss_r_degraded_scale,
                                                        gnss_r_severe_scale);
        }
        else if (gnss_mahalanobis > gnss_mahalanobis_reject_threshold)
        {
            gnss_reject_count++;
            ROS_WARN_THROTTLE(1.0,
                              "Rejecting GNSS update: innovation %.3f m mahalanobis %.3f exceeds %.3f updates=%d weak=%d reject=%d",
                              innovation_norm,
                              gnss_mahalanobis,
                              gnss_mahalanobis_reject_threshold,
                              gnss_update_count,
                              gnss_weak_count,
                              gnss_reject_count);
            return;
        }
    }
    else if (innovation_norm > gnss_adaptive_reject_threshold)
    {
        gnss_reject_count++;
        ROS_WARN_THROTTLE(1.0,
                          "Rejecting GNSS update: innovation %.3f m exceeds threshold %.3f m updates=%d weak=%d reject=%d",
                          innovation_norm,
                          gnss_adaptive_reject_threshold,
                          gnss_update_count,
                          gnss_weak_count,
                          gnss_reject_count);
        return;
    }

    double motion_consistency_error = 0.0;
    double motion_consistency_scale = 1.0;
    bool motion_consistency_checked = false;
    double motion_consistency_score = 1.0;
    if (enable_gnss_motion_consistency && has_synced_odom_position)
    {
        // 运动一致性不直接比较绝对位置，而是比较一段时间内 GNSS delta 与
        // odom delta。这样能发现 GNSS 漂移/跳点，同时降低 frame 平移误差的影响。
        if (!last_gnss_motion_reference_initialized)
        {
            last_gnss_motion_reference_initialized = true;
            last_gnss_motion_reference = z_gnss;
            last_odom_motion_reference = synced_odom_position;
        }
        else
        {
            const Vector3d gnss_delta = z_gnss - last_gnss_motion_reference;
            const Vector3d odom_delta = synced_odom_position - last_odom_motion_reference;
            if (gnss_delta.head<2>().norm() >= gnss_motion_consistency_min_motion ||
                odom_delta.head<2>().norm() >= gnss_motion_consistency_min_motion)
            {
                motion_consistency_checked = true;
                motion_consistency_error = (gnss_delta - odom_delta).head<2>().norm();
                if (motion_consistency_error > gnss_motion_consistency_reject_threshold)
                {
                    gnss_reject_count++;
                    gnss_motion_inconsistent_count++;
                    ROS_WARN_THROTTLE(1.0,
                                      "Rejecting GNSS update: motion inconsistency %.3f m exceeds %.3f updates=%d inconsistent=%d reject=%d",
                                      motion_consistency_error,
                                      gnss_motion_consistency_reject_threshold,
                                      gnss_update_count,
                                      gnss_motion_inconsistent_count,
                                      gnss_reject_count);
                    return;
                }
                motion_consistency_scale = bounded_adaptive_scale(motion_consistency_error,
                                                                  gnss_motion_consistency_threshold,
                                                                  gnss_motion_consistency_reject_threshold,
                                                                  gnss_motion_consistency_max_scale);
                motion_consistency_score = descending_score(motion_consistency_error,
                                                            gnss_motion_consistency_threshold,
                                                            gnss_motion_consistency_reject_threshold);
                if (motion_consistency_scale > 1.0)
                {
                    gnss_motion_inconsistent_count++;
                }
                last_gnss_motion_reference = z_gnss;
                last_odom_motion_reference = synced_odom_position;
            }
        }
    }

    const double horizontal_std = std::sqrt(std::max(R_base(0, 0), R_base(1, 1)));
    const double covariance_score = descending_score(horizontal_std,
                                                     gnss_good_covariance_xy,
                                                     gnss_poor_covariance_xy);
    const double nis_score = enable_gnss_mahalanobis_gate
                                 ? descending_score(gnss_mahalanobis,
                                                    gnss_mahalanobis_weak_threshold,
                                                    gnss_mahalanobis_reject_threshold)
                                 : descending_score(innovation_norm,
                                                    gnss_adaptive_threshold,
                                                    gnss_adaptive_reject_threshold);
    const double status_score = msg->status.status >= gnss_min_status ? 1.0 : 0.0;
    const double gnss_data_health_score = clamp01(0.45 * covariance_score +
                                                  0.35 * motion_consistency_score +
                                                  0.20 * status_score);
    double gnss_health_score = enable_gnss_health_score
                                   ? gnss_health_score_from_factors(covariance_score,
                                                                    nis_score,
                                                                    motion_consistency_score,
                                                                    status_score)
                                   : 1.0;
    if (odom_lost)
    {
        gnss_health_score = std::max(gnss_health_score, gnss_data_health_score);
    }

    double odom_gnss_distance = 0.0;
    double odom_gnss_scale = 1.0;
    double effective_odom_health_score = odom_lost ? 0.0 : last_odom_health_score;
    if (enable_odom_gnss_consistency_health && has_synced_odom_position)
    {
        odom_gnss_distance = (z_gnss - synced_odom_position).head<2>().norm();
        last_odom_gnss_consistency_score = descending_score(odom_gnss_distance,
                                                            odom_gnss_consistency_threshold,
                                                            odom_gnss_consistency_poor_threshold);
        last_odom_gnss_consistency_scale = 1.0;
        const bool gnss_motion_supported =
            !enable_gnss_motion_consistency || motion_consistency_checked;
        const bool gnss_consistency_trusted =
            gnss_motion_supported &&
            (gnss_health_score >= gnss_health_trust_threshold ||
             gnss_data_health_score >= gnss_health_trust_threshold);
        if (gnss_consistency_trusted)
        {
            last_odom_gnss_consistency_scale = bounded_adaptive_scale(odom_gnss_distance,
                                                                      odom_gnss_consistency_threshold,
                                                                      odom_gnss_consistency_poor_threshold,
                                                                      odom_gnss_consistency_max_scale);
            last_odom_gnss_consistency_time = stamp;
        }
        if (last_odom_health_score > odom_weak_health_threshold &&
            !gnss_consistency_trusted)
        {
            odom_gnss_scale = bounded_adaptive_scale(odom_gnss_distance,
                                                     odom_gnss_consistency_threshold,
                                                     odom_gnss_consistency_poor_threshold,
                                                     gnss_motion_consistency_max_scale);
        }
        if (gnss_consistency_trusted)
        {
            effective_odom_health_score = std::min(effective_odom_health_score,
                                                   last_odom_gnss_consistency_score);
            gnss_health_score = std::max(gnss_health_score, gnss_data_health_score);
        }
    }
    else
    {
        last_odom_gnss_consistency_score = 1.0;
        last_odom_gnss_consistency_scale = 1.0;
    }
    const bool trusted_gnss_against_weak_odom =
        gnss_health_score >= gnss_health_trust_threshold &&
        effective_odom_health_score <= odom_weak_health_threshold;
    last_gnss_health_score = gnss_health_score;

    if (enable_gnss_mahalanobis_gate &&
        enable_gnss_nis_state_machine &&
        gnss_nis_decision.reject &&
        !trusted_gnss_against_weak_odom)
    {
        gnss_reject_count++;
        ROS_WARN_THROTTLE(1.0,
                          "Rejecting GNSS update: NIS state=%s innovation %.3f m nis %.3f isolated_until %.3f data_score %.2f odom_score %.2f updates=%d weak=%d reject=%d",
                          gnss_nis_monitor.state_name(),
                          innovation_norm,
                          gnss_mahalanobis,
                          gnss_nis_monitor.isolated_until,
                          gnss_data_health_score,
                          effective_odom_health_score,
                          gnss_update_count,
                          gnss_weak_count,
                          gnss_reject_count);
        return;
    }
    if (enable_gnss_mahalanobis_gate &&
        gnss_mahalanobis > gnss_mahalanobis_reject_threshold &&
        !trusted_gnss_against_weak_odom)
    {
        gnss_reject_count++;
        ROS_WARN_THROTTLE(1.0,
                          "Rejecting GNSS update: immediate NIS reject innovation %.3f m nis %.3f exceeds %.3f data_score %.2f odom_score %.2f updates=%d weak=%d reject=%d",
                          innovation_norm,
                          gnss_mahalanobis,
                          gnss_mahalanobis_reject_threshold,
                          gnss_data_health_score,
                          effective_odom_health_score,
                          gnss_update_count,
                          gnss_weak_count,
                          gnss_reject_count);
        return;
    }

    if (enable_gnss_health_score && gnss_health_score < gnss_min_health_score)
    {
        gnss_reject_count++;
        ROS_WARN_THROTTLE(1.0,
                          "Rejecting GNSS update: health score %.2f below %.2f cov_score %.2f nis_score %.2f motion_score %.2f updates=%d reject=%d",
                          gnss_health_score,
                          gnss_min_health_score,
                          covariance_score,
                          nis_score,
                          motion_consistency_score,
                          gnss_update_count,
                          gnss_reject_count);
        return;
    }

    double gnss_scale = trusted_gnss_against_weak_odom
                            ? motion_consistency_scale
                            : adaptive_observation_scale(innovation_norm,
                                                         gnss_adaptive_threshold,
                                                         gnss_adaptive_reject_threshold,
                                                         gnss_adaptive_max_scale);
    // 多个健康指标共同决定最终 R scale。所有 scale 都只会增大 R 或保持不变，
    // 避免弱观测通过某一项指标“抵消”另一项风险。
    if (enable_gnss_mahalanobis_gate && !trusted_gnss_against_weak_odom)
    {
        gnss_scale = std::max(gnss_scale,
                              bounded_adaptive_scale(gnss_mahalanobis,
                                                     gnss_mahalanobis_weak_threshold,
                                                     gnss_mahalanobis_reject_threshold,
                                                     gnss_adaptive_max_scale));
        if (enable_gnss_nis_state_machine)
        {
            gnss_scale = std::max(gnss_scale, gnss_nis_decision.scale);
        }
    }
    gnss_scale = std::max(gnss_scale, motion_consistency_scale);
    gnss_scale = std::max(gnss_scale, odom_gnss_scale);
    if (enable_gnss_health_score && !trusted_gnss_against_weak_odom)
    {
        const double low_health_scale = 1.0 + std::pow(1.0 - gnss_health_score, 2.0) *
                                                  (gnss_health_low_score_max_scale - 1.0);
        gnss_scale = std::max(gnss_scale, low_health_scale);
    }

    Matrix3d R = R_base * gnss_scale;
    if (gnss_health_score >= gnss_health_trust_threshold &&
        effective_odom_health_score <= odom_weak_health_threshold &&
        gnss_healthy_odom_weak_scale > 0.0)
    {
        R *= std::min(1.0, gnss_healthy_odom_weak_scale);
    }

    gnss_update_count++;
    if (gnss_scale > 1.0)
    {
        gnss_weak_count++;
        ROS_WARN_THROTTLE(1.0,
                          "GNSS observation health=WEAK score %.2f cov_score %.2f nis_score %.2f motion_score %.2f innovation %.3f m mahalanobis %.3f motion_error %.3f checked=%d scale %.2f odom_score %.2f odom_gnss_score %.2f odom_gnss_dist %.3f updates=%d weak=%d inconsistent=%d reject=%d",
                          gnss_health_score,
                          covariance_score,
                          nis_score,
                          motion_consistency_score,
                          innovation_norm,
                          gnss_mahalanobis,
                          motion_consistency_error,
                          static_cast<int>(motion_consistency_checked),
                          gnss_scale,
                          effective_odom_health_score,
                          last_odom_gnss_consistency_score,
                          odom_gnss_distance,
                          gnss_update_count,
                          gnss_weak_count,
                          gnss_motion_inconsistent_count,
                          gnss_reject_count);
        if (enable_gnss_nis_state_machine)
        {
            ROS_WARN_THROTTLE(1.0,
                              "GNSS NIS monitor state=%s nis %.3f state_scale %.2f window=%zu",
                              gnss_nis_monitor.state_name(),
                              gnss_mahalanobis,
                              gnss_nis_decision.scale,
                              gnss_nis_monitor.window.size());
        }
    }
    else
    {
        ROS_INFO_THROTTLE(5.0,
                          "GNSS observation health=HEALTHY score %.2f cov_score %.2f nis_score %.2f motion_score %.2f innovation %.3f m mahalanobis %.3f odom_score %.2f odom_gnss_score %.2f odom_gnss_dist %.3f updates=%d weak=%d inconsistent=%d reject=%d",
                          gnss_health_score,
                          covariance_score,
                          nis_score,
                          motion_consistency_score,
                          innovation_norm,
                          gnss_mahalanobis,
                          effective_odom_health_score,
                          last_odom_gnss_consistency_score,
                          odom_gnss_distance,
                          gnss_update_count,
                          gnss_weak_count,
                          gnss_motion_inconsistent_count,
                          gnss_reject_count);
    }

    // GNSS observation is fixed to 3D position. The historical finite-difference
    // GNSS velocity pseudo-observation is intentionally disabled for data/data2
    // odom-dropout experiments.
    MatrixXd H_update = H;
    VectorXd innovation_update = innovation;
    MatrixXd R_update = R;

    MatrixXd S = H_update * StateCovariance * H_update.transpose() + R_update;
    MatrixXd K = StateCovariance * H_update.transpose() * S.inverse();
    VectorXd dx = VectorXd::Zero(errorstateSize);
    dx = K * innovation_update;
    // GNSS update 仍然产生 15D dx，因此统一通过 boxplus() 修正 nominal state。
    // 对于标准位置观测，dx 中的 dtheta/dv/dbias 可能因为 P 的交叉协方差而非零。
    X_state = boxplus(X_state, dx);
    joseph_covariance_update(H_update, K, R_update);

    Vector3d position_delta = dx.segment<3>(0);
    if (odom_lost && enable_gnss_position_snap_when_odom_lost)
    {
        const Vector3d forced_position_delta = z_gnss - X_state.segment<3>(0);
        X_state.segment<3>(0) = z_gnss;
        position_delta += forced_position_delta;
    }
    for (auto &state_and_imu : sys_seq)
    {
        // GNSS 更新发生在当前状态上，但 replay buffer 中还缓存着未来可能用于
        // 延迟 odom 更新的历史 nominal state。同步平移增量可以避免下一次
        // odom 回放时重新使用旧的 GNSS 前状态。
        state_and_imu.first.segment<3>(0) += position_delta;
    }
    push_accepted_gnss_sample(stamp, z_gnss);
    last_gnss_update_time = stamp;
    last_accepted_gnss_position = z_gnss;
    last_accepted_gnss_position_time = stamp;
    last_accepted_gnss_position_initialized = true;

    append_pose_to_path(gnss_path_msg,
                        gnss_path_pub,
                        world_frame_id,
                        z_gnss,
                        Quaterniond(X_state(3), X_state(4), X_state(5), X_state(6)),
                        msg->header.stamp,
                        gnss_path_counter);
}
/// @brief Convert a 3D Lie-algebra rotation vector to a rotation matrix.
Matrix3d lie_algebra_2_rotation(Vector3d v)
{

    Eigen::Matrix3d R;
    double theta = v.norm();

    if (theta < 1e-6)
    {
        R = Eigen::Matrix3d::Identity();
    }
    else
    {
        Eigen::Matrix3d skew;
        skew << 0, -v(2), v(1),
            v(2), 0, -v(0),
            -v(1), v(0), 0;
        R = Eigen::Matrix3d::Identity() + skew / theta * std::sin(theta) + skew * skew / theta / theta * (1 - std::cos(theta));
        // R = skew.exp();
    }

    return R;
}
/// @brief Apply a 15D error-state increment to the 16D nominal state.
VectorXd boxplus(VectorXd x, VectorXd dx)
{
    VectorXd x_plus(x.rows());
    // dx uses the 15D minimal error-state layout:
    //   [dp(0:2), dtheta(3:5), dv(6:8), dbg(9:11), dba(12:14)].
    // The nominal state uses a 4D quaternion, so only the position/velocity/bias
    // parts are direct additions. Attitude must be composed on SO(3).
    x_plus.segment<3>(kStatePositionOffset) =
        x.segment<3>(kStatePositionOffset) +
        dx.segment<3>(kErrorPositionOffset);

    Vector3d dtheta = dx.segment<3>(kErrorRotationOffset);
    Matrix3d dR = lie_algebra_2_rotation(dtheta);
    Quaterniond x_q(x(kStateQuaternionOffset),
                    x(kStateQuaternionOffset + 1),
                    x(kStateQuaternionOffset + 2),
                    x(kStateQuaternionOffset + 3));
    x_q.normalize();
    Matrix3d x_R = x_q.toRotationMatrix();
    Matrix3d x_R_plus = x_R * dR;
    Quaterniond x_q_plus(x_R_plus);
    x_q_plus.normalize();
    x_plus(kStateQuaternionOffset) = x_q_plus.w();
    x_plus(kStateQuaternionOffset + 1) = x_q_plus.x();
    x_plus(kStateQuaternionOffset + 2) = x_q_plus.y();
    x_plus(kStateQuaternionOffset + 3) = x_q_plus.z();

    x_plus.segment<3>(kStateVelocityOffset) =
        x.segment<3>(kStateVelocityOffset) +
        dx.segment<3>(kErrorVelocityOffset);

    x_plus.segment<3>(kStateGyroBiasOffset) =
        x.segment<3>(kStateGyroBiasOffset) +
        dx.segment<3>(kErrorGyroBiasOffset);

    x_plus.segment<3>(kStateAccelBiasOffset) =
        x.segment<3>(kStateAccelBiasOffset) +
        dx.segment<3>(kErrorAccelBiasOffset);

    return x_plus;
}

/// @brief ROS node entry point: parameters, subscribers, publishers, and EKF init.
int main(int argc, char **argv)
{
  // Pinning is an optimization for the original test machine. Failure is not
  // fatal; ROS still runs normally, so keep it as a warning-level startup detail.
  int core_id = 5;
  cpu_set_t cpuset;
  CPU_ZERO(&cpuset);
  CPU_SET(core_id, &cpuset);
  if (sched_setaffinity(0, sizeof(cpu_set_t), &cpuset) == -1) 
  {
      std::cerr << "Failed to set CPU affinity for thread: ekf "<< std::endl;
  } 
  else 
  {
      std::cout << "Successfully set CPU affinity to core " << core_id << std::endl;
  }
    ros::init(argc, argv, "ekf");
    ros::NodeHandle n("~");
    // Private topic names are remapped by launch/ekf_lidar.launch. Keep the
    // private names stable so launch files can adapt datasets without source
    // edits.
    ros::Subscriber s1 = n.subscribe("imu", 1000, imu_callback, ros::TransportHints().tcpNoDelay());
    ros::Subscriber s2 = n.subscribe("bodyodometry_primary", 40, vioodom_primary_callback, ros::TransportHints().tcpNoDelay());
    ros::Subscriber s3 = n.subscribe("bodyodometry_fallback", 40, vioodom_fallback_callback, ros::TransportHints().tcpNoDelay());
    ros::Subscriber s5 = n.subscribe("gnss_fix", 40, gnss_fix_callback, ros::TransportHints().tcpNoDelay());
    odom_pub = n.advertise<nav_msgs::Odometry>("ekf_odom", 1000);             // freq = imu freq
    ahead_odom_pub = n.advertise<nav_msgs::Odometry>("ahead_ekf_odom", 1000); // freq = imu freq
    cam_odom_pub = n.advertise<nav_msgs::Odometry>("cam_ekf_odom", 1000);
    input_path_pub = n.advertise<nav_msgs::Path>("input_path", 10, true);
    ekf_path_pub = n.advertise<nav_msgs::Path>("ekf_path", 10, true);
    measurement_path_pub = n.advertise<nav_msgs::Path>("measurement_path", 10, true);
    gnss_path_pub = n.advertise<nav_msgs::Path>("gnss_path", 10, true);
    ekf_segments_pub = n.advertise<visualization_msgs::MarkerArray>("ekf_segments", 10, true);
    ekf_arrows_pub = n.advertise<visualization_msgs::MarkerArray>("ekf_arrows", 10, true);

    // Parameter loading mirrors launch/ekf_lidar.launch. Defaults above are only
    // fallbacks; reproducible runs should record the launch command and any
    // overridden parameters.
    n.getParam("gyro_cov", gyro_cov);
    n.getParam("acc_cov", acc_cov);
    n.getParam("position_cov", position_cov);
    n.getParam("q_rp_cov", q_rp_cov);
    n.getParam("q_yaw_cov", q_yaw_cov);
    n.getParam("imu_trans_x", imu_trans_x);
    n.getParam("imu_trans_y", imu_trans_y);
    n.getParam("imu_trans_z", imu_trans_z);
    n.getParam("cutoff_freq", cutoff_freq);
    n.getParam("enable_output_motion_smoothing", enable_output_motion_smoothing);
    n.getParam("output_smoothing_natural_freq", output_smoothing_natural_freq);
    n.getParam("output_smoothing_damping_ratio", output_smoothing_damping_ratio);
    n.getParam("output_smoothing_max_accel", output_smoothing_max_accel);
    n.getParam("output_smoothing_max_correction_speed", output_smoothing_max_correction_speed);
    n.getParam("output_smoothing_normal_natural_freq", output_smoothing_normal_natural_freq);
    n.getParam("output_smoothing_normal_max_accel", output_smoothing_normal_max_accel);
    n.getParam("output_smoothing_normal_max_correction_speed", output_smoothing_normal_max_correction_speed);
    n.getParam("output_smoothing_release_error", output_smoothing_release_error);
    n.getParam("output_smoothing_recovery_duration", output_smoothing_recovery_duration);
    n.getParam("offset_px", offset_px);
    n.getParam("offset_py", offset_py);
    n.getParam("offset_pz", offset_pz);
    n.getParam("publish_warmup_frames", publish_warmup_frames);
    n.getParam("odom_jump_threshold", odom_jump_threshold);
    n.getParam("innovation_reject_threshold", innovation_reject_threshold);
    n.getParam("innovation_reset_threshold", innovation_reset_threshold);
    n.getParam("odom_use_msg_covariance", odom_use_msg_covariance);
    n.getParam("odom_msg_min_position_cov", odom_msg_min_position_cov);
    n.getParam("odom_msg_min_orientation_cov", odom_msg_min_orientation_cov);
    n.getParam("enable_odom_realign", enable_odom_realign);
    n.getParam("enable_adaptive_observation_covariance", enable_adaptive_observation_covariance);
    n.getParam("odom_adaptive_threshold", odom_adaptive_threshold);
    n.getParam("odom_adaptive_reject_threshold", odom_adaptive_reject_threshold);
    n.getParam("odom_adaptive_max_scale", odom_adaptive_max_scale);
    n.getParam("odom_loss_timeout", odom_loss_timeout);
    n.getParam("enable_odom_recovery_guard", enable_odom_recovery_guard);
    n.getParam("odom_recovery_frames", odom_recovery_frames);
    n.getParam("odom_recovery_scale", odom_recovery_scale);
    n.getParam("odom_recovery_min_scale", odom_recovery_min_scale);
    n.getParam("enable_gnss_velocity_when_odom_lost", enable_gnss_velocity_when_odom_lost);
    if (enable_gnss_velocity_when_odom_lost)
    {
        ROS_WARN("GNSS velocity pseudo-observation is deprecated and forced off; GNSS will be used as a position observation only");
        enable_gnss_velocity_when_odom_lost = false;
    }
    n.getParam("gnss_velocity_min_dt", gnss_velocity_min_dt);
    n.getParam("gnss_velocity_max_dt", gnss_velocity_max_dt);
    n.getParam("gnss_velocity_cov", gnss_velocity_cov);
    n.getParam("gnss_velocity_window_size", gnss_velocity_window_size);
    n.getParam("gnss_velocity_smoothing_alpha", gnss_velocity_smoothing_alpha);
    n.getParam("gyro_bias_rw_cov", gyro_bias_rw_cov);
    n.getParam("acc_bias_rw_cov", acc_bias_rw_cov);
    n.getParam("gnss_adaptive_threshold", gnss_adaptive_threshold);
    n.getParam("gnss_adaptive_reject_threshold", gnss_adaptive_reject_threshold);
    n.getParam("gnss_adaptive_max_scale", gnss_adaptive_max_scale);
    n.getParam("odom_realign_settle_frames", odom_realign_settle_frames);
    n.getParam("odom_source_switch_grace", odom_source_switch_grace);
    n.getParam("use_gnss", use_gnss);
    n.getParam("enable_gnss_cold_start", enable_gnss_cold_start);
    n.getParam("gnss_cold_start_delay", gnss_cold_start_delay);
    n.getParam("gnss_cold_start_frame_id", gnss_cold_start_frame_id);
    n.getParam("gnss_update_only_when_odom_lost", gnss_update_only_when_odom_lost);
    n.getParam("enable_gnss_position_snap_when_odom_lost", enable_gnss_position_snap_when_odom_lost);
    n.getParam("gnss_use_msg_covariance", gnss_use_msg_covariance);
    n.getParam("gnss_min_interval", gnss_min_interval);
    n.getParam("gnss_min_cov_xy", gnss_min_cov_xy);
    n.getParam("gnss_min_cov_z", gnss_min_cov_z);
    n.getParam("gnss_cov_scale", gnss_cov_scale);
    n.getParam("gnss_position_covariance_floor_xy", gnss_position_covariance_floor_xy);
    n.getParam("gnss_position_covariance_floor_z", gnss_position_covariance_floor_z);
    n.getParam("enable_gnss_mahalanobis_gate", enable_gnss_mahalanobis_gate);
    n.getParam("gnss_mahalanobis_weak_threshold", gnss_mahalanobis_weak_threshold);
    n.getParam("gnss_mahalanobis_reject_threshold", gnss_mahalanobis_reject_threshold);
    n.getParam("enable_gnss_motion_consistency", enable_gnss_motion_consistency);
    n.getParam("gnss_motion_consistency_min_motion", gnss_motion_consistency_min_motion);
    n.getParam("gnss_motion_consistency_threshold", gnss_motion_consistency_threshold);
    n.getParam("gnss_motion_consistency_reject_threshold", gnss_motion_consistency_reject_threshold);
    n.getParam("gnss_motion_consistency_max_scale", gnss_motion_consistency_max_scale);
    n.getParam("gnss_healthy_odom_weak_scale", gnss_healthy_odom_weak_scale);
    n.getParam("enable_gnss_health_score", enable_gnss_health_score);
    n.getParam("gnss_good_covariance_xy", gnss_good_covariance_xy);
    n.getParam("gnss_poor_covariance_xy", gnss_poor_covariance_xy);
    n.getParam("gnss_min_health_score", gnss_min_health_score);
    n.getParam("gnss_health_trust_threshold", gnss_health_trust_threshold);
    n.getParam("odom_weak_health_threshold", odom_weak_health_threshold);
    n.getParam("gnss_health_low_score_max_scale", gnss_health_low_score_max_scale);
    n.getParam("enable_gnss_nis_state_machine", enable_gnss_nis_state_machine);
    n.getParam("gnss_health_window_size", gnss_health_window_size);
    n.getParam("gnss_degraded_count_threshold", gnss_degraded_count_threshold);
    n.getParam("gnss_severe_count_threshold", gnss_severe_count_threshold);
    n.getParam("gnss_recover_count_threshold", gnss_recover_count_threshold);
    n.getParam("gnss_isolation_time", gnss_isolation_time);
    n.getParam("gnss_nis_degraded_threshold", gnss_nis_degraded_threshold);
    n.getParam("gnss_nis_severe_threshold", gnss_nis_severe_threshold);
    n.getParam("gnss_r_degraded_scale", gnss_r_degraded_scale);
    n.getParam("gnss_r_severe_scale", gnss_r_severe_scale);
    n.getParam("enable_odom_gnss_consistency_health", enable_odom_gnss_consistency_health);
    n.getParam("odom_gnss_consistency_threshold", odom_gnss_consistency_threshold);
    n.getParam("odom_gnss_consistency_poor_threshold", odom_gnss_consistency_poor_threshold);
    n.getParam("odom_gnss_consistency_max_scale", odom_gnss_consistency_max_scale);
    n.getParam("odom_gnss_consistency_timeout", odom_gnss_consistency_timeout);
    n.getParam("enable_gnss_yaw_alignment", enable_gnss_yaw_alignment);
    n.getParam("gnss_require_yaw_alignment_before_update", gnss_require_yaw_alignment_before_update);
    n.getParam("gnss_alignment_min_samples", gnss_alignment_min_samples);
    n.getParam("gnss_alignment_max_samples", gnss_alignment_max_samples);
    n.getParam("gnss_alignment_min_motion", gnss_alignment_min_motion);
    n.getParam("gnss_alignment_sample_interval", gnss_alignment_sample_interval);
    n.getParam("gnss_odom_sync_max_dt", gnss_odom_sync_max_dt);
    n.getParam("gnss_alignment_max_residual", gnss_alignment_max_residual);
    n.getParam("enable_gnss_alignment_refinement", enable_gnss_alignment_refinement);
    n.getParam("gnss_alignment_refinement_min_samples", gnss_alignment_refinement_min_samples);
    n.getParam("gnss_alignment_refinement_max_samples", gnss_alignment_refinement_max_samples);
    n.getParam("gnss_alignment_refinement_min_motion", gnss_alignment_refinement_min_motion);
    n.getParam("gnss_alignment_refinement_max_residual", gnss_alignment_refinement_max_residual);
    n.getParam("gnss_alignment_refinement_gain", gnss_alignment_refinement_gain);
    n.getParam("gnss_alignment_refinement_max_yaw_step", gnss_alignment_refinement_max_yaw_step);
    n.getParam("gnss_alignment_refinement_max_translation_step", gnss_alignment_refinement_max_translation_step);
    n.getParam("gnss_min_status", gnss_min_status);
    n.getParam("path_publish_stride", path_publish_stride);
    n.getParam("path_max_points", path_max_points);
    n.getParam("arrow_publish_stride", arrow_publish_stride);
    n.getParam("arrow_max_markers", arrow_max_markers);
    

    cout << "Q:" << gyro_cov << " " << acc_cov << " R: " << position_cov << " " << q_rp_cov << " " << q_yaw_cov << endl;

    std::vector<double> Rri, tri, rotationimu;
    n.getParam("Rr_i", Rri);
    n.getParam("tr_i", tri);
    n.getParam("scale_g", scale_g);
    n.getParam("rotation_imu", rotationimu);
    Rr_i = Quaterniond(Rri.at(0), Rri.at(1), Rri.at(2), Rri.at(3)).toRotationMatrix();
    for (int i=0; i<3; i++){
        for (int j=0; j<3; j++){
            rotation_imu(i, j) = rotationimu[i*3+j];
        }
    }
    cout << "rotation_imu = " << endl 
         << rotation_imu << endl;
    cout << "scale_g = " << scale_g << endl;
    tr_i << tri.at(0), tri.at(1), tri.at(2);
    cout << "Rr_i: " << endl
         << Rr_i << endl;
    cout << "tr_i: " << endl
         << tr_i << endl;

    initsys();
    cout << "initsys" << endl;


    ros::spin();
}

/// @brief Publish a short-horizon predicted odometry state for feed-forward consumers.
void ahead_system_pub(const Eigen::VectorXd &X_state_in, ros::Time stamp)
{
    nav_msgs::Odometry odom_fusion;
    odom_fusion.header.stamp = stamp;
    odom_fusion.header.frame_id = world_frame_id;
    // odom_fusion.header.frame_id = "imu";

    Quaterniond q;
    q = X_state_in.segment<4>(3);
    odom_fusion.pose.pose.orientation.w = q.w();
    odom_fusion.pose.pose.orientation.x = q.x();
    odom_fusion.pose.pose.orientation.y = q.y();
    odom_fusion.pose.pose.orientation.z = q.z();
    odom_fusion.twist.twist.linear.x = X_state_in(7);
    odom_fusion.twist.twist.linear.y = X_state_in(8);
    odom_fusion.twist.twist.linear.z = X_state_in(9);

    Vector3d pos_center(X_state_in(0), X_state_in(1), X_state_in(2)), pos_center2;
    pos_center2 = pos_center + q.toRotationMatrix() * Vector3d(imu_trans_x, imu_trans_y, imu_trans_z);
    odom_fusion.pose.pose.position.x = pos_center2(0);
    odom_fusion.pose.pose.position.y = pos_center2(1);
    odom_fusion.pose.pose.position.z = pos_center2(2);

    ahead_odom_pub.publish(odom_fusion);
}

/// @brief Reset the output smoother after hard state discontinuities.
void reset_output_motion_smoother()
{
    output_filter_initialized = false;
    output_filter_state = Vector3d::Zero();
    output_filter_velocity = Vector3d::Zero();
    last_output_filter_time = -1.0;
    output_smoothing_recovery_until = -1.0;
}

/// @brief Second-order damped output smoother with low-latency healthy tracking.
void smooth_output_motion(const Vector3d &raw_position,
                          const Vector3d &raw_velocity,
                          const ros::Time &stamp,
                          Vector3d &output_position,
                          Vector3d &output_velocity)
{
    output_position = raw_position;
    output_velocity = raw_velocity;
    if (!enable_output_motion_smoothing)
    {
        reset_output_motion_smoother();
        return;
    }

    double dt_filter = last_output_filter_time > 0.0
                           ? stamp.toSec() - last_output_filter_time
                           : dt;
    if (dt_filter <= 1.0e-5 && sample_freq > 1.0e-5)
    {
        dt_filter = 1.0 / sample_freq;
    }
    if (dt_filter <= 1.0e-5)
    {
        dt_filter = 0.01;
    }
    dt_filter = std::max(0.001, std::min(0.1, dt_filter));

    if (!output_filter_initialized || last_output_filter_time < 0.0)
    {
        output_filter_state = raw_position;
        output_filter_velocity = raw_velocity;
        output_filter_initialized = true;
        last_output_filter_time = stamp.toSec();
        output_position = output_filter_state;
        output_velocity = output_filter_velocity;
        return;
    }

    const bool output_smoothing_recovery_mode_active =
        output_smoothing_recovery_output_active(stamp.toSec()) ||
        odom_is_lost_at(stamp.toSec()) ||
        last_odom_health_score <= odom_weak_health_threshold;
    const double tracking_error =
        (raw_position - output_filter_state).norm();
    const bool output_smoothing_low_latency_mode_active =
        !output_smoothing_recovery_mode_active &&
        tracking_error <= std::max(0.0, output_smoothing_release_error);
    if (output_smoothing_low_latency_mode_active)
    {
        output_filter_state = raw_position;
        output_filter_velocity = raw_velocity;
        last_output_filter_time = stamp.toSec();
        output_position = raw_position;
        output_velocity = raw_velocity;
        return;
    }

    const Vector3d previous_position = output_filter_state;
    const Vector3d previous_velocity = output_filter_velocity;
    const double natural_freq = output_smoothing_recovery_mode_active
                                    ? output_smoothing_natural_freq
                                    : output_smoothing_normal_natural_freq;
    const double max_accel = output_smoothing_recovery_mode_active
                                 ? output_smoothing_max_accel
                                 : output_smoothing_normal_max_accel;
    const double max_correction_speed = output_smoothing_recovery_mode_active
                                            ? output_smoothing_max_correction_speed
                                            : output_smoothing_normal_max_correction_speed;
    const double omega =
        2.0 * PI * std::max(1.0e-3, natural_freq);
    const double damping_ratio =
        std::max(0.0, output_smoothing_damping_ratio);
    const Vector3d position_error = raw_position - previous_position;
    const Vector3d velocity_error = raw_velocity - previous_velocity;

    const Vector3d second_order_position_correction_velocity =
        clamp_vector_norm(omega * position_error,
                          std::max(0.0, max_correction_speed));
    Vector3d acceleration =
        omega * second_order_position_correction_velocity +
        2.0 * damping_ratio * omega * velocity_error;
    acceleration = clamp_vector_norm(acceleration,
                                     std::max(0.0, max_accel));

    const Vector3d candidate_velocity = previous_velocity + acceleration * dt_filter;
    const Vector3d candidate_position =
        previous_position + candidate_velocity * dt_filter;

    output_filter_state = candidate_position;
    output_filter_velocity = candidate_velocity;
    last_output_filter_time = stamp.toSec();
    output_position = output_filter_state;
    output_velocity = output_filter_velocity;
}

/// @brief Publish the main fused EKF odometry and append visualization paths/markers.
void system_pub(const Eigen::VectorXd &X_state_in, ros::Time stamp)
{
    nav_msgs::Odometry odom_fusion;
    odom_fusion.header.stamp = stamp;
    odom_fusion.header.frame_id = world_frame_id;

    Quaterniond q;
    q.w() = X_state_in(3);
    q.x() = X_state_in(4);
    q.y() = X_state_in(5);
    q.z() = X_state_in(6);
    odom_fusion.pose.pose.orientation.w = q.w();
    odom_fusion.pose.pose.orientation.x = q.x();
    odom_fusion.pose.pose.orientation.y = q.y();
    odom_fusion.pose.pose.orientation.z = q.z();

    Vector3d pos_center(X_state_in(0), X_state_in(1), X_state_in(2));
    Vector3d pos_center_world = pos_center + q.toRotationMatrix() * Vector3d(imu_trans_x, imu_trans_y, imu_trans_z);
    Vector3d raw_velocity(X_state_in(7), X_state_in(8), X_state_in(9));
    Vector3d pos_center_output = pos_center_world;
    Vector3d output_velocity = raw_velocity;
    if (!enable_output_motion_smoothing && cutoff_freq > 1.0e-5)
    {
        if (!output_filter_initialized)
        {
            output_filter_state = pos_center_world;
            output_filter_initialized = true;
        }
        else
        {
            double dt_filter = dt;
            if (dt_filter <= 1.0e-5 && sample_freq > 1.0e-5)
            {
                dt_filter = 1.0 / sample_freq;
            }
            if (dt_filter > 1.0e-5)
            {
                double alpha = exp(-2.0 * PI * cutoff_freq * dt_filter);
                output_filter_state = alpha * output_filter_state + (1.0 - alpha) * pos_center_world;
            }
            else
            {
                output_filter_state = pos_center_world;
            }
        }
        pos_center_output = output_filter_state;
        output_velocity = raw_velocity;
    }
    if (enable_output_motion_smoothing)
    {
        smooth_output_motion(pos_center_world, raw_velocity, stamp, pos_center_output, output_velocity);
    }

    odom_fusion.pose.pose.position.x = pos_center_output(0);
    odom_fusion.pose.pose.position.y = pos_center_output(1);
    odom_fusion.pose.pose.position.z = pos_center_output(2);

    odom_fusion.pose.pose.position.x += offset_px;
    odom_fusion.pose.pose.position.y += offset_py;
    odom_fusion.pose.pose.position.z += offset_pz;
    odom_fusion.twist.twist.linear.x = output_velocity(0);
    odom_fusion.twist.twist.linear.y = output_velocity(1);
    odom_fusion.twist.twist.linear.z = output_velocity(2);

    static int cnt = 0;
    if(cnt < publish_warmup_frames){
        cnt++;
        return;
    } 

    odom_pub.publish(odom_fusion);
    const Vector3d output_position(odom_fusion.pose.pose.position.x,
                                   odom_fusion.pose.pose.position.y,
                                   odom_fusion.pose.pose.position.z);
    append_pose_to_path(ekf_path_msg, ekf_path_pub, world_frame_id, output_position, q, stamp, ekf_path_counter);
    append_ekf_arrow_marker(world_frame_id, output_position, q, stamp);
    if (ekf_path_counter % std::max(1, path_publish_stride) == 0)
    {
        append_pose_to_ekf_segments(world_frame_id,
                                    output_position,
                                    stamp);
    }
}
/// @brief Publish the current odom measurement pose that entered the EKF update.
void cam_system_pub(ros::Time stamp)
{
    nav_msgs::Odometry odom_fusion;
    odom_fusion.header.stamp = stamp;
    odom_fusion.header.frame_id = world_frame_id;
    odom_fusion.pose.pose.position.x = Z_measurement(0);
    odom_fusion.pose.pose.position.y = Z_measurement(1);
    odom_fusion.pose.pose.position.z = Z_measurement(2);
    Quaterniond q;

    q = Z_measurement.segment<4>(3);

    odom_fusion.pose.pose.orientation.w = q.w();
    odom_fusion.pose.pose.orientation.x = q.x();
    odom_fusion.pose.pose.orientation.y = q.y();
    odom_fusion.pose.pose.orientation.z = q.z();
    cam_odom_pub.publish(odom_fusion);
    append_pose_to_path(measurement_path_msg, measurement_path_pub, world_frame_id, Vector3d(odom_fusion.pose.pose.position.x,
                                                                                              odom_fusion.pose.pose.position.y,
                                                                                              odom_fusion.pose.pose.position.z),
                        q, stamp, measurement_path_counter);
}

// process model
/// @brief Initialize nominal state, error-state covariance, process noise, and odom R.
void initsys()
{
    // 名义状态和误差状态维度必须同时维护：
    //   stateSize=16        X=[p3, q4, v3, bg3, ba3]
    //   errorstateSize=15   dx=[dp3, dtheta3, dv3, dbg3, dba3]
    //   measurementSize=7   odom 原始观测 [p3, q4]
    //   inputSize=6         IMU 输入噪声 [gyro3, acc3]
    //
    // Rt 的实际维度是 measurementSize-1=6，因为 EKF 中使用的是 6D pose
    // residual，而不是直接对 7D [p,q] 做线性差分。
    stateSize = 16;
    errorstateSize = 15;
    measurementSize = 7;
    inputSize = 6;

    X_state = VectorXd::Zero(stateSize);
    X_state(kStateQuaternionOffset) = 1.0;
    X_state(kStateQuaternionOffset + 1) = 0;
    X_state(kStateQuaternionOffset + 2) = 0;
    X_state(kStateQuaternionOffset + 3) = 0;
    X_state(kStateVelocityOffset) = 0;
    X_state(kStateVelocityOffset + 1) = 0;
    X_state(kStateVelocityOffset + 2) = 0;
    // Initial gyro and accelerometer biases are parameterized as constants here.
    X_state.segment<3>(kStateGyroBiasOffset) = bg_0;
    X_state.segment<3>(kStateAccelBiasOffset) = ba_0;
    Z_measurement = VectorXd::Zero(measurementSize);
    StateCovariance = MatrixXd::Identity(errorstateSize, errorstateSize);

    Kt_kalmanGain = MatrixXd::Identity(stateSize, measurementSize); // Kt
    // Ct_stateToMeasurement = MatrixXd::Identity(stateSize, measurementSize);         // Ct

    Qt = MatrixXd::Identity(inputSize, inputSize);
    Rt = MatrixXd::Identity(measurementSize - 1, measurementSize - 1);

    // Smaller Q trusts IMU propagation more; smaller Rt trusts odom pose updates more.
    Qt.topLeftCorner(3, 3) = gyro_cov * Qt.topLeftCorner(3, 3);
    Qt.bottomRightCorner(3, 3) = acc_cov * Qt.bottomRightCorner(3, 3);
    Rt.topLeftCorner(3, 3) = position_cov * Rt.topLeftCorner(3, 3);
    Rt.bottomRightCorner(3, 3) = q_rp_cov * Rt.bottomRightCorner(3, 3);
    Rt.bottomRightCorner(1, 1) = q_yaw_cov * Rt.bottomRightCorner(1, 1);
    current_odom_Rt = Rt;
}


/// @brief Extract the current nominal state components from X_state.
void getState(Vector3d &p, Quaterniond &q, Vector3d &v, Vector3d &bg, Vector3d &ba)
{
    p = X_state.segment<3>(kStatePositionOffset);
    q = Quaterniond(X_state(kStateQuaternionOffset),
                    X_state(kStateQuaternionOffset + 1),
                    X_state(kStateQuaternionOffset + 2),
                    X_state(kStateQuaternionOffset + 3));
    v = X_state.segment<3>(kStateVelocityOffset);
    bg = X_state.segment<3>(kStateGyroBiasOffset);
    ba = X_state.segment<3>(kStateAccelBiasOffset);
}


/// @brief Discrete IMU propagation for the 16D nominal state.
VectorXd propagate_nominal_state(VectorXd X_state, Vector3d gyro, Vector3d acc, double dt)
{
    // 名义状态预测使用当前 IMU 样本做零阶保持：
    //   gyro_unbias = gyro - bg
    //   acc_world   = gravity + R(q) * (acc - ba)
    //   p_k = p + v*dt + 0.5*acc_world*dt^2
    //   q_k = q * Exp((gyro-bg)*dt)
    //   v_k = v + acc_world*dt
    //
    // ng/na/nbg/nba 当前默认是 0，保留这些变量是为了支持后续显式噪声采样或
    // bias random walk 建模。
    VectorXd updated_X_state(VectorXd::Zero(stateSize));

    const Vector3d v = X_state.segment<3>(kStateVelocityOffset);
    const Vector3d bg = X_state.segment<3>(kStateGyroBiasOffset);
    const Vector3d ba = X_state.segment<3>(kStateAccelBiasOffset);
    Quaterniond q(X_state(kStateQuaternionOffset),
                  X_state(kStateQuaternionOffset + 1),
                  X_state(kStateQuaternionOffset + 2),
                  X_state(kStateQuaternionOffset + 3));
    q.normalize();
    const Vector3d unbiased_gyro = gyro - bg - ng;
    const Vector3d world_acc = gravity + q * (acc - ba - na);

    updated_X_state.segment<3>(kStatePositionOffset) =
        X_state.segment<3>(kStatePositionOffset) + v * dt + 0.5 * world_acc * dt * dt;

    Quaterniond updated_q = (q * delta_quaternion_from_gyro(unbiased_gyro, dt)).normalized();
    updated_X_state(kStateQuaternionOffset) = updated_q.w();
    updated_X_state(kStateQuaternionOffset + 1) = updated_q.x();
    updated_X_state(kStateQuaternionOffset + 2) = updated_q.y();
    updated_X_state(kStateQuaternionOffset + 3) = updated_q.z();
    updated_X_state.segment<3>(kStateVelocityOffset) =
        X_state.segment<3>(kStateVelocityOffset) + world_acc * dt;
    updated_X_state.segment<3>(kStateGyroBiasOffset) =
        X_state.segment<3>(kStateGyroBiasOffset) + nbg * dt;
    updated_X_state.segment<3>(kStateAccelBiasOffset) =
        X_state.segment<3>(kStateAccelBiasOffset) + nba * dt;
    return updated_X_state;
}


/// @brief Measurement model for odom pose: return nominal [p,q].
VectorXd g_model()
{
    VectorXd g(VectorXd::Zero(measurementSize));

    // odom measurement model returns the nominal pose [p,q]. The residual is
    // not computed here because quaternion residuals require SO(3) logarithm,
    // which is handled by build_odom_pose_residual().
    g.segment<7>(0) = X_state.segment<7>(kStatePositionOffset);
    return g;
}

/// @brief Return the skew-symmetric matrix used in SO(3) Jacobians.
Matrix3d hat(Vector3d v)
{
    Matrix3d v_hat;
    v_hat << 0, -v(2), v(1),
        v(2), 0, -v(0),
        -v(1), v(0), 0;
    return v_hat;
}

/// @brief Error-state process Jacobian F for IMU prediction.
MatrixXd diff_f_diff_x(Quaterniond q_last, Vector3d gyro, Vector3d acc, Vector3d bg_last, Vector3d ba_last)
{
    // Continuous-time error-state Jacobian F for
    //   dx=[dp,dtheta,dv,dbg,dba].
    //
    // Non-zero blocks:
    //   d(dp_dot)/d(dv)        = I
    //   d(dtheta_dot)/dtheta   = -hat(gyro-bg)
    //   d(dtheta_dot)/d(dbg)   = -I
    //   d(dv_dot)/dtheta       = -R(q) * hat(acc-ba)
    //   d(dv_dot)/d(dba)       = -R(q)
    //
    // Bias random walk terms are modeled through process noise; with the
    // current Qt/V layout there are no direct F blocks for dbg/dba.
    MatrixXd diff_f_diff_x_jacobian(MatrixXd::Zero(errorstateSize, errorstateSize));
    diff_f_diff_x_jacobian.block<3, 3>(kErrorPositionOffset, kErrorVelocityOffset) =
        Eigen::Matrix3d::Identity();
    diff_f_diff_x_jacobian.block<3, 3>(kErrorRotationOffset, kErrorRotationOffset) =
        -hat(gyro - bg_last);
    diff_f_diff_x_jacobian.block<3, 3>(kErrorRotationOffset, kErrorGyroBiasOffset) =
        -Eigen::Matrix3d::Identity();
    diff_f_diff_x_jacobian.block<3, 3>(kErrorVelocityOffset, kErrorRotationOffset) =
        -q_last.toRotationMatrix() * hat(acc - ba_last);
    diff_f_diff_x_jacobian.block<3, 3>(kErrorVelocityOffset, kErrorAccelBiasOffset) =
        -q_last.toRotationMatrix();
    return diff_f_diff_x_jacobian;
}

/// @brief Process-noise Jacobian V mapping IMU noise into the 15D error state.
MatrixXd diff_f_diff_n(Quaterniond q_last)
{
    // V maps IMU white noise n=[n_gyro, n_acc] into dx. With the current model:
    //   gyro noise affects dtheta_dot
    //   acc noise affects dv_dot after rotation into world frame
    // Bias random-walk noise is represented by nbg/nba variables in the nominal
    // propagation, but Qt is only 6x6 here; extending bias process noise would
    // require increasing inputSize and adding dbg/dba noise blocks.
    MatrixXd diff_f_diff_n_jacobian(MatrixXd::Zero(errorstateSize, inputSize));
    diff_f_diff_n_jacobian.block<3, 3>(kErrorRotationOffset, 0) =
        -Eigen::Matrix3d::Identity();
    diff_f_diff_n_jacobian.block<3, 3>(kErrorVelocityOffset, 3) =
        -q_last.toRotationMatrix();

    return diff_f_diff_n_jacobian;
}

/// @brief Odom measurement Jacobian H from 15D error state to 6D pose residual.
MatrixXd diff_g_diff_x()
{
    // Odom observes pose only:
    //   residual_p      ~= dp
    //   residual_theta  ~= dtheta
    // The velocity and bias columns remain zero; they can still be corrected
    // indirectly through cross-covariance P(position/orientation, velocity/bias).
    MatrixXd diff_g_diff_x_jacobian(MatrixXd::Zero(measurementSize - 1, errorstateSize));
    diff_g_diff_x_jacobian.block<3, 3>(kResidualPositionOffset, kErrorPositionOffset) =
        MatrixXd::Identity(3, 3);
    diff_g_diff_x_jacobian.block<3, 3>(kResidualRotationOffset, kErrorRotationOffset) =
        MatrixXd::Identity(3, 3);

    return diff_g_diff_x_jacobian;
}

/// @brief Measurement-noise Jacobian for the 6D odom residual.
MatrixXd diff_g_diff_v()
{
    MatrixXd diff_g_diff_v_jacobian(MatrixXd::Identity(measurementSize - 1, measurementSize - 1));

    return diff_g_diff_v_jacobian;
}
