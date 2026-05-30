#include "ekf.h"

#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Range.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/MarkerArray.h>
#include <Eigen/Eigen>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <unsupported/Eigen/MatrixFunctions>
#include <algorithm>
#include <cmath>
#include <deque>
#include <limits>
#include <string>
// #include <geometry_msgs/Accel.h>
#include "conversion.h"

using namespace std;
using namespace Eigen;
// 20200531: time synchronization
// 20200105: ekf_node_vio.cpp and ekf_node_mocap.cpp merge into one (ekf_node_vio.cpp) and the differences between them is the odom format
// X_state: p q v gb ab   with time stamp aligned between imu and img
/*
    EKF model
    prediction:
    xt~ = xt-1 + dt*f(xt-1, ut, 0)
    sigmat~ = Ft*sigmat-1*Ft' + Vt*Qt*Vt'
    Update:
    Kt = sigmat~*Ct'*(Ct*sigmat~*Ct' + Wt*Rt*Wt')^-1
    xt = xt~ + Kt*(zt - g(xt~,0))
    sigmat = sigmat~ - Kt*Ct*sigmat~
*/
/*
   -pi ~ pi crossing problem:
   1. the model prpagation: X_state should be limited to [-pi,pi] after predicting and updating
   2. inovation crossing: (measurement - g(X_state)) should also be limited to [-pi,pi] when getting the inovation.
   z_measurement is normally in [-pi~pi]
*/

// imu frame is imu body frame

// odom: pose px,py pz orientation qw qx qy qz
// imu: acc: x y z gyro: wx wy wz

#define TimeSync 1 // time synchronize or not
#define RePub 0    // re publish the odom when repropagation

#define POS_DIFF_THRESHOLD (0.8f)

ros::Publisher odom_pub, ahead_odom_pub;
ros::Publisher cam_odom_pub;
ros::Publisher acc_filtered_pub;
ros::Publisher input_path_pub, ekf_path_pub, measurement_path_pub;
ros::Publisher ekf_segments_pub;
ros::Publisher ekf_arrows_pub;
ros::Publisher gnss_path_pub;
ros::Time imu_back_time = ros::Time(0), imu_front_time = ros::Time(0);

// state
geometry_msgs::Pose pose;
Vector3d position, orientation, velocity;

// Now set up the relevant matrices
// states X [p q pdot]  [px,py,pz, wx,wy,wz, vx,vy,vz]
size_t stateSize;            // x = [p q pdot bg ba]
size_t errorstateSize;       // x = [p q pdot bg ba]
size_t stateSize_pqv;        // x = [p q pdot]
size_t measurementSize;      // z = [p q]
size_t inputSize;            // u = [w a]
VectorXd X_state(stateSize); // x (in most literature)
VectorXd u_input;
VectorXd Z_measurement;              // z
MatrixXd StateCovariance;            // sigma
MatrixXd Kt_kalmanGain;              // Kt
VectorXd X_state_correct(stateSize); // x (in most literature)
MatrixXd StateCovariance_correct;    // sigma
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

// Vector3d q_last;
//! changed by wz
Quaterniond q_last;
Vector3d bg_last;
Vector3d ba_last;
// Qt imu covariance matrix  smaller believe system(imu) more
double imu_trans_x = 0.0;
double imu_trans_y = 0.0;
double imu_trans_z = 0.0;
double gyro_cov = 0.01;
double acc_cov = 0.01;
// Rt visual odomtry covariance smaller believe measurement more
double position_cov = 0.1;
double q_rp_cov = 0.1;
double q_yaw_cov = 0.1;
double scale_g;
double dt = 0.005; // second
double t_last, t_now;
bool first_frame_imu = true;
bool first_frame_tag_odom = true;
bool ekf_initialized = false;
bool test_odomtag_call = false;
bool odomtag_call = false;

double time_now, time_last;
double time_odom_tag_now;
// double diff_time;

double cutoff_freq = 20;
double sample_freq = 120;
int publish_warmup_frames = 0;
double odom_jump_threshold = 2.0;
double odom_reset_threshold = 6.0;
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
bool enable_gnss_velocity_when_odom_lost = true;
double gnss_velocity_min_dt = 0.05;
double gnss_velocity_max_dt = 1.5;
double gnss_velocity_cov = 1.0;
double gnss_adaptive_threshold = 3.0;
double gnss_adaptive_reject_threshold = 5.0;
double gnss_adaptive_max_scale = 25.0;
int odom_realign_settle_frames = 20;
double odom_source_switch_grace = 0.5;
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
double gnss_innovation_gate = 15.0;
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
int gnss_alignment_max_samples = 30;
double gnss_alignment_min_motion = 3.0;
double gnss_alignment_sample_interval = 0.5;
double gnss_odom_sync_max_dt = 0.2;
double gnss_alignment_max_residual = 1.0;
int gnss_min_status = 0;
int reject_reinit_limit = 10;
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
std::deque<nav_msgs::Odometry::ConstPtr> pending_odom_measurements;
const size_t max_pending_odom_measurements = 200;

bool odom_is_lost_at(double stamp_sec);
void process_vioodom(const nav_msgs::Odometry::ConstPtr &msg);
void drain_pending_odom_measurements();

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

void symmetrize_covariance(MatrixXd &covariance)
{
    covariance = 0.5 * (covariance + covariance.transpose());
}

void joseph_covariance_update(const MatrixXd &H, const MatrixXd &K, const MatrixXd &R)
{
    const MatrixXd I = MatrixXd::Identity(errorstateSize, errorstateSize);
    const MatrixXd IKH = I - K * H;
    StateCovariance = IKH * StateCovariance * IKH.transpose() + K * R * K.transpose();
    symmetrize_covariance(StateCovariance);
}

//zyh added
double offset_px, offset_py, offset_pz;
Eigen::Vector3d first_odom_get_;

// world frame points velocity
deque<pair<VectorXd, sensor_msgs::Imu>> sys_seq;
deque<MatrixXd> cov_seq;
double dt_0_rp; // the dt for the first frame in repropagation
void seq_keep(const sensor_msgs::Imu::ConstPtr &imu_msg)
{
#define seqsize 100
    if (sys_seq.size() < seqsize)
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
bool search_proper_frame(double odom_time)
{
    if (sys_seq.size() == 0)
    {
        ROS_ERROR("sys_seq.size() == 0. if appear this error, should check the code");
        return false;
    }
    if (sys_seq.size() == 1)
    {
        ROS_ERROR("sys_seq.size() == 0. if appear this error, should check the code");
        return false;
    }

    size_t rightframe = sys_seq.size() - 1;
    bool find_proper_frame = false;
    for (size_t i = 1; i < sys_seq.size(); i++) // TODO: it better to search from the middle instead in the front
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
void re_propagate()
{
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

        //! changed by wz
        bg_last = sys_seq[i].first.segment<3>(10); // last X4
        ba_last = sys_seq[i].first.segment<3>(13); // last X5

        //! changed by wz
        Ft = MatrixXd::Identity(errorstateSize, errorstateSize) + dt * diff_f_diff_x(q_last, u_gyro, u_acc, bg_last, ba_last);

        Vt = dt * diff_f_diff_n(q_last);

        //! changed by wz
        X_state = upate_state_Quaterniond_F_model(X_state, u_gyro, u_acc, dt);

        StateCovariance = Ft * StateCovariance * Ft.transpose() + Vt * Qt * Vt.transpose();
        symmetrize_covariance(StateCovariance);

#if RePub
        system_pub(X_state, sys_seq[i].second.header.stamp); // choose to publish the repropagation or not
#endif
    }
}
void imu_callback(const sensor_msgs::Imu::ConstPtr &msg)
{
    // wmywmy
    // imu_time = msg->header.stamp;
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
    // wmywmy

    // seq_keep(msg);
    // nav_msgs::Odometry odom_fusion;
    // your code for propagation
    if (!first_frame_tag_odom)
    { // get the initial pose and orientation in the first frame of measurement
        if (first_frame_imu)
        {
            first_frame_imu = false;
            time_now = new_msg->header.stamp.toSec();
            time_last = time_now;
#if TimeSync
            seq_keep(new_msg); // keep before propagation
#endif

            if (ekf_initialized) {
                system_pub(X_state, new_msg->header.stamp);
            }
            // cout << "first frame imu" << endl;
        }
        else
        {
#if TimeSync
            seq_keep(new_msg); // keep before propagation
#endif
            // cout << "\033[1;32m[ INFO] [IMU] [TimeSync] [seq_keep] [OK] \033[0m" << endl;
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

            if (odomtag_call)
            {
                odomtag_call = false;
                // diff_time = time_now - time_odom_tag_now;
                // if(diff_time<0)
                // {
                //     cout << "diff time: " << diff_time << endl;  //???!!! exist !!!???
                //     cout << "timeimu: " << time_now - 1.60889e9 << " time_odom: " << time_odom_tag_now - 1.60889e9 << endl;
                //     // cout << "diff time: " << diff_time << endl;  //about 30ms
                // }
            }
            MatrixXd Ft;
            MatrixXd Vt;

            u_gyro(0) = new_msg->angular_velocity.x;
            u_gyro(1) = new_msg->angular_velocity.y;
            u_gyro(2) = new_msg->angular_velocity.z;
            u_acc(0) = new_msg->linear_acceleration.x;
            u_acc(1) = new_msg->linear_acceleration.y;
            u_acc(2) = new_msg->linear_acceleration.z;

            //! changed by wz
            q_last.w() = X_state(3);
            q_last.x() = X_state(4);
            q_last.y() = X_state(5);
            q_last.z() = X_state(6);
            // cout << "q_last" << endl
            //      << q_last << endl;

            bg_last = X_state.segment<3>(10); // last X4
            // cout << "bg_last" << endl
            //  << bg_last << endl;

            ba_last = X_state.segment<3>(13); // last X5

            // cout << "ba_last" << endl
            //  << ba_last << endl;
            //! changed by wz
            // cout << "dt:" << dt << endl;
            Ft = MatrixXd::Identity(errorstateSize, errorstateSize) + dt * diff_f_diff_x(q_last, u_gyro, u_acc, bg_last, ba_last);

            // cout << "Ft" << endl
            //      << Ft << endl;

            Vt = dt * diff_f_diff_n(q_last);
            const bool odom_lost_for_prediction =
                odom_is_lost_at(time_now) && last_accepted_gnss_velocity_initialized;
            const Vector3d position_before_prediction = X_state.segment<3>(0);
            X_state = upate_state_Quaterniond_F_model(X_state, u_gyro, u_acc, dt);
            if (odom_lost_for_prediction)
            {
                X_state.segment<3>(0) =
                    position_before_prediction + last_accepted_gnss_velocity * dt;
                X_state.segment<3>(7) = last_accepted_gnss_velocity;
            }
           

            StateCovariance = Ft * StateCovariance * Ft.transpose() + Vt * Qt * Vt.transpose();
            symmetrize_covariance(StateCovariance);

            time_last = time_now;

            // Eigen::VectorXd X_state_ahead = X_state + 0.01 * F_model(u_gyro, u_acc);
            //! changed by wz
            Eigen::VectorXd X_state_ahead = upate_state_Quaterniond_F_model(X_state, u_gyro, u_acc, 0.01);

            // if(test_odomtag_call) //no frequency boost
            // {
            //     test_odomtag_call = false;
            //     system_pub(msg->header.stamp);
            // }
            if (ekf_initialized) {
                system_pub(X_state, new_msg->header.stamp);
                ahead_system_pub(X_state_ahead, new_msg->header.stamp);
            }
            drain_pending_odom_measurements();

            // cout << "[IMU] [TimeSync] [OK]" << endl;

            // system_pub(X_state, ros::Time::now());
            // ahead_system_pub(X_state_ahead, ros::Time::now());
        }
    }
}

// Rotation from the camera frame to the IMU frame
Matrix3d Rc_i;
Vector3d tc_i; //  cam in imu frame
int cnt = 0;
Vector3d INNOVATION_;
Matrix3d Rr_i;
Vector3d tr_i; //  rigid body in imu frame
int consecutive_reject_count = 0;
nav_msgs::Path input_path_msg;
nav_msgs::Path ekf_path_msg;
nav_msgs::Path measurement_path_msg;
nav_msgs::Path gnss_path_msg;
bool output_filter_initialized = false;
Vector3d output_filter_state = Vector3d::Zero();
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

void record_odom_position_for_gnss_sync(const ros::Time &stamp, const Vector3d &position);
bool lookup_odom_position_for_gnss_sync(const ros::Time &stamp, Vector3d &position);

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

void ensure_ekf_segment(const std::string &frame_id, const ros::Time &stamp)
{
    if (ekf_segment_markers.markers.empty())
    {
        ekf_segment_markers.markers.push_back(make_ekf_segment_marker(frame_id, stamp, 0));
        ekf_active_segment_index = 0;
    }
}

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

//! changed by wz
VectorXd get_pose_from_VIOodom(const nav_msgs::Odometry::ConstPtr &msg)
{
    // cout << "get_pose_from_VIOodom" << endl;
    Matrix3d Rr_w; // rigid body in world
    Vector3d tr_w;
    Matrix3d Ri_w;
    Vector3d ti_w;
    Vector3d p_temp;
    p_temp(0) = msg->pose.pose.position.x;
    p_temp(1) = msg->pose.pose.position.y;
    p_temp(2) = msg->pose.pose.position.z;
    // quaternion2euler:  ZYX  roll pitch yaw
    Quaterniond q;
    q.w() = msg->pose.pose.orientation.w;
    q.x() = msg->pose.pose.orientation.x;
    q.y() = msg->pose.pose.orientation.y;
    q.z() = msg->pose.pose.orientation.z;
    q.normalize();

    // Euler transform
    //  Ri_w = q.toRotationMatrix();
    //  ti_w = p_temp;
    Rr_w = q.toRotationMatrix();
    tr_w = p_temp;
    Ri_w = Rr_w * Rr_i.inverse();
    ti_w = tr_w - Ri_w * tr_i;
    // Vector3d euler = mat2euler(Ri_w);
    //! changed by wz
    Quaterniond q_wi = Quaterniond(Ri_w);

    // VectorXd pose = VectorXd::Random(6);
    //! changed by wz
    VectorXd pose = VectorXd::Random(7);
    pose.segment<3>(0) = ti_w;
    // pose.segment<3>(3) = euler;
    //! changed by wz
    pose.segment<4>(3) = Vector4d(q_wi.w(), q_wi.x(), q_wi.y(), q_wi.z());

    // cout << "pose: " << pose << endl;

    return pose;
}

double yaw_from_quaternion(const Quaterniond &q_in)
{
    Quaterniond q = q_in.normalized();
    return std::atan2(2.0 * (q.w() * q.z() + q.x() * q.y()),
                      1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z()));
}

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

double clamp01(double value)
{
    return std::max(0.0, std::min(1.0, value));
}

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

struct SensorHealthMonitor
{
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

    void reset()
    {
        window.clear();
        state = HEALTHY;
        isolated_until = -1.0;
        normal_recover_count = 0;
    }

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
    odom_update_count++;
    if (scale > 1.0)
    {
        odom_weak_count++;
        ROS_WARN_THROTTLE(1.0,
                          "Odom observation health=WEAK innovation %.3f m scale %.2f gnss_score %.2f odom_gnss_score %.2f odom_gnss_scale %.2f updates=%d weak=%d realign=%d settle_left=%d",
                          innovation_norm,
                          scale,
                          last_gnss_health_score,
                          last_odom_gnss_consistency_score,
                          last_odom_gnss_consistency_scale,
                          odom_update_count,
                          odom_weak_count,
                          odom_realign_count,
                          odom_realign_settle_count);
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
    consecutive_reject_count = 0;
    last_odom_message_time = stamp.toSec();
    odom_ever_initialized = true;
    odom_loss_reported = false;
    last_gnss_motion_reference_initialized = false;
    last_accepted_gnss_position_initialized = false;
    last_accepted_gnss_velocity_initialized = false;
    gnss_nis_monitor.reset();
    ekf_path_msg.poses.clear();
    ekf_path_counter = 0;
    output_filter_initialized = false;
    start_new_ekf_segment(world_frame_id, stamp);
}

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

void update_lastest_state()
{
    MatrixXd Ct;
    MatrixXd Wt;
    Ct = diff_g_diff_x();
    // cout << "Ct: " << Ct << endl;
    Wt = diff_g_diff_v();
    // cout << "Wt: " << Wt << endl;

    VectorXd gg = g_model();
    // VectorXd innovation = Z_measurement - gg;
    // VectorXd innovation_t = gg;
    VectorXd innovation = VectorXd::Zero(6);
    innovation.segment<3>(0) = Z_measurement.segment<3>(0) - gg.segment<3>(0);
    Quaterniond q_gg(gg(3), gg(4), gg(5), gg(6));
    q_gg.normalize();
    Quaterniond q_Z_measurement(Z_measurement(3), Z_measurement(4), Z_measurement(5), Z_measurement(6));
    q_Z_measurement.normalize();
    Quaterniond error_q = q_gg.inverse() * q_Z_measurement;
    error_q.normalize();
    innovation.segment<3>(3) = rotation_2_lie_algebra(error_q.toRotationMatrix());
    VectorXd innovation_t = gg;
    // Prevent innovation changing suddenly when euler from -Pi to Pi
    float pos_diff = sqrt(innovation(0) * innovation(0) + innovation(1) * innovation(1) + innovation(2) * innovation(2));
    if (pos_diff > POS_DIFF_THRESHOLD)
    {
        ROS_WARN_THROTTLE(5.0, "posintion diff too much between measurement and model prediction!!!   pos_diff setting: %f  but the diff measured is %f ", POS_DIFF_THRESHOLD, pos_diff);
        // return;
    }
    const double odom_scale = odom_health_scale(pos_diff, time_odom_tag_now);
    const MatrixXd R_odom = odom_scale * Wt * current_odom_Rt * Wt.transpose();
    Kt_kalmanGain = StateCovariance * Ct.transpose() * (Ct * StateCovariance * Ct.transpose() + R_odom).inverse();


    INNOVATION_ = innovation_t.segment<3>(3);
    // X_state += Kt_kalmanGain * (innovation);
    X_state = boxplus(X_state, Kt_kalmanGain * (innovation));

    joseph_covariance_update(Ct, Kt_kalmanGain, R_odom);

    // ROS_INFO("time cost: %f\n", (clock() - t) / CLOCKS_PER_SEC);
    // cout << "z " << Z_measurement(2) << " k " << Kt_kalmanGain(2) << " inn " << innovation(2) << endl;

    test_odomtag_call = true;
    odomtag_call = true;

    if (INNOVATION_(0) > 6 || INNOVATION_(1) > 6 || INNOVATION_(2) > 6)
        cout << "\ninnovation: \n"
             << INNOVATION_ << endl;
    if (INNOVATION_(0) < -6 || INNOVATION_(1) < -6 || INNOVATION_(2) < -6)
        cout << "\ninnovation: \n"
             << INNOVATION_ << endl;
    // monitor the position changing
    if (cnt == 10 || cnt == 50 || cnt == 90)
    {
        // cout << "Ct: \n" << Ct << "\nWt:\n" << Wt << endl;
        // cout << "Kt_kalmanGain: \n" << Kt_kalmanGain << endl;
        // cout << "\ninnovation: \n" << Kt_kalmanGain*innovation  << "\ndt:\n" << dt << endl;
        // cout << "\ninnovation: \n" << Kt_kalmanGain*innovation  << endl;
        // cout << "\ninnovation: \n" << INNOVATION_ << endl;
    }
    cnt++;
    if (cnt > 100)
        cnt = 101;
}
Vector3d rotation_2_lie_algebra(Matrix3d R)
{

    Eigen::Vector3d omega;
    double theta = std::acos((R.trace() - 1) / 2);

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

void process_vioodom(const nav_msgs::Odometry::ConstPtr &msg)
{ // assume that the odom_tag from camera is sychronized with the imus and without delay. !!!

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
            // X_state.segment<3>(3) = odom_pose.segment<3>(3);
            //! changed by wz
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

        first_odom_get_ = last_pos;
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
#if TimeSync
        if (sys_seq.empty() || msg->header.stamp > imu_back_time + ros::Duration(1.0e-4))
        {
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
#endif
        double odom_step = (last_pos - Vector3d(msg->pose.pose.position.x,
                                                msg->pose.pose.position.y,
                                                msg->pose.pose.position.z))
                               .norm();
        if (odom_step > odom_jump_threshold)
        {
            ROS_WARN("Detected odom jump %.3f m at %.3f s", odom_step, msg->header.stamp.toSec());
            VectorXd raw_odom_pose = get_pose_from_VIOodom(msg);
            if (enable_odom_realign && odom_measurement_position_initialized)
            {
                realign_odom_frame(raw_odom_pose, msg->header.stamp, odom_step);
            }
            else
            {
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
        // Eigen::Vector3d euler_odom(odom_pose(3), odom_pose(4), odom_pose(5));
        //! changed by wz
        Eigen::Quaterniond q_odom(odom_pose(3), odom_pose(4), odom_pose(5), odom_pose(6));
        Matrix3d R_odom;
        // R_odom = euler2mat(euler_odom);
        //! changed by wz
        R_odom = q_odom.toRotationMatrix();

        Z_measurement.segment<3>(0) = odom_pose.segment<3>(0);
        Z_measurement.segment<4>(3) = odom_pose.segment<4>(3);


#if TimeSync
        // call back to the proper time
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

#endif

        // cam_system_pub(msg->header.stamp);

#if !TimeSync // no aligned
        MatrixXd Ct;
        MatrixXd Wt;
        Ct = diff_g_diff_x();
        Wt = diff_g_diff_v();

        Kt_kalmanGain = StateCovariance * Ct.transpose() * (Ct * StateCovariance * Ct.transpose() + Wt * Rt * Wt.transpose()).inverse();
        VectorXd gg = g_model();
        VectorXd innovation = Z_measurement - gg;
        VectorXd innovation_t = gg;

        // Prevent innovation changing suddenly when euler from -Pi to Pi
        float pos_diff = sqrt(innovation(0) * innovation(0) + innovation(1) * innovation(1) + innovation(2) * innovation(2));
        if (pos_diff > POS_DIFF_THRESHOLD)
        {
            ROS_WARN_THROTTLE(5.0, "posintion diff too much between measurement and model prediction!!!   pos_diff setting: %f  but the diff measured is %f ", POS_DIFF_THRESHOLD, pos_diff);
            return;
        }
        if (innovation(3) > 6)
            innovation(3) -= 2 * PI;
        if (innovation(3) < -6)
            innovation(3) += 2 * PI;
        if (innovation(4) > 6)
            innovation(4) -= 2 * PI;
        if (innovation(4) < -6)
            innovation(4) += 2 * PI;
        if (innovation(5) > 6)
            innovation(5) -= 2 * PI;
        if (innovation(5) < -6)
            innovation(5) += 2 * PI;
        INNOVATION_ = innovation_t.segment<3>(3);
        X_state += Kt_kalmanGain * (innovation);
        1.4571 X_state(3) -= 2 * PI;
        if (X_state(3) < -PI)
            X_state(3) += 2 * PI;
        if (X_state(4) > PI)
            X_state(4) -= 2 * PI;
        if (X_state(4) < -PI)
            X_state(4) += 2 * PI;
        if (X_state(5) > PI)
            X_state(5) -= 2 * PI;
        if (X_state(5) < -PI)
            X_state(5) += 2 * PI;
        joseph_covariance_update(Ct, Kt_kalmanGain, Wt * Rt * Wt.transpose());

        // ROS_INFO("time cost: %f\n", (clock() - t) / CLOCKS_PER_SEC);
        // cout << "z " << Z_measurement(2) << " k " << Kt_kalmanGain(2) << " inn " << innovation(2) << endl;

        test_odomtag_call = true;
        odomtag_call = true;

        if (INNOVATION_(0) > 6 || INNOVATION_(1) > 6 || INNOVATION_(2) > 6)
            cout << "\ninnovation: \n"
                 << INNOVATION_ << endl;
        if (INNOVATION_(0) < -6 || INNOVATION_(1) < -6 || INNOVATION_(2) < -6)
            cout << "\ninnovation: \n"
                 << INNOVATION_ << endl;
        // monitor the position changing
        if ((innovation(0) > 1.5) || (innovation(1) > 1.5) || (innovation(2) > 1.5) ||
            (innovation(0) < -1.5) || (innovation(1) < -1.5) || (innovation(2) < -1.5))
            ROS_ERROR("posintion diff too much between measurement and model prediction!!!");
        if (cnt == 10 || cnt == 50 || cnt == 90)
        {
            // cout << "Ct: \n" << Ct << "\nWt:\n" << Wt << endl;
            // cout << "Kt_kalmanGain: \n" << Kt_kalmanGain << endl;
            // cout << "\ninnovation: \n" << Kt_kalmanGain*innovation  << "\ndt:\n" << dt << endl;
            // cout << "\ninnovation: \n" << Kt_kalmanGain*innovation  << endl;
            // cout << "\ninnovation: \n" << INNOVATION_ << endl;
        }
        cnt++;
        if (cnt > 100)
            cnt = 101;

#else // time sync
        MatrixXd Ct;
        MatrixXd Wt;

        // re-prediction for the rightframe
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

        bg_last = sys_seq[0].first.segment<3>(10); // last X4
        ba_last = sys_seq[0].first.segment<3>(13); // last X5

        // Ft = MatrixXd::Identity(stateSize, stateSize) + dt * diff_f_diff_x(q_last, u_gyro, u_acc, bg_last, ba_last);
        //! changed by wz
        Ft = MatrixXd::Identity(errorstateSize, errorstateSize) + dt * diff_f_diff_x(q_last, u_gyro, u_acc, bg_last, ba_last);

        // std::cout << "Ft" << std::endl
        //           << Ft << std::endl;

        Vt = dt * diff_f_diff_n(q_last);

        // std::cout << "Vt" << std::endl
        //   << Vt << std::endl;

        // X_state += dt * F_model(u_gyro, u_acc);
        //! changed by wz
        X_state = upate_state_Quaterniond_F_model(X_state, u_gyro, u_acc, dt);
        // std::cout << "X_state" << std::endl
        //   << X_state << std::endl;

        StateCovariance = Ft * StateCovariance * Ft.transpose() + Vt * Qt * Vt.transpose();
        symmetrize_covariance(StateCovariance);
        // std::cout << "StateCovariance" << std::endl
        //           << StateCovariance << std::endl;

        // re-update for the rightframe
        Ct = diff_g_diff_x();
        // std::cout << "Ct" << std::endl
        //           << Ct << std::endl;
        Wt = diff_g_diff_v();

        // std::cout << "Wt" << std::endl
        //           << Wt << std::endl;

        VectorXd gg = g_model();
        // std::cout << "gg" << std::endl
        //           << gg << std::endl;
        // std::cout << "Z_measurement" << std::endl
        //           << Z_measurement << std::endl;
        // VectorXd innovation = Z_measurement - gg;
        VectorXd innovation = VectorXd::Zero(6);
        innovation.segment<3>(0) = Z_measurement.segment<3>(0) - gg.segment<3>(0);
        // std::cout << "innovation_first" << std::endl
        //           << innovation << std::endl;
        Quaterniond q_gg(gg(3), gg(4), gg(5), gg(6));
        q_gg.normalize();
        Quaterniond q_Z_measurement(Z_measurement(3), Z_measurement(4), Z_measurement(5), Z_measurement(6));
        q_Z_measurement.normalize();
        // Quaterniond error_q = q_Z_measurement * q_gg.inverse();
        Quaterniond error_q = q_gg.inverse() * q_Z_measurement;
        error_q.normalize();
        // cout << "error_q.vec()" << endl
        //      << error_q.vec() << endl;
        Matrix3d error_R = error_q.toRotationMatrix();
        // 旋转矩阵李代数
        innovation.segment<3>(3) = rotation_2_lie_algebra(error_R);
        // std::cout << "innovation" << std::endl
        //           << innovation << std::endl;
        // std::cout << "\033[1;32m innovation" << std::endl
        //           << innovation << "\033[0m" << std::endl;
        // std::cout << "\033[1;33m position diff: " << sqrt(innovation(0) * innovation(0) + innovation(1) * innovation(1) + innovation(2) * innovation(2)) << std::endl;
        // std::cout << "angle diff: " << sqrt(innovation(3) * innovation(3) + innovation(4) * innovation(4) + innovation(5) * innovation(5)) << "\033[0m" << std::endl;
        VectorXd innovation_t = gg;

        float pos_diff = sqrt(innovation(0) * innovation(0) + innovation(1) * innovation(1) + innovation(2) * innovation(2));
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
        const double odom_scale = odom_health_scale(pos_diff, msg->header.stamp.toSec());
        const MatrixXd R_odom_meas = odom_scale * Wt * current_odom_Rt * Wt.transpose();
        Kt_kalmanGain = StateCovariance * Ct.transpose() * (Ct * StateCovariance * Ct.transpose() + R_odom_meas).inverse();
        consecutive_reject_count = 0;

  

        // Quaterniond test_q = Quaterniond(X_state(3), X_state(4), X_state(5), X_state(6)) * error_q;
        // std::cout << "\033[1;31m test_q" << std::endl
        //           << test_q << "\033[0m" << std::endl;

        INNOVATION_ = innovation_t.segment<3>(3);

        VectorXd dx(errorstateSize);
        dx = VectorXd::Zero(errorstateSize);
        dx += Kt_kalmanGain * (innovation);
        // dx += (innovation);
        // dx.segment<6>(0) = innovation.segment<6>(0);
        // std::cout << "dx" << std::endl
        //           << dx << std::endl;

        // X_state += Kt_kalmanGain * (innovation);
        X_state = boxplus(X_state, dx);
        Vector3d X_state_p = X_state.segment<3>(0);
        VectorXd final_innovation = VectorXd::Zero(6);
        final_innovation.segment<3>(0) = Z_measurement.segment<3>(0) - X_state_p;
        // std::cout << "\033[1;32m Z_measurement.segment<3>(0)" << std::endl
        //           << Z_measurement.segment<3>(0) << "\033[0m" << std::endl;
        // std::cout << "X_state_p" << std::endl
        //           << X_state_p << std::endl;

        Quaterniond q_X_state_q(X_state(3), X_state(4), X_state(5), X_state(6));
        Quaterniond q_Z_measurement_(Z_measurement(3), Z_measurement(4), Z_measurement(5), Z_measurement(6));
        Quaterniond error_q_ = q_Z_measurement_ * q_X_state_q.inverse();
        Matrix3d error_R_ = error_q_.toRotationMatrix();
        final_innovation.segment<3>(3) = rotation_2_lie_algebra(error_R_);
        // std::cout << "\033[1;32m final_innovation" << std::endl
        //           << final_innovation << "\033[0m" << std::endl;
        // std::cout << "\033[1;33m position diff: " << sqrt(final_innovation(0) * final_innovation(0) + final_innovation(1) * final_innovation(1) + final_innovation(2) * final_innovation(2)) << std::endl;
        // std::cout << "angle diff: " << sqrt(final_innovation(3) * final_innovation(3) + final_innovation(4) * final_innovation(4) + final_innovation(5) * final_innovation(5)) << "\033[0m" << std::endl;

        // std::cout << "X_state_update" << std::endl
        //           << X_state << std::endl;
        joseph_covariance_update(Ct, Kt_kalmanGain, R_odom_meas);

        // system_pub(X_state, sys_seq[0].second.header.stamp); // choose to publish the repropagation or not
        // std::cout << "re_propagate" << std::endl;

        re_propagate();

#endif
        cam_system_pub(msg->header.stamp);
    }
}

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

void vioodom_primary_callback(const nav_msgs::Odometry::ConstPtr &msg)
{
    if (should_use_odom_source(msg, "primary", true))
    {
        process_vioodom(msg);
    }
}

void vioodom_fallback_callback(const nav_msgs::Odometry::ConstPtr &msg)
{
    if (should_use_odom_source(msg, "fallback", false))
    {
        process_vioodom(msg);
    }
}

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

bool initialize_filter_from_gnss(const Vector3d &gnss_local, const ros::Time &stamp)
{
    if (!enable_gnss_cold_start)
    {
        return false;
    }

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
    last_gnss_motion_reference_initialized = false;
    gnss_nis_monitor.reset();
    output_filter_initialized = false;

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

void update_gnss_alignment(const Vector3d &gnss_local, const Vector3d &odom_position, const ros::Time &stamp)
{
    if (!gnss_alignment_initialized)
    {
        gnss_alignment_R = Matrix3d::Identity();
        gnss_alignment_offset = odom_position - gnss_local;
        gnss_alignment_initialized = true;
        gnss_path_msg.header.frame_id = world_frame_id;
        ROS_INFO("Initialized GNSS translation offset from odom measurement: %.3f %.3f %.3f",
                 gnss_alignment_offset.x(),
                 gnss_alignment_offset.y(),
                 gnss_alignment_offset.z());
    }

    if (!enable_gnss_yaw_alignment || gnss_yaw_alignment_initialized)
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
    while (static_cast<int>(gnss_alignment_pairs.size()) > gnss_alignment_max_samples)
    {
        gnss_alignment_pairs.pop_front();
    }

    if (static_cast<int>(gnss_alignment_pairs.size()) < gnss_alignment_min_samples)
    {
        return;
    }

    const Vector3d gnss_motion = gnss_alignment_pairs.back().first - gnss_alignment_pairs.front().first;
    const Vector3d odom_motion = gnss_alignment_pairs.back().second - gnss_alignment_pairs.front().second;
    if (gnss_motion.head<2>().norm() < gnss_alignment_min_motion ||
        odom_motion.head<2>().norm() < gnss_alignment_min_motion)
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
    if (residual_max > gnss_alignment_max_residual)
    {
        ROS_WARN_THROTTLE(1.0,
                          "Skipping GNSS yaw alignment: residual mean %.3f max %.3f exceeds %.3f yaw_delta %.3f",
                          residual_mean,
                          residual_max,
                          gnss_alignment_max_residual,
                          yaw_delta);
        return;
    }

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
}

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

    const Vector3d z_gnss = gnss_alignment_R * gnss_local + gnss_alignment_offset;
    const Vector3d innovation = z_gnss - X_state.segment<3>(0);
    const double innovation_norm = innovation.norm();

    MatrixXd H = MatrixXd::Zero(3, errorstateSize);
    H.block<3, 3>(0, 0) = Matrix3d::Identity();

    Matrix3d R_base = Matrix3d::Zero();
    if (gnss_use_msg_covariance && msg->position_covariance_type != sensor_msgs::NavSatFix::COVARIANCE_TYPE_UNKNOWN)
    {
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
        if (last_accepted_gnss_position_initialized)
        {
            const double accepted_dt = stamp - last_accepted_gnss_position_time;
            if (accepted_dt >= gnss_velocity_min_dt && accepted_dt <= gnss_velocity_max_dt)
            {
                last_accepted_gnss_velocity =
                    (z_gnss - last_accepted_gnss_position) / accepted_dt;
                last_accepted_gnss_velocity_initialized = true;
            }
        }
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

    MatrixXd H_update = H;
    VectorXd innovation_update = innovation;
    MatrixXd R_update = R;
    bool using_gnss_velocity = false;
    Vector3d gnss_velocity = Vector3d::Zero();
    if (odom_lost &&
        enable_gnss_velocity_when_odom_lost &&
        last_accepted_gnss_position_initialized)
    {
        const double gnss_dt = stamp - last_accepted_gnss_position_time;
        if (gnss_dt >= gnss_velocity_min_dt && gnss_dt <= gnss_velocity_max_dt)
        {
            gnss_velocity = (z_gnss - last_accepted_gnss_position) / gnss_dt;
            H_update = MatrixXd::Zero(6, errorstateSize);
            H_update.block<3, 3>(0, 0) = Matrix3d::Identity();
            H_update.block<3, 3>(3, 6) = Matrix3d::Identity();
            innovation_update = VectorXd::Zero(6);
            innovation_update.segment<3>(0) = innovation;
            innovation_update.segment<3>(3) = gnss_velocity - X_state.segment<3>(7);
            R_update = MatrixXd::Zero(6, 6);
            R_update.block<3, 3>(0, 0) = R;
            R_update.block<3, 3>(3, 3) =
                Matrix3d::Identity() * std::max(1.0e-4, gnss_velocity_cov * gnss_scale);
            using_gnss_velocity = true;
            ROS_INFO_THROTTLE(2.0,
                              "GNSS velocity pseudo-update active: dt %.3f s vel %.3f %.3f %.3f odom_score %.2f",
                              gnss_dt,
                              gnss_velocity.x(),
                              gnss_velocity.y(),
                              gnss_velocity.z(),
                              effective_odom_health_score);
        }
    }

    MatrixXd S = H_update * StateCovariance * H_update.transpose() + R_update;
    MatrixXd K = StateCovariance * H_update.transpose() * S.inverse();
    VectorXd dx = VectorXd::Zero(errorstateSize);
    dx = K * innovation_update;
    X_state = boxplus(X_state, dx);
    joseph_covariance_update(H_update, K, R_update);

    Vector3d position_delta = dx.segment<3>(0);
    Vector3d velocity_delta = Vector3d::Zero();
    if (using_gnss_velocity)
    {
        velocity_delta = dx.segment<3>(6);
    }
    if (odom_lost && enable_gnss_position_snap_when_odom_lost)
    {
        const Vector3d forced_position_delta = z_gnss - X_state.segment<3>(0);
        X_state.segment<3>(0) = z_gnss;
        position_delta += forced_position_delta;
        if (enable_gnss_velocity_when_odom_lost && last_accepted_gnss_velocity_initialized)
        {
            const Vector3d forced_velocity_delta =
                last_accepted_gnss_velocity - X_state.segment<3>(7);
            X_state.segment<3>(7) = last_accepted_gnss_velocity;
            velocity_delta += forced_velocity_delta;
            using_gnss_velocity = true;
        }
    }
    for (auto &state_and_imu : sys_seq)
    {
        state_and_imu.first.segment<3>(0) += position_delta;
        if (using_gnss_velocity)
        {
            state_and_imu.first.segment<3>(7) += velocity_delta;
        }
    }
    if (last_accepted_gnss_position_initialized)
    {
        const double accepted_dt = stamp - last_accepted_gnss_position_time;
        if (accepted_dt >= gnss_velocity_min_dt && accepted_dt <= gnss_velocity_max_dt)
        {
            last_accepted_gnss_velocity =
                (z_gnss - last_accepted_gnss_position) / accepted_dt;
            last_accepted_gnss_velocity_initialized = true;
        }
    }
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
VectorXd boxplus(VectorXd x, VectorXd dx)
{
    VectorXd x_plus(x.rows());
    x_plus(0) = x(0) + dx(0);
    x_plus(1) = x(1) + dx(1);
    x_plus(2) = x(2) + dx(2);

    Vector3d dv(dx(3), dx(4), dx(5));
    Matrix3d dR = lie_algebra_2_rotation(dv);
    Quaterniond x_q(x(3), x(4), x(5), x(6));
    x_q.normalize();
    Matrix3d x_R = x_q.toRotationMatrix();
    Matrix3d x_R_plus = x_R * dR;
    Quaterniond x_q_plus(x_R_plus);
    x_q_plus.normalize();
    x_plus(3) = x_q_plus.w();
    x_plus(4) = x_q_plus.x();
    x_plus(5) = x_q_plus.y();
    x_plus(6) = x_q_plus.z();

    x_plus(7) = x(7) + dx(6);
    x_plus(8) = x(8) + dx(7);
    x_plus(9) = x(9) + dx(8);

    x_plus(10) = x(10) + dx(9);
    x_plus(11) = x(11) + dx(10);
    x_plus(12) = x(12) + dx(11);

    x_plus(13) = x(13) + dx(12);
    x_plus(14) = x(14) + dx(13);
    x_plus(15) = x(15) + dx(14);

    return x_plus;
}

Quaterniond q_gt, q_gt0;
bool first_gt = true;
void gt_callback(const nav_msgs::Odometry::ConstPtr &msg)
{
    q_gt.w() = msg->pose.pose.orientation.w;
    q_gt.x() = msg->pose.pose.orientation.x;
    q_gt.y() = msg->pose.pose.orientation.y;
    q_gt.z() = msg->pose.pose.orientation.z;

    if (first_gt && !first_frame_tag_odom)
    {
        first_gt = false;
        q_gt0 = q_gt;
        // q_gt0 = q_gt0.normalized();
    }
}

int main(int argc, char **argv)
{
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
    ros::Subscriber s1 = n.subscribe("imu", 1000, imu_callback, ros::TransportHints().tcpNoDelay());
    ros::Subscriber s2 = n.subscribe("bodyodometry_primary", 40, vioodom_primary_callback, ros::TransportHints().tcpNoDelay());
    ros::Subscriber s3 = n.subscribe("bodyodometry_fallback", 40, vioodom_fallback_callback, ros::TransportHints().tcpNoDelay());
    ros::Subscriber s4 = n.subscribe("gt_", 40, gt_callback, ros::TransportHints().tcpNoDelay());
    ros::Subscriber s5 = n.subscribe("gnss_fix", 40, gnss_fix_callback, ros::TransportHints().tcpNoDelay());
    odom_pub = n.advertise<nav_msgs::Odometry>("ekf_odom", 1000);             // freq = imu freq
    ahead_odom_pub = n.advertise<nav_msgs::Odometry>("ahead_ekf_odom", 1000); // freq = imu freq
    cam_odom_pub = n.advertise<nav_msgs::Odometry>("cam_ekf_odom", 1000);
    acc_filtered_pub = n.advertise<geometry_msgs::PoseStamped>("acc_filtered", 1000);
    input_path_pub = n.advertise<nav_msgs::Path>("input_path", 10, true);
    ekf_path_pub = n.advertise<nav_msgs::Path>("ekf_path", 10, true);
    measurement_path_pub = n.advertise<nav_msgs::Path>("measurement_path", 10, true);
    gnss_path_pub = n.advertise<nav_msgs::Path>("gnss_path", 10, true);
    ekf_segments_pub = n.advertise<visualization_msgs::MarkerArray>("ekf_segments", 10, true);
    ekf_arrows_pub = n.advertise<visualization_msgs::MarkerArray>("ekf_arrows", 10, true);

    n.getParam("gyro_cov", gyro_cov);
    n.getParam("acc_cov", acc_cov);
    n.getParam("position_cov", position_cov);
    n.getParam("q_rp_cov", q_rp_cov);
    n.getParam("q_yaw_cov", q_yaw_cov);
    n.getParam("imu_trans_x", imu_trans_x);
    n.getParam("imu_trans_y", imu_trans_y);
    n.getParam("imu_trans_z", imu_trans_z);
    n.getParam("cutoff_freq", cutoff_freq);
    n.getParam("offset_px", offset_px);
    n.getParam("offset_py", offset_py);
    n.getParam("offset_pz", offset_pz);
    n.getParam("publish_warmup_frames", publish_warmup_frames);
    n.getParam("odom_jump_threshold", odom_jump_threshold);
    n.getParam("odom_reset_threshold", odom_reset_threshold);
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
    n.getParam("enable_gnss_velocity_when_odom_lost", enable_gnss_velocity_when_odom_lost);
    n.getParam("gnss_velocity_min_dt", gnss_velocity_min_dt);
    n.getParam("gnss_velocity_max_dt", gnss_velocity_max_dt);
    n.getParam("gnss_velocity_cov", gnss_velocity_cov);
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
    n.getParam("gnss_innovation_gate", gnss_innovation_gate);
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
    n.getParam("gnss_min_status", gnss_min_status);
    n.getParam("reject_reinit_limit", reject_reinit_limit);
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

void acc_f_pub(Vector3d acc, ros::Time stamp)
{
    geometry_msgs::PoseStamped Accel_filtered;
    Accel_filtered.header.frame_id = world_frame_id;
    Accel_filtered.header.stamp = stamp;
    // Accel_filtered.header.stamp = ros::Time::now();
    Vector3d Acc_ = get_filtered_acc(acc);
    Accel_filtered.pose.position.x = Acc_[0];
    Accel_filtered.pose.position.y = Acc_[1];
    Accel_filtered.pose.position.z = Acc_[2];

    Quaterniond q;
    // q = euler2quaternion(X_state.segment<3>(3));
    //! changed by wz
    q = X_state.segment<4>(3);
    // q = q.normalized();
    Accel_filtered.pose.orientation.w = q.w();
    Accel_filtered.pose.orientation.x = q.x();
    Accel_filtered.pose.orientation.y = q.y();
    Accel_filtered.pose.orientation.z = q.z();


    cout << "q_gt0: " << quaternion2euler(q_gt0) << endl;
    cout << "q_gt: " << quaternion2euler(q_gt) << endl;
    cout << "q_vio: " << mat2euler(q_gt0.toRotationMatrix() * euler2quaternion(X_state.segment<3>(3)).toRotationMatrix()) << endl;
    cout << "q_gt0*vio != q_gt: " << quaternion2euler(q_gt0 * q) << endl;
    // cout << "q_gt0*vio*q_gt0^-1 = q_gt: " << quaternion2euler(q_gt0 * q * q_gt0.inverse()) << endl;   //q1*euler2quaternion(V)*q1.inverse()  = q1.toRotationMatrix() * V  TODO why?

    acc_filtered_pub.publish(Accel_filtered);
}
void ahead_system_pub(const Eigen::VectorXd &X_state_in, ros::Time stamp)
{
    nav_msgs::Odometry odom_fusion;
    odom_fusion.header.stamp = stamp;
    odom_fusion.header.frame_id = world_frame_id;
    // odom_fusion.header.frame_id = "imu";

    Quaterniond q;
    // q = euler2quaternion(X_state_in.segment<3>(3));
    //! changed by wz
    q = X_state_in.segment<4>(3);
    odom_fusion.pose.pose.orientation.w = q.w();
    odom_fusion.pose.pose.orientation.x = q.x();
    odom_fusion.pose.pose.orientation.y = q.y();
    odom_fusion.pose.pose.orientation.z = q.z();
    //! changed by wz
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
void system_pub(const Eigen::VectorXd &X_state_in, ros::Time stamp)
{
    nav_msgs::Odometry odom_fusion;
    odom_fusion.header.stamp = stamp;
    odom_fusion.header.frame_id = world_frame_id;
    // odom_fusion.header.frame_id = world_frame_id;
    // odom_fusion.header.frame_id = "imu";

    Quaterniond q;
    // q = euler2quaternion(X_state_in.segment<3>(3));
    //! changed by wz
    q.w() = X_state_in(3);
    q.x() = X_state_in(4);
    q.y() = X_state_in(5);
    q.z() = X_state_in(6);
    odom_fusion.pose.pose.orientation.w = q.w();
    odom_fusion.pose.pose.orientation.x = q.x();
    odom_fusion.pose.pose.orientation.y = q.y();
    odom_fusion.pose.pose.orientation.z = q.z();
    //! changed by wz
    odom_fusion.twist.twist.linear.x = X_state_in(7);
    odom_fusion.twist.twist.linear.y = X_state_in(8);
    odom_fusion.twist.twist.linear.z = X_state_in(9);

    Vector3d pos_center(X_state_in(0), X_state_in(1), X_state_in(2));
    Vector3d pos_center_world = pos_center + q.toRotationMatrix() * Vector3d(imu_trans_x, imu_trans_y, imu_trans_z);
    Vector3d pos_center_output = pos_center_world;
    if (cutoff_freq > 1.0e-5)
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
    }

    odom_fusion.pose.pose.position.x = pos_center_output(0);
    odom_fusion.pose.pose.position.y = pos_center_output(1);
    odom_fusion.pose.pose.position.z = pos_center_output(2);

    odom_fusion.pose.pose.position.x += offset_px;
    odom_fusion.pose.pose.position.y += offset_py;
    odom_fusion.pose.pose.position.z += offset_pz;

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
void cam_system_pub(ros::Time stamp)
{
    nav_msgs::Odometry odom_fusion;
    odom_fusion.header.stamp = stamp;
    odom_fusion.header.frame_id = world_frame_id;
    odom_fusion.pose.pose.position.x = Z_measurement(0);
    odom_fusion.pose.pose.position.y = Z_measurement(1);
    odom_fusion.pose.pose.position.z = Z_measurement(2);
    Quaterniond q;

    // q = euler2quaternion(Z_measurement.segment<3>(3));
    //! changed by wz
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
void initsys()
{
    //  camera position in the IMU frame = (0.05, 0.05, 0)
    // camera orientaion in the IMU frame = Quaternion(0, 1, 0, 0); w x y z, respectively
    //					   RotationMatrix << 1, 0, 0,
    //							             0, -1, 0,
    //                                       0, 0, -1;
    // set the cam2imu params
    Rc_i = Quaterniond(0, 1, 0, 0).toRotationMatrix();
    // cout << "R_cam" << endl << Rc_i << endl;
    tc_i << 0.05, 0.05, 0;

    //  rigid body position in the IMU frame = (0, 0, 0.04)
    // rigid body orientaion in the IMU frame = Quaternion(1, 0, 0, 0); w x y z, respectively
    //					   RotationMatrix << 1, 0, 0,
    //						 	             0, 1, 0,
    //                                       0, 0, 1;

    // states X [p q pdot bg ba]  [px,py,pz, wx,wy,wz, vx,vy,vz bgx,bgy,bgz bax,bay,baz]
    // stateSize = 15;                      // x = [p q pdot bg ba]

    stateSize = 16; //! changed by wz

    errorstateSize = 15; // ! changed by wz

    // stateSize_pqv = 9;                   // x = [p q pdot]

    stateSize_pqv = 10; // ! changed by wz

    // measurementSize = 6;                 // z = [p q]

    measurementSize = 7; //! changed by wz

    inputSize = 6;                       // u = [w a]
    X_state = VectorXd::Zero(stateSize); // x
    // velocity
    // X_state(6) = 0;
    // X_state(7) = 0;
    // X_state(8) = 0;
    X_state(3) = 1.0; //! changed by wz
    X_state(4) = 0;
    X_state(5) = 0;
    X_state(6) = 0;
    X_state(7) = 0;
    X_state(8) = 0;
    X_state(9) = 0;
    // bias
    X_state.segment<3>(10) = bg_0; //! changed by wz
    X_state.segment<3>(13) = ba_0;
    u_input = VectorXd::Zero(inputSize);
    Z_measurement = VectorXd::Zero(measurementSize); // z
    // StateCovariance = MatrixXd::Identity(stateSize, stateSize);     // sigma
    //! changed by wz
    StateCovariance = MatrixXd::Identity(errorstateSize, errorstateSize); // sigma

    Kt_kalmanGain = MatrixXd::Identity(stateSize, measurementSize); // Kt
    // Ct_stateToMeasurement = MatrixXd::Identity(stateSize, measurementSize);         // Ct
    X_state_correct = X_state;
    StateCovariance_correct = StateCovariance;

    Qt = MatrixXd::Identity(inputSize, inputSize); // 6x6 input [gyro acc]covariance
    // Rt = MatrixXd::Identity(measurementSize, measurementSize); // 6x6 measurement [p q]covariance
    //! changed by wz
    Rt = MatrixXd::Identity(measurementSize - 1, measurementSize - 1); // 6x6 measurement [p q]covariance
    // MatrixXd temp_Rt = MatrixXd::Identity(measurementSize, measurementSize);

    // You should also tune these parameters
    // Q imu covariance matrix; Rt visual odomtry covariance matrix
    // //Rt visual odomtry covariance smaller believe measurement more
    Qt.topLeftCorner(3, 3) = gyro_cov * Qt.topLeftCorner(3, 3);
    Qt.bottomRightCorner(3, 3) = acc_cov * Qt.bottomRightCorner(3, 3);
    Rt.topLeftCorner(3, 3) = position_cov * Rt.topLeftCorner(3, 3);
    Rt.bottomRightCorner(3, 3) = q_rp_cov * Rt.bottomRightCorner(3, 3);
    Rt.bottomRightCorner(1, 1) = q_yaw_cov * Rt.bottomRightCorner(1, 1);
    current_odom_Rt = Rt;
}


//! changed by wz
void getState(Vector3d &p, Quaterniond &q, Vector3d &v, Vector3d &bg, Vector3d &ba)
{
    p = X_state.segment<3>(0);
    // q = X_state.segment<4>(3);
    q = Quaterniond(X_state(3), X_state(4), X_state(5), X_state(6));
    v = X_state.segment<3>(7);
    bg = X_state.segment<3>(10);
    ba = X_state.segment<3>(13);
}


//! changed by wz
VectorXd get_filtered_acc(Vector3d acc)
{
    Vector3d ba;
    ba = X_state.segment<3>(13);
    return ((acc - ba - na));
}


//! changed by wz
VectorXd F_model(Vector3d gyro, Vector3d acc)
{
    // IMU is in FLU frame
    // Transform IMU frame into "world" frame whose original point is FLU's original point and the XOY plain is parallel with the ground and z axis is up
    VectorXd f(VectorXd::Zero(stateSize));
    Vector3d p, v, bg, ba;
    Quaterniond q;
    getState(p, q, v, bg, ba);
    f.segment<3>(0) = v;                          // 0,1,2
    f.segment<3>(4) = q * (gyro - bg - ng) * 0.5; // 4,5,6
    f.segment<3>(7) = gravity + q * (acc - ba - na);
    f.segment<3>(10) = nbg;
    f.segment<3>(13) = nba;

    return f;
}

//? added by wz
VectorXd upate_state_Quaterniond_F_model(VectorXd X_state, Vector3d gyro, Vector3d acc, double dt)
{
    // IMU is in FLU frame
    // Transform IMU frame into "world" frame whose original point is FLU's original point and the XOY plain is parallel with the ground and z axis is up
    VectorXd upate_X_state(VectorXd::Zero(stateSize));

    const Vector3d v = X_state.segment<3>(7);
    const Vector3d bg = X_state.segment<3>(10);
    const Vector3d ba = X_state.segment<3>(13);
    Quaterniond q(X_state(3), X_state(4), X_state(5), X_state(6));
    q.normalize();
    const Vector3d unbiased_gyro = gyro - bg - ng;
    const Vector3d world_acc = gravity + q * (acc - ba - na);

    upate_X_state.segment<3>(0) = X_state.segment<3>(0) + v * dt + 0.5 * world_acc * dt * dt;

    Quaterniond upate_q = (q * delta_quaternion_from_gyro(unbiased_gyro, dt)).normalized();
    upate_X_state(3) = upate_q.w();
    upate_X_state(4) = upate_q.x();
    upate_X_state(5) = upate_q.y();
    upate_X_state(6) = upate_q.z();
    upate_X_state.segment<3>(7) = X_state.segment<3>(7) + world_acc * dt;
    upate_X_state.segment<3>(10) = X_state.segment<3>(10) + nbg * dt;
    upate_X_state.segment<3>(13) = X_state.segment<3>(13) + nba * dt;
    return upate_X_state;
}


//! changed by wz
VectorXd g_model()
{
    VectorXd g(VectorXd::Zero(measurementSize));

    g.segment<7>(0) = X_state.segment<7>(0);



    return g;
}


//? added by wz
Matrix3d hat(Vector3d v)
{
    Matrix3d v_hat;
    v_hat << 0, -v(2), v(1),
        v(2), 0, -v(0),
        -v(1), v(0), 0;
    return v_hat;
}

//! changed by wz
MatrixXd diff_f_diff_x(Quaterniond q_last, Vector3d gyro, Vector3d acc, Vector3d bg_last, Vector3d ba_last)
{

   
    MatrixXd diff_f_diff_x_jacobian(MatrixXd::Zero(errorstateSize, errorstateSize));
    diff_f_diff_x_jacobian.block<3, 3>(0, 6) = Eigen::Matrix3d::Identity(); // dp/dv
	    diff_f_diff_x_jacobian.block<3, 3>(3, 3) = -hat(gyro - bg_last);       // d(dtheta)/d(theta)  missing term fixed
    diff_f_diff_x_jacobian.block<3, 3>(3, 9) = -Eigen::Matrix3d::Identity();
    diff_f_diff_x_jacobian.block<3, 3>(6, 3) = -q_last.toRotationMatrix() * hat(acc - ba_last); //!!!!
    diff_f_diff_x_jacobian.block<3, 3>(6, 12) = -q_last.toRotationMatrix();
    return diff_f_diff_x_jacobian;
}



//! changed by wz
MatrixXd diff_f_diff_n(Quaterniond q_last)
{
    MatrixXd diff_f_diff_n_jacobian(MatrixXd::Zero(errorstateSize, inputSize));
    diff_f_diff_n_jacobian.block<3, 3>(3, 0) = -Eigen::Matrix3d::Identity();
    diff_f_diff_n_jacobian.block<3, 3>(6, 3) = -q_last.toRotationMatrix();

    return diff_f_diff_n_jacobian;
}


//! changed by wz
MatrixXd diff_g_diff_x()
{
 

    MatrixXd diff_g_diff_x_jacobian(MatrixXd::Zero(measurementSize - 1, errorstateSize));
    diff_g_diff_x_jacobian.block<3, 3>(0, 0) = MatrixXd::Identity(3, 3);
    diff_g_diff_x_jacobian.block<3, 3>(3, 3) = MatrixXd::Identity(3, 3);

    return diff_g_diff_x_jacobian;
}

//! changed by wz
MatrixXd diff_g_diff_v()
{
    MatrixXd diff_g_diff_v_jacobian(MatrixXd::Identity(measurementSize - 1, measurementSize - 1));

    return diff_g_diff_v_jacobian;
}
