#ifndef MotionEstimation_EKF_H
#define MotionEstimation_EKF_H

#include <Eigen/Dense>

#include <ros/ros.h>

using namespace std;
using namespace Eigen;
//! @brief Common variables
const double PI = 3.141592653589793;
const double TAU = 6.283185307179587;

/// @brief Publish the main fused EKF odometry and visualization paths.
void system_pub(const Eigen::VectorXd &X_state_in, ros::Time stamp);
/// @brief Publish a short-horizon predicted EKF odometry state.
void ahead_system_pub(const Eigen::VectorXd &X_state_in, ros::Time stamp);
/// @brief Publish the odom measurement pose used by the EKF update.
void cam_system_pub(ros::Time stamp);

// process model
//
// Core EKF convention used by src/ekf_node_vio_timesync_with_acc_pub.cpp:
//   Nominal state X is 16D:
//     X = [p(3), q_wxyz(4), v(3), bg(3), ba(3)]
//   Error state dx is 15D:
//     dx = [dp(3), dtheta(3), dv(3), dbg(3), dba(3)]
//   StateCovariance is P(dx), not P(X). Quaternion uncertainty is stored as
//   dtheta, and every Kalman correction must be injected with boxplus().
//   Odom observations are kept as 7D [p,q] only until the residual is built;
//   the actual EKF residual is 6D [dp, dtheta].
/// @brief Initialize nominal state, covariance, process noise, and odom measurement noise.
void initsys();
/// @brief Extract nominal EKF state components X=[p,q,v,bg,ba].
void getState(Vector3d &p, Quaterniond &q, Vector3d &v, Vector3d &bg, Vector3d &ba); // p q v bias
/// @brief Odom pose measurement model returning nominal [p,q].
VectorXd g_model();

/// @brief Error-state process Jacobian F for IMU prediction, dx_dot ~= F dx.
MatrixXd diff_f_diff_x(Quaterniond q_last, Vector3d gyro, Vector3d acc, Vector3d bg_last, Vector3d ba_last); //  p q v bias
/// @brief Process-noise Jacobian V mapping IMU noise [gyro,acc] into dx.
MatrixXd diff_f_diff_n(Quaterniond q_last);
/// @brief Discrete propagation of the 16D nominal state using one IMU sample.
VectorXd propagate_nominal_state(VectorXd X_state, Vector3d gyro, Vector3d acc, double dt);
/// @brief Odom measurement Jacobian H from 15D error state to 6D pose residual.
MatrixXd diff_g_diff_x();
/// @brief Measurement-noise Jacobian W for the 6D odom residual.
MatrixXd diff_g_diff_v();

/// @brief Convert a rotation matrix residual to a Lie-algebra rotation vector.
Vector3d rotation_2_lie_algebra(Matrix3d R);
/// @brief Convert a Lie-algebra rotation vector to a rotation matrix.
Matrix3d lie_algebra_2_rotation(Vector3d v);

/// @brief Apply a 15D error-state increment to the 16D nominal state.
VectorXd boxplus(VectorXd x, VectorXd dx);

/// @brief Return the skew-symmetric matrix for SO(3) Jacobian calculations.
Matrix3d hat(Vector3d v);

#endif
