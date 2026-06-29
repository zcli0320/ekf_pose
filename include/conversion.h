#include <Eigen/Geometry>
#include <Eigen/Dense>
using namespace std;
using namespace Eigen;

// Rotation conversion convention shared by the EKF node:
//   Euler vector = [roll, pitch, yaw]
//   Rotation order = ZYX = Rz(yaw) * Ry(pitch) * Rx(roll)
//   Quaternion scalar order in APIs = (w, x, y, z)
// These utilities are for conversion and diagnostics; the EKF state keeps
// orientation as a quaternion and applies corrections through SO(3) increments.
/// @brief Convert ZYX roll-pitch-yaw Euler angles to a quaternion.
Quaterniond euler2quaternion(Vector3d euler);
/// @brief Convert a quaternion to a rotation matrix.
Matrix3d quaternion2mat(Quaterniond q);
/// @brief Convert a rotation matrix to ZYX roll-pitch-yaw Euler angles.
Vector3d mat2euler(Matrix3d m);
/// @brief Convert a rotation matrix to a quaternion.
Quaterniond mat2quaternion(Matrix3d m);
/// @brief Convert ZYX roll-pitch-yaw Euler angles to a rotation matrix.
Matrix3d euler2mat(Vector3d euler);
/// @brief Convert a quaternion to ZYX roll-pitch-yaw Euler angles.
Vector3d quaternion2euler(Quaterniond q);
/// @brief Extract yaw from a quaternion using the ZYX convention.
double quaternion2yaw(Quaterniond q);
/// @brief Build the mapping from Euler-angle rates to body angular velocity.
Matrix3d w_Euler2Body(Vector3d q);
/// @brief Build the inverse mapping from body angular velocity to Euler-angle rates.
Matrix3d w_Body2Euler(Vector3d q);
