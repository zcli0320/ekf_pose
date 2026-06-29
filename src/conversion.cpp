#include <cmath>
#include <Eigen/Geometry>
#include <Eigen/Dense>
using namespace std;
using namespace Eigen;

/**
 * Rotation conversion helpers used by the EKF node.
 *
 * Convention:
 *   - Euler vector layout is [roll, pitch, yaw].
 *   - Composition order is ZYX: R = Rz(yaw) * Ry(pitch) * Rx(roll).
 *   - Quaternion storage follows Eigen's constructor order (w, x, y, z).
 *
 * Keep these helpers consistent with yaw_from_quaternion() in the EKF node.
 * Mixing Euler conventions is a common source of sign and yaw-frame bugs.
 **/
/// @brief Convert ZYX roll-pitch-yaw Euler angles to a quaternion.
Quaterniond euler2quaternion(Vector3d euler)
{
  double cr = cos(euler(0)/2);
  double sr = sin(euler(0)/2);
  double cp = cos(euler(1)/2);
  double sp = sin(euler(1)/2);
  double cy = cos(euler(2)/2);
  double sy = sin(euler(2)/2);
  Quaterniond q;
  q.w() = cr*cp*cy + sr*sp*sy;
  q.x() = sr*cp*cy - cr*sp*sy;
  q.y() = cr*sp*cy + sr*cp*sy;
  q.z() = cr*cp*sy - sr*sp*cy;
  return q; 
}


/// @brief Convert a quaternion to a rotation matrix.
Matrix3d quaternion2mat(Quaterniond q)
{
  // Closed-form rotation matrix for a unit quaternion. EKF callers normalize
  // orientation before use so quaternion scale drift does not leak into R.
  Matrix3d m;
  double a = q.w(), b = q.x(), c = q.y(), d = q.z();
  m << a*a + b*b - c*c - d*d, 2*(b*c - a*d), 2*(b*d+a*c),
       2*(b*c+a*d), a*a - b*b + c*c - d*d, 2*(c*d - a*b),
       2*(b*d - a*c), 2*(c*d+a*b), a*a-b*b - c*c + d*d;
  return m;
}

/// @brief Convert a rotation matrix to ZYX roll-pitch-yaw Euler angles.
Vector3d mat2euler(Matrix3d m)
{ 
  // ZYX inverse mapping. Pitch uses asin(-R20), so this utility inherits the
  // usual Euler singularity near +/-90 deg pitch. The EKF state itself avoids
  // that singularity by storing orientation as a quaternion.
  double r = atan2(m(2, 1), m(2, 2));
  double p = asin(-m(2, 0));
  double y = atan2(m(1, 0), m(0, 0));
  Vector3d rpy(r, p, y);
  return rpy;
}

/// @brief Convert a rotation matrix to a quaternion.
Quaterniond mat2quaternion(Matrix3d m)
{
  //return euler2quaternion(mat2euler(m));
  Quaterniond q;
  double a, b, c, d;
  a = sqrt(1 + m(0, 0) + m(1, 1) + m(2, 2))/2;
  b = (m(2, 1) - m(1, 2))/(4*a);
  c = (m(0, 2) - m(2, 0))/(4*a);
  d = (m(1, 0) - m(0, 1))/(4*a);
  q.w() = a; q.x() = b; q.y() = c; q.z() = d;
  return q;
}

//ZYX
/// @brief Convert ZYX roll-pitch-yaw Euler angles to a rotation matrix.
Matrix3d euler2mat(Vector3d euler)
{
  // Rz(yaw) * Ry(pitch) * Rx(roll), matching euler2quaternion().
  double cr = cos(euler(0));
  double sr = sin(euler(0));
  double cp = cos(euler(1));
  double sp = sin(euler(1));
  double cy = cos(euler(2));
  double sy = sin(euler(2));
  Matrix3d m;
  m << cp*cy,  -cr*sy + sr*sp*cy, sr*sy + cr*sp*cy, 
       cp*sy,  cr*cy + sr*sp*sy,  -sr*cy + cr*sp*sy, 
       -sp,    sr*cp,             cr*cp;
  return m;
}

/// @brief Convert a quaternion to ZYX roll-pitch-yaw Euler angles.
Vector3d quaternion2euler(Quaterniond q)
{
  return mat2euler(quaternion2mat(q));
}

/// @brief Extract yaw from a quaternion using the same ZYX convention as the EKF.
double quaternion2yaw(Quaterniond q)
{
  double a = q.w(), b = q.x(), c = q.y(), d = q.z();
  double y = atan2(2*(b*c+a*d), (a*a + b*b - c*c - d*d));
  return y;
}

//Euler motion equation
//ZYX Euler angles velocity to body frame wx wy wz  w_phi,  w_theta,  w_psi     q: roll pitch yaw  (phi theta psi)
/// @brief Build the mapping from ZYX Euler-angle rates to body angular velocity.
Matrix3d w_Euler2Body(Vector3d q)
{
    // q is [roll, pitch, yaw]. G maps Euler rates to body angular velocity:
    //   omega_body = G(q) * [roll_dot, pitch_dot, yaw_dot].
    double cr = cos( q(0));
    double sr = sin( q(0));
    double cp = cos( q(1));
    double sp = sin( q(1));

    Matrix3d G;
    G << 1.,    0.,       -sp,
         0.,    cr,   cp * sr,
         0.,   -sr,   cp * cr;
    
    // Vector3d w(0,0,0);
    // w = G * qdot;
    return G;
} 
/// @brief Build the inverse mapping from body angular velocity to ZYX Euler-angle rates.
Matrix3d w_Body2Euler(Vector3d q)
{
    // Inverse of w_Euler2Body(). cp is guarded by a small epsilon to avoid a
    // hard division-by-zero at pitch = +/-90 deg. This does not remove the
    // Euler singularity; it only prevents numeric explosion in utility usage.
    double cr = cos( q(0));
    double sr = sin( q(0));
    double cp = cos( q(1)) + 0.00000001;
    double sp = sin( q(1));

    Matrix3d G_inv;
    G_inv << 1., sp*sr/cp, cr*sp/cp,
             0.,       cr,      -sr,
             0.,    sr/cp,    cr/cp;

    // Vector3d qdot(0,0,0);
    // qdot = G_inv * w;
    return G_inv;
}
// Vector3d Body2Euler(Vector3d q, Vector3d w)
// {
//     Vector3d qdot(0,0,0);
//     Matrix3d G_inv;
//     G_inv << cos(q.y),   0.,    sin(q.y),
//              (sin(q.x)*sin(q.y))/(cos(q.x)+0.00000001), 1., -(cos(q.y)*sin(q.x))/(cos(q.x)+0.00000001),
//              -sin(q.y)/(cos(q.x)+0.00000001), 0., cos(q.y)/(cos(q.x)+0.00000001);
//     qdot = G_inv * w;
//     return qdot;
// }
