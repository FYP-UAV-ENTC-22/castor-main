// Frame conversions between PX4 and ROS (REP-103), kept in one place.
//
//   PX4  world: NED (x north, y east, z down)    body: FRD (x forward, y right, z down)
//   ROS  world: ENU (x east, y north, z up)      body: FLU (x forward, y left, z up)
//
// Quaternions are Hamilton, stored (w, x, y, z), and rotate body -> world, which
// is PX4's VehicleOdometry.q convention and ROS's.

#pragma once

#include <array>
#include <cmath>

namespace castor_vehicle_interface::frames {

using Vec3 = std::array<double, 3>;
using Quat = std::array<double, 4>;  // w, x, y, z

// World position / velocity: NED <-> ENU is the same swap-and-negate both ways.
inline Vec3 ned_to_enu(const Vec3 &v) { return {v[1], v[0], -v[2]}; }
inline Vec3 enu_to_ned(const Vec3 &v) { return {v[1], v[0], -v[2]}; }

// Body vectors (e.g. angular velocity): FRD <-> FLU flips y and z.
inline Vec3 frd_to_flu(const Vec3 &v) { return {v[0], -v[1], -v[2]}; }

inline Quat quat_mul(const Quat &a, const Quat &b) {
  return {
    a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3],
    a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2],
    a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1],
    a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0],
  };
}

inline Quat quat_conj(const Quat &q) { return {q[0], -q[1], -q[2], -q[3]}; }

// Rotate v by q (body -> world when q is a body->world attitude).
inline Vec3 rotate(const Quat &q, const Vec3 &v) {
  const Quat p{0.0, v[0], v[1], v[2]};
  const Quat r = quat_mul(quat_mul(q, p), quat_conj(q));
  return {r[1], r[2], r[3]};
}

// q_enu_flu = q(NED->ENU) * q_ned_frd * q(FLU->FRD).
// NED->ENU is a pi rotation about (1, 1, 0)/sqrt(2); FLU->FRD is pi about x.
inline Quat px4_to_ros_attitude(const Quat &q_ned_frd) {
  const double s = std::sqrt(0.5);
  const Quat ned_to_enu{0.0, s, s, 0.0};
  const Quat flu_to_frd{0.0, 1.0, 0.0, 0.0};
  Quat q = quat_mul(quat_mul(ned_to_enu, q_ned_frd), flu_to_frd);
  // Keep w >= 0 so equal attitudes compare equal.
  if (q[0] < 0.0) {
    q = {-q[0], -q[1], -q[2], -q[3]};
  }
  return q;
}

inline double wrap_pi(double a) {
  a = std::fmod(a + M_PI, 2.0 * M_PI);
  if (a < 0.0) {
    a += 2.0 * M_PI;
  }
  return a - M_PI;
}

// Heading: ENU yaw is measured from east counter-clockwise, NED yaw from north clockwise.
inline double enu_to_ned_yaw(double yaw_enu) { return wrap_pi(M_PI / 2.0 - yaw_enu); }

// Yaw rate about the world z axis flips sign because ENU z is up and NED z is down.
inline double enu_to_ned_yaw_rate(double rate_enu) { return -rate_enu; }

inline bool all_finite(const Vec3 &v) {
  return std::isfinite(v[0]) && std::isfinite(v[1]) && std::isfinite(v[2]);
}

}  // namespace castor_vehicle_interface::frames
