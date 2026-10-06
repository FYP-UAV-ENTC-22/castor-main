// What flying a trained policy on the aircraft needs on top of the training env's processing (flycrane.hpp),
// free of ROS so it can be unit-tested. The values come from the model's manifest (models/README.md: flight and
// policy.point_local); they were measured in Mugesram's PX4 harness (castor_marl_v1, removed after 241b700):
//
//   point     the policy calls a particular point on the drone its position (the Falcon task: 0.03 m above the
//             cable tie point). It gets that point's position and velocity, and its setpoint is for that point;
//             RAPTOR flies the body origin, so the setpoint is moved back by the same offset.
//   filter    a first-order low-pass on the velocity feedforward: the raw one (increment / dt) rocks the S500.
//   goal box  goals outside the positions seen in training are clamped into them.

#pragma once

#include <algorithm>
#include <cmath>

#include "castor_policy/flycrane.hpp"

namespace castor_policy {

inline Vec3 cross(const Vec3 &a, const Vec3 &b) {
  return {a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]};
}

// The drone as the policy sees it: position and velocity of the point `point_local` (body frame) instead of the
// body origin. Orientation and angular velocity (world frame) are the body's.
inline BodyState policy_point(const BodyState &body, const Vec3 &point_local) {
  // qualified: with std::array arguments, argument-dependent lookup would find std::apply
  const Vec3 arm = castor_policy::apply(rotation(body.orientation), point_local);
  const Vec3 spin = cross(body.angular_velocity, arm);
  BodyState p = body;
  for (int i = 0; i < 3; ++i) {
    p.position[i] = body.position[i] + arm[i];
    p.linear_velocity[i] = body.linear_velocity[i] + spin[i];
  }
  return p;
}

// The body-origin setpoint for a setpoint of the policy's point, at the setpoint's yaw: RAPTOR holds the drone
// near level at that heading, so the point sits at Rz(yaw) * point_local from the body origin.
inline Vec3 body_setpoint(const Vec3 &point_setpoint, double yaw, const Vec3 &point_local) {
  const double c = std::cos(yaw), s = std::sin(yaw);
  return {point_setpoint[0] - (c * point_local[0] - s * point_local[1]),
          point_setpoint[1] - (s * point_local[0] + c * point_local[1]), point_setpoint[2] - point_local[2]};
}

// First-order low-pass with time constant tau; tau <= 0 passes x through.
inline Vec3 low_pass(const Vec3 &previous, const Vec3 &x, double dt, double tau) {
  if (tau <= 0.0) return x;
  const double a = dt / (tau + dt);
  return {previous[0] + a * (x[0] - previous[0]), previous[1] + a * (x[1] - previous[1]),
          previous[2] + a * (x[2] - previous[2])};
}

inline Vec3 clamp_box(const Vec3 &v, const Vec3 &lo, const Vec3 &hi) {
  return {std::clamp(v[0], lo[0], hi[0]), std::clamp(v[1], lo[1], hi[1]), std::clamp(v[2], lo[2], hi[2])};
}

}  // namespace castor_policy
