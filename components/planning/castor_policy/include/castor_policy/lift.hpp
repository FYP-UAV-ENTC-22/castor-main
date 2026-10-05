// Getting the payload into the state the policy was trained from, free of ROS so it can be unit-tested:
// the lift (every drone climbs straight up at the same speed until the payload hangs at the lift height) and the
// hand-over check (this drone's cable taut, the payload clear of the ground). The team does the same thing at the
// same time because the system layer's mission node starts them together; the checks are per drone, and the
// mission node waits for every teammate to pass before the policy flies.
//
// Same conventions as flycrane.hpp: world frame ENU, rotations body -> world, positions of the body origin.

#pragma once

#include <algorithm>
#include <cmath>
#include <string>

#include "castor_policy/flycrane.hpp"

namespace castor_policy {

// Distance from this drone's cable tie point to its anchor on the payload.
inline double cable_span(const Vec3 &drone_position, const Quat &drone_orientation, const Vec3 &mount_local,
                         const Vec3 &payload_position, const Quat &payload_orientation, const Vec3 &anchor_local) {
  // qualified: with std::array arguments, argument-dependent lookup would find std::apply
  const Vec3 m = castor_policy::apply(rotation(drone_orientation), mount_local);
  const Vec3 a = castor_policy::apply(rotation(payload_orientation), anchor_local);
  return norm({drone_position[0] + m[0] - payload_position[0] - a[0],
               drone_position[1] + m[1] - payload_position[1] - a[1],
               drone_position[2] + m[2] - payload_position[2] - a[2]});
}

struct LiftConfig {
  double target_height{1.0};  // payload centre above the ground [m]
  double speed{0.15};         // climb speed [m/s]
  double accel{0.1};          // ramp in and out [m/s^2]
  double max_lead{0.3};       // the setpoint waits when it is this far above a drone that cannot keep up [m]
  double max_climb{3.5};      // give up after climbing this far without the payload reaching the height [m]
};

struct Lift {
  double start_z{0.0};
  double speed{0.0};  // current climb speed
  bool reached{false};  // the payload got to the height; slowing down
  bool done{false};     // stopped with the payload at the height
  bool failed{false};   // climbed max_climb without getting there
  std::string why;

  void reset(double setpoint_z) {
    *this = Lift{};
    start_z = setpoint_z;
  }
};

// One step: moves the setpoint up (position and velocity feedforward) and updates the lift's state.
inline void lift_step(Lift &lift, Vec3 &sp_position, Vec3 &sp_velocity, double drone_z, double payload_z,
                      double dt, const LiftConfig &cfg) {
  sp_velocity = {0.0, 0.0, 0.0};
  if (lift.done || lift.failed) return;
  if (!lift.reached && payload_z >= cfg.target_height) lift.reached = true;
  if (!lift.reached && sp_position[2] - lift.start_z > cfg.max_climb) {
    lift.failed = true;
    lift.speed = 0.0;
    lift.why = "climbed " + std::to_string(cfg.max_climb) + " m and the payload is still below the lift height";
    return;
  }
  const double wanted = lift.reached ? 0.0 : cfg.speed;
  lift.speed = wanted > lift.speed ? std::min(wanted, lift.speed + cfg.accel * dt)
                                   : std::max(wanted, lift.speed - cfg.accel * dt);
  if (sp_position[2] - drone_z > cfg.max_lead && !lift.reached) {
    return;  // anti-windup: hold the setpoint until the drone catches up
  }
  sp_position[2] += lift.speed * dt;
  sp_velocity[2] = lift.speed;
  if (lift.reached && lift.speed <= 0.0) lift.done = true;
}

struct HandoverConfig {
  double cable_length{2.0};
  double taut_tolerance{0.05};  // span at least cable_length - this
  double payload_height{0.03};
  double min_clearance{0.2};    // payload bottom above the ground [m]
};

struct Handover {
  bool ok{false};
  std::string reason;
};

inline Handover handover_check(double span, double payload_z, const HandoverConfig &cfg) {
  if (span < cfg.cable_length - cfg.taut_tolerance) {
    return {false, "cable slack: span " + std::to_string(span) + " m of " + std::to_string(cfg.cable_length) + " m"};
  }
  const double clearance = payload_z - cfg.payload_height / 2.0;
  if (clearance < cfg.min_clearance) {
    return {false, "payload " + std::to_string(clearance) + " m above the ground, needs " +
                       std::to_string(cfg.min_clearance) + " m"};
  }
  return {true, ""};
}

}  // namespace castor_policy
