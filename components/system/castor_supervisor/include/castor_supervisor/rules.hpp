// Pure decision rules for the supervisor, separate from ROS and YASMIN so they
// can be unit-tested.

#pragma once

#include <string>

namespace castor_supervisor {

struct VehicleView {
  bool fresh{false};          // a VehicleState arrived within vehicle_timeout_s
  bool fc_connected{false};   // ...and it says the FC link is up
  bool armed{false};
  bool failsafe{false};
  bool preflight_ok{false};
};

// States in which a stack restart (image update) is harmless.
inline bool state_allows_update(const std::string &state) {
  return state == "BOOT" || state == "WAIT_COMPONENTS" || state == "IDLE";
}

// The update gate: only when idle and the vehicle is known to be disarmed. With
// fc.enabled false (a bench Pi with no flight controller) there is nothing to
// fly, so only the state matters.
inline bool update_allowed(const std::string &state, bool fc_enabled, const VehicleView &v) {
  if (!state_allows_update(state)) {
    return false;
  }
  if (!fc_enabled) {
    return true;
  }
  return v.fresh && v.fc_connected && !v.armed;
}

// The FC acts first on an emergency (failsafe); the supervisor follows it.
// Losing the FC link while armed is treated the same way.
inline bool emergency(bool fc_enabled, bool in_armed_state, const VehicleView &v, std::string *reason) {
  if (v.fresh && v.fc_connected && v.failsafe) {
    *reason = "flight controller reports failsafe";
    return true;
  }
  if (fc_enabled && in_armed_state && !(v.fresh && v.fc_connected)) {
    *reason = "flight controller link lost while armed";
    return true;
  }
  return false;
}

// A latched emergency may only be cleared once the vehicle is safe again.
inline bool emergency_clearable(bool fc_enabled, const VehicleView &v) {
  if (!fc_enabled) {
    return true;
  }
  return v.fresh && v.fc_connected && !v.armed && !v.failsafe;
}

}  // namespace castor_supervisor
