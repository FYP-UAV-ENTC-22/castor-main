// The simple mission state machine (take off, lift the payload with the team, fly the MARL
// policy, hold, land), free of ROS so it can be unit-tested. The node in
// src/mission_node.cpp feeds it inputs at a fixed rate and carries out its outputs.
//
//   WAIT_VEHICLE -> IDLE -takeoff-> ARMING -> TAKING_OFF (PX4 Takeoff) -> HANDOVER (-> RAPTOR)
//     A take-off that arrives before the vehicle is connected is refused (with a note), never kept for later:
//     a stale command must not arm the drone minutes afterwards. The ground station waits for IDLE first.
//     ARMING switches PX4 to Takeoff first and arms there: PX4 boots in Position mode, which needs stick input,
//     so without RC it refuses to arm in it (and reports its preflight checks failing for it); armed in Takeoff
//     it climbs at once, to a height where the cables are still slack.
//   -> HOVER (RAPTOR holds, cables slack) -team in RAPTOR-> LIFT (planning climbs; cables go taut, payload lifts)
//     -> LIFTED (planning holds the setpoint) -settled + hand-over check-> READY
//     -> goal + team READY -> MARL -goal reached-> HOLD -new goal-> MARL
//   land: any airborne state -> LANDING (PX4 Land) -> disarm once PX4 reports landed -> IDLE
//   PX4 failsafe, or PX4 leaving RAPTOR while we fly it -> FC_OVERRIDE (planning off, PX4 in charge)
//
// The take-off, lift and hand-over check follow Mugesram's PX4 harness (castor_marl_v1, removed after 241b700), where
// one process flew all three drones: the policy only ever trained from taut cables and a payload in the air.
//
// Temporary: this is replaced by the real supervisor and mission trees later.

#pragma once

#include <string>

namespace castor_supervisor::mission {

inline const std::string WAIT_VEHICLE = "WAIT_VEHICLE", IDLE = "IDLE", ARMING = "ARMING",
                         TAKING_OFF = "TAKING_OFF", HANDOVER = "HANDOVER", HOVER = "HOVER", LIFT = "LIFT",
                         LIFTED = "LIFTED", READY = "READY", MARL = "MARL", HOLD = "HOLD", LANDING = "LANDING",
                         FC_OVERRIDE = "FC_OVERRIDE";

// PX4 vehicle_status nav_state values used here (VehicleStatus.msg).
constexpr int NAV_AUTO_LOITER = 4, NAV_AUTO_TAKEOFF = 17, NAV_AUTO_LAND = 18;

struct Config {
  double takeoff_height{2.0};   // m above home; below where the cables go taut
  double height_tolerance{0.2};
  int raptor_nav_state{23};     // NAVIGATION_STATE_EXTERNAL1
  double arming_timeout_s{40.0};  // PX4 switches to Takeoff and arms in a few seconds; SITL can be slower
  double takeoff_timeout_s{60.0};
  double handover_timeout_s{10.0};
  double lift_timeout_s{60.0};
  double settle_s{4.0};          // LIFTED: hold this long before the hand-over check
  double goal_reached_holdoff_s{1.5};  // planning status is 1 Hz: ignore goal_reached from before the new goal
};

struct Inputs {
  bool vehicle_ok{false};  // fresh vehicle state with the FC connected
  std::string vehicle_problem;  // why not, for the log: "no vehicle state", "PX4 link down", ...
  bool armed{false}, landed{true}, failsafe{false}, preflight_ok{false};
  int nav_state{0};
  double altitude{0.0};    // m above the vehicle's local origin (home)
  bool takeoff_cmd{false}, land_cmd{false};  // edges, consumed this step
  bool new_goal{false};    // a goal arrived this step
  bool has_goal{false};    // a goal is held (it arrived while READY, MARL or HOLD)
  bool team_in_raptor{false};  // every teammate is in HOVER or a later RAPTOR state
  bool team_ready{false};  // every teammate in READY, MARL or HOLD
  bool planning_alive{false};  // a fresh planning status
  bool planning_ready{false};  // model loaded with the right input width
  bool lift_done{false}, lift_failed{false};
  bool handover_ok{false};
  bool goal_reached{false};
  double time_in_state{0.0};  // s on the ROS clock: simulated time in stack_sim
};

enum class Command { NONE, ARM, DISARM, TAKEOFF, RAPTOR, LAND };
enum class Planning { OFF, STEADY, LIFT, POLICY };

struct Outputs {
  std::string state;
  Command command{Command::NONE};  // what PX4 should be told (the node rate-limits repeats)
  Planning planning{Planning::OFF};
  std::string note;                // why a transition happened or a request was refused, for the log
};

// RAPTOR flies the drone in these; PX4 leaving RAPTOR in them is an override.
inline bool in_raptor(const std::string &s) {
  return s == HOVER || s == LIFT || s == LIFTED || s == READY || s == MARL || s == HOLD;
}
// The payload is in the air and the policy may fly it; goals are taken here.
inline bool payload_ready(const std::string &s) { return s == READY || s == MARL || s == HOLD; }
inline bool airborne(const std::string &s) {
  return s == TAKING_OFF || s == HANDOVER || in_raptor(s) || s == FC_OVERRIDE;
}

inline Outputs step(const std::string &state, const Inputs &in, const Config &cfg) {
  Outputs out{state, Command::NONE, Planning::OFF, ""};
  const auto go = [&out](const std::string &s, const std::string &why) {
    out.state = s;
    out.note = why;
  };

  if (state != WAIT_VEHICLE && !in.vehicle_ok) {
    go(WAIT_VEHICLE, "lost the vehicle state; PX4 is on its own");
    return out;
  }

  if (state == WAIT_VEHICLE) {
    if (in.vehicle_ok) {
      go(in.armed ? FC_OVERRIDE : IDLE, in.armed ? "vehicle already armed" : "vehicle connected");
    } else if (in.takeoff_cmd) {
      out.note = "takeoff refused: waiting for the vehicle (" + in.vehicle_problem + "); send it again once IDLE";
    } else if (in.land_cmd) {
      out.note = "land ignored: waiting for the vehicle (" + in.vehicle_problem + ")";
    }
    return out;
  }

  if (state == IDLE) {
    if (in.takeoff_cmd) {
      if (in.armed || !in.landed) {
        out.note = "takeoff refused: vehicle is armed or not landed";
      } else {
        go(ARMING, "takeoff requested: PX4 Takeoff mode, then arm");
        out.command = Command::TAKEOFF;
      }
    } else if (in.land_cmd) {
      out.note = "land ignored: on the ground";
    }
    return out;
  }

  // Everything below can be armed.
  if (in.land_cmd) {
    if (state == ARMING) {
      go(IDLE, "land requested before take-off");
      out.command = Command::DISARM;
      return out;
    }
    if (airborne(state)) {
      go(LANDING, "land requested");
      out.command = Command::LAND;
      return out;
    }
  }
  if (state != ARMING && state != LANDING && !in.armed) {
    go(IDLE, "vehicle disarmed");
    return out;
  }
  if (airborne(state) && state != FC_OVERRIDE && in.failsafe) {
    go(FC_OVERRIDE, "PX4 failsafe");
    return out;
  }
  if (in_raptor(state) && in.nav_state != cfg.raptor_nav_state) {
    go(FC_OVERRIDE, "PX4 left RAPTOR");
    return out;
  }

  if (state == ARMING) {
    if (in.armed) {
      go(TAKING_OFF, "armed");
    } else if (in.time_in_state > cfg.arming_timeout_s) {
      go(IDLE, in.nav_state != NAV_AUTO_TAKEOFF ? "arming timed out: PX4 did not switch to Takeoff"
                                                : (in.preflight_ok ? "arming timed out: PX4 refused to arm"
                                                                   : "arming timed out: PX4 preflight checks fail"));
    } else if (in.nav_state != NAV_AUTO_TAKEOFF) {
      out.command = Command::TAKEOFF;
    } else if (in.preflight_ok) {
      out.command = Command::ARM;
    }
  } else if (state == TAKING_OFF) {
    if (in.altitude >= cfg.takeoff_height - cfg.height_tolerance && in.nav_state == NAV_AUTO_LOITER) {
      go(HANDOVER, "at take-off height");
      out.command = Command::RAPTOR;
    } else if (in.time_in_state > cfg.takeoff_timeout_s) {
      go(LANDING, "take-off timed out at " + std::to_string(in.altitude).substr(0, 4) + " m of " +
                    std::to_string(cfg.takeoff_height).substr(0, 4) + " m, PX4 nav_state " +
                    std::to_string(in.nav_state));
      out.command = Command::LAND;
    } else if (in.nav_state != NAV_AUTO_TAKEOFF && in.nav_state != NAV_AUTO_LOITER) {
      out.command = Command::TAKEOFF;
    }
  } else if (state == HANDOVER) {
    if (in.nav_state == cfg.raptor_nav_state) {
      go(HOVER, "RAPTOR in control");
    } else if (in.time_in_state > cfg.handover_timeout_s) {
      go(LANDING, "PX4 did not switch to RAPTOR");
      out.command = Command::LAND;
    } else {
      out.command = Command::RAPTOR;
    }
  } else if (state == HOVER) {
    if (in.team_in_raptor && in.planning_alive) {
      go(LIFT, "team in RAPTOR: lifting the payload");
      out.planning = Planning::LIFT;
    }
  } else if (state == LIFT) {
    out.planning = Planning::LIFT;
    if (in.lift_done) {
      go(LIFTED, "payload at the lift height");
      out.planning = Planning::STEADY;
    } else if (in.lift_failed || in.time_in_state > cfg.lift_timeout_s) {
      go(LIFTED, in.lift_failed ? "lift gave up; holding" : "lift timed out; holding");
      out.planning = Planning::STEADY;
    }
  } else if (state == LIFTED) {
    out.planning = Planning::STEADY;
    if (in.time_in_state > cfg.settle_s && in.handover_ok) {
      go(READY, "hand-over check passed");
    }
  } else if (payload_ready(state)) {
    if (state == READY) {
      out.planning = Planning::STEADY;
      if (in.has_goal && in.team_ready && in.planning_ready) {
        go(MARL, "goal and team ready");
        out.planning = Planning::POLICY;
      } else if (in.new_goal) {
        out.note = !in.team_ready ? "goal held: waiting for the team to pass the hand-over check"
                                  : "goal held: planning has no usable model";
      }
    } else {
      out.planning = Planning::POLICY;
      if (state == MARL && in.goal_reached && in.time_in_state > cfg.goal_reached_holdoff_s) {
        go(HOLD, "goal reached");
      } else if (state == HOLD && in.new_goal) {
        go(MARL, "new goal");
      } else if (state == MARL && in.new_goal) {
        out.note = "goal updated";
      }
    }
  } else if (state == LANDING) {
    if (!in.armed) {
      go(IDLE, "landed and disarmed");
    } else if (in.landed) {
      out.command = Command::DISARM;  // PX4's own auto-disarm is off: a drone hanging on a cable can look landed
    } else if (in.nav_state != NAV_AUTO_LAND) {
      out.command = Command::LAND;
    }
  }
  // FC_OVERRIDE: wait for PX4 to land and disarm, or for a land command (handled above).
  return out;
}

}  // namespace castor_supervisor::mission
