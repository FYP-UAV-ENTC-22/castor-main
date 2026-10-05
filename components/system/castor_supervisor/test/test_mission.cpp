#include <gtest/gtest.h>

#include "castor_supervisor/mission.hpp"

using namespace castor_supervisor::mission;

namespace {

Inputs ok() {
  Inputs in;
  in.vehicle_ok = true;
  in.preflight_ok = true;
  return in;
}

Inputs flying(int nav_state) {
  Inputs in = ok();
  in.armed = true;
  in.landed = false;
  in.nav_state = nav_state;
  in.altitude = 2.0;
  return in;
}

const Config cfg;

}  // namespace

TEST(Mission, WaitsForTheVehicle) {
  EXPECT_EQ(step(WAIT_VEHICLE, Inputs{}, cfg).state, WAIT_VEHICLE);
  EXPECT_EQ(step(WAIT_VEHICLE, ok(), cfg).state, IDLE);
}

TEST(Mission, TakeoffSwitchesToTakeoffModeBeforeArming) {
  // PX4 boots in Position mode; without sticks it fails preflight there, which must not block the request.
  Inputs in = ok();
  in.preflight_ok = false;
  in.takeoff_cmd = true;
  auto out = step(IDLE, in, cfg);
  EXPECT_EQ(out.state, ARMING);
  EXPECT_EQ(out.command, Command::TAKEOFF);

  in.takeoff_cmd = false;
  out = step(ARMING, in, cfg);  // still in Position: ask for Takeoff again, never arm here
  EXPECT_EQ(out.command, Command::TAKEOFF);

  in.nav_state = NAV_AUTO_TAKEOFF;  // in Takeoff, but its checks have not passed yet
  out = step(ARMING, in, cfg);
  EXPECT_EQ(out.state, ARMING);
  EXPECT_EQ(out.command, Command::NONE);

  in.preflight_ok = true;
  out = step(ARMING, in, cfg);
  EXPECT_EQ(out.command, Command::ARM);

  in.time_in_state = cfg.arming_timeout_s + 1.0;
  out = step(ARMING, in, cfg);
  EXPECT_EQ(out.state, IDLE);
  EXPECT_NE(out.note.find("refused to arm"), std::string::npos);
}

TEST(Mission, TakeoffRefusedWhenArmedOrAirborne) {
  Inputs in = ok();
  in.takeoff_cmd = true;
  in.landed = false;
  EXPECT_EQ(step(IDLE, in, cfg).state, IDLE);
}

TEST(Mission, ArmedStartsTakeoffThenHandsOverAtHeight) {
  Inputs in = ok();
  in.armed = true;
  in.nav_state = NAV_AUTO_TAKEOFF;
  auto out = step(ARMING, in, cfg);
  EXPECT_EQ(out.state, TAKING_OFF);
  EXPECT_EQ(out.command, Command::NONE);  // armed in Takeoff, PX4 is already climbing

  in = flying(NAV_AUTO_TAKEOFF);
  in.altitude = 1.0;
  out = step(TAKING_OFF, in, cfg);
  EXPECT_EQ(out.state, TAKING_OFF);
  EXPECT_EQ(out.command, Command::NONE);  // PX4 is already taking off

  in = flying(NAV_AUTO_LOITER);
  out = step(TAKING_OFF, in, cfg);
  EXPECT_EQ(out.state, HANDOVER);
  EXPECT_EQ(out.command, Command::RAPTOR);

  in.nav_state = cfg.raptor_nav_state;
  EXPECT_EQ(step(HANDOVER, in, cfg).state, HOVER);
}

TEST(Mission, LiftStartsOnceTheWholeTeamIsInRaptor) {
  Inputs in = flying(cfg.raptor_nav_state);
  in.planning_alive = true;
  auto out = step(HOVER, in, cfg);
  EXPECT_EQ(out.state, HOVER);
  EXPECT_EQ(out.planning, Planning::OFF);  // RAPTOR holds by itself

  in.team_in_raptor = true;
  in.planning_alive = false;
  EXPECT_EQ(step(HOVER, in, cfg).state, HOVER);  // nobody to fly the lift

  in.planning_alive = true;
  out = step(HOVER, in, cfg);
  EXPECT_EQ(out.state, LIFT);
  EXPECT_EQ(out.planning, Planning::LIFT);
}

TEST(Mission, LiftEndsWithThePayloadUpOrGivesUpHolding) {
  Inputs in = flying(cfg.raptor_nav_state);
  in.planning_alive = true;
  auto out = step(LIFT, in, cfg);
  EXPECT_EQ(out.state, LIFT);
  EXPECT_EQ(out.planning, Planning::LIFT);

  in.lift_done = true;
  out = step(LIFT, in, cfg);
  EXPECT_EQ(out.state, LIFTED);
  EXPECT_EQ(out.planning, Planning::STEADY);

  in.lift_done = false;
  in.lift_failed = true;
  EXPECT_EQ(step(LIFT, in, cfg).state, LIFTED);
  in.lift_failed = false;
  in.time_in_state = cfg.lift_timeout_s + 1.0;
  EXPECT_EQ(step(LIFT, in, cfg).state, LIFTED);
}

TEST(Mission, HandoverWaitsToSettleThenNeedsTheCheck) {
  Inputs in = flying(cfg.raptor_nav_state);
  in.planning_alive = true;
  in.handover_ok = true;
  in.time_in_state = cfg.settle_s - 0.5;
  auto out = step(LIFTED, in, cfg);
  EXPECT_EQ(out.state, LIFTED);
  EXPECT_EQ(out.planning, Planning::STEADY);

  in.time_in_state = cfg.settle_s + 0.5;
  in.handover_ok = false;
  EXPECT_EQ(step(LIFTED, in, cfg).state, LIFTED);  // holds the formation, never hands a slack rig to the policy

  in.handover_ok = true;
  out = step(LIFTED, in, cfg);
  EXPECT_EQ(out.state, READY);
  EXPECT_EQ(out.planning, Planning::STEADY);
}

TEST(Mission, GoalWaitsForTheTeamAndAModel) {
  Inputs in = flying(cfg.raptor_nav_state);
  in.has_goal = in.new_goal = true;
  in.planning_ready = true;
  auto out = step(READY, in, cfg);
  EXPECT_EQ(out.state, READY);
  EXPECT_EQ(out.planning, Planning::STEADY);

  in.team_ready = true;
  in.planning_ready = false;
  EXPECT_EQ(step(READY, in, cfg).state, READY);

  in.planning_ready = true;
  out = step(READY, in, cfg);
  EXPECT_EQ(out.state, MARL);
  EXPECT_EQ(out.planning, Planning::POLICY);
}

TEST(Mission, GoalReachedHoldsAndKeepsThePolicyRunning) {
  Inputs in = flying(cfg.raptor_nav_state);
  in.has_goal = true;
  in.goal_reached = true;
  in.time_in_state = 0.5;  // status could be about the previous goal
  EXPECT_EQ(step(MARL, in, cfg).state, MARL);
  in.time_in_state = 2.0;
  auto out = step(MARL, in, cfg);
  EXPECT_EQ(out.state, HOLD);
  EXPECT_EQ(out.planning, Planning::POLICY);

  in.goal_reached = false;
  in.new_goal = true;
  out = step(HOLD, in, cfg);
  EXPECT_EQ(out.state, MARL);
  EXPECT_EQ(out.planning, Planning::POLICY);
}

TEST(Mission, LandFromAnyAirborneState) {
  for (const auto &s : {TAKING_OFF, HANDOVER, HOVER, LIFT, LIFTED, READY, MARL, HOLD, FC_OVERRIDE}) {
    Inputs in = flying(cfg.raptor_nav_state);
    in.land_cmd = true;
    const auto out = step(s, in, cfg);
    EXPECT_EQ(out.state, LANDING) << s;
    EXPECT_EQ(out.command, Command::LAND) << s;
    EXPECT_EQ(out.planning, Planning::OFF) << s;
  }
  Inputs in = ok();
  in.armed = false;
  EXPECT_EQ(step(LANDING, in, cfg).state, IDLE);
}

TEST(Mission, PX4LeavingRaptorOrFailsafeStopsPlanning) {
  Inputs in = flying(NAV_AUTO_LOITER);
  in.has_goal = true;
  auto out = step(MARL, in, cfg);
  EXPECT_EQ(out.state, FC_OVERRIDE);
  EXPECT_EQ(out.planning, Planning::OFF);

  in = flying(NAV_AUTO_LOITER);
  EXPECT_EQ(step(LIFT, in, cfg).state, FC_OVERRIDE);

  in = flying(cfg.raptor_nav_state);
  in.failsafe = true;
  out = step(HOLD, in, cfg);
  EXPECT_EQ(out.state, FC_OVERRIDE);
  EXPECT_EQ(out.planning, Planning::OFF);
}

TEST(Mission, LosingTheVehicleStopsEverything) {
  Inputs in = flying(cfg.raptor_nav_state);
  in.vehicle_ok = false;
  const auto out = step(MARL, in, cfg);
  EXPECT_EQ(out.state, WAIT_VEHICLE);
  EXPECT_EQ(out.planning, Planning::OFF);
}

TEST(Mission, DisarmInFlightGoesIdle) {
  Inputs in = flying(cfg.raptor_nav_state);
  in.armed = false;
  EXPECT_EQ(step(HOVER, in, cfg).state, IDLE);
}
