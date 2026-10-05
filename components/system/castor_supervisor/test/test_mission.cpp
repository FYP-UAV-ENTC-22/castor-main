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

TEST(Mission, GoalWaitsForTheTeamAndAModel) {
  Inputs in = flying(cfg.raptor_nav_state);
  in.has_goal = in.new_goal = true;
  in.planning_ready = true;
  auto out = step(HOVER, in, cfg);
  EXPECT_EQ(out.state, HOVER);
  EXPECT_FALSE(out.planning_enabled);

  in.team_ready = true;
  in.planning_ready = false;
  EXPECT_EQ(step(HOVER, in, cfg).state, HOVER);

  in.planning_ready = true;
  out = step(HOVER, in, cfg);
  EXPECT_EQ(out.state, MARL);
  EXPECT_TRUE(out.planning_enabled);
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
  EXPECT_TRUE(out.planning_enabled);

  in.goal_reached = false;
  in.new_goal = true;
  out = step(HOLD, in, cfg);
  EXPECT_EQ(out.state, MARL);
  EXPECT_TRUE(out.planning_enabled);
}

TEST(Mission, LandFromAnyAirborneState) {
  for (const auto &s : {TAKING_OFF, HANDOVER, HOVER, MARL, HOLD, FC_OVERRIDE}) {
    Inputs in = flying(cfg.raptor_nav_state);
    in.land_cmd = true;
    const auto out = step(s, in, cfg);
    EXPECT_EQ(out.state, LANDING) << s;
    EXPECT_EQ(out.command, Command::LAND) << s;
    EXPECT_FALSE(out.planning_enabled) << s;
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
  EXPECT_FALSE(out.planning_enabled);

  in = flying(cfg.raptor_nav_state);
  in.failsafe = true;
  out = step(HOLD, in, cfg);
  EXPECT_EQ(out.state, FC_OVERRIDE);
  EXPECT_FALSE(out.planning_enabled);
}

TEST(Mission, LosingTheVehicleStopsEverything) {
  Inputs in = flying(cfg.raptor_nav_state);
  in.vehicle_ok = false;
  const auto out = step(MARL, in, cfg);
  EXPECT_EQ(out.state, WAIT_VEHICLE);
  EXPECT_FALSE(out.planning_enabled);
}

TEST(Mission, DisarmInFlightGoesIdle) {
  Inputs in = flying(cfg.raptor_nav_state);
  in.armed = false;
  EXPECT_EQ(step(HOVER, in, cfg).state, IDLE);
}
