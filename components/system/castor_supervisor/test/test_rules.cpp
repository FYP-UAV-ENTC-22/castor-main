#include <gtest/gtest.h>

#include <string>

#include "castor_supervisor/rules.hpp"

using castor_supervisor::emergency;
using castor_supervisor::emergency_clearable;
using castor_supervisor::update_allowed;
using castor_supervisor::VehicleView;

namespace {
VehicleView disarmed() { return {true, true, false, false, true}; }
VehicleView armed() { return {true, true, true, false, true}; }
VehicleView stale() { return {false, false, false, false, false}; }
}  // namespace

TEST(UpdateGate, OnlyIdleStatesAndDisarmed) {
  EXPECT_TRUE(update_allowed("IDLE", true, disarmed()));
  EXPECT_TRUE(update_allowed("BOOT", true, disarmed()));
  EXPECT_FALSE(update_allowed("IDLE", true, armed()));
  EXPECT_FALSE(update_allowed("MISSION", true, disarmed()));
  EXPECT_FALSE(update_allowed("READY", true, disarmed()));
  EXPECT_FALSE(update_allowed("EMERGENCY", true, disarmed()));
}

TEST(UpdateGate, UnknownVehicleStateDenies) {
  // FC expected but silent: the vehicle might be flying with a dead link.
  EXPECT_FALSE(update_allowed("IDLE", true, stale()));
}

TEST(UpdateGate, BenchPiWithoutFc) {
  EXPECT_TRUE(update_allowed("IDLE", false, stale()));
  EXPECT_FALSE(update_allowed("MISSION", false, stale()));
}

TEST(Emergency, FollowsFailsafe) {
  std::string why;
  VehicleView v = armed();
  v.failsafe = true;
  EXPECT_TRUE(emergency(true, true, v, &why));
  EXPECT_NE(why.find("failsafe"), std::string::npos);
}

TEST(Emergency, LinkLossOnlyCountsWhenArmedState) {
  std::string why;
  EXPECT_TRUE(emergency(true, true, stale(), &why));
  EXPECT_NE(why.find("link lost"), std::string::npos);
  EXPECT_FALSE(emergency(true, false, stale(), &why));
  EXPECT_FALSE(emergency(false, true, stale(), &why));
  EXPECT_FALSE(emergency(true, true, armed(), &why));
}

TEST(Emergency, ClearOnlyWhenSafe) {
  EXPECT_TRUE(emergency_clearable(true, disarmed()));
  EXPECT_FALSE(emergency_clearable(true, armed()));
  EXPECT_FALSE(emergency_clearable(true, stale()));
  VehicleView v = disarmed();
  v.failsafe = true;
  EXPECT_FALSE(emergency_clearable(true, v));
  EXPECT_TRUE(emergency_clearable(false, stale()));
}
