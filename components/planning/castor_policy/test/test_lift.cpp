#include <gtest/gtest.h>

#include <cmath>

#include "castor_policy/lift.hpp"

using namespace castor_policy;

namespace {
Quat yaw_quat(double yaw) { return {std::cos(yaw / 2), 0.0, 0.0, std::sin(yaw / 2)}; }
}  // namespace

TEST(Lift, CableSpanUsesBothBodies) {
  // Drone yawed 90 deg with its mount 0.2 m ahead and 0.1 m down; payload yawed 90 deg with its anchor 0.5 m ahead.
  const double span = cable_span({0, 0, 2}, yaw_quat(M_PI / 2), {0.2, 0, -0.1}, {0, 0, 0}, yaw_quat(M_PI / 2),
                                 {0.5, 0, 0});
  // mount at (0, 0.2, 1.9), anchor at (0, 0.5, 0)
  EXPECT_NEAR(span, std::hypot(0.3, 1.9), 1e-9);
}

TEST(Lift, RampsUpClimbsThenStopsAtTheHeight) {
  LiftConfig cfg;
  Lift lift;
  Vec3 sp{0, 0, 1.5}, vel{};
  lift.reset(sp[2]);
  const double dt = 0.02;
  lift_step(lift, sp, vel, sp[2], 0.0, dt, cfg);
  EXPECT_NEAR(vel[2], cfg.accel * dt, 1e-12);  // ramping, not a step to full speed
  for (int i = 0; i < 200; ++i) lift_step(lift, sp, vel, sp[2], 0.0, dt, cfg);
  EXPECT_NEAR(vel[2], cfg.speed, 1e-12);
  EXPECT_FALSE(lift.reached);

  // the payload gets there: slow down to a stop, then report done
  int steps = 0;
  while (!lift.done && steps++ < 1000) lift_step(lift, sp, vel, sp[2], 1.0, dt, cfg);
  EXPECT_TRUE(lift.done);
  EXPECT_NEAR(vel[2], 0.0, 1e-12);
  EXPECT_LE(steps, static_cast<int>(cfg.speed / cfg.accel / dt) + 2);
  const double z = sp[2];
  lift_step(lift, sp, vel, sp[2], 1.0, dt, cfg);
  EXPECT_EQ(sp[2], z);  // done holds still
}

TEST(Lift, WaitsForADroneThatFallsBehind) {
  LiftConfig cfg;
  Lift lift;
  Vec3 sp{0, 0, 2.0}, vel{};
  lift.reset(sp[2]);
  lift.speed = cfg.speed;
  lift_step(lift, sp, vel, 2.0 - cfg.max_lead - 0.01, 0.0, 0.02, cfg);
  EXPECT_EQ(sp[2], 2.0);
  EXPECT_EQ(vel[2], 0.0);
}

TEST(Lift, GivesUpAfterMaxClimb) {
  LiftConfig cfg;
  cfg.max_climb = 0.5;
  Lift lift;
  Vec3 sp{0, 0, 1.0}, vel{};
  lift.reset(sp[2]);
  for (int i = 0; i < 2000 && !lift.failed; ++i) lift_step(lift, sp, vel, sp[2], 0.0, 0.02, cfg);
  EXPECT_TRUE(lift.failed);
  EXPECT_FALSE(lift.done);
  EXPECT_LT(sp[2] - 1.0, cfg.max_climb + 0.01);
}

TEST(Lift, HandoverNeedsATautCableAndAnAirbornePayload) {
  HandoverConfig cfg;
  EXPECT_TRUE(handover_check(1.99, 1.0, cfg).ok);
  EXPECT_TRUE(handover_check(2.01, 1.0, cfg).ok);  // a distance joint stretches a little
  const auto slack = handover_check(1.9, 1.0, cfg);
  EXPECT_FALSE(slack.ok);
  EXPECT_NE(slack.reason.find("slack"), std::string::npos);
  const auto low = handover_check(2.0, 0.1, cfg);
  EXPECT_FALSE(low.ok);
  EXPECT_NE(low.reason.find("ground"), std::string::npos);
}
