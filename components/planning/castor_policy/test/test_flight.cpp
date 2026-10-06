#include <gtest/gtest.h>

#include <cmath>

#include "castor_policy/flight.hpp"

using namespace castor_policy;

namespace {
Quat yaw_quat(double yaw) { return {std::cos(yaw / 2), 0.0, 0.0, std::sin(yaw / 2)}; }
}  // namespace

TEST(Flight, PolicyPointMovesPositionAndVelocity) {
  BodyState body;
  body.position = {1, 2, 3};
  body.orientation = yaw_quat(M_PI / 2);
  body.linear_velocity = {0.1, 0, 0};
  body.angular_velocity = {0, 0, 1.0};  // world frame, 1 rad/s about z
  const BodyState p = policy_point(body, {0.2, 0, -0.15});
  // the point sits 0.2 m along body x = world y, 0.15 m down
  EXPECT_NEAR(p.position[0], 1.0, 1e-12);
  EXPECT_NEAR(p.position[1], 2.2, 1e-12);
  EXPECT_NEAR(p.position[2], 2.85, 1e-12);
  // v + w x r = (0.1, 0, 0) + (0, 0, 1) x (0, 0.2, -0.15) = (0.1 - 0.2, 0, 0)
  EXPECT_NEAR(p.linear_velocity[0], -0.1, 1e-12);
  EXPECT_NEAR(p.linear_velocity[1], 0.0, 1e-12);
  EXPECT_EQ(p.angular_velocity, body.angular_velocity);
}

TEST(Flight, BodySetpointUndoesThePointAtTheSetpointYaw) {
  const Vec3 point_local{0.2, 0, -0.1575};
  BodyState body;
  body.position = {0.5, -0.3, 2.9};
  body.orientation = yaw_quat(2.0943951);  // 120 deg, a drone facing outward
  const BodyState p = policy_point(body, point_local);
  const Vec3 back = body_setpoint(p.position, 2.0943951, point_local);
  for (int i = 0; i < 3; ++i) EXPECT_NEAR(back[i], body.position[i], 1e-9) << i;
}

TEST(Flight, LowPassSmoothsAndZeroTauPassesThrough) {
  const Vec3 x{1, 0, 0};
  const Vec3 y = low_pass({0, 0, 0}, x, 0.02, 0.1);
  EXPECT_NEAR(y[0], 0.02 / 0.12, 1e-12);
  EXPECT_EQ(low_pass({0, 0, 0}, x, 0.02, 0.0), x);
}

TEST(Flight, GoalsAreClampedIntoTheTrainingBox) {
  const Vec3 g = clamp_box({1.7, -0.2, 0.1}, {-1, -1, 0.5}, {1, 1, 1.5});
  EXPECT_EQ(g, (Vec3{1.0, -0.2, 0.5}));
}

TEST(Flight, MaxSpeedCapsTheStepKeepingItsDirection) {
  Setpoint sp;
  advance(sp, {3.0f, 4.0f, 0.0f}, {0, 0, 0}, 0.05, 0.02, 1.5, 1.0);  // asks for 12.5 m/s
  EXPECT_NEAR(norm(sp.velocity), 1.0, 1e-9);
  EXPECT_NEAR(sp.position[0] / sp.position[1], 0.75, 1e-9);
  Setpoint free;
  advance(free, {3.0f, 4.0f, 0.0f}, {0, 0, 0}, 0.05, 0.02, 1.5);
  EXPECT_NEAR(norm(free.velocity), 12.5, 1e-9);
}
