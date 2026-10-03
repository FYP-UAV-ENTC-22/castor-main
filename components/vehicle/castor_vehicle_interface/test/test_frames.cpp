#include <gtest/gtest.h>

#include <cmath>

#include "castor_vehicle_interface/frames.hpp"

namespace f = castor_vehicle_interface::frames;

namespace {

constexpr double kTol = 1e-9;

void expect_vec(const f::Vec3 &a, const f::Vec3 &b) {
  for (int i = 0; i < 3; ++i) {
    EXPECT_NEAR(a[i], b[i], kTol) << "component " << i;
  }
}

void expect_quat(const f::Quat &a, const f::Quat &b) {
  // q and -q are the same rotation.
  const double sign = (a[0] * b[0] + a[1] * b[1] + a[2] * b[2] + a[3] * b[3]) < 0 ? -1.0 : 1.0;
  for (int i = 0; i < 4; ++i) {
    EXPECT_NEAR(a[i], sign * b[i], 1e-9) << "component " << i;
  }
}

f::Quat yaw_quat(double yaw) { return {std::cos(yaw / 2), 0.0, 0.0, std::sin(yaw / 2)}; }

}  // namespace

TEST(Frames, PositionSwap) {
  // 1 m north, 2 m east, 3 m down -> 2 m east, 1 m north, 3 m below.
  expect_vec(f::ned_to_enu({1, 2, 3}), {2, 1, -3});
  expect_vec(f::enu_to_ned(f::ned_to_enu({0.3, -4.0, 7.5})), {0.3, -4.0, 7.5});
}

TEST(Frames, LevelFacingNorthIsEnuYawNinety) {
  // PX4 identity attitude = level, nose north. In ENU that is yaw +90 deg.
  expect_quat(f::px4_to_ros_attitude({1, 0, 0, 0}), yaw_quat(M_PI / 2));
}

TEST(Frames, LevelFacingEastIsEnuYawZero) {
  // NED yaw +90 deg (clockwise from north) = nose east = ENU yaw 0.
  expect_quat(f::px4_to_ros_attitude(yaw_quat(M_PI / 2)), {1, 0, 0, 0});
}

TEST(Frames, AttitudeRotatesBodyAxesCorrectly) {
  // Nose pitched up 30 deg while facing north: the FLU x axis should point
  // north and up in ENU.
  const double pitch = M_PI / 6;
  const f::Quat q_ned{std::cos(pitch / 2), 0.0, std::sin(pitch / 2), 0.0};  // +pitch about FRD y = nose up
  const f::Quat q_enu = f::px4_to_ros_attitude(q_ned);
  expect_vec(f::rotate(q_enu, {1, 0, 0}), {0.0, std::cos(pitch), std::sin(pitch)});
  // FLU z (up through the top of the airframe) tilts back toward south.
  expect_vec(f::rotate(q_enu, {0, 0, 1}), {0.0, -std::sin(pitch), std::cos(pitch)});
}

TEST(Frames, BodyRates) {
  // Roll right (FRD +x), pitch up (FRD +y), yaw clockwise seen from above (FRD +z).
  expect_vec(f::frd_to_flu({0.1, 0.2, 0.3}), {0.1, -0.2, -0.3});
}

TEST(Frames, WorldVelocityToBody) {
  // Facing east and flying north at 1 m/s: in FLU that is 1 m/s to the left.
  const f::Quat q = f::px4_to_ros_attitude(yaw_quat(M_PI / 2));
  const f::Vec3 v_body = f::rotate(f::quat_conj(q), f::ned_to_enu({1, 0, 0}));
  expect_vec(v_body, {0, 1, 0});
}

TEST(Frames, Yaw) {
  EXPECT_NEAR(f::enu_to_ned_yaw(0.0), M_PI / 2, kTol);        // east
  EXPECT_NEAR(f::enu_to_ned_yaw(M_PI / 2), 0.0, kTol);        // north
  EXPECT_NEAR(std::fabs(f::enu_to_ned_yaw(-M_PI / 2)), M_PI, kTol);  // south
  EXPECT_NEAR(f::enu_to_ned_yaw(M_PI), -M_PI / 2, kTol);      // west
  EXPECT_NEAR(f::enu_to_ned_yaw_rate(0.4), -0.4, kTol);
}

TEST(Frames, WrapPi) {
  EXPECT_NEAR(f::wrap_pi(3 * M_PI / 2), -M_PI / 2, kTol);
  EXPECT_NEAR(f::wrap_pi(-3 * M_PI / 2), M_PI / 2, kTol);
  EXPECT_NEAR(f::wrap_pi(0.25), 0.25, kTol);
}

TEST(Frames, Finite) {
  EXPECT_TRUE(f::all_finite({0, 1, 2}));
  EXPECT_FALSE(f::all_finite({0, NAN, 2}));
  EXPECT_FALSE(f::all_finite({INFINITY, 0, 0}));
}
