#include <gtest/gtest.h>

#include <cmath>

#include "castor_policy/flycrane.hpp"

using namespace castor_policy;

namespace {
Quat yaw_quat(double yaw) { return {std::cos(yaw / 2), 0.0, 0.0, std::sin(yaw / 2)}; }
}  // namespace

TEST(Flycrane, RotationMatchesIsaacLabRowMajor) {
  // 90 deg about z: body x -> world y. Isaac Lab matrix_from_quat gives [[0,-1,0],[1,0,0],[0,0,1]].
  const Mat3 r = rotation(yaw_quat(M_PI / 2));
  const Mat3 expected{0, -1, 0, 1, 0, 0, 0, 0, 1};
  for (int i = 0; i < 9; ++i) EXPECT_NEAR(r[i], expected[i], 1e-12) << i;
  const Vec3 v = apply(r, {1, 0, 0});
  EXPECT_NEAR(v[1], 1.0, 1e-12);
}

TEST(Flycrane, FrameLayoutForThreeDrones) {
  BodyState own;
  own.position = {1, 2, 3};
  own.linear_velocity = {4, 5, 6};
  own.angular_velocity = {7, 8, 9};
  const auto f = frame({0.1, 0.2, 0.3}, Quat{}, {0, 1, 0}, own, {1.1, 1.2, 1.3}, Quat{});
  ASSERT_EQ(f.size(), 45u);
  EXPECT_FLOAT_EQ(f[0], 0.1f);   // load position
  EXPECT_FLOAT_EQ(f[3], 1.0f);   // load R[0][0]
  EXPECT_FLOAT_EQ(f[12], 0.0f);  // one-hot at 12..14
  EXPECT_FLOAT_EQ(f[13], 1.0f);
  EXPECT_FLOAT_EQ(f[15], 1.0f);  // own position
  EXPECT_FLOAT_EQ(f[27], 4.0f);  // own linear velocity after own R (18..26)
  EXPECT_FLOAT_EQ(f[30], 7.0f);  // own angular velocity
  EXPECT_FLOAT_EQ(f[33], 1.0f);  // goal - load
  EXPECT_FLOAT_EQ(f[35], 1.0f);
  EXPECT_FLOAT_EQ(f[36], 1.0f);  // R_goal R_load^T = I
  EXPECT_FLOAT_EQ(f[37], 0.0f);
  EXPECT_FLOAT_EQ(f[44], 1.0f);
}

TEST(Flycrane, DifferenceMatrixIsGoalTimesLoadTranspose) {
  const auto f = frame({0, 0, 0}, yaw_quat(0.3), {1}, BodyState{}, {0, 0, 0}, yaw_quat(1.0));
  const Mat3 expected = rotation(yaw_quat(0.7));
  for (int i = 0; i < 9; ++i) EXPECT_NEAR(f[34 + i], expected[i], 1e-6) << i;
}

TEST(Flycrane, HistoryBackfillsThenKeepsOldestFirst) {
  History h(3);
  h.push({1});
  EXPECT_EQ(h.flat(), (std::vector<float>{1, 1, 1}));
  h.push({2});
  h.push({3});
  h.push({4});
  EXPECT_EQ(h.flat(), (std::vector<float>{2, 3, 4}));
  h.clear();
  h.push({9});
  EXPECT_EQ(h.flat(), (std::vector<float>{9, 9, 9}));
}

TEST(Flycrane, AdvanceIntegratesAndDerivesVelocity) {
  Setpoint sp;
  sp.position = {0, 0, 1};
  advance(sp, {1.0f, -0.5f, 0.0f}, {0, 0, 1}, 0.05, 0.02, 1.5);
  EXPECT_NEAR(sp.position[0], 0.05, 1e-9);
  EXPECT_NEAR(sp.position[1], -0.025, 1e-9);
  EXPECT_NEAR(sp.velocity[0], 2.5, 1e-9);
  EXPECT_NEAR(sp.velocity[1], -1.25, 1e-9);
  EXPECT_FALSE(sp.leashed);
}

TEST(Flycrane, LeashKeepsSetpointNearTheDrone) {
  Setpoint sp;
  sp.position = {1.48, 0, 0};
  advance(sp, {1.0f, 0.0f, 0.0f}, {0, 0, 0}, 0.05, 0.02, 1.5);
  EXPECT_TRUE(sp.leashed);
  EXPECT_NEAR(sp.position[0], 1.5, 1e-9);
}

TEST(Flycrane, LocalFrameMapsTheAnchorAndRotates) {
  // Local frame = world shifted to (5, -2, 0) and yawed by +90 deg.
  const auto lf = LocalFrame::from({5, -2, 0.2}, M_PI / 2, {0, 0, 0}, 0.0);
  const Vec3 a = lf.position({5, -2, 0.2});
  EXPECT_NEAR(a[0], 0.0, 1e-12);
  EXPECT_NEAR(a[2], 0.0, 1e-12);
  const Vec3 b = lf.position({5, -1, 1.2});  // 1 m world +y = local +x, 1 m up
  EXPECT_NEAR(b[0], 1.0, 1e-12);
  EXPECT_NEAR(b[1], 0.0, 1e-12);
  EXPECT_NEAR(b[2], 1.0, 1e-12);
  EXPECT_NEAR(lf.yaw(M_PI / 2), 0.0, 1e-12);
}

TEST(Flycrane, AngleBetween) {
  EXPECT_NEAR(angle_between(yaw_quat(0.2), yaw_quat(0.6)), 0.4, 1e-9);
  const Quat q = yaw_quat(0.5);
  EXPECT_NEAR(angle_between(q, Quat{-q.w, -q.x, -q.y, -q.z}), 0.0, 1e-6);  // double cover
}
