#include <gtest/gtest.h>

#include <stdexcept>

#include "castor_policy/team.hpp"

using castor_policy::expected_obs_dim;
using castor_policy::one_hot;

TEST(Team, OneHot) {
  EXPECT_EQ(one_hot(3, 0), (std::vector<float>{1, 0, 0}));
  EXPECT_EQ(one_hot(3, 2), (std::vector<float>{0, 0, 1}));
  EXPECT_EQ(one_hot(1, 0), (std::vector<float>{1}));
  EXPECT_EQ(one_hot(6, 4), (std::vector<float>{0, 0, 0, 0, 1, 0}));
}

TEST(Team, OneHotRejectsBadSlots) {
  EXPECT_THROW(one_hot(3, 3), std::invalid_argument);
  EXPECT_THROW(one_hot(3, -1), std::invalid_argument);
  EXPECT_THROW(one_hot(0, 0), std::invalid_argument);
}

TEST(Team, ObsDimMatchesTrainedPolicies) {
  // The 3-drone paper policy benchmarked on drone1: 135 inputs.
  EXPECT_EQ(expected_obs_dim(42, 3, 3), 135u);
  // flyhex with six drones: (24 + 18 + 6) * 3.
  EXPECT_EQ(expected_obs_dim(42, 6, 3), 144u);
}
