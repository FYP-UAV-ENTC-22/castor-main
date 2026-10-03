// Team identity -> policy observation pieces. Kept free of ROS so it can be unit-tested.

#pragma once

#include <cstddef>
#include <stdexcept>
#include <string>
#include <vector>

namespace castor_policy {

// One-hot agent id: 1 at team_index, 0 elsewhere. This is the slice the
// flycrane envs put in every observation frame (indices 12..12+N in the paper
// env, built with torch.eye(num_drones) in flyhex).
inline std::vector<float> one_hot(int team_size, int team_index) {
  if (team_size < 1) {
    throw std::invalid_argument("team_size must be >= 1, got " + std::to_string(team_size));
  }
  if (team_index < 0 || team_index >= team_size) {
    throw std::invalid_argument("team_index must be in 0.." + std::to_string(team_size - 1) + ", got " +
                                std::to_string(team_index));
  }
  std::vector<float> v(static_cast<std::size_t>(team_size), 0.0f);
  v[static_cast<std::size_t>(team_index)] = 1.0f;
  return v;
}

// Width of one actor observation. Both the paper env (45 x 3 = 135 for three
// drones) and flyhex use (frame_base + team_size) per frame, `history` frames.
inline std::size_t expected_obs_dim(int frame_base, int team_size, int history) {
  return static_cast<std::size_t>(frame_base + team_size) * static_cast<std::size_t>(history);
}

}  // namespace castor_policy
