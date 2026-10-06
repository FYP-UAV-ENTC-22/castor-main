// The raptor_v1.1 flycrane policy's observation and action processing, free of
// ROS so it can be unit-tested. Mirrors MARL_mav_carry_ext
// tasks/directMARL/hover/marl_hover_env.py (control_mode "raptor", partial_obs):
//
//   one frame = load position (3), load rotation row-major (9), one-hot (N),
//               own position (3), own rotation (9), own linear velocity (3),
//               own angular velocity (3), goal position - load position (3),
//               R_goal * R_load^T row-major (9)                -> 42 + N
//   observation = the last `history` frames, oldest first; the first frame after
//                 a reset fills every slot (Isaac Lab CircularBuffer)
//   action (3) = position increment: sp_pos += a * step_scale (no faster than
//                setpoint_max_speed when the task sets one), sp_vel = increment / step_dt,
//                sp_pos leashed to the drone
//
// Everything is in the world frame (ENU, z up), velocities included, as in the env.

#pragma once

#include <array>
#include <cmath>
#include <cstddef>
#include <deque>
#include <vector>

namespace castor_policy {

using Vec3 = std::array<double, 3>;
using Mat3 = std::array<double, 9>;  // row-major
struct Quat {
  double w{1.0}, x{0.0}, y{0.0}, z{0.0};
};

inline Mat3 rotation(const Quat &q) {
  const double n = std::sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
  const double w = q.w / n, x = q.x / n, y = q.y / n, z = q.z / n;
  return {1 - 2 * (y * y + z * z), 2 * (x * y - z * w),     2 * (x * z + y * w),
          2 * (x * y + z * w),     1 - 2 * (x * x + z * z), 2 * (y * z - x * w),
          2 * (x * z - y * w),     2 * (y * z + x * w),     1 - 2 * (x * x + y * y)};
}

inline Vec3 apply(const Mat3 &r, const Vec3 &v) {
  return {r[0] * v[0] + r[1] * v[1] + r[2] * v[2], r[3] * v[0] + r[4] * v[1] + r[5] * v[2],
          r[6] * v[0] + r[7] * v[1] + r[8] * v[2]};
}

// a * b^T
inline Mat3 mul_transpose(const Mat3 &a, const Mat3 &b) {
  Mat3 out{};
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      out[3 * i + j] = a[3 * i] * b[3 * j] + a[3 * i + 1] * b[3 * j + 1] + a[3 * i + 2] * b[3 * j + 2];
    }
  }
  return out;
}

inline double yaw_of(const Quat &q) {
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

// Rotation angle between two orientations [rad], as Isaac Lab's quat_error_magnitude.
inline double angle_between(const Quat &a, const Quat &b) {
  const double na = std::sqrt(a.w * a.w + a.x * a.x + a.y * a.y + a.z * a.z);
  const double nb = std::sqrt(b.w * b.w + b.x * b.x + b.y * b.y + b.z * b.z);
  const double dot = std::fabs(a.w * b.w + a.x * b.x + a.y * b.y + a.z * b.z) / (na * nb);
  return 2.0 * std::acos(std::fmin(1.0, dot));
}

inline double norm(const Vec3 &v) { return std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]); }

struct BodyState {
  Vec3 position{};
  Quat orientation{};
  Vec3 linear_velocity{};   // world frame
  Vec3 angular_velocity{};  // world frame
};

// One observation frame, laid out as the env's partial_obs branch.
inline std::vector<float> frame(const Vec3 &load_position, const Quat &load_orientation,
                                const std::vector<float> &one_hot, const BodyState &own, const Vec3 &goal_position,
                                const Quat &goal_orientation) {
  std::vector<float> f;
  f.reserve(42 + one_hot.size());
  const auto put3 = [&f](const Vec3 &v) { f.insert(f.end(), {float(v[0]), float(v[1]), float(v[2])}); };
  const auto put9 = [&f](const Mat3 &m) {
    for (double x : m) f.push_back(static_cast<float>(x));
  };
  const Mat3 r_load = rotation(load_orientation);
  put3(load_position);
  put9(r_load);
  f.insert(f.end(), one_hot.begin(), one_hot.end());
  put3(own.position);
  put9(rotation(own.orientation));
  put3(own.linear_velocity);
  put3(own.angular_velocity);
  put3({goal_position[0] - load_position[0], goal_position[1] - load_position[1], goal_position[2] - load_position[2]});
  put9(mul_transpose(rotation(goal_orientation), r_load));
  return f;
}

// The last `length` frames, oldest first.
class History {
public:
  explicit History(std::size_t length) : length_(length) {}
  void clear() { frames_.clear(); }
  void push(const std::vector<float> &f) {
    if (frames_.empty()) {
      frames_.assign(length_, f);  // first push after a reset fills every slot
      return;
    }
    frames_.pop_front();
    frames_.push_back(f);
  }
  std::vector<float> flat() const {
    std::vector<float> out;
    for (const auto &f : frames_) out.insert(out.end(), f.begin(), f.end());
    return out;
  }

private:
  std::size_t length_;
  std::deque<std::vector<float>> frames_;
};

// The env's _update_setpoints: an absolute position setpoint the policy nudges.
struct Setpoint {
  Vec3 position{};
  Vec3 velocity{};
  double yaw{0.0};
  bool leashed{false};  // the leash bound on the last step
};

// max_speed <= 0: no cap (setpoint_max_speed None in the env).
inline void advance(Setpoint &sp, const std::array<float, 3> &action, const Vec3 &drone_position, double step_scale,
                    double step_dt, double leash, double max_speed = 0.0) {
  Vec3 inc{};
  for (int i = 0; i < 3; ++i) inc[i] = static_cast<double>(action[i]) * step_scale;
  if (max_speed > 0.0) {
    // actions are unbounded: the step scale is only the speed at unit action
    const double k = std::fmin(max_speed * step_dt / std::fmax(norm(inc), 1e-9), 1.0);
    for (double &x : inc) x *= k;
  }
  for (int i = 0; i < 3; ++i) {
    sp.position[i] += inc[i];
    sp.velocity[i] = inc[i] / step_dt;
  }
  const Vec3 offset{sp.position[0] - drone_position[0], sp.position[1] - drone_position[1],
                    sp.position[2] - drone_position[2]};
  const double d = std::fmax(norm(offset), 1e-6);
  const double scale = std::fmin(leash / d, 1.0);
  sp.leashed = scale < 1.0;
  for (int i = 0; i < 3; ++i) sp.position[i] = drone_position[i] + offset[i] * scale;
}

// World frame -> the vehicle's own local frame (PX4's odometry origin and heading),
// from one simultaneous pair of poses: world (ground truth) and local (odometry).
struct LocalFrame {
  Vec3 world_anchor{}, local_anchor{};
  double yaw_offset{0.0};  // world yaw - local yaw

  static LocalFrame from(const Vec3 &world_position, double world_yaw, const Vec3 &local_position, double local_yaw) {
    LocalFrame f;
    f.world_anchor = world_position;
    f.local_anchor = local_position;
    f.yaw_offset = std::remainder(world_yaw - local_yaw, 2.0 * M_PI);
    return f;
  }
  Vec3 rotate(const Vec3 &v) const {  // by -yaw_offset about z
    const double c = std::cos(yaw_offset), s = std::sin(yaw_offset);
    return {c * v[0] + s * v[1], -s * v[0] + c * v[1], v[2]};
  }
  Vec3 position(const Vec3 &p) const {
    const Vec3 r = rotate({p[0] - world_anchor[0], p[1] - world_anchor[1], p[2] - world_anchor[2]});
    return {r[0] + local_anchor[0], r[1] + local_anchor[1], r[2] + local_anchor[2]};
  }
  Vec3 velocity(const Vec3 &v) const { return rotate(v); }
  double yaw(double world_yaw) const { return std::remainder(world_yaw - yaw_offset, 2.0 * M_PI); }
};

}  // namespace castor_policy
