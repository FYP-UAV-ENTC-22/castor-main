// Onboard runner for the flycrane MARL policy (raptor_v1.1: the policy emits a
// position increment per drone and RAPTOR on the FC flies the resulting setpoint),
// and for the steps that get the payload into the state the policy trained from.
//
//   <ns>/planning/command  castor_interfaces/PlanningCommand   from the system layer: mode + payload goal
//   own state, payload pose (world frame; Isaac ground truth in stack_sim for now, localization later)
//   <ns>/vehicle/odom      the vehicle's own local frame, to express setpoints in it
//   -> <ns>/vehicle/setpoint castor_interfaces/PositionSetpoint  at rate_hz while active
//   -> <ns>/planning/status  castor_interfaces/PolicyStatus      1 Hz
//
// Modes (PlanningCommand): off publishes nothing; steady streams a fixed setpoint (the
// last one flown, or the drone's position); lift climbs straight up until the payload
// hangs at the model's lift height (lift.hpp); policy runs the MARL policy, whose
// observation and action processing (flycrane.hpp) mirror the training env.
// The runner publishes nothing when the command is stale or an input it needs is
// stale; RAPTOR then holds the vehicle where it is (its 200 ms setpoint timeout).
// Setpoints carry over between steady, lift and policy; after off or a stale input
// they restart at the drone. Every policy start refills the history and seeds its
// setpoint at the drone, target yaw = the setpoint's (the env's spawn heading).
// The hand-over check (own cable taut, payload clear of the ground) is computed in
// every mode from the model's rig geometry and reported on the status.
//
// Step trigger (step_trigger): "timer" (default, a Pi) runs a wall-clock timer at
// rate_hz. "payload" (stack_sim) runs one step per new payload pose: the simulator
// publishes it every 1/rate_hz of simulated time, so the policy steps on the time
// PX4 and the physics live in however fast the simulation runs, and always on the
// sample of that instant (a /clock timer would fire before the sample is taken,
// one step late). A sample whose stamp is not newer than the last stepped one (a
// duplicate from a bridge restart) does not step.

#include <pthread.h>
#include <sched.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cstring>
#include <filesystem>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include "castor_interfaces/msg/planning_command.hpp"
#include "castor_interfaces/msg/policy_status.hpp"
#include "castor_interfaces/msg/position_setpoint.hpp"
#include "castor_policy/flight.hpp"
#include "castor_policy/flycrane.hpp"
#include "castor_policy/lift.hpp"
#include "castor_policy/team.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"

#if CASTOR_HAVE_ORT
#include <onnxruntime_cxx_api.h>
#endif

using namespace std::chrono_literals;
using Clock = std::chrono::steady_clock;
using castor_policy::BodyState;
using castor_policy::Quat;
using castor_policy::Vec3;

namespace {

Vec3 vec(const geometry_msgs::msg::Point &p) { return {p.x, p.y, p.z}; }
Vec3 vec(const geometry_msgs::msg::Vector3 &v) { return {v.x, v.y, v.z}; }
Quat quat(const geometry_msgs::msg::Quaternion &q) { return {q.w, q.x, q.y, q.z}; }

template <typename T>
struct Stamped {
  T msg;
  Clock::time_point at;
};

}  // namespace

class PolicyRunner : public rclcpp::Node {
public:
  PolicyRunner() : Node("policy_runner") {
    team_size_ = static_cast<int>(declare_parameter<int64_t>("team_size", 0));
    team_index_ = static_cast<int>(declare_parameter<int64_t>("team_index", -1));
    const auto ns = declare_parameter<std::string>("robot_namespace", "");
    // The launch file resolves a model package (models/README.md) into these parameters.
    model_id_ = declare_parameter<std::string>("model_id", "");
    model_path_ = declare_parameter<std::string>("model_path", "/var/lib/castor/models/policy.onnx");
    rate_hz_ = declare_parameter<double>("rate_hz", 50.0);  // raptor_v1.1 trains the policy at 50 Hz
    step_trigger_ = declare_parameter<std::string>("step_trigger", "timer");
    if (step_trigger_ != "timer" && step_trigger_ != "payload") {
      throw std::invalid_argument("step_trigger must be timer or payload, not '" + step_trigger_ + "'");
    }
    const int frame_base = static_cast<int>(declare_parameter<int64_t>("obs_frame_base", 42));
    history_len_ = static_cast<int>(declare_parameter<int64_t>("history", 3));
    run_inference_ = declare_parameter<bool>("run_inference_every_step", false);
    const int fifo_priority = static_cast<int>(declare_parameter<int64_t>("sched_fifo_priority", 0));
    // Action processing, from marl_hover_env_cfg.py (raptor_v1.1).
    step_scale_ = declare_parameter<double>("setpoint_step_scale", 0.05);
    leash_ = declare_parameter<double>("setpoint_leash", 1.5);
    max_speed_ = declare_parameter<double>("setpoint_max_speed", 0.0);  // <= 0: none
    // Flying it on this airframe (flight.hpp; the model manifest's flight section and policy.point_local).
    point_local_ = vec3_param("policy_point_local").value_or(Vec3{0.0, 0.0, 0.0});
    velocity_filter_s_ = declare_parameter<double>("velocity_filter_s", 0.0);
    velocity_gain_ = declare_parameter<double>("velocity_gain", 1.0);
    goal_lo_ = vec3_param("goal_box_min");
    goal_hi_ = vec3_param("goal_box_max");
    // "Goal reached", the env's goal_achieved_range / goal_achieved_ori_range.
    goal_pos_tol_ = declare_parameter<double>("goal_position_tolerance", 0.3);
    goal_ori_tol_ = declare_parameter<double>("goal_orientation_tolerance", 0.4);
    command_timeout_s_ = declare_parameter<double>("command_timeout_s", 0.5);
    state_timeout_s_ = declare_parameter<double>("state_timeout_s", 0.2);
    // The rig the model was trained for (its manifest's rig section), for the lift and the hand-over check.
    rig_mount_ = vec3_param("rig_mount_local");
    rig_anchor_ = vec3_param("rig_anchor_local");
    handover_cfg_.cable_length = declare_parameter<double>("rig_cable_length", 0.0);
    handover_cfg_.payload_height = declare_parameter<double>("rig_payload_height", 0.0);
    handover_cfg_.taut_tolerance = declare_parameter<double>("handover_taut_tolerance", handover_cfg_.taut_tolerance);
    handover_cfg_.min_clearance = declare_parameter<double>("handover_min_clearance", handover_cfg_.min_clearance);
    lift_cfg_.target_height = declare_parameter<double>("lift_height", lift_cfg_.target_height);
    lift_cfg_.speed = declare_parameter<double>("lift_speed", lift_cfg_.speed);
    lift_cfg_.accel = declare_parameter<double>("lift_accel", lift_cfg_.accel);
    lift_cfg_.max_climb = declare_parameter<double>("lift_max_climb", handover_cfg_.cable_length + 1.5);
    // World-frame state. stack_sim: the simulator's ground truth, published into this robot's domain.
    const auto own = declare_parameter<std::string>("own_state_prefix", "/sim/" + ns + "/state");
    const auto payload = declare_parameter<std::string>("payload_state_prefix", "/sim/payload/state");

    one_hot_ = castor_policy::one_hot(team_size_, team_index_);  // throws on a bad team config
    expected_dim_ = castor_policy::expected_obs_dim(frame_base, team_size_, history_len_);
    history_ = castor_policy::History(static_cast<std::size_t>(history_len_));
    obs_.assign(expected_dim_, 0.0f);

    std::string hot;
    for (float x : one_hot_) {
      hot += (hot.empty() ? "" : ", ") + std::to_string(static_cast<int>(x));
    }
    RCLCPP_INFO(get_logger(), "team slot %d of %d, one-hot [%s], expected obs width %zu, %.0f Hz (%s); state from %s, %s",
                team_index_, team_size_, hot.c_str(), expected_dim_, rate_hz_,
                step_trigger_ == "payload" ? "one step per payload sample" : "wall-clock timer", own.c_str(),
                payload.c_str());

    load_model();
    if (fifo_priority > 0) {
      set_fifo(fifo_priority);
    }

    const std::string root = ns.empty() ? "" : "/" + ns;
    const auto qos = rclcpp::SensorDataQoS();
    own_pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      own + "/pose", qos, [this](const geometry_msgs::msg::PoseStamped &m) { own_pose_ = {m, Clock::now()}; });
    own_twist_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>(
      own + "/twist", qos, [this](const geometry_msgs::msg::TwistStamped &m) { own_twist_ = {m, Clock::now()}; });
    own_twist_w_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>(
      own + "/twist_inertial", qos,
      [this](const geometry_msgs::msg::TwistStamped &m) { own_twist_w_ = {m, Clock::now()}; });
    payload_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      payload + "/pose", qos, [this](const geometry_msgs::msg::PoseStamped &m) { on_payload(m); });
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      root + "/vehicle/odom", 10, [this](const nav_msgs::msg::Odometry &m) { odom_ = {m, Clock::now()}; });
    command_sub_ = create_subscription<castor_interfaces::msg::PlanningCommand>(
      "command", 10, [this](const castor_interfaces::msg::PlanningCommand &m) { command_ = {m, Clock::now()}; });

    setpoint_pub_ = create_publisher<castor_interfaces::msg::PositionSetpoint>(root + "/vehicle/setpoint", 10);
    setpoint_frame_ = ns.empty() ? "odom" : ns + "/odom";
    status_pub_ = create_publisher<castor_interfaces::msg::PolicyStatus>("status", 10);
    const auto period = std::chrono::duration<double>(1.0 / rate_hz_);
    period_ = std::chrono::duration_cast<Clock::duration>(period);
    next_due_ = Clock::now() + period_;
    window_start_ = Clock::now();
    if (step_trigger_ == "timer") {
      step_timer_ = create_wall_timer(period, [this] { step(); });
    }
    status_timer_ = create_wall_timer(1s, [this] { publish_status(); });
  }

private:
  std::optional<Vec3> vec3_param(const std::string &name) {
    const auto v = declare_parameter<std::vector<double>>(name, std::vector<double>{});
    if (v.size() != 3) return std::nullopt;
    return Vec3{v[0], v[1], v[2]};
  }

  void load_model() {
#if CASTOR_HAVE_ORT
    if (!std::filesystem::exists(model_path_)) {
      RCLCPP_WARN(get_logger(), "no model at %s; running without one", model_path_.c_str());
      return;
    }
    try {
      env_ = std::make_unique<Ort::Env>(ORT_LOGGING_LEVEL_WARNING, "castor_policy");
      Ort::SessionOptions opts;
      // One thread: multi-threaded ORT was slower and spin-burned cores on the
      // Pi 5 benchmark (2026-09-29).
      opts.SetIntraOpNumThreads(1);
      opts.SetInterOpNumThreads(1);
      opts.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);
      session_ = std::make_unique<Ort::Session>(*env_, model_path_.c_str(), opts);

      Ort::AllocatorWithDefaultOptions alloc;
      input_name_ = session_->GetInputNameAllocated(0, alloc).get();
      output_name_ = session_->GetOutputNameAllocated(0, alloc).get();
      const auto shape = session_->GetInputTypeInfo(0).GetTensorTypeAndShapeInfo().GetShape();
      model_dim_ = shape.empty() || shape.back() < 0 ? 0 : static_cast<std::size_t>(shape.back());
      const auto out_shape = session_->GetOutputTypeInfo(0).GetTensorTypeAndShapeInfo().GetShape();
      action_dim_ = out_shape.empty() || out_shape.back() < 0 ? 0 : static_cast<std::size_t>(out_shape.back());
      model_loaded_ = true;
      obs_ok_ = model_dim_ == expected_dim_;
      if (!obs_ok_) {
        RCLCPP_ERROR(get_logger(),
                     "model %s takes %zu inputs but this team needs %zu: was it trained for team size %d?",
                     model_path_.c_str(), model_dim_, expected_dim_, team_size_);
        return;
      }
      if (action_dim_ != 3) {
        RCLCPP_ERROR(get_logger(),
                     "model %s outputs %zu actions; the raptor_v1.1 policy outputs 3 (a position increment). "
                     "An ACCBR checkpoint (5) cannot drive RAPTOR; it will not be run",
                     model_path_.c_str(), action_dim_);
      }
      const auto t0 = Clock::now();
      infer();
      last_inference_ms_ = std::chrono::duration<double, std::milli>(Clock::now() - t0).count();
      const std::string name = model_id_.empty() ? model_path_ : model_id_ + " (" + model_path_ + ")";
      RCLCPP_INFO(get_logger(), "loaded %s: %zu inputs, %zu outputs, first inference %.3f ms", name.c_str(),
                  model_dim_, action_dim_, last_inference_ms_);
    } catch (const Ort::Exception &e) {
      RCLCPP_ERROR(get_logger(), "failed to load %s: %s", model_path_.c_str(), e.what());
      model_loaded_ = false;
    }
#else
    RCLCPP_WARN(get_logger(), "built without ONNX Runtime; model %s not loaded", model_path_.c_str());
#endif
  }

#if CASTOR_HAVE_ORT
  std::vector<float> infer() {
    const std::array<int64_t, 2> shape{1, static_cast<int64_t>(obs_.size())};
    auto mem = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
    Ort::Value input = Ort::Value::CreateTensor<float>(mem, obs_.data(), obs_.size(), shape.data(), shape.size());
    const char *in_names[] = {input_name_.c_str()};
    const char *out_names[] = {output_name_.c_str()};
    auto out = session_->Run(Ort::RunOptions{nullptr}, in_names, &input, 1, out_names, 1);
    const float *data = out[0].GetTensorData<float>();
    return {data, data + out[0].GetTensorTypeAndShapeInfo().GetElementCount()};
  }
#endif

  void set_fifo(int priority) {
    sched_param sp{};
    sp.sched_priority = priority;
    const int err = pthread_setschedparam(pthread_self(), SCHED_FIFO, &sp);
    if (err != 0) {
      RCLCPP_WARN(get_logger(), "SCHED_FIFO %d refused (%s); the container needs cap_add SYS_NICE and an rtprio ulimit",
                  priority, std::strerror(err));
    } else {
      RCLCPP_INFO(get_logger(), "running SCHED_FIFO priority %d", priority);
    }
  }

  template <typename T>
  bool fresh(const std::optional<Stamped<T>> &s, Clock::time_point now, double timeout_s) const {
    return s && std::chrono::duration<double>(now - s->at).count() < timeout_s;
  }

  // The mode the system layer asks for; off when its command is stale or unknown.
  std::string requested_mode(Clock::time_point now) const {
    using C = castor_interfaces::msg::PlanningCommand;
    if (!fresh(command_, now, command_timeout_s_)) return C::OFF;
    const auto &m = command_->msg.mode;
    return (m == C::STEADY || m == C::LIFT || m == C::POLICY) ? m : C::OFF;
  }

  // Why `mode` cannot be flown right now, or "" if it can.
  std::string blocker(const std::string &mode, Clock::time_point now) const {
    using C = castor_interfaces::msg::PlanningCommand;
    if (mode == C::POLICY) {
      if (!model_loaded_) return "no model loaded";
      if (!obs_ok_) return "model input width does not match the team";
      if (action_dim_ != 3) return "model does not output a 3-D position increment";
    }
    if (!fresh(own_pose_, now, state_timeout_s_) || !fresh(own_twist_, now, state_timeout_s_) ||
        !fresh(own_twist_w_, now, state_timeout_s_)) {
      return "own state stale";
    }
    if (mode != C::STEADY && !fresh(payload_pose_, now, state_timeout_s_)) return "payload pose stale";
    if (!fresh(odom_, now, state_timeout_s_)) return "vehicle odometry stale";
    return "";
  }

  // The commanded goal position, clamped into the box the policy saw in training when the model gives one.
  Vec3 goal_position() const {
    const Vec3 g = vec(command_->msg.goal.position);
    return goal_lo_ && goal_hi_ ? castor_policy::clamp_box(g, *goal_lo_, *goal_hi_) : g;
  }

  void update_goal_errors() {
    goal_pos_err_ = goal_ori_err_ = -1.0;
    if (!command_ || !payload_pose_) return;
    const auto &g = command_->msg.goal;
    const auto &p = payload_pose_->msg.pose;
    const Vec3 goal = goal_position();
    const Vec3 d{goal[0] - p.position.x, goal[1] - p.position.y, goal[2] - p.position.z};
    goal_pos_err_ = castor_policy::norm(d);
    goal_ori_err_ = castor_policy::angle_between(quat(g.orientation), quat(p.orientation));
  }

  // Own cable span and the hand-over check, in every mode, whenever the poses are fresh.
  void update_handover(Clock::time_point now) {
    cable_span_ = payload_z_ = -1.0;
    handover_ = {};
    const bool own = fresh(own_pose_, now, state_timeout_s_);
    const bool load = fresh(payload_pose_, now, state_timeout_s_);
    if (load) payload_z_ = payload_pose_->msg.pose.position.z;
    if (!rig_mount_ || !rig_anchor_ || handover_cfg_.cable_length <= 0.0) {
      handover_.reason = "no rig geometry (a model package's manifest gives it)";
      return;
    }
    if (!own || !load) {
      handover_.reason = !own ? "own state stale" : "payload pose stale";
      return;
    }
    const auto &d = own_pose_->msg.pose;
    const auto &p = payload_pose_->msg.pose;
    cable_span_ = castor_policy::cable_span(vec(d.position), quat(d.orientation), *rig_mount_, vec(p.position),
                                            quat(p.orientation), *rig_anchor_);
    handover_ = castor_policy::handover_check(cable_span_, payload_z_, handover_cfg_);
  }

  void enter(const std::string &mode) {
    using C = castor_interfaces::msg::PlanningCommand;
    RCLCPP_INFO(get_logger(), "mode %s -> %s", mode_.c_str(), mode.c_str());
    if (mode == C::LIFT) lift_pending_ = true;
    if (mode == C::POLICY) policy_pending_ = true;
    mode_ = mode;
  }

  // Stop publishing. The next active step starts again at the drone, in a freshly anchored frame.
  void deactivate(const std::string &reason) {
    if (active_) {
      RCLCPP_INFO(get_logger(), "%s stopped%s%s", mode_.c_str(), reason.empty() ? "" : ": ", reason.c_str());
    }
    active_ = false;
    inactive_reason_ = reason;
    hold_valid_ = false;
    policy_pending_ = true;
  }

  void on_payload(const geometry_msgs::msg::PoseStamped &m) {
    payload_pose_ = {m, Clock::now()};
    if (step_trigger_ != "payload") return;
    const rclcpp::Time stamp(m.header.stamp, RCL_ROS_TIME);
    if (last_payload_stamp_ && stamp < *last_payload_stamp_ - rclcpp::Duration::from_seconds(1.0)) {
      RCLCPP_WARN(get_logger(), "payload time went back %.1f s (simulator restarted?); following it",
                  (*last_payload_stamp_ - stamp).seconds());
      last_payload_stamp_.reset();
      deactivate("simulator restarted");
    }
    if (last_payload_stamp_ && stamp <= *last_payload_stamp_) return;  // duplicate or out of order
    if (last_payload_stamp_) {
      // How far the sample spacing is from the policy's period, in the stamps' (simulated) time.
      const double late_ms = ((stamp - *last_payload_stamp_).seconds() - 1.0 / rate_hz_) * 1000.0;
      max_late_ms_ = std::max(max_late_ms_, late_ms);
    }
    last_payload_stamp_ = stamp;
    step();
  }

  void step() {
    const auto now = Clock::now();
    if (step_trigger_ == "timer") {
      const double late_ms = std::chrono::duration<double, std::milli>(now - next_due_).count();
      max_late_ms_ = std::max(max_late_ms_, late_ms);
      next_due_ += period_;
      if (now - next_due_ > 10 * period_) {
        next_due_ = now + period_;  // fell far behind (e.g. suspended); resynchronise
      }
    }
    ++steps_;
    update_goal_errors();
    update_handover(now);

    using C = castor_interfaces::msg::PlanningCommand;
    const std::string mode = requested_mode(now);
    if (mode != mode_) enter(mode);
    if (mode == C::OFF) {
      deactivate("");
#if CASTOR_HAVE_ORT
      if (run_inference_ && model_loaded_ && obs_ok_) {  // timing only, on whatever obs_ holds
        const auto t0 = Clock::now();
        try {
          infer();
        } catch (const Ort::Exception &e) {
          RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000, "inference failed: %s", e.what());
          return;
        }
        last_inference_ms_ = std::chrono::duration<double, std::milli>(Clock::now() - t0).count();
        window_ms_.push_back(last_inference_ms_);
      }
#endif
      return;
    }
    const std::string why = blocker(mode, now);
    if (!why.empty()) {
      deactivate(why);
      return;
    }

    const auto &pose = own_pose_->msg.pose;
    BodyState own;
    own.position = vec(pose.position);
    own.orientation = quat(pose.orientation);
    own.linear_velocity = vec(own_twist_w_->msg.twist.linear);
    // Pegasus' twist carries body-frame rates; the env observes world-frame ones.
    own.angular_velocity = castor_policy::apply(castor_policy::rotation(own.orientation), vec(own_twist_->msg.twist.angular));

    if (!hold_valid_) {
      const auto &o = odom_->msg.pose.pose;
      local_ = castor_policy::LocalFrame::from(own.position, castor_policy::yaw_of(own.orientation), vec(o.position),
                                               castor_policy::yaw_of(quat(o.orientation)));
      hold_ = {};
      hold_.position = own.position;
      hold_.yaw = castor_policy::yaw_of(own.orientation);
      hold_valid_ = true;
      RCLCPP_INFO(get_logger(), "%s: setpoint seeded at (%.2f, %.2f, %.2f), yaw %.2f; world->local yaw offset %.3f rad",
                  mode.c_str(), own.position[0], own.position[1], own.position[2], hold_.yaw, local_.yaw_offset);
    }

    const double dt = 1.0 / rate_hz_;
    if (mode == C::STEADY) {
      hold_.velocity = {0.0, 0.0, 0.0};
    } else if (mode == C::LIFT) {
      if (lift_pending_) {
        lift_.reset(hold_.position[2]);
        lift_pending_ = false;
        RCLCPP_INFO(get_logger(), "lift: climbing at %.2f m/s until the payload is at %.2f m (now %.2f m)",
                    lift_cfg_.speed, lift_cfg_.target_height, payload_z_);
      }
      const bool was_done = lift_.done, was_failed = lift_.failed;
      castor_policy::lift_step(lift_, hold_.position, hold_.velocity, own.position[2], payload_z_, dt, lift_cfg_);
      if (lift_.done && !was_done) {
        RCLCPP_INFO(get_logger(), "lift: done, payload at %.2f m, climbed %.2f m, cable span %.3f m", payload_z_,
                    hold_.position[2] - lift_.start_z, cable_span_);
      }
      if (lift_.failed && !was_failed) RCLCPP_ERROR(get_logger(), "lift: %s", lift_.why.c_str());
    } else {
#if CASTOR_HAVE_ORT
      if (!policy_step(own, dt)) return;
#else
      deactivate("built without ONNX Runtime");
      return;
#endif
    }
    publish_setpoint(hold_);
    active_ = true;
    inactive_reason_.clear();
  }

#if CASTOR_HAVE_ORT
  // One policy step: observation, inference, setpoint integration. Moves hold_ to the policy's setpoint.
  // The policy sees, and sets, its own point on the drone (point_local_); hold_ is the body origin's.
  bool policy_step(const BodyState &own, double dt) {
    const BodyState point = castor_policy::policy_point(own, point_local_);
    if (policy_pending_) {
      sp_ = {};
      sp_.position = point.position;
      sp_.yaw = hold_.yaw;  // the heading the drone has been holding
      filtered_velocity_ = {0.0, 0.0, 0.0};
      history_.clear();
      policy_pending_ = false;
      RCLCPP_INFO(get_logger(), "policy: started at (%.2f, %.2f, %.2f), yaw %.2f; step %.3f m, speed cap %.2f m/s, "
                  "velocity filter %.2f s", point.position[0], point.position[1], point.position[2], sp_.yaw,
                  step_scale_, max_speed_, velocity_filter_s_);
    }
    const Vec3 goal = goal_position();
    if (goal != last_goal_) {
      const Vec3 asked = vec(command_->msg.goal.position);
      if (goal != asked) {
        RCLCPP_WARN(get_logger(), "goal (%.2f, %.2f, %.2f) is outside the trained box; flying to (%.2f, %.2f, %.2f)",
                    asked[0], asked[1], asked[2], goal[0], goal[1], goal[2]);
      }
      last_goal_ = goal;
    }
    const auto &load = payload_pose_->msg.pose;
    history_.push(castor_policy::frame(vec(load.position), quat(load.orientation), one_hot_, point, goal,
                                       quat(command_->msg.goal.orientation)));
    obs_ = history_.flat();

    std::vector<float> action;
    const auto t0 = Clock::now();
    try {
      action = infer();
    } catch (const Ort::Exception &e) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000, "inference failed: %s", e.what());
      deactivate("inference failed");
      return false;
    }
    last_inference_ms_ = std::chrono::duration<double, std::milli>(Clock::now() - t0).count();
    window_ms_.push_back(last_inference_ms_);
    if (action.size() != 3 || !std::all_of(action.begin(), action.end(), [](float a) { return std::isfinite(a); })) {
      deactivate("non-finite action");
      return false;
    }

    castor_policy::advance(sp_, {action[0], action[1], action[2]}, point.position, step_scale_, dt, leash_, max_speed_);
    leash_steps_ += sp_.leashed ? 1 : 0;
    const Vec3 v{sp_.velocity[0] * velocity_gain_, sp_.velocity[1] * velocity_gain_, sp_.velocity[2] * velocity_gain_};
    filtered_velocity_ = castor_policy::low_pass(filtered_velocity_, v, dt, velocity_filter_s_);
    hold_.position = castor_policy::body_setpoint(sp_.position, sp_.yaw, point_local_);
    hold_.velocity = filtered_velocity_;
    hold_.yaw = sp_.yaw;
    return true;
  }
#endif

  // The setpoint (world frame, body origin) in the vehicle's own frame, to RAPTOR.
  void publish_setpoint(const castor_policy::Setpoint &sp) {
    const Vec3 p = local_.position(sp.position);
    const Vec3 v = local_.velocity(sp.velocity);
    castor_interfaces::msg::PositionSetpoint out;
    out.header.stamp = get_clock()->now();
    out.header.frame_id = setpoint_frame_;
    out.position.x = p[0];
    out.position.y = p[1];
    out.position.z = p[2];
    out.velocity.x = v[0];
    out.velocity.y = v[1];
    out.velocity.z = v[2];
    out.yaw = local_.yaw(sp.yaw);
    out.yaw_rate = 0.0;
    setpoint_pub_->publish(out);
  }

  void publish_status() {
    const auto now = Clock::now();
    const double window = std::chrono::duration<double>(now - window_start_).count();
    castor_interfaces::msg::PolicyStatus s;
    s.header.stamp = get_clock()->now();
    s.team_size = static_cast<uint32_t>(team_size_);
    s.team_index = static_cast<uint32_t>(team_index_);
    s.one_hot = one_hot_;
    s.model_loaded = model_loaded_;
    s.obs_dim_ok = obs_ok_;
    s.expected_obs_dim = static_cast<uint32_t>(expected_dim_);
    s.model_obs_dim = static_cast<uint32_t>(model_dim_);
    s.loop_rate_hz = window > 0 ? steps_ / window : 0.0;
    s.max_jitter_ms = std::max(0.0, max_late_ms_);
    s.last_inference_ms = last_inference_ms_;
    s.inference_samples = static_cast<uint32_t>(window_ms_.size());
    s.inference_mean_ms = s.inference_p50_ms = s.inference_p99_ms = s.inference_max_ms = -1.0;
    s.model_id = model_id_;
    s.mode = mode_;
    s.active = active_;
    s.inactive_reason = inactive_reason_;
    s.lift_done = lift_.done;
    s.lift_failed = lift_.failed;
    s.handover_ok = handover_.ok;
    s.handover_reason = handover_.reason;
    s.cable_span = cable_span_;
    s.payload_z = payload_z_;
    s.goal_position_error = goal_pos_err_;
    s.goal_orientation_error = goal_ori_err_;
    s.goal_reached = goal_pos_err_ >= 0.0 && goal_pos_err_ < goal_pos_tol_ && goal_ori_err_ >= 0.0 &&
                     goal_ori_err_ < goal_ori_tol_;
    if (!window_ms_.empty()) {
      std::sort(window_ms_.begin(), window_ms_.end());
      const auto pct = [this](double q) {
        return window_ms_[std::min(window_ms_.size() - 1, static_cast<std::size_t>(q * window_ms_.size()))];
      };
      double sum = 0.0;
      for (double x : window_ms_) sum += x;
      s.inference_mean_ms = sum / window_ms_.size();
      s.inference_p50_ms = pct(0.50);
      s.inference_p99_ms = pct(0.99);
      s.inference_max_ms = window_ms_.back();
      if (++windows_since_log_ >= 10) {
        RCLCPP_INFO(get_logger(), "inference over 1 s: n=%u mean %.3f p50 %.3f p99 %.3f max %.3f ms, loop %.1f Hz",
                    s.inference_samples, s.inference_mean_ms, s.inference_p50_ms, s.inference_p99_ms,
                    s.inference_max_ms, s.loop_rate_hz);
        windows_since_log_ = 0;
      }
      window_ms_.clear();
    }
    if (active_ && leash_steps_ > 0) {
      RCLCPP_WARN(get_logger(), "setpoint leash bound on %u steps in the last second", leash_steps_);
    }
    leash_steps_ = 0;
    status_pub_->publish(s);
    steps_ = 0;
    max_late_ms_ = 0.0;
    window_start_ = now;
  }

  int team_size_{0}, team_index_{-1}, history_len_{3};
  std::string model_id_, model_path_, setpoint_frame_;
  std::string step_trigger_{"timer"};
  std::optional<rclcpp::Time> last_payload_stamp_;
  double rate_hz_{50.0}, step_scale_{0.05}, leash_{1.5}, goal_pos_tol_{0.3}, goal_ori_tol_{0.4};
  double max_speed_{0.0}, velocity_filter_s_{0.0}, velocity_gain_{1.0};
  Vec3 point_local_{}, filtered_velocity_{}, last_goal_{};
  std::optional<Vec3> goal_lo_, goal_hi_;
  double command_timeout_s_{0.5}, state_timeout_s_{0.2};
  bool run_inference_{false};
  std::vector<float> one_hot_, obs_;
  std::size_t expected_dim_{0}, model_dim_{0}, action_dim_{0};
  bool model_loaded_{false}, obs_ok_{false};
  double last_inference_ms_{-1.0};

  castor_policy::History history_{3};
  castor_policy::Setpoint sp_;    // the policy's own integrator
  castor_policy::Setpoint hold_;  // what is published: world frame, body origin
  bool hold_valid_{false};        // hold_ and local_ are anchored
  castor_policy::LocalFrame local_;
  std::string mode_{"off"};
  bool active_{false}, lift_pending_{false}, policy_pending_{true};
  std::string inactive_reason_;
  castor_policy::Lift lift_;
  castor_policy::LiftConfig lift_cfg_;
  castor_policy::HandoverConfig handover_cfg_;
  castor_policy::Handover handover_;
  std::optional<Vec3> rig_mount_, rig_anchor_;
  double cable_span_{-1.0}, payload_z_{-1.0};
  double goal_pos_err_{-1.0}, goal_ori_err_{-1.0};
  uint32_t leash_steps_{0};

  std::optional<Stamped<geometry_msgs::msg::PoseStamped>> own_pose_, payload_pose_;
  std::optional<Stamped<geometry_msgs::msg::TwistStamped>> own_twist_, own_twist_w_;
  std::optional<Stamped<nav_msgs::msg::Odometry>> odom_;
  std::optional<Stamped<castor_interfaces::msg::PlanningCommand>> command_;

  Clock::duration period_{};
  Clock::time_point next_due_, window_start_;
  double max_late_ms_{0.0};
  uint64_t steps_{0};
  std::vector<double> window_ms_;
  int windows_since_log_{0};

#if CASTOR_HAVE_ORT
  std::unique_ptr<Ort::Env> env_;
  std::unique_ptr<Ort::Session> session_;
  std::string input_name_, output_name_;
#endif

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr own_pose_sub_, payload_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr own_twist_sub_, own_twist_w_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<castor_interfaces::msg::PlanningCommand>::SharedPtr command_sub_;
  rclcpp::Publisher<castor_interfaces::msg::PositionSetpoint>::SharedPtr setpoint_pub_;
  rclcpp::Publisher<castor_interfaces::msg::PolicyStatus>::SharedPtr status_pub_;
  rclcpp::TimerBase::SharedPtr step_timer_, status_timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<PolicyRunner>());
  } catch (const std::invalid_argument &e) {
    RCLCPP_FATAL(rclcpp::get_logger("policy_runner"), "bad team configuration: %s", e.what());
    rclcpp::shutdown();
    return 2;
  }
  rclcpp::shutdown();
  return 0;
}
