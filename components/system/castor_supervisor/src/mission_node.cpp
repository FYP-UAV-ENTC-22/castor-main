// CASTOR mission node (temporary, simplest form): one per drone, in the system layer.
//
//   /team/command "takeoff" | "land"  (castor_interfaces/TeamCommand, from the ground station)
//   /team/goal                        (geometry_msgs/PoseStamped: payload goal, world frame ENU)
//   <ns>/vehicle/state, <ns>/vehicle/odom   what PX4 reports, through the vehicle component
//   <ns>/planning/status              whether the policy has a model and has reached the goal
//   /<other>/system/mission           teammates' mission state (only system state crosses between drones)
//   -> <ns>/vehicle/command           arm, takeoff, RAPTOR, land: the vehicle component forwards them to PX4
//   -> <ns>/planning/command          enable + goal while the policy should fly (refreshed at 10 Hz)
//   -> <ns>/system/mission            this drone's mission state, 5 Hz
//
// The state machine is castor_supervisor/mission.hpp. The node never talks to PX4
// itself. Teammates are found by topic name (any /<ns>/system/mission the zenoh
// bridge makes visible); teammate_namespaces names them explicitly instead.

#include <algorithm>
#include <chrono>
#include <map>
#include <memory>
#include <optional>
#include <regex>
#include <string>
#include <utility>
#include <vector>

#include "castor_interfaces/msg/mission_state.hpp"
#include "castor_interfaces/msg/planning_command.hpp"
#include "castor_interfaces/msg/policy_status.hpp"
#include "castor_interfaces/msg/team_command.hpp"
#include "castor_interfaces/msg/vehicle_command.hpp"
#include "castor_interfaces/msg/vehicle_state.hpp"
#include "castor_supervisor/mission.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;
using Clock = std::chrono::steady_clock;
using castor_interfaces::msg::MissionState;
using castor_interfaces::msg::PlanningCommand;
using castor_interfaces::msg::PolicyStatus;
using castor_interfaces::msg::TeamCommand;
using castor_interfaces::msg::VehicleCommand;
using castor_interfaces::msg::VehicleState;
namespace m = castor_supervisor::mission;

namespace {
double since(Clock::time_point t) { return std::chrono::duration<double>(Clock::now() - t).count(); }
}  // namespace

class MissionNode : public rclcpp::Node {
public:
  MissionNode() : Node("mission") {
    robot_id_ = static_cast<uint32_t>(declare_parameter<int64_t>("robot_id", 0));
    ns_ = declare_parameter<std::string>("robot_namespace", "");
    team_size_ = static_cast<int>(declare_parameter<int64_t>("team_size", 1));
    team_index_ = static_cast<uint32_t>(declare_parameter<int64_t>("team_index", 0));
    cfg_.takeoff_height = declare_parameter<double>("takeoff_height", cfg_.takeoff_height);
    cfg_.height_tolerance = declare_parameter<double>("height_tolerance", cfg_.height_tolerance);
    cfg_.raptor_nav_state = static_cast<int>(declare_parameter<int64_t>("raptor_nav_state", cfg_.raptor_nav_state));
    vehicle_timeout_s_ = declare_parameter<double>("vehicle_timeout_s", 1.0);
    teammate_timeout_s_ = declare_parameter<double>("teammate_timeout_s", 1.5);
    command_repeat_s_ = declare_parameter<double>("command_repeat_s", 1.0);
    for (const auto &t : declare_parameter<std::vector<std::string>>("teammate_namespaces", std::vector<std::string>{})) {
      watch("/" + t + "/system/mission");
    }

    const std::string root = ns_.empty() ? "" : "/" + ns_;
    vehicle_sub_ = create_subscription<VehicleState>(root + "/vehicle/state", 10, [this](const VehicleState &s) {
      vehicle_ = s;
      vehicle_at_ = Clock::now();
    });
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      root + "/vehicle/odom", 10, [this](const nav_msgs::msg::Odometry &o) { altitude_ = o.pose.pose.position.z; });
    status_sub_ = create_subscription<PolicyStatus>(root + "/planning/status", 10,
                                                    [this](const PolicyStatus &s) { planning_ = s; });
    team_cmd_sub_ = create_subscription<TeamCommand>("/team/command", 10, [this](const TeamCommand &c) {
      if (!c.robot_ids.empty() && std::find(c.robot_ids.begin(), c.robot_ids.end(), robot_id_) == c.robot_ids.end()) {
        return;
      }
      takeoff_cmd_ |= c.command == TeamCommand::TAKEOFF;
      land_cmd_ |= c.command == TeamCommand::LAND;
    });
    goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/team/goal", 10, [this](const geometry_msgs::msg::PoseStamped &g) {
        if (!m::at_height(state_)) {
          RCLCPP_WARN(get_logger(), "goal ignored in %s: goals are taken once at take-off height", state_.c_str());
          return;
        }
        goal_ = g.pose;
        new_goal_ = true;
        RCLCPP_INFO(get_logger(), "goal (%.2f, %.2f, %.2f)", g.pose.position.x, g.pose.position.y, g.pose.position.z);
      });

    vehicle_cmd_pub_ = create_publisher<VehicleCommand>(root + "/vehicle/command", 10);
    planning_pub_ = create_publisher<PlanningCommand>(root + "/planning/command", 10);
    state_pub_ = create_publisher<MissionState>("mission", 10);

    entered_ = Clock::now();
    step_timer_ = create_wall_timer(50ms, [this] { step(); });
    planning_timer_ = create_wall_timer(100ms, [this] { publish_planning(); });
    state_timer_ = create_wall_timer(200ms, [this] { publish_state(); });
    discover_timer_ = create_wall_timer(1s, [this] { discover_teammates(); });

    RCLCPP_INFO(get_logger(), "robot %u, team slot %u of %d, take-off height %.2f m, RAPTOR nav_state %d", robot_id_,
                team_index_, team_size_, cfg_.takeoff_height, cfg_.raptor_nav_state);
  }

private:
  void discover_teammates() {
    static const std::regex pattern("^/([a-z][a-z0-9_]*)/system/mission$");
    for (const auto &[topic, types] : get_topic_names_and_types()) {
      std::smatch match;
      if (std::regex_match(topic, match, pattern) && match[1] != ns_) {
        watch(topic);
      }
    }
  }

  void watch(const std::string &topic) {
    if (teammate_subs_.count(topic)) {
      return;
    }
    teammate_subs_[topic] = create_subscription<MissionState>(topic, 10, [this](const MissionState &s) {
      if (s.robot_id != robot_id_) teammates_[s.team_index] = {s, Clock::now()};
    });
    RCLCPP_INFO(get_logger(), "watching teammate %s", topic.c_str());
  }

  bool team_ready() const {
    if (!m::at_height(state_)) return false;
    int ready = 0;
    for (const auto &[index, t] : teammates_) {
      if (static_cast<int>(index) < team_size_ && index != team_index_ && since(t.second) < teammate_timeout_s_ &&
          m::at_height(t.first.state)) {
        ++ready;
      }
    }
    return ready >= team_size_ - 1;
  }

  void step() {
    m::Inputs in;
    in.vehicle_ok = vehicle_ && since(vehicle_at_) < vehicle_timeout_s_ && vehicle_->fc_connected;
    if (vehicle_) {
      in.armed = vehicle_->armed;
      in.landed = vehicle_->landed;
      in.failsafe = vehicle_->failsafe;
      in.preflight_ok = vehicle_->preflight_checks_pass;
      in.nav_state = vehicle_->nav_state;
    }
    in.altitude = altitude_;
    in.takeoff_cmd = std::exchange(takeoff_cmd_, false);
    in.land_cmd = std::exchange(land_cmd_, false);
    in.new_goal = std::exchange(new_goal_, false);
    in.has_goal = goal_.has_value();
    in.team_ready = team_ready_ = team_ready();
    in.planning_ready = planning_ && planning_->model_loaded && planning_->obs_dim_ok;
    in.goal_reached = planning_ && planning_->goal_reached;
    in.time_in_state = since(entered_);

    const auto out = m::step(state_, in, cfg_);
    if (!out.note.empty()) {
      RCLCPP_INFO(get_logger(), "%s%s%s: %s", state_.c_str(), out.state != state_ ? " -> " : "",
                  out.state != state_ ? out.state.c_str() : "", out.note.c_str());
    }
    if (out.state != state_ || (in.new_goal && (state_ == m::MARL || state_ == m::HOLD))) {
      entered_ = Clock::now();  // a new goal restarts MARL's goal-reached hold-off too
    }
    if (out.state != state_) {
      if (!m::at_height(out.state)) goal_.reset();
      state_ = out.state;
      publish_state();
    }
    planning_enabled_ = out.planning_enabled;
    send(out.command);
  }

  // Each transition sends its command at once; states that keep asking repeat it
  // at most every command_repeat_s.
  void send(m::Command c) {
    if (c == m::Command::NONE) {
      return;
    }
    if (c == last_command_ && since(last_command_at_) < command_repeat_s_) {
      return;
    }
    VehicleCommand msg;
    msg.stamp = now();
    switch (c) {
      case m::Command::ARM: msg.command = VehicleCommand::ARM; break;
      case m::Command::DISARM: msg.command = VehicleCommand::DISARM; break;
      case m::Command::TAKEOFF:
        msg.command = VehicleCommand::TAKEOFF;
        msg.altitude = static_cast<float>(cfg_.takeoff_height);
        break;
      case m::Command::RAPTOR: msg.command = VehicleCommand::RAPTOR; break;
      case m::Command::LAND: msg.command = VehicleCommand::LAND; break;
      case m::Command::NONE: return;
    }
    vehicle_cmd_pub_->publish(msg);
    last_command_ = c;
    last_command_at_ = Clock::now();
  }

  void publish_planning() {
    if (!planning_enabled_ && !planning_was_enabled_) {
      return;  // nothing to say; the policy is off by default
    }
    PlanningCommand c;
    c.header.stamp = now();
    c.header.frame_id = "map";
    c.enable = planning_enabled_ && goal_.has_value();
    if (goal_) c.goal = *goal_;
    planning_pub_->publish(c);
    planning_was_enabled_ = planning_enabled_;
  }

  void publish_state() {
    MissionState s;
    s.header.stamp = now();
    s.robot_id = robot_id_;
    s.team_index = team_index_;
    s.state = state_;
    s.team_ready = team_ready_;
    if (goal_) s.goal = *goal_;
    state_pub_->publish(s);
  }

  uint32_t robot_id_{0}, team_index_{0};
  std::string ns_;
  int team_size_{1};
  m::Config cfg_;
  double vehicle_timeout_s_{1.0}, teammate_timeout_s_{1.5}, command_repeat_s_{1.0};

  std::string state_{m::WAIT_VEHICLE};
  Clock::time_point entered_;
  std::optional<VehicleState> vehicle_;
  Clock::time_point vehicle_at_;
  double altitude_{0.0};
  std::optional<PolicyStatus> planning_;
  std::optional<geometry_msgs::msg::Pose> goal_;
  bool takeoff_cmd_{false}, land_cmd_{false}, new_goal_{false};
  bool planning_enabled_{false}, planning_was_enabled_{false}, team_ready_{false};
  m::Command last_command_{m::Command::NONE};
  Clock::time_point last_command_at_;
  std::map<uint32_t, std::pair<MissionState, Clock::time_point>> teammates_;
  std::map<std::string, rclcpp::Subscription<MissionState>::SharedPtr> teammate_subs_;

  rclcpp::Subscription<VehicleState>::SharedPtr vehicle_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<PolicyStatus>::SharedPtr status_sub_;
  rclcpp::Subscription<TeamCommand>::SharedPtr team_cmd_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_cmd_pub_;
  rclcpp::Publisher<PlanningCommand>::SharedPtr planning_pub_;
  rclcpp::Publisher<MissionState>::SharedPtr state_pub_;
  rclcpp::TimerBase::SharedPtr step_timer_, planning_timer_, state_timer_, discover_timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MissionNode>());
  rclcpp::shutdown();
  return 0;
}
