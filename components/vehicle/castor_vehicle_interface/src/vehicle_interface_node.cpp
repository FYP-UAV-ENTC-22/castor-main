// Bridges PX4's uXRCE-DDS topics to CASTOR's ROS-convention topics.
//
//   /fmu/out/vehicle_odometry   (NED/FRD) -> odom  nav_msgs/Odometry (ENU/FLU)
//   /fmu/out/vehicle_status_vN            -> state castor_interfaces/VehicleState
//   setpoint castor_interfaces/PositionSetpoint (ENU) -> /fmu/in/trajectory_setpoint (NED)
//
// Every other component uses only the right-hand side, so nothing outside this
// container depends on px4_msgs or on PX4's frame conventions.
//
// Safety: this node never arms, never changes mode and never sends a command.
// Setpoint forwarding is off unless enable_setpoint_output is set, and even then
// only finite position + velocity + yaw targets are forwarded (RAPTOR rejects
// anything else). RAPTOR itself holds position when setpoints stop for 200 ms.

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <optional>
#include <string>

#include "castor_interfaces/msg/position_setpoint.hpp"
#include "castor_interfaces/msg/vehicle_state.hpp"
#include "castor_vehicle_interface/frames.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "px4_msgs/msg/offboard_control_mode.hpp"
#include "px4_msgs/msg/trajectory_setpoint.hpp"
#include "px4_msgs/msg/vehicle_odometry.hpp"
#include "px4_msgs/msg/vehicle_status.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;
namespace f = castor_vehicle_interface::frames;
using px4_msgs::msg::OffboardControlMode;
using px4_msgs::msg::TrajectorySetpoint;
using px4_msgs::msg::VehicleOdometry;
using px4_msgs::msg::VehicleStatus;

namespace {

// PX4's uxrce_dds_client appends _v<N> to a topic when its message has a
// non-zero MESSAGE_VERSION (utilities.hpp: generate_topic_name). px4_msgs is
// generated from the same PX4 commit, so the constant here always matches.
std::string versioned(const std::string &topic, uint32_t version) {
  return version == 0 ? topic : topic + "_v" + std::to_string(version);
}

double nan_to_zero(float v) { return std::isfinite(v) ? static_cast<double>(v) : 0.0; }

}  // namespace

class VehicleInterface : public rclcpp::Node {
public:
  VehicleInterface() : Node("vehicle_interface") {
    const auto px4_ns = declare_parameter<std::string>("px4_namespace", "");
    world_frame_ = declare_parameter<std::string>("world_frame", "odom");
    body_frame_ = declare_parameter<std::string>("body_frame", "base_link");
    link_timeout_s_ = declare_parameter<double>("link_timeout_s", 1.0);
    enable_setpoint_output_ = declare_parameter<bool>("enable_setpoint_output", false);
    publish_offboard_mode_ = declare_parameter<bool>("publish_offboard_control_mode", true);

    const std::string fmu = px4_ns.empty() ? "/fmu" : "/" + px4_ns + "/fmu";
    const std::string odom_topic = versioned(fmu + "/out/vehicle_odometry", VehicleOdometry::MESSAGE_VERSION);
    const std::string status_topic = versioned(fmu + "/out/vehicle_status", VehicleStatus::MESSAGE_VERSION);
    const std::string traj_topic = versioned(fmu + "/in/trajectory_setpoint", TrajectorySetpoint::MESSAGE_VERSION);

    // PX4's writers are best effort; SensorDataQoS (best effort, volatile) matches them.
    odom_sub_ = create_subscription<VehicleOdometry>(
      odom_topic, rclcpp::SensorDataQoS(), [this](const VehicleOdometry &m) { on_odometry(m); });
    status_sub_ = create_subscription<VehicleStatus>(
      status_topic, rclcpp::SensorDataQoS(), [this](const VehicleStatus &m) { on_status(m); });

    // Reliable publishers match both reliable and best-effort subscribers.
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("odom", 10);
    state_pub_ = create_publisher<castor_interfaces::msg::VehicleState>("state", 10);

    setpoint_sub_ = create_subscription<castor_interfaces::msg::PositionSetpoint>(
      "setpoint", 10, [this](const castor_interfaces::msg::PositionSetpoint &s) { on_setpoint(s); });
    if (enable_setpoint_output_) {
      traj_pub_ = create_publisher<TrajectorySetpoint>(traj_topic, 10);
      if (publish_offboard_mode_) {
        offboard_pub_ = create_publisher<OffboardControlMode>(fmu + "/in/offboard_control_mode", 10);
      }
    }

    state_timer_ = create_wall_timer(100ms, [this] { publish_state(); });

    RCLCPP_INFO(get_logger(), "PX4 topics: %s, %s; setpoint output %s%s", odom_topic.c_str(),
                status_topic.c_str(), enable_setpoint_output_ ? "ENABLED -> " : "disabled",
                enable_setpoint_output_ ? traj_topic.c_str() : "");
  }

private:
  void on_odometry(const VehicleOdometry &m) {
    if (!std::isfinite(m.q[0]) || !std::isfinite(m.position[0])) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "odometry without a valid pose; not republished");
      return;
    }
    if (m.pose_frame != VehicleOdometry::POSE_FRAME_NED) {
      // FRD world frame: NED with an arbitrary heading offset. Converted the
      // same way, so ENU yaw carries that offset too.
      RCLCPP_WARN_ONCE(get_logger(), "odometry pose_frame is %u, not NED; yaw is relative to an arbitrary heading",
                       m.pose_frame);
    }

    const f::Quat q = f::px4_to_ros_attitude({m.q[0], m.q[1], m.q[2], m.q[3]});
    const f::Vec3 p = f::ned_to_enu({m.position[0], m.position[1], m.position[2]});
    const f::Vec3 v_px4{nan_to_zero(m.velocity[0]), nan_to_zero(m.velocity[1]), nan_to_zero(m.velocity[2])};

    // nav_msgs/Odometry carries twist in the child (body) frame.
    f::Vec3 v_body;
    if (m.velocity_frame == VehicleOdometry::VELOCITY_FRAME_BODY_FRD) {
      v_body = f::frd_to_flu(v_px4);
    } else {
      v_body = f::rotate(f::quat_conj(q), f::ned_to_enu(v_px4));
    }
    const f::Vec3 w = f::frd_to_flu(
      {nan_to_zero(m.angular_velocity[0]), nan_to_zero(m.angular_velocity[1]), nan_to_zero(m.angular_velocity[2])});

    nav_msgs::msg::Odometry o;
    o.header.stamp = now();
    o.header.frame_id = world_frame_;
    o.child_frame_id = body_frame_;
    o.pose.pose.position.x = p[0];
    o.pose.pose.position.y = p[1];
    o.pose.pose.position.z = p[2];
    o.pose.pose.orientation.w = q[0];
    o.pose.pose.orientation.x = q[1];
    o.pose.pose.orientation.y = q[2];
    o.pose.pose.orientation.z = q[3];
    o.twist.twist.linear.x = v_body[0];
    o.twist.twist.linear.y = v_body[1];
    o.twist.twist.linear.z = v_body[2];
    o.twist.twist.angular.x = w[0];
    o.twist.twist.angular.y = w[1];
    o.twist.twist.angular.z = w[2];
    // Diagonal variances only. NED->ENU swaps x and y; body-frame variances are
    // unchanged by the FRD->FLU sign flips. Twist variances are reported in
    // PX4's velocity frame and only approximately apply in the body frame.
    o.pose.covariance[0] = nan_to_zero(m.position_variance[1]);
    o.pose.covariance[7] = nan_to_zero(m.position_variance[0]);
    o.pose.covariance[14] = nan_to_zero(m.position_variance[2]);
    o.pose.covariance[21] = nan_to_zero(m.orientation_variance[0]);
    o.pose.covariance[28] = nan_to_zero(m.orientation_variance[1]);
    o.pose.covariance[35] = nan_to_zero(m.orientation_variance[2]);
    o.twist.covariance[0] = nan_to_zero(m.velocity_variance[0]);
    o.twist.covariance[7] = nan_to_zero(m.velocity_variance[1]);
    o.twist.covariance[14] = nan_to_zero(m.velocity_variance[2]);
    odom_pub_->publish(o);
  }

  void on_status(const VehicleStatus &m) {
    last_status_ = now();
    armed_ = m.arming_state == VehicleStatus::ARMING_STATE_ARMED;
    nav_state_ = m.nav_state;
    failsafe_ = m.failsafe;
    preflight_ok_ = m.pre_flight_checks_pass;
  }

  void publish_state() {
    castor_interfaces::msg::VehicleState s;
    s.header.stamp = now();
    if (last_status_) {
      const double age = (now() - *last_status_).seconds();
      s.seconds_since_fc_message = static_cast<float>(age);
      s.fc_connected = age < link_timeout_s_;
    } else {
      s.seconds_since_fc_message = -1.0f;
      s.fc_connected = false;
    }
    // Report the last known values even when the link is stale; consumers must
    // check fc_connected before trusting them.
    s.armed = armed_;
    s.nav_state = nav_state_;
    s.failsafe = failsafe_;
    s.preflight_checks_pass = preflight_ok_;
    state_pub_->publish(s);
  }

  void on_setpoint(const castor_interfaces::msg::PositionSetpoint &sp) {
    if (!traj_pub_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "setpoint received but enable_setpoint_output is false; not forwarded to PX4");
      return;
    }
    const f::Vec3 p = f::enu_to_ned({sp.position.x, sp.position.y, sp.position.z});
    const f::Vec3 v = f::enu_to_ned({sp.velocity.x, sp.velocity.y, sp.velocity.z});
    const double yaw = f::enu_to_ned_yaw(sp.yaw);
    const double yaw_rate = f::enu_to_ned_yaw_rate(sp.yaw_rate);
    if (!f::all_finite(p) || !f::all_finite(v) || !std::isfinite(yaw) || !std::isfinite(yaw_rate)) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000, "rejected setpoint with a non-finite field");
      return;
    }

    const uint64_t t_us = static_cast<uint64_t>(now().nanoseconds() / 1000);
    if (offboard_pub_) {
      OffboardControlMode mode{};
      mode.timestamp = t_us;
      mode.position = true;
      mode.velocity = true;
      offboard_pub_->publish(mode);
    }

    constexpr float nan = std::numeric_limits<float>::quiet_NaN();
    TrajectorySetpoint t{};
    t.timestamp = t_us;
    for (int i = 0; i < 3; ++i) {
      t.position[i] = static_cast<float>(p[i]);
      t.velocity[i] = static_cast<float>(v[i]);
      t.acceleration[i] = nan;
      t.jerk[i] = nan;
    }
    t.yaw = static_cast<float>(yaw);
    t.yawspeed = static_cast<float>(yaw_rate);
    traj_pub_->publish(t);
  }

  std::string world_frame_, body_frame_;
  double link_timeout_s_{1.0};
  bool enable_setpoint_output_{false}, publish_offboard_mode_{true};

  std::optional<rclcpp::Time> last_status_;
  bool armed_{false}, failsafe_{false}, preflight_ok_{false};
  uint8_t nav_state_{0};

  rclcpp::Subscription<VehicleOdometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<VehicleStatus>::SharedPtr status_sub_;
  rclcpp::Subscription<castor_interfaces::msg::PositionSetpoint>::SharedPtr setpoint_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<castor_interfaces::msg::VehicleState>::SharedPtr state_pub_;
  rclcpp::Publisher<TrajectorySetpoint>::SharedPtr traj_pub_;
  rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_pub_;
  rclcpp::TimerBase::SharedPtr state_timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VehicleInterface>());
  rclcpp::shutdown();
  return 0;
}
