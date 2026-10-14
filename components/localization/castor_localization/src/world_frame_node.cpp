// Places this vehicle's odometry in the team's world frame.
//
//   <ns>/vehicle/odom          nav_msgs/Odometry, the vehicle's own local frame (PX4's estimate, ENU)
//   <ns>/vehicle/local_origin  sensor_msgs/NavSatFix, where that frame's origin is on the globe
//   datum (parameter)          latitude, longitude [deg], altitude [m AMSL] of the world frame's origin
//   -> <ns>/localization/odom          nav_msgs/Odometry: the same estimate, pose in the world frame (ENU, datum at
//                                      the origin); twist in the body frame, as nav_msgs defines it
//   -> <ns>/localization/map_to_odom   geometry_msgs/TransformStamped, latched: the local frame in the world frame
//
// Both frames are east-north-up, so the transform is the translation datum.hpp gives. Nothing is published
// without a datum or before the vehicle reports its origin: a consumer then sees a stale input and does not fly.
// The whole team must be given the same datum.

#include <cmath>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "castor_localization/datum.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"

using castor_localization::Geodetic;

class WorldFrame : public rclcpp::Node {
public:
  WorldFrame() : Node("world_frame") {
    const auto ns = declare_parameter<std::string>("robot_namespace", "");
    world_frame_ = declare_parameter<std::string>("world_frame", "map");
    const auto datum = declare_parameter<std::vector<double>>("datum", std::vector<double>{});
    if (datum.size() == 3 && std::isfinite(datum[0]) && std::isfinite(datum[1]) && std::isfinite(datum[2])) {
      datum_ = Geodetic{datum[0], datum[1], datum[2]};
      RCLCPP_INFO(get_logger(), "datum lat %.7f lon %.7f alt %.2f m: world frame '%s'", datum[0], datum[1], datum[2],
                  world_frame_.c_str());
    } else {
      RCLCPP_WARN(get_logger(), "no datum (launch with datum:=<lat>,<lon>,<alt>): nothing is published, and "
                                "planning will not fly on this vehicle's own estimate");
    }

    const std::string root = ns.empty() ? "" : "/" + ns;
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("odom", 10);
    transform_pub_ =
      create_publisher<geometry_msgs::msg::TransformStamped>("map_to_odom", rclcpp::QoS(1).transient_local());
    origin_sub_ = create_subscription<sensor_msgs::msg::NavSatFix>(
      root + "/vehicle/local_origin", rclcpp::QoS(1).transient_local(),
      [this](const sensor_msgs::msg::NavSatFix &m) { on_origin(m); });
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      root + "/vehicle/odom", 10, [this](const nav_msgs::msg::Odometry &m) { on_odom(m); });
  }

private:
  void on_origin(const sensor_msgs::msg::NavSatFix &m) {
    if (!datum_) return;
    const auto t = castor_localization::enu_from_datum(*datum_, {m.latitude, m.longitude, m.altitude});
    const bool moved = !offset_ || std::fabs(t[0] - (*offset_)[0]) > 1e-4 || std::fabs(t[1] - (*offset_)[1]) > 1e-4 ||
                       std::fabs(t[2] - (*offset_)[2]) > 1e-4;
    offset_ = t;
    if (!moved) return;
    RCLCPP_INFO(get_logger(), "local frame '%s' is at (%.3f, %.3f, %.3f) m in the world frame", m.header.frame_id.c_str(),
                t[0], t[1], t[2]);
    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = now();
    tf.header.frame_id = world_frame_;
    tf.child_frame_id = m.header.frame_id;
    tf.transform.translation.x = t[0];
    tf.transform.translation.y = t[1];
    tf.transform.translation.z = t[2];
    tf.transform.rotation.w = 1.0;
    transform_pub_->publish(tf);
  }

  void on_odom(const nav_msgs::msg::Odometry &m) {
    if (!offset_) return;
    nav_msgs::msg::Odometry o = m;  // same stamp, attitude, body-frame twist and covariances
    o.header.frame_id = world_frame_;
    o.pose.pose.position.x += (*offset_)[0];
    o.pose.pose.position.y += (*offset_)[1];
    o.pose.pose.position.z += (*offset_)[2];
    odom_pub_->publish(o);
  }

  std::string world_frame_;
  std::optional<Geodetic> datum_;
  std::optional<std::array<double, 3>> offset_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TransformStamped>::SharedPtr transform_pub_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr origin_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WorldFrame>());
  rclcpp::shutdown();
  return 0;
}
