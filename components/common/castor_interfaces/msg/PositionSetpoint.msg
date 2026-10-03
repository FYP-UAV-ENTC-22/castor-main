# A position/velocity/yaw target for RAPTOR, in the ENU world frame (REP-103).
# The vehicle component converts it to PX4's NED trajectory_setpoint. RAPTOR
# rejects a setpoint unless position, velocity, yaw and yaw_rate are all finite.

std_msgs/Header header
geometry_msgs/Point position     # [m]   ENU
geometry_msgs/Vector3 velocity   # [m/s] ENU
float64 yaw                      # [rad] ENU, 0 = facing east, counter-clockwise positive
float64 yaw_rate                 # [rad/s] about ENU z
