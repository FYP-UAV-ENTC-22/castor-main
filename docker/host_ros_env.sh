# shellcheck shell=bash
# Join the CASTOR containers' ROS 2 graph (onboard stack, simulator) from a ROS 2
# install on the host itself, after sourcing that install:
#
#   source /opt/ros/humble/setup.bash        # or jazzy
#   source docker/host_ros_env.sh
#   ros2 topic list
#
# Domain 20, Fast DDS, UDP on 127.0.0.1 only (fastdds_host.xml has why not shared
# memory). Containers on host networking share the ros2 daemon's port with the
# host, so a daemon started by another distro or other settings answers your CLI
# with "!rclpy.ok()" faults: this stops it, and the next ros2 command starts a
# fresh one with these settings.
_castor_docker_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export ROS_DOMAIN_ID=20
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE="$_castor_docker_dir/fastdds_host.xml"
export FASTDDS_DEFAULT_PROFILES_FILE="$FASTRTPS_DEFAULT_PROFILES_FILE"
unset ROS_LOCALHOST_ONLY ROS_AUTOMATIC_DISCOVERY_RANGE _castor_docker_dir
if command -v ros2 >/dev/null; then
    ros2 daemon stop >/dev/null 2>&1 || true
else
    echo "host_ros_env.sh: source your ROS 2 setup.bash first" >&2
fi
