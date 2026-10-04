# shellcheck shell=bash
# Sourced by Isaac Sim's setup_python_env.sh (so by python.sh, isaaclab.sh and
# everything Pegasus or Isaac Lab starts): the ROS 2 Jazzy build bundled with
# the Isaac ROS 2 bridge, Python 3.11. Appended, never global: the bundled
# libcrypto would shadow the system's and break apt, curl and openssl.
export ROS_DISTRO=jazzy
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
case ":${LD_LIBRARY_PATH:-}:" in
    *":/isaac-sim/exts/isaacsim.ros2.bridge/jazzy/lib:"*) ;;
    *) export LD_LIBRARY_PATH="${LD_LIBRARY_PATH:+$LD_LIBRARY_PATH:}/isaac-sim/exts/isaacsim.ros2.bridge/jazzy/lib" ;;
esac
