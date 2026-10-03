#!/bin/bash
# Print the apt packages the given ROS source trees need at run time: their exec
# dependencies (<depend> and <exec_depend>), resolved through rosdep. Packages
# that are part of the trees themselves are left out.
#
#   resolve_runtime_deps.sh <src dir>...   > runtime-apt.txt
#
# The runtime image installs exactly this list, so it never needs build tools
# or rosdep itself.
set -euo pipefail

[ $# -ge 1 ] || { echo "usage: resolve_runtime_deps.sh <src dir>..." >&2; exit 2; }

# With a ROS environment sourced, --ignore-src also skips every package already
# installed under AMENT_PREFIX_PATH (rclcpp, yasmin, ...), which is exactly what
# the runtime image is missing. Resolve against the source trees alone.
clean_env=(env -u AMENT_PREFIX_PATH -u ROS_PACKAGE_PATH -u CMAKE_PREFIX_PATH -u COLCON_PREFIX_PATH)

keys=$("${clean_env[@]}" rosdep keys --rosdistro jazzy --ignore-src --dependency-types exec --from-paths "$@" | sort -u)
[ -n "$keys" ] || exit 0

# shellcheck disable=SC2086
"${clean_env[@]}" rosdep resolve --rosdistro jazzy $keys \
    | grep -v '^#' \
    | tr ' ' '\n' \
    | grep -v '^$' \
    | sort -u
