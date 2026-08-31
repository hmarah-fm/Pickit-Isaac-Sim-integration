#!/usr/bin/env bash
set -euo pipefail

readonly ISAAC_ROS_JAZZY_LIB=/isaac-sim/exts/isaacsim.ros2.bridge/jazzy/lib

if [[ ! -d "${ISAAC_ROS_JAZZY_LIB}" ]]; then
  echo "ERROR: Isaac Sim internal Jazzy bridge libraries were not found:"        "${ISAAC_ROS_JAZZY_LIB}" >&2
  exit 2
fi

# Do not source /opt/ros/jazzy/setup.bash here. Isaac Sim uses its embedded
# Python 3.11 runtime and its own ROS 2 Jazzy bridge libraries.
export ROS_DISTRO=jazzy
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
export LD_LIBRARY_PATH="${ISAAC_ROS_JAZZY_LIB}${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"

exec /isaac-sim/runheadless.sh "$@"
