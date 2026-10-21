#!/usr/bin/env bash
# Source this file from a WSL Ubuntu shell.
PDG_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# Verified local DDS settings for this WSL workstation.
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-45}"
export ROS_AUTOMATIC_DISCOVERY_RANGE="${PDG_DISCOVERY_RANGE:-LOCALHOST}"
source /opt/ros/jazzy/setup.bash
if [[ -f "$PDG_ROOT/ros_ws/install/setup.bash" ]]; then
  source "$PDG_ROOT/ros_ws/install/setup.bash"
fi
source "$PDG_ROOT/.venv/bin/activate"
export UV_CACHE_DIR="$PDG_ROOT/.cache/uv"
export MPLCONFIGDIR="$PDG_ROOT/.cache/matplotlib"
export ROS_LOG_DIR="$PDG_ROOT/.runtime/ros-log"
mkdir -p "$UV_CACHE_DIR" "$MPLCONFIGDIR" "$ROS_LOG_DIR"
