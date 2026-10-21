#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
if grep -q CHANGE_ME "$ROOT/config/motion_capture.yaml"; then
  echo 'Set the Vicon hostname in config/motion_capture.yaml first.' >&2
  exit 2
fi
# Requires a built/sourced Crazyswarm2 workspace. No flight command is issued here.
exec ros2 launch crazyflie launch.py backend:=cpp \
  crazyflies_yaml_file:="$ROOT/config/crazyflies.yaml" \
  motion_capture_yaml_file:="$ROOT/config/motion_capture.yaml"
