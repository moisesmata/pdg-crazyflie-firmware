#!/usr/bin/env bash
# Print vendor subject/segment names and occlusion flags. No radio connection.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SDK="$ROOT/ros_ws/src/motion_capture_tracking/motion_capture_tracking/deps/libmotioncapture/deps/vicon-datastream-sdk/Vicon/CrossMarket/DataStream/ViconDataStreamSDK_CPP"
BINARY="$ROOT/.tools/vicon_subjects"
if [[ ! -x "$BINARY" || "$ROOT/scripts/vicon_subjects.cpp" -nt "$BINARY" ]]; then
  LIBRARY="$("$ROOT/.venv/bin/python" -c 'import importlib.util; print(importlib.util.find_spec("motioncapture").origin)')"
  mkdir -p "$ROOT/.tools"
  # python3-config intentionally supplies the matching system Python 3.12 library.
  read -r -a LINK_FLAGS <<< "$(python3-config --embed --ldflags)"
  g++ -std=c++17 -I"$SDK" "$ROOT/scripts/vicon_subjects.cpp" "$LIBRARY" \
    "${LINK_FLAGS[@]}" -Wl,-rpath,"$(dirname "$LIBRARY")" -o "$BINARY"
fi
exec timeout 15s "$BINARY" "${1:-127.0.0.1:801}"
