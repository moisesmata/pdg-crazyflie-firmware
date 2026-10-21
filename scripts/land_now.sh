#!/usr/bin/env bash
# Operator landing request; never arms or takes off.
set -eo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=45
export PDG_DISCOVERY_RANGE=LOCALHOST
source "$ROOT/scripts/local_env.sh"
exec python "$ROOT/scripts/request_landing.py"
