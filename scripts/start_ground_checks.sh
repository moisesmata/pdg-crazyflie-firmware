#!/usr/bin/env bash
# Start only the Vicon receiver and radio server; no arming or flight mission.
set -eo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"
REPAIR=0
PROPS_REMOVED=0
export PDG_CONTROLLER_ID=0
export PDG_VICON_ONLY=0
for arg in "$@"; do
  case "$arg" in
    --repair-firmware) REPAIR=1 ;;
    --props-removed) PROPS_REMOVED=1 ;;
    --baseline-pid) export PDG_CONTROLLER_ID=1 ;;
    --vicon-only) export PDG_VICON_ONLY=1; export PDG_CONTROLLER_ID=1 ;;
    *) echo "Unknown option: $arg" >&2; exit 2 ;;
  esac
done
if [[ "$REPAIR" == 1 && "$PROPS_REMOVED" != 1 ]]; then
  echo 'Firmware repair requires --props-removed.' >&2
  exit 2
fi
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=45
export PDG_DISCOVERY_RANGE=LOCALHOST
source "$ROOT/scripts/local_env.sh"
mkdir -p "$ROOT/output/setup"
if [[ ! -d /dev/bus/usb ]]; then
  echo 'USB device nodes are unavailable. Run this in a normal Ubuntu/WSL terminal with the Crazyradio attached.' >&2
  exit 2
fi
URI="$(python -c 'from pathlib import Path; import yaml; print(yaml.safe_load(Path("config/crazyflies.yaml").read_text())["robots"]["Crazyflie"]["uri"])')"
timeout 120s python "$ROOT/scripts/probe_drone.py" --uri "$URI" 2>&1 | tee "$ROOT/output/setup/recovery-probe.log"
if [[ "$PDG_VICON_ONLY" == 1 ]]; then
  python - <<'PY_CHECK'
from pathlib import Path
import json
v=json.loads(Path('output/setup/drone-probe.json').read_text())['parameters']
if any(v.get(name)=='1' for name in ['deck.bcFlow','deck.bcFlow2','deck.bcZRanger2']):
    raise SystemExit('Vicon-only mode currently requires the Flow/Z-ranger deck removed: motion.disable does not disable range. Power off before removing the deck.')
print('Vicon-only sensor check: no initialized Flow/Z-ranger deck; optical flow will also be disabled.')
PY_CHECK
fi
SELFTEST="$(python -c 'from pathlib import Path; import json; print(json.loads(Path("output/setup/drone-probe.json").read_text())["parameters"].get("system.selftestPassed", "missing"))')"
if [[ "$SELFTEST" != 1 ]]; then
  echo 'Drone startup self-test did not pass. No tracking/radio server or flight is started.' >&2
  echo 'Stop other clients, disconnect USB and battery for 10 seconds, then power on level and stationary before retrying. Do not override the self-test.' >&2
  exit 2
fi
IDENTITY="$(python -c 'from pathlib import Path; import json; v=json.loads(Path("output/setup/drone-probe.json").read_text())["parameters"]; print(int(v.get("pdg.apiVersion",-1))==1 and int(v.get("pdg.controllerId",-1))==6)')"
if [[ "$IDENTITY" != True ]]; then
  if [[ "$REPAIR" != 1 ]]; then
    echo 'Expected PDG identity missing. With props removed, rerun with --repair-firmware --props-removed.' >&2
    exit 2
  fi
  python "$ROOT/scripts/flash_cf2.py" --flash --props-removed 2>&1 | tee "$ROOT/output/setup/recovery-flash.log"
else
  echo 'Expected PDG API/controller identity is installed; no reflash needed.'
fi
printf 'Project: %s\nROS domain: %s\nDDS: %s\n' "$ROOT" "$ROS_DOMAIN_ID" "$RMW_IMPLEMENTATION"
printf 'Requested controller ID: %s (0=project Mellinger, 1=stock PID baseline)\n' "$PDG_CONTROLLER_ID"
printf 'Starting tracking and radio server. No arming/takeoff command is sent. Leave this terminal running.\n'
"$ROOT/scripts/launch_crazyswarm.sh" 2>&1 | tee "$ROOT/output/setup/server-console.log"
