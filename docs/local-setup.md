# This workstation: CF 2.1+ brushed and local Windows Vicon

All local files are under `moises-experiments/pdg-crazyflie-firmware`.
The firmware target for the stated CF 2.1+ **brushed** hardware is `cf2`.
The tracked subject, root segment and ROS robot are named `Crazyflie`.
Vicon DataStream is reachable at `127.0.0.1:801` from this WSL environment.
The radio address is configured as `E7E7E7E7E7`; channel 80 / 2M is still an unverified starting setting.

## Open a normal Ubuntu/WSL terminal

```bash
cd /mnt/c/Users/gravitas/Documents/Codex/2026-10-20/ple/moises-experiments/pdg-crazyflie-firmware
source scripts/local_env.sh
python scripts/check_vicon.py --receiver --seconds 15
./scripts/inspect_vicon.sh
python scripts/scan_radio.py
```

Do not start a second mocap receiver when the project launch is running: omit `--receiver` in that case.
The environment defaults to Cyclone DDS, ROS domain 45 and local ROS discovery. Both terminals must source it. Fast DDS did not deliver reliably inside this WSL command environment; Cyclone delivered live poses at approximately 122 Hz with a maximum observed message age of 1.5 ms. These are middleware receive measurements, not camera-to-controller latency or clock/frame calibration. An existing `RMW_IMPLEMENTATION` or `ROS_DOMAIN_ID` is preserved; `PDG_DISCOVERY_RANGE` overrides the local discovery setting.

The radio scan does not arm or fly the drone. Copy a discovered URI into `config/crazyflies.yaml`, keeping the Vicon/ROS name unchanged.
The assistant's command sandbox does not expose `/dev/bus/usb`, even though sysfs lists the Crazyradio (`1915:7777`).
A normal WSL terminal needs the Crazyradio attached through usbipd and appropriate USB permissions.

The DataStream diagnostic found `subject=Crazyflie` and `root=Crazyflie`. Its first sample was an old occluded frame; subsequent samples were live and unoccluded, at approximately (0.560, 2.318, 1.531) metres. `scripts/inspect_vicon.sh` samples five frames so the initial sample does not determine the result. The receiver suppresses occluded bodies. The tracked position differs from the example start/ground coordinates; ground-body height and frame alignment remain to be measured.

## Reproduce the software setup

ROS 2 Jazzy and the system dependencies were already installed on this Ubuntu 24.04 machine.
No system packages were changed.

```bash
./scripts/setup_local.sh
source scripts/local_env.sh
mkdir -p ros_ws/src
vcs import --recursive ros_ws/src < ros.repos
python scripts/patch_heading_commands.py
rosdep check --from-paths ros_ws/src --ignore-src
(cd ros_ws && colcon build --symlink-install --executor sequential --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF)
source scripts/local_env.sh
./scripts/bootstrap_toolchain_linux.sh  # only if .tools lacks the compiler
make firmware PLATFORM=cf2
python -m pytest -q
pdg mission --height 1.0 --output output/dry-run
python scripts/simulation_sweep.py
```

The local compiler is xPack Arm GCC 14.2.1-1.1, downloaded from its release and verified against the published SHA-256.

The local Crazyswarm C++ patch uses firmware commands TAKEOFF_2 (7) and LAND_2 (8), with `useCurrentYaw=1`, instead of the pinned library's legacy commands that target yaw zero. It affects both individual and broadcast takeoff/landing requests. The patched server advertises `high_level.preserve_yaw=true`; heading-preserving hover preflight refuses to arm if that marker is absent. Run `scripts/patch_heading_commands.py` after importing the pinned sources and before building. `tests/test_heading_packets.cpp` verifies every payload byte against the firmware/cflib layout.

`scripts/align_ground_estimator.py --apply` initializes Kalman position and yaw from a stationary tracked pad and resets the estimator. It requires fresh unarmed supervisor status, adequate battery, one pose publisher, a level body near the calibrated pad, and stable position/orientation. It never arms or flies. This fixes estimator initialization; it does not establish the physical orientation of the rigid body's axes or remove marker symmetry.

ROS arming service completion does not confirm that the supervisor can fly. The first takeoff attempt reached the radio 60 ms before the firmware announced readiness, and was ignored. Mission transport now waits for fresh supervisor ARMED and CAN_FLY flags before takeoff, and confirms disarming after landing. Subsequent first-hover launches require at least 4.0 V at rest. Mission logs include battery/supervisor status samples and voltage/status age in health abort reasons.

The initial 3.7 V airborne threshold was too conservative: a charged pack fell from 4.12 V at rest to 3.679 V under motor load. The hover configuration now requests landing below 3.5 V, still above the pinned CF2 firmware's 3.2 V low/3.0 V critical thresholds (`platform_defaults_cf2.h`). Other missions retain their previous default threshold. Abort handling now continues landing supervision and only disarms after a fresh stationary ground-pose proxy.

The next attempt exceeded the 8 cm lateral deviation limit after rising about 6 cm. The firmware subsequently reported a crash and unarmed status. A completed hover has not been demonstrated. Before retrying, inspect the vehicle and verify physical body axes with props removed. Raw `acc.x/y/z` logging is configured on `/Crazyflie/imu`; after restarting the server, `scripts/check_body_axes.py` compares raw accelerometer gravity with gravity predicted by the tracked body orientation during a stationary nose-up tilt. This check does not arm or fly and does not establish the rigid-body origin or certify a complete calibration.

The passive near-level comparison found approximately 2.1 degrees of IMU/Vicon gravity disagreement, but cannot establish yaw alignment. Subsequent props-off nose-up and right-side-up measurements exposed disagreements of 45.7 and 34.4 degrees. A fixed marker-to-airframe rotation fitted from these poses and the earlier level capture is approximately [-0.29, -3.61, 177.73] degrees in xyz Euler order. Applying it offline reduced an independent nose-down capture from 30.25 degrees disagreement to 2.77 degrees median (3.01 maximum, 59 samples). Reports and raw samples are in `output/setup/body-frame-*.json` and `body-axes-*-samples.json`.

`config/body-frame.yaml` stores the independently validated right multiplier: q_world_body = q_world_marker * q_marker_body. The project launch routes the original Vicon poses to `/poses_raw`; `scripts/relay_body_pose.py` publishes corrected orientations to `/poses`, preserving world positions, frame IDs and source timestamps. Raw mocap TF children use the `_marker` suffix. The relay drops invalid/stale inputs and requires exactly one raw pose publisher. It advertises the finite pose deadline QoS required by the C++ server. Ten transform/configuration tests and an isolated ROS-domain relay smoke test passed; confirm the corrected live poses with props removed before flight. Do not edit the Tracker object's local axes and also retain this correction: that would apply two corrections. Recalibrate after changing the marker geometry or object axes.

A parameter audit also found that the pinned C++ server's ROS parameter mirror retained pre-override values. The launch now supplies per-robot declaration overrides using the same global/type/robot precedence as firmware writes. ROS parameter queries consequently represent configured values; they are not fresh hardware readbacks. The controller's reported mass was 0.029 kg, which excludes the attached markers. No mass or gain adjustment was made without a measurement.
Planner dependencies use `uv.lock`; ROS revisions use `ros.repos`; all 10 workspace packages built successfully. Local cflib and direct Vicon diagnostic dependencies are pinned in `scripts/setup_local.sh`.
Firmware builds never flash a device.

## Before a physical experiment

Use `./scripts/launch_crazyswarm.sh` to start the Vicon receiver and C++ radio server.
The project launch does not start teleop or a flight mission.
From a second terminal with `scripts/local_env.sh` sourced, run `python scripts/check_hardware.py` for a read-only check of the pose and PDG firmware identity.
Check live `/poses`, the real radio URI, PDG firmware parameters and clock freshness.
Measure the vehicle mass, marker/body origin and ground-body height; adjust `config/nominal.yaml` to the actual room and vehicle before considering execution.
The supplied mass, ground-body height, start coordinates and target remain examples.
Build output alone does not establish that PDG firmware is installed on the drone.
Follow `docs/hardware.md` for the separate, platform-matched flashing step and explicit mission execution command.

## Repository issues reproduced during setup

1. **Read-only preflight parameters were never fetched by the default launch.** The mission reads `pdg.apiVersion` and `pdg.controllerId` through ROS GetParameters, but upstream's C++ server defaults `firmware_params.query_all_values_on_connect` to false. Their declarations have no value unless fetched. The new `launch/pdg.launch.py` explicitly enables querying and loads the project configs. Two configuration regression tests cover this setting and robot/frame consistency.
2. **The lock selected pytest 9, incompatible with Jazzy launch_testing.** With ROS sourced, pytest failed plugin registration because Jazzy's `pytest_pycollect_makemodule(path, parent)` hook uses the removed `path` argument. The dev requirement and lock now select pytest 8.4.2.
3. **The original bootstrap supported only macOS.** A Linux x86_64 compiler helper and ROS-compatible local Python setup/environment scripts were added.
4. **The default launch also inherited joystick teleop.** The local project launch starts only mocap and the C++ server, consistent with a single mission command source.

The original README did not explain the experiment workflow; it now points to this guide and the hardware documentation.
The example hostname, robot name and radio address were deployment settings, not algorithm defects.
The Vicon occlusion and sandbox USB visibility are runtime/environment blockers, not planner test failures.

## Validation completed on this machine

- All 16 tests passed with ROS Jazzy sourced and pytest 8.4.2.
- The 1 m dry-run mission converged in 13 iterations and passed independent rollout validation.
- The vertical, east, west and diagonal simulation sweep cases all converged and passed validation.
- The `cf2` firmware built with required arming and linked `controllerOutOfTree` and `pdgLimitCollective` symbols. Images and manifest are in `output/firmware/`; no device was flashed.
- `rosdep check --from-paths ros_ws/src --ignore-src` reported all system dependencies satisfied.
- Live SDK connectivity and Vicon subject/segment identity were confirmed, including advancing unoccluded pose samples after an initially stale occluded frame. The pinned ROS receiver delivered 1,432 real poses in a 12 s check with Cyclone DDS, at about 122 Hz and max observed ROS message age 1.5 ms.
- All 10 pinned ROS packages built. A synthetic receiver check also validated the ROS message path. Upstream reports a guard-condition exception during SIGINT shutdown; valid pose measurements were recorded before shutdown. This shutdown behavior is retained in receiver logs and has not been patched in the pinned dependency.
- Radio discovery could not run inside the assistant sandbox because USB device nodes were not exposed. The radio channel/rate and installed device firmware remain unverified.

Logs and JSON reports are under `output/setup/`; dry-run plots and plans are under `output/dry-run/`, and the sweep is under `output/smoke-study/`.
