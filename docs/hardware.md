# Crazyswarm2 and Vicon integration

Default data path: Vicon Tracker/DataStream → Crazyswarm2 `motion_capture_tracking` (Vicon backend, vendor rigid bodies) → `/poses` and radio external-pose packets → onboard Kalman estimator. Our host subscribes to the same `/poses` stream. OPT4SMART is an optional future alternate source; do not run two external-pose pipelines for the same vehicle.

Reference implementation: https://github.com/IMRCLab/crazyswarm2 and https://github.com/IMRCLab/motion_capture_tracking/tree/ros2. Configuration fields and service/message interfaces were checked against their current source; see `ros.repos` for pinned versions. ROS runtime integration is not validated on the development Mac.

## Linux ROS workstation

1. Use a supported ROS 2 environment (for example Ubuntu 24.04/Jazzy) and the Crazyswarm2 installation instructions. Install radio permissions and Vicon backend dependencies per upstream docs.
2. `mkdir -p ros_ws/src && vcs import --recursive ros_ws/src < ros.repos`
3. Install ROS package dependencies with rosdep and build/source this workspace. Also install this host package into a Python environment compatible with your ROS interpreter; do not mix the Mac .venv into Linux ROS.
4. Edit `config/crazyflies.yaml`: the real radio URI, robot name, initial rigid-body position, and calibrated logging/settings. The Vicon rigid body name must match `cf1` (or change the robot name consistently).
5. Edit `config/motion_capture.yaml` hostname. Set Vicon's frame, metre units, body origin and orientation consistently with ENU/body-z-up. `world` is a frame identifier, not a coordinate transformation. A nonmatching system needs an explicit transform adapter before flight.
6. Match `config/nominal.yaml` to the room and measured vehicle. `ground_body_height` is the tracked body-origin height at actual ground contact, not necessarily zero. All planner coordinates are absolute in the configured frame. `--height X` means rise X above that ground-body height.
7. Launch `./scripts/launch_crazyswarm.sh`. The project launch explicitly enables `firmware_params.query_all_values_on_connect` so the read-only PDG identity parameters are available to preflight, and starts no joystick teleop. This launches the server/receiver, not takeoff. Keep exactly one command-producing mission node per vehicle.
8. Observe `/poses`, validate freshness, pose origin/orientation, and robot names before any motor operation. The mission rejects stale, duplicate-source and mismatched-frame observations. Clock synchronization is required.

## Firmware and flight

Build the correct platform (`cf2` for CF 2.x brushed, `cf21bl` for CF 2.1 Brushless). Never flash a platform-mismatched image. Flashing is a separate manual Bitcraze operation; no build/test command flashes automatically. Requiring explicit motor arming is enabled in our app config.

Crazyswarm configuration uses `stabilizer.controller: 0`: the compiled default auto-selects our OOT wrapper. Selecting `2` directly runs stock Mellinger and bypasses the wrapper. The mission checks the app API/controller IDs and requires capability injection disabled.

Run `pdg mission --height 1.0` first: this solves and validates a plan without ROS or flight. An explicit `pdg mission --height 1.0 --execute` connects to ROS, checks the configured ground start, arms and takes off, verifies the takeoff endpoint, streams descent, relinquishes low-level priority with `notify_setpoints_stop`, and requests a nominal high-level landing. Disarm occurs only after a settled pose/velocity proxy near the configured ground and pad. This is not an independent contact sensor.

On stale pose, tracking error, or streaming delay, execution requests a best-effort high-level landing; it does not cut motors in the air. This cannot guarantee recovery during radio loss, localization loss, or propulsion faults. Upstream supervisor/watchdog behavior still applies. No custom onboard verified backup is implemented.

## Capability hook (disabled)

`pdg.capEnable=0` is the nominal default. The optional `pdg.maxCollective` cap acts on **legacy Mellinger collective-command units**, before stock mixing/battery compensation. It is not a limit in Newtons, not a cap on each motor, and not a calibrated effectiveness-loss injection. No individual rotor degradation is implemented. Calibration, torque-reserve analysis, and fault injection are separate future experiments.

Use the same Python minor version as the ROS installation. A practical Jazzy setup is `uv venv --python /usr/bin/python3 --system-site-packages .venv`, source `/opt/ros/jazzy/setup.bash` and the Crazyswarm workspace, then `uv pip install --python .venv/bin/python -e '.[dev]'`. The system-site-packages access is needed for apt-installed rclpy and generated messages. The dependency lock applies to host planning; ROS dependencies are managed by the ROS workspace/rosdep.

The nominal configuration uses a 1.8 s minimum descent time to avoid starting with the shortest physically admissible maneuver. The SCvx solver still optimizes flight time within its configured bounds. Do not infer minimum-time optimality independent of those bounds or a hardware tracking margin from this example.

For the prepared CF 2.1+ brushed / local Windows Vicon workstation, see [local setup](local-setup.md). ROS Jazzy launch_testing requires pytest below 9; the dev dependency is constrained accordingly.
