# pdg-crazyflie-firmware

Crazyflie firmware app and host software for powered-descent guidance (PDG)
experiments using Crazyswarm2 and Vicon.

## What runs where

- **Host:** solve a virtual rocket PDG problem; scale its position, velocity and
  acceleration into the lab; stream full-state references through Crazyswarm2.
- **Crazyflie:** estimate state and track references using the onboard Mellinger
  controller wrapped by the project firmware app.
- **Vicon:** provide measured pose to the onboard estimator and host experiment.

The default planner uses the mass-depleting, gimbaled rocket model from Szmuk,
Reynolds & Açıkmeşe (2020). The drone emulates scaled **translation**; its attitude
is derived from the force required under Earth gravity. See
[rocket model, scaling and limitations](docs/rocket-emulation.md).

## Hardware workstation setup

Run the following from the repository root on the **Linux/Ubuntu/WSL ROS
workstation** with Crazyradio access. Do not use a Mac Python environment for ROS.
The prepared setup uses ROS 2 Jazzy, Cyclone DDS and ROS domain 45.

For a new workstation, follow [local setup](docs/local-setup.md) to install host
and ROS dependencies, import `ros.repos`, apply `scripts/patch_heading_commands.py`,
and build the ROS workspace. The heading patch must be applied before building.
Install the matching firmware app: `cf2` for CF 2.x/2.1+ brushed; `cf21bl` for
CF 2.1 Brushless. `make firmware PLATFORM=cf2` builds without flashing; follow
[hardware integration](docs/hardware.md) for flashing and frame setup. A host-only
planner change does not require reflashing an already compatible app.

Check these files against the current aircraft and room:

| File | Values to verify |
| --- | --- |
| `config/crazyflies.yaml` | Radio URI, robot name and logging |
| `config/motion_capture.yaml` | Vicon hostname, subject and stream |
| `config/tracker-frame.yaml`, `config/body-frame.yaml` | Physical body axes and selected frame correction |
| Your rocket mission YAML | Landing position, ground body-origin height, lab scales, workspace, drone parameters and expected tracker settings |

The supplied setup names the robot and Vicon subject `Crazyflie` in frame
`world`. Body axes, metre units and timestamps must agree throughout the path.
After changing markers or Tracker object axes, repeat frame validation and ground
calibration. Do not apply both a Tracker-axis correction and the old host correction.
The checked-in 44 g effective drone mass and motor-force limits are assumptions,
not independent identification of your aircraft.

## 1. Prepare and validate the trajectory before flight

Copy the example so that room-specific changes remain explicit:

```bash
source scripts/local_env.sh
cp config/rocket-emulation.yaml config/rocket-lab.yaml
# Edit config/rocket-lab.yaml for the current aircraft, pad and room.
pdg plan --config config/rocket-lab.yaml --output output/rocket-prepared
```

This command sends no flight commands. Both `target-validation.json` and
`emulation-validation.json` must report `passed: true`. Inspect
`target-trajectory.png` and `lab-reference.png`. The solver can take several
minutes; prepare the plan before connecting the flight battery.

The unchanged example maps to approximately **2 m above the pad**, starting at
world XY **[-1, 0] m**, with landing near **[0, 0] m**. Its checked descent lasts
about 13.4 s; takeoff, settling and landing add about 18 s. Account for the whole
sequence when assessing battery endurance. This new rocket reference has passed
numerical validation but has not been flown. Archived successful 2 m runs used
the legacy quadrotor planner.

For a smaller first trial, change `emulation.length_scale_m_per_target_unit`
in your lab YAML and rerun validation. Keep the target rocket problem separate
from the lab mapping. Changing time scale also changes speed, acceleration and
endurance. `emulation.landing_position_enu_m[2]` must equal
`execution.ground_body_height`: the tracked body's height at ground contact.

## 2. Start tracking and radio — terminal A

Close cfclient or any other process using the same radio. For the prepared
**Vicon-only, Flow/Z-ranger-deck-removed** setup:

```bash
source scripts/local_env.sh
bash scripts/start_ground_checks.sh --vicon-only
```

This probes the aircraft and starts the receiver/radio server without arming or
taking off. It initially selects **stock PID**, not the PDG Mellinger tracker.
Leave this terminal running. Run only one receiver and one command-producing
mission for the aircraft. OPT4SMART is an optional alternate integration, not a
second concurrent external-pose source in this workflow.

## 3. Verify tracking and configure the tracker — terminal B

With the aircraft stationary, unarmed and motors stopped:

```bash
source scripts/local_env.sh
python scripts/check_hardware.py --vicon-only --controller-id 1
python scripts/check_stream_continuity.py --seconds 30
python scripts/check_mocap_path.py --seconds 30 --output output/rocket-ground-audit
```

Inspect the reports for fresh external/onboard pose, continuous orientation,
correct frames, unarmed supervisor and zero motor commands. These helper scripts
use the prepared `Crazyflie`/`world` setup; they do not automatically inherit every
field from `rocket-lab.yaml`.

The current workstation's tracker helper uses the local, ignored
`output/setup/pdg-tracker-candidate.json`. It must exist and contain reviewed
settings for this aircraft; it is **not supplied by cloning the repository**.
Do not copy unknown gains or mass/thrust calibration from another aircraft.
If that calibration artifact is available:

```bash
python scripts/configure_pdg_tracker.py         # Preview parameter changes
python scripts/configure_pdg_tracker.py --apply # Apply while unarmed; no flight
python scripts/check_hardware.py --vicon-only --controller-id 6
```

The mission's `execution.expected_tracker_parameters` must match those settings.
Reapply and verify after a battery power cycle or server restart. Selecting stock
Mellinger directly as controller 2 bypasses the project wrapper; the project path
uses controller 0/auto and reports project controller ID 6.

Before the rocket test, check the tracker with the prepared low-altitude hover:

```bash
python scripts/hover_check.py --config config/pdg-mellinger-hover.yaml --vicon-only --controller mellinger --height 0.6
```

This is a preview. Once the current ground calibration, physical clearance,
battery and tracker have been checked, append `--execute` to perform that hover.
This helper requires the local calibration/flash reports described in
[bring-up notes](docs/flight-today.md). A fresh clone is not immediately flight-ready.

## 4. Replay the rocket trajectory on hardware

First revalidate the saved target with the exact lab configuration:

```bash
pdg mission --config config/rocket-lab.yaml \
  --plan-file output/rocket-prepared/target-plan.json \
  --output output/rocket-flight-review
```

Without `--execute`, this does not connect to ROS or send flight commands.
The loader checks target model, units and solver provenance, then reruns target
and mapped-drone validation. Use **`target-plan.json`**, not the derived
`flight-reference.json`. Changes to the rocket problem require a fresh solve.

For `stage_to_start: false`, place the aircraft on the ground **directly below
the mapped initial XY**; it climbs vertically there. The example starts near
[-1, 0] m and descends toward the origin. Do not re-zero Vicon to move the start.
The room must contain the complete mapped path and landing continuation.

After successful tracker checkout, with a charged battery and the flight area
clear, the following command **arms, takes off, streams the descent and lands**:

```bash
pdg mission --config config/rocket-lab.yaml \
  --plan-file output/rocket-prepared/target-plan.json \
  --output "output/rocket-flight-$(date -u +%Y%m%dT%H%M%SZ)" \
  --execute
```

Use a fresh output directory for each attempt. The ordinary mode retains pose,
start-state, tracking-error, deadline and voltage checks. **Do not add
`--controller-trial` as a routine workaround:** that experimental mode disables
host airborne voltage and descent tracking-error aborts. A failed preflight or
tracking check needs investigation.

## Landing and reviewing a run

Keep the tracking/radio server running through flight and touchdown. From another
terminal, this requests an early high-level landing:

```bash
bash scripts/land_now.sh
```

It is a landing request, not an emergency motor cut or independent touchdown
confirmation. Stopping the server is not a landing command. A best-effort landing
can fail during radio, localization or propulsion loss. Verify the aircraft is
physically grounded and motors stopped after any interrupted attempt.

Each flight directory contains the target plan, validation reports, mapped
reference, execution configuration and `mission.jsonl`. Compare:

- `sample`: measured versus commanded lab position.
- `rocket_emulation_sample`: inverse-mapped measured position/velocity versus the
  virtual rocket reference; virtual mass and attitude remain reference quantities.
- Start checks, telemetry and landing/recovery events: distinguish a completed
  reference stream from a confirmed landing and disarm.

These hardware experiments evaluate tracking of scaled rocket translation.
Closed-loop virtual-rocket replanning and fault-recovery guarantees are future work.

## Software regression tests and legacy experiments

```bash
source scripts/local_env.sh
python -m pytest -q
```

Tests do not arm, fly or flash. The native heading-packet test requires the pinned
Crazyswarm2 source headers in `ros_ws/src`; a checkout without that dependency
fails that test even when the Python tests pass.

Select a legacy quadrotor configuration explicitly to reproduce old experiments,
for example `pdg plan --config config/nominal.yaml`. See
[validation evidence](docs/validation.md), [hardware details](docs/hardware.md)
and [historical trial notes](docs/pdg-trial.md) for additional context.
