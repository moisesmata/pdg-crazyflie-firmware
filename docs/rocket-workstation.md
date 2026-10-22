# Rocket landing on this workstation

> The active launcher now selects the [2.75 m test](rocket-2p75m.md).
> The 2 m instructions below describe the previous setup. Use the new start
> position and plan for the next flight.

Repository: `/home/gravitas/workspaces/moises-experiments/pdg/pdg-crazyflie-firmware`.
The operator confirmed the same CF 2.1+ brushed aircraft, removed Flow/Z-ranger
deck, unchanged Vicon axes and pad, and clear route on 2026-10-21.

`config/rocket-lab.yaml` preserves the supplied 2 m example. The saved target
was independently revalidated into `output/rocket-setup-review`: both target
and mapped-drone validation passed. The rocket-powered phase maps to 13.398 s,
with peak speed 0.232 m/s. Takeoff, settling and landing bring the nominal
sequence to about 31.4 s, plus touchdown confirmation.

The aircraft climbs vertically from world XY **[-1, 0] m**, to about **2 m above
the pad**, then descends toward world XY [0, 0]. Keep the original Vicon origin.
The tracked body origin at contact is Z=0.014326675795018673 m.

## Start the server in terminal A

Use a **normal Ubuntu/WSL terminal with Crazyradio USB access**. This chat's
command environment has no `/dev/bus/usb`; no fresh hardware check or flight
can be completed here. Close other radio clients. Place the grounded aircraft
below world XY [-1, 0], stationary with motors stopped, and use a charged pack.

```bash
cd /home/gravitas/workspaces/moises-experiments/pdg/pdg-crazyflie-firmware
bash scripts/rocket.sh server
```

Leave that terminal running through touchdown. This probes firmware/self-test
and starts Vicon and the PID baseline server; it does not arm. The last recorded
radio probe identified the PDG app already installed. No reflash is needed for
the host rocket planner; the fresh startup probe verifies its API again.

## Complete ground commissioning in terminal B

```bash
cd /home/gravitas/workspaces/moises-experiments/pdg/pdg-crazyflie-firmware
bash scripts/rocket.sh check
```

This checks PID identity and fresh telemetry, audits continuity and Vicon-to-
estimator agreement, captures the stationary ground pose, prepares a hover at
that measured position, and applies the recorded Mellinger tracker settings
while unarmed and stopped. It finishes by checking project controller 6.
Any failure stops the sequence. The script expects a newly started PID server;
after a failed/repeated check with Mellinger already selected, restart terminal A.

The restored local `output/setup/pdg-tracker-candidate.json` uses recorded
`ctrlMel.mass=0.044` and `ctrlMel.massThrust=135456.8069962816`, Kalman, disabled
optical flow, disabled capability injection, and controller 0/auto. Gains are
unchanged. These values reproduce this aircraft's recorded effective tracker
calibration; they are not a new physical mass/thrust identification.
The previous rocket attempt aborted before arming because the stock mass was
0.029, whereas the mission expects 0.044. The diagnostic now honors an explicit
`--controller-id 6` in Vicon-only mode.

## Preview, then check the tracker with a hover

```bash
bash scripts/rocket.sh hover
bash scripts/rocket.sh hover --execute
```

Only the second command arms and performs the 0.6 m hover. Ground capture must
match the rocket start within 8 cm horizontally and the existing pad height
within 1 cm. A fresh matching radio probe can establish the installed firmware
identity when this checkout lacks the historical flash receipt; the mission
also checks live firmware and health before arming. No flash record is fabricated.
Confirm physical touchdown and stopped motors. Recharge or swap the battery
before the longer rocket mission. If power cycled or the server restarted,
repeat server and ground commissioning to reapply the tracker settings.

## Preview and run the rocket trajectory

```bash
bash scripts/rocket.sh review
bash scripts/rocket.sh fly
bash scripts/rocket.sh fly --execute
```

`review` and `fly` without `--execute` independently validate the saved rocket
target and mapped reference without connecting to ROS. The final command arms,
takes off, streams the descent and lands. Each attempt has a new UTC-stamped
output folder. Pose, start-state and tracking checks remain active. At the operator’s
request, `config/rocket-lab.yaml` now sets `minimum_flight_voltage: null`: the
host does not initiate landing for low airborne voltage. The host minimum launch voltage is also disabled with
`min_launch_voltage: null`, including the mission’s separate ground-voltage
minimum. Finite/fresh battery telemetry, normal supervisor state, tracking and
other mission checks and firmware battery protections remain active. A rejection requires investigation; do not use controller-trial mode
to bypass it.

From another terminal, request an early bounded landing with:

```bash
cd /home/gravitas/workspaces/moises-experiments/pdg/pdg-crazyflie-firmware
bash scripts/rocket.sh land
```

Keep tracking and radio running until physically grounded and motors stopped.
This requests landing; it is not an emergency motor cut or independent touchdown
confirmation.

## Software and provenance

The Python environment is installed. The ROS source revisions match `ros.repos`
and include the heading-preserving packet patch. `ros_ws/src` links to the
existing pinned source tree under Documents/Codex/2026-10-20/ple; build and
install outputs are local to this checkout. Keep that source tree available.
No firmware was flashed, aircraft parameter writes sent, arming or flight
performed by this setup session. Fresh hardware commissioning is pending USB
access from the normal WSL terminal.

The drone tracks scaled rocket translation. The validation is numerical
feedforward feasibility, not closed-loop flight validation or reproduction of
rocket mass depletion and attitude. See `docs/rocket-emulation.md` for the model.

## Verified setup results

All 10 ROS packages built successfully. Both local executables and their shared
libraries resolve after sourcing the project environment; the radio server
contains the heading-preserving patch. All 137 tests pass. Both target and
mapped-drone numerical validation passed.
