# Prepared PDG trial

The requested mission takes off near the landing origin, stages sideways, and
starts powered descent at **(+1 m X, 0 m Y, 2.5 m height)**, ending at **(0,0,0)**
in a landing frame. The operator confirmed 2.8 m overhead clearance and 0.5 m
around the route, and chose sideways staging rather than moving the ground start.

Vicon's tracked body origin is 0.0143267 m above the landing surface at contact.
Accordingly, the world-coordinate descent starts at `[1,0,2.5143267]` and ends at
`[0,0,0.0143267]`. Tracker's world/object axes must remain unchanged. This offset
makes the physical landing surface the operator's Z=0 without commanding the
tracked body through the floor.

The accepted offline configuration is `config/pdg-2p5m-1m-candidate.yaml`.
The accepted plan, independent rollout plot and mission overview are under
`output/pdg-2p5m-1m-prepared/`. It converged in 14 iterations to a seven-second
descent and passed independent dense nonlinear validation. This is a numerical
check, not a continuous-time or hardware guarantee. Earlier five-second solves
were rejected and retained in their original output directories. The accepted
solve uses 21 nodes, a seven-second minimum duration, and an explicit normalized
control-change penalty of 1.0; the optimizer's default objective remains unchanged.

| Phase | Configured duration |
|---|---:|
| Vertical takeoff near origin | 8 s |
| Settle at height | 2 s |
| Stage sideways to +1 m X | 6 s |
| Settle at descent start | 2 s |
| Optimized PDG descent to origin | 7 s |
| Bounded normal landing | 8 s |

The nominal sequence is about 33 seconds plus ground confirmation and service
latency. Battery endurance for that sequence has **not** been demonstrated. The
completed PID controller trial lasted 18.84 seconds and its minimum logged loaded
voltage was 3.362 V, starting around 3.916 V. It is not evidence of adequate reserve
for the larger mission. A complete charge, rather than a partially charged swap,
is needed for the next hardware comparison.

## Tracker and model

The successful baseline was stock PID, Kalman, Vicon full pose and onboard IMU,
with Flow deck removed. This repository's PDG path uses a host SCvx optimizer and
an onboard out-of-tree wrapper around stock Mellinger. The optimizer does not run
on the drone, and PDG is not a replacement state estimator.

During the PID hold, median collective was 58,468.6. Using the installed CF 2.1+
normalization of 0.12 N per rotor gives an effective hover mass estimate of
43.65 g. Ten quasi-static samples and no saturated motor samples were used.
This is **not** a scale measurement or a thrust-stand calibration; inertia also
remains illustrative. The Mellinger snapshot still had mass 29 g and gain 132,000,
which would give only 37,552.7 collective at hover before position feedback.

The provisional tracker uses `ctrlMel.mass=0.044` and
`ctrlMel.massThrust=135456.807`, matching the observed hover command. These values
are prepared in `output/setup/pdg-tracker-candidate.json`; they have not been
applied or flown. The larger plan uses a separate **assumed 48 g upper case** for
rotor-force constraints. Because the planner's translational input is specific
thrust, this is a planning headroom assumption, not a claim that actual mass is
48 g. Neither assumption replaces hardware verification.

The separate smaller candidate `config/pdg-candidate.yaml` descends vertically
from 60 cm to 20 cm before normal landing. Separate numerical feasibility solves
passed at assumed masses 40, 44 and 48 g. This sweep is not a proof of robustness
to an unknown plant or to model errors in closed-loop flight.

## Next hardware step

First commission the provisional Mellinger tracker at 60 cm. No firmware flash
is needed. Start the familiar Vicon-only server with a fully charged battery,
level drone on the measured pad, Flow deck removed, and markers unchanged:

```bash
bash scripts/start_ground_checks.sh --vicon-only
```

That launcher deliberately starts in the proven PID configuration. In a separate
terminal with the same project environment, preview and then apply the prepared
tracker settings while the drone is unarmed with four zero motor commands:

```bash
source scripts/local_env.sh
python scripts/configure_pdg_tracker.py
python scripts/configure_pdg_tracker.py --apply
```

The helper checks fresh external/onboard pose and supervisor status, battery
operation, pad height, stopped motors and an unarmed state. It configures the
sensor/feedforward values before selecting the project tracker, and records the
original values and acknowledged changes. It never arms or takes off. Parameter
acknowledgements and the ROS mirror are not independent hardware readbacks.

Preview the Mellinger checkout:

```bash
python scripts/hover_check.py --config config/pdg-mellinger-hover.yaml --vicon-only --controller mellinger --height 0.6
```

Only after physical readiness, append `--execute`. The hover checks the expected
mass/gain so a stale 29 g setup cannot pass silently. The ordinary `--vicon-only`
command without an explicit controller still means PID. The existing
`--controller-trial` hover mode remains restricted to PID. For a full PDG mission,
the explicit flag accepts the calibrated Vicon-only Mellinger setup and omits host
voltage and descent tracking-error aborts. It retains startup, telemetry freshness,
staging readiness, firmware checks and bounded landing supervision.

After any battery power cycle or server restart, apply the tracker configuration
again; do not assume prior runtime settings or controller selection survived.
The next full mission also checks the expected tracker parameters.

## Reuse the reviewed plan

This command revalidates the accepted plan without solving or connecting to ROS:

```bash
pdg mission --config config/pdg-2p5m-1m-candidate.yaml --plan-file output/pdg-2p5m-1m-prepared/plan.json --output output/pdg-2p5m-1m-review
```

After Mellinger checkout and an endurance decision, actual execution additionally
requires `--execute` and a fresh output directory. It should not be run while the
tracker/endurance items are unresolved. Saved-plan loading checks model/problem,
coordinates/units, array shapes, node dynamics and a fresh independent rollout.
It avoids re-solving while the flight battery is connected.

Keep the tracking/radio server running throughout flight and landing. The staging
GoTo holds its endpoint while the mission verifies the initial state; normal
completion then streams the PDG reference. On an exception the mission queues a
bounded landing and supervises recovery. A recovery landing is vertical near the
current position, not a return-to-origin maneuver. Server loss can prevent new
commands; stopping the server does not itself stop an onboard command.

## Repository changes relevant to the larger trial

- Sideways staging is explicit; the old mission could only take off vertically at
  its planned initial XY position.
- GoTo yaw is radians in this pinned C++ backend and firmware, despite the .srv
  comment saying degrees. The request builder has a unit regression test.
- The bounded landing is queued before low-level stream priority is released,
  avoiding the prior idle high-level motor-off window at handoff.
- PDG recovery now monitors for fresh settled ground state before disarming and
  reports unconfirmed recovery on persistent telemetry loss.
- The former indefinite GoTo landing continuation remains removed.
- Optional control smoothing addresses the oscillation seen on the larger solve;
  nonconverged or independently invalid solutions remain rejected.


## Latest hardware outcome and negative-X preparation

The 60 cm Mellinger checkout completed its hold with 0.591–0.593 m height
above the pad and under 3.9 cm lateral deviation; landing/disarming was confirmed.
The operator reported a very stable flight.

The subsequent +X full-sequence attempt reached 2.5 m and staged sideways, but
failed its staging readiness check before any PDG descent setpoint was sent.
Firmware received-pose logs showed approximately 93° and 172° orientation
changes in successive 95 ms intervals before the loss of control. Raw ROS mocap
was not recorded during staging, so this does not independently identify Tracker
as the source. The battery also dropped to 3.36 V under load. Final motor commands
were zero, but the host could not confirm disarming; physical state must be checked.
See `output/pdg-trial-20261021T102223Z/result.json`.

The requested new start is [-1, 0, 2.5] m in the landing frame.
`config/pdg-2p5m-minus1m-candidate.yaml` and
`output/pdg-2p5m-minus1m-prepared/plan.json` contain the new independently solved
7 s descent, with the same model and limits. The solver converged in 14 iterations
and the independent nonlinear rollout passed. This is software preparation, not
hardware clearance. Next investigate the orientation discontinuities while unarmed,
inspect the aircraft after the loss, fit a fresh battery, and reapply tracker settings.

Every mission now logs external pose inputs during takeoff, staging and descent,
including timestamp-rejected samples. The read-only `scripts/check_mocap_path.py`
also stores source, received and estimated quaternions, and flags large consecutive
orientation changes. Use `--seconds 30 --output output/<unique-audit-name>` to
preserve evidence. No frame correction or pose filter has been added.


Negative-X hardware trial: `output/pdg-minusx-trial-20261021T103743Z`. Ground pose-path audit passed with 317 paired samples; no orientation jumps. The full attempt again failed staging before PDG descent. This time the upstream pose stayed continuous, while gyro measurements confirmed a real spin. Motor 3 was repeatedly saturated, with loaded battery voltage about 3.4 V. The aircraft spiraled down and flipped (operator observation). Final ground-height proxy, disarming and zero motor commands were confirmed by telemetry. This supports investigating propulsion/yaw authority and battery condition; it does not establish one failed component. Repeated high-altitude attempts and a change of staging direction have not resolved the issue.


Requested 30-second Mellinger hold at 60 cm: `output/hover-20261021T104518Z`. The new configurable hold passed 41 hover/execution tests, including early recovery during the long hold. Hardware reached its hover, but the unchanged 3.50 V host landing trigger fired at 8.57 seconds after takeoff and 2.56 seconds into hold. Landing/disarming and zero motors were confirmed. This is a threshold-limited run, not evidence that the battery was fully depleted or a completed 30-second hold. Per-motor balance and height results are saved in result.json.
