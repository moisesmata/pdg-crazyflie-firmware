# Physical bring-up: CF 2.1+ brushed

Run hardware commands in a normal Ubuntu/WSL terminal, from this repository, with `source scripts/local_env.sh` sourced. The assistant sandbox cannot open the Crazyradio.

## Completed

- Discovered and saved the real URI: `radio://0/80/2M` (default E7E7E7E7E7 address).
- Read the device: stock/non-PDG parameter set, PID controller selected, Kalman estimator selected, high-level commands disabled.
- The initial battery measurement was only 3.336 V. That reading is below this project's 3.7 V critical threshold. A charged battery is required; the operator has reported charging/swapping it.
- Verified that the built flash binary matches the checked ELF using upstream's exact objcopy section exclusions. Flash SHA-256: `d54c71148710361835f92b0bdd448665b94c74755d3986595e82f2df4bff2928`.
- Captured 1,208 stable Vicon ground-position samples near `[0.0197364, 0.0918443, 1.0159466]` m, with under 1 mm range in each axis. The operator confirmed the drone was level on its intended takeoff/landing surface.
- That capture's Vicon attitude was approximately roll -6.65°, pitch +5.56°, yaw +173.58°. Do not assume those marker axes coincide with the vehicle axes. The radio IMU simultaneously reported roll about -0.8° and pitch about +0.9° before external-pose forwarding was running.

## Flash and verify

With propellers removed and a charged battery fitted:

```bash
python scripts/flash_cf2.py --flash --props-removed
```

The helper refreshes the read-only probe and requires measured voltage >= 3.8 V, verifies the manifest hashes, warm-boots the discovered URI, checks CF2 bootloader protocol, flashes only the STM32 firmware, reboots, and reads the PDG API parameters. It does not arm or fly. Reports go to `output/setup/drone-probe.json` and `output/setup/flash-result.json`.

Without `--flash`, the helper only checks and prints the planned image/target. The completed flash was verified: `flash-result.json` reports both `flash_complete` and `identity_verified`. Post-flash values are PDG API 1, controller ID 6, capability disabled, and battery approximately 4.13 V.

## Coordinate calibration

Verify the Vicon rigid-body +X axis points through the physical nose and +Z is the vehicle up axis. Correct the object axes in Vicon if needed. For the current planner's initial heading convention, place the vehicle with its nose along world +X. Repeat a stationary measurement:

```bash
python scripts/calibrate_ground.py --receiver --seconds 10
```

Do not use `--receiver` if the project receiver is already running. The script reads poses and writes a report; it issues no radio or motor commands. It rejects missing/stale/wrong-frame data, motion over 1 cm, and orientation more than 5° from the planner's initial identity orientation. These checks do not establish physical body-frame alignment on their own.

The total mass with five markers (four peripheral 14 mm / 17 mm-base markers and one top-center marker) is currently unmeasured. No marker mass was verified from the supplied seller website. The illustrative 32 g model remains uncalibrated. The stationary pose must not be treated as a mass or thrust calibration.

After obtaining an actual mass, `--mass-grams N --write-candidate` creates `config/flight-today.yaml` for a 0.3 m vertical mission over the measured pad. It does not replace the active nominal file or command flight. Room clearance and physical frame agreement must be checked before executing any candidate.

## Next flight stage

After firmware verification and corrected coordinate alignment, run the project launch and the read-only hardware check. The prepared `scripts/hover_check.py` previews a 20 cm high-level takeoff, 2 s hold, and landing. Only `--execute` sends motor/flight commands. It uses `config/hover-today.yaml`, which is created after accepted ground calibration. The transport requires charged-battery status, an unarmed ready supervisor, fresh tracking and agreement between onboard estimator and Vicon before arming. Live pose/status limits trigger a best-effort landing request. This is not a verified recovery controller; radio or localization loss can prevent recovery. Optimized PDG descent remains a later step because its vehicle mass is unmeasured. No motor command or takeoff has been sent in this session.


## Operator landing command

Keep the radio/Vicon server running during the flight. To request an early landing from another WSL terminal, run:

```bash
bash /mnt/c/Users/gravitas/Documents/Codex/2026-10-20/ple/moises-experiments/pdg-crazyflie-firmware/scripts/land_now.sh
```

This requests a 3 s high-level landing; it does not cut airborne motors and does not independently confirm touchdown. Stopping the tracking/radio server is not a landing command.

Bring-up also reproduced a host interoperability issue: cflib returns a shorthand URI (`radio://0/80/2M`), whereas the pinned C++ server's broadcast-URI parser requires the address suffix. The probe now expands the standard address, the launcher normalizes it, and a regression test covers this case. A legacy mocap-enabled flag is also derived from the vendor-tracking configuration for the C++ server's no-data warning.

Hover startup evaluates vertical tilt and yaw separately, each capped at 5 degrees, so a combination of small acceptable tilt and heading errors is not treated as excessive tilt. The measured pad is in `config/hover-today.yaml`; the PDG mass remains unmeasured.


## PID diagnostic baseline (2026-10-21)

The operator reports successful prior command-based flight with the same five markers/mounts,
a Flow deck, the Kalman estimator, and stock PID. The corrected-frame Mellinger attempt
aborted on yaw change, after about 5.5 cm rise and 3 cm lateral displacement. It recovered
and confirmed disarmed. This does not establish whether controller tuning or actuator
headroom caused the remaining problem.

For a controlled comparison, keep the corrected Vicon pose and Kalman estimator and start:

```bash
bash scripts/start_ground_checks.sh --baseline-pid
```

This selects stock PID (`stabilizer.controller=1`) already present in the verified image;
it does not flash, arm, or fly. Without the flag, the wrapper selects the project controller
again (`controller=0`, auto OOT/Mellinger). Each launch logs four individual motor commands,
gyro yaw rate, and onboard yaw. These commands are PWM ratios, not measured motor RPM.
The startup probe now records Flow deck initialization and `motion.disable` when available.
No deck input is disabled by this diagnostic mode.

Read-only verification and preview:

```bash
source scripts/local_env.sh
python scripts/check_hardware.py --controller-id 1
python scripts/hover_check.py --controller pid
```

Only after physical readiness and fresh ground checks, `--execute` on the hover command
performs the guarded 20 cm hover. The explicit PID exception applies only to this diagnostic
hover; optimized PDG missions still require the project controller. Battery, tracking,
heading, supervisor and recovery checks stay active. Mass remains unmeasured.


Live PID startup confirmed `CONTROLLER: Using PID (1) controller`. After a ground
power-cycle, the read-only check passed with supervisor 513, fresh corrected Vicon,
and consistent onboard pose. The fresh startup probe reported `bcFlow2=1`,
`bcZRanger2=1`, and `motion.disable=0`: the Kalman estimator has Vicon full pose,
IMU, and enabled deck inputs. The first PID attempt was not armed because ground
voltage was 3.953 V, below the existing 4.00 V launch minimum.


## PID attempt and telemetry interruption

The charged-battery PID attempt `output/hover-20261021T073201Z` started at 4.133 V.
It aborted at 3.174 seconds for Vicon tilt over 15 degrees. The operator observed
rotation with little tipping/sliding. The last recorded rise was about 1.5 cm;
recorded individual motor commands peaked at 52020/65535, below saturation.
All onboard telemetry ceased near 2.63 seconds after takeoff. Recovery requested
landing but could not confirm touchdown because status became stale. Later the
server console reported supervisor disarming; this does not replace physical
confirmation that motors stopped. No further flight was attempted.

Local firmware source `log.c` deletes all log blocks when `crtpIsConnected()` is
false. Radio inactivity timeout defaults to 1000 ms. The pinned server sends
latency probes at its warnings frequency, originally 1 Hz. This timing has no
margin when external pose packets are absent. It is a plausible mechanism for
persistent loss of logs; it does not prove the cause of the interruption.
The launch now sets diagnostics/probes to 10 Hz (the firmware timeout is unchanged).
A runtime onboard-pose watchdog requests landing after 0.25 seconds without a
pose update, in addition to existing external tracking and battery guards.

After an unarmed server restart, `python scripts/check_stream_continuity.py`
records 30 seconds of Vicon, onboard pose, supervisor, and individual motor logs.
It sends no parameters or flight commands, requires unarmed status and zero motor
commands, and fails on gaps over 0.25 seconds (2.5 seconds for 1 Hz status).
Its report and samples are in `output/setup/stream-continuity*.json`.


## Marker layout update

The operator moved the top marker slightly toward the nose and updated the
Tracker model. The previous frame calibration/configuration and raw captures
were archived under `output/setup/before-marker-change-20261021T074108Z/`.
The old correction is disabled pending a fresh three-pose fit plus independent
nose-down validation. Explicit hover execution now also validates the body-frame
configuration before importing/starting the ROS mission.

The first continuity report included startup discovery delays, not sustained
in-stream gaps: actual maximum adjacent gaps were about 18 ms Vicon, 64 ms
onboard pose, and 86 ms actuators. The monitor now waits up to 15 seconds for
all streams before beginning its 30-second interval. A subsequent test saw all
streams stop together partway through, during the operator's setup/model changes;
it did not establish 30 seconds of continuity and must be repeated after setup.


## Requested comparison: native Tracker full pose, IMU, Kalman, PID

The operator chose to remove the Flow deck physically. In this pinned firmware,
`motion.disable=1` disables optical flow but does not disable the range sensor.
The new startup flag `--vicon-only` therefore refuses an initialized Flow/Z-ranger
deck, selects PID (1), Kalman (2), and sets `motion.disable=1`. It needs no reflash.
Physical marker geometry must remain fixed when the deck is removed.

For Tracker-native alignment, with the drone level and its physical nose along
Global +X, pause the stream, select Crazyflie, and use Object Manipulator /
Alignment Tool: primary local +X to Global X, secondary local +Z to Global Z,
then Align and resume. The global origin is unchanged. Tracker 4.4 guide pages
200–201 document these alignment controls:
https://help.vicon.com/download/attachments/376309877/Vicon%20Tracker%20User%20Guide.pdf

Start `bash scripts/start_ground_checks.sh --vicon-only` after removing the deck
with power off and completing alignment. Ground checks use
`python scripts/check_hardware.py --vicon-only`. The hover preview is
`python scripts/hover_check.py --vicon-only`; only `--execute` performs flight.
This explicitly selected native-axis mode requires the operator's alignment
confirmation recorded in `config/tracker-frame.yaml`, with the old body rotation
disabled. Fresh onboard/external attitude agreement, charged battery, tracking,
and supervisor checks still apply. Its confirmation is currently false, awaiting
the operator's completed alignment; a fresh pad/heading capture is also required.


Native Tracker alignment is now operator-confirmed. With the Flow deck removed,
the live Vicon-only check passed (PID 1, Kalman 2, motion.disable 1, all three
deck initialization flags 0), battery 4.101 V and unarmed supervisor 513.
The fresh pad is [0.0142974351, 0.0032781411, 0.0143266758] m;
attitude is approximately [-1.8843, 0.5252, 1.4234] degrees RPY.
The 30-second unarmed continuity test passed: largest Vicon gap 17.8 ms,
onboard gap 77.8 ms, actuator gap 81.0 ms, status gap 983.6 ms; motors zero.
The native-axis confirmation and new hover pad/heading are saved. Physical
props-on readiness and a fresh preflight are pending before flight.


The first native-frame Vicon-only PID attempt is `output/hover-20261021T082104Z`.
It started at 4.090 V and aborted at about 2.59 seconds because orientation change
(including yaw) exceeded 15 degrees. The operator observed clean lift and rotation.
Telemetry stayed live; recovery landing and disarm (supervisor 513) were confirmed.
Vicon and onboard yaw both increased, reaching about 7.6 degrees by 2.39 seconds,
while gyro Z was about +52 deg/s. At that point M1/M3 commands were about
55944/57517 and M2/M4 about 29326/38272. These are commands, not measured thrust
or RPM; they do not prove a propeller/motor fault. The logged Vicon rise before
abort was only 4.4 mm; do not equate that log with the operator's perceived rise.

For the next diagnostic restart, a 20 Hz yaw_control log block records PID desired
yaw, desired yaw rate, attitude/rate proportional outputs, measured yaw and gyro Z.
This will distinguish a changing heading target from insufficient/opposite yaw
response. No further flight was commanded while preparing this logging.


## One-meter target and host packet handling

The operator requested a 1 m target and confirmed 1.3 m vertical clearance plus
0.5 m horizontal clearance, standing clear. `hover_check.py --height 1.0` keeps
the four-second ascent, two-second hold, existing yaw/tilt/drift/battery guards,
and four-second landing. Height validation now allows 0.10 to 1.00 m.

Attempt `output/hover-20261021T083021Z` started near 4.027 V and requested landing
at 1.60 seconds after a stale external-pose source timestamp. Onboard/motor/yaw
telemetry gaps were under 57 ms before abort; desired yaw stayed near 1.781 degrees.
The host had incorrectly latched one stale packet as a permanent fault, preventing
its recovery monitor from accepting subsequent fresh poses. The operator confirmed
the drone grounded with motors stopped; a subsequent live check read supervisor 513.

`PoseStream` now rejects delayed/duplicate/future packets without refreshing the
pose watchdog. Accepted source age counts against the same 150 ms watchdog;
wrong frames and invalid quaternions remain fatal. Recovery supervises for its
existing timeout while waiting for valid telemetry to return, and never confirms
landing/disarms blindly with persistent tracking loss. 64 targeted tests pass,
including delayed-packet recovery, genuine starvation, and persistent-loss cases.

Radio packet counts/latency were added to mission status logs. A later read-only
radio audit received no status messages, so no new latency conclusion is possible
until server telemetry is restarted. Tracker's normal 126 Hz stream exceeded the
upstream 120 Hz rate-warning threshold; the diagnostic range is now 80–180 Hz to
reduce false warning traffic. The flight freshness limit remains unchanged.


After the charged-battery restart, the Vicon-only ground check passed at 4.143 V,
supervisor 513. A 30-second continuity test passed (Vicon max gap 9.4 ms,
onboard pose 75.8 ms, actuators 70.5 ms). The 10-second radio audit received 11
status samples: reported latency median 5 ms, range 4–10 ms, and maximum ROS
status delivery age 0.83 ms. Counter totals were UC RX/TX 8327/8484 and BC
1302/1319; these approximately matched intervals do not establish an exact
packet-loss rate. These are ground results, not proof of behavior under motor load.
The next 1 m attempt awaits the operator standing clear; no flight was sent during
these measurements.

## Received-pose audit and vertical control diagnostic

The post-reference-review unarmed ground audit passed with 106 paired samples.
Firmware input age was at most 17 ms; received pose matched host Vicon within
1.005 mm / 0.097 degrees, and Kalman matched host Vicon within 0.793 mm /
2.216 degrees. This verifies the stationary full-pose path, not airborne behavior.
No takeoff or arming command was sent during this check.

The last 1 m attempt (`output/hover-20261021T084231Z`) aborted at 1.877 s on
8 cm lateral displacement. It later reached approximately 0.421 m rise during
recovery, with touchdown/disarm verified by a subsequent fresh ground check.
Desired yaw stayed near -6.21 degrees while actual yaw departed from it;
the controller commanded opposing yaw correction. Radio latency was generally
4–6 ms, with one 19 ms sample. This does not establish a mechanical root cause.

The installed build correctly selects CF 2.1+ propellers and battery compensation.
Read-only PID parameters show the stock thrust base of 36000. Offline inversion
of the battery-compensation curve suggests substantially higher collective near
liftoff, but uses asynchronous PWM and 1 Hz voltage logs; it is not a calibrated
thrust or mass measurement. Two logged motor outputs reached 65535 after 5.25 s,
during recovery, not at initial liftoff. Battery voltage dropped from 4.127 V
before takeoff to approximately 3.60 V under load. These observations motivate
checking control demand and actuator headroom rather than changing estimator code.

A new 5 Hz, 24-byte `vertical_control` log contains raw PID collective, desired
height/vertical speed, estimated height/vertical speed, and battery voltage.
Names/types were verified against the installed log TOC. The mission transport
records it alongside yaw and motor outputs. It requires a server restart, not a
firmware flash. No gains, thrust-base settings, or flight limits were changed.

## Landing incident and revised hover envelope (2026-10-21)

The 09:09:39 UTC attempt aborted at 1.902 s on 3.483 V battery status.
The maximum estimated rise during recovery was 0.423 m. Telemetry and operator
confirmed touchdown and stopped motors. Timed Land ended while estimated height
was still 0.061 m (approximately 0.046 m above the measured pad), explaining the
operator's observed abrupt stop near the ground.

A host change replaced the end of timed Land with GoTo at the pad target. Although
unit tests and a native planner check confirmed that it avoided the timed motor
stop, its indefinite position hold was inappropriate on monitor timeout or server
loss. The 09:18:37 UTC attempt aborted on 8 cm lateral displacement at 2.033 s,
reached an estimated 0.535 m rise during recovery, and timed out without confirming
touchdown. The operator stopped the server and reported motors continuing on the
ground, later confirming that they stopped. The assistant could not command
disarm after the server stopped. This GoTo landing change was retracted. It must
not be restored as a landing fix; simulator/unit success did not cover this failure.

At the operator's request to allow PID to settle, the active hover profile now
allows 0.30 m lateral displacement, 25 degrees tilt, and no yaw/full-attitude-change
abort. Four seconds of settling follow the four-second ascent; endpoint position
tolerance is 0.20 m. The one-metre height target and existing vertical envelope,
tracking freshness, battery checks, initial alignment, and observed-touchdown
disarming remain. No PID gains, mass, thrust base, or estimator weights changed.

Landing uses the onboard bounded Land command over eight seconds, monitored for
an additional three seconds. It does not command an indefinite GoTo. This slower
trajectory reduces descent demand but does not make timed landing contact-aware:
the firmware still ends its landing trajectory on a timer. Ground confirmation
remains necessary; never assume a mission exception or stopping the server stops
an already active onboard command. Both attempt logs and result reports remain
in their respective output/hover-* directories.

The wider-envelope attempt at 09:28:40 UTC reached 1.154 m above the measured
pad (onboard peak), corroborated by received mocap (1.154 m). Maximum onboard
horizontal deviation was 0.0452 m. The operator reported smooth climb and descent.
The former 8 cm vertical overshoot allowance triggered recovery at 3.612 s.
The eight-second bounded landing finished with fresh ground/disarmed status 513
and all motor commands zero. A sustained hover was not demonstrated.

For a subsequent attempt, vertical overshoot allowance is now 0.20 m, giving a
1.20 m maximum height above the pad for a 1 m target, within the operator's
confirmed 1.30 m overhead clearance. The lower height bound, 30 cm lateral bound,
25 degree tilt bound, battery/freshness checks and eight-second bounded landing
remain. PID gains and thrust base are unchanged. Tests exercise 15 cm overshoot
acceptance and rejection beyond 20 cm. No flight has yet used this vertical allowance.

The 09:33:06 UTC attempt entered the hover phase. During seconds 4.5–8.5,
188 host samples stayed between 1.078 and 1.146 m above the pad, with at most
2.99 cm lateral deviation in that interval (6.14 cm maximum over the pre-abort
flight). Peak rise was 1.154 m. Battery voltage 3.494 V triggered landing at
8.528 s, after 0.514 s of the explicitly labelled two-second hold. Telemetry
confirmed landing, supervisor 513 and four zero motor commands. The operator
reported the hover looked okay and confirmed stopped on the ground.
Received-pose age was at most 11 ms; radio latency ranged 4–15 ms.

For the operator-requested repeat, settling is reduced from four to two seconds
based on the measured steady interval; ascent remains four seconds and hold two
seconds. This starts nominal landing at eight rather than ten seconds to preserve
battery reserve. Gains, envelope, voltage limits and bounded eight-second Land
remain unchanged. The repeat requires another fully charged battery.

## Optional controller trial, explicitly requested after the pending test

The operator requested removal of host abort thresholds and accepted crash and
low-battery risks. `scripts/hover_check.py --vicon-only --height 1.0
--controller-trial` previews a separate Vicon-only PID mode with no host voltage
thresholds, lateral/vertical/tilt/full-attitude envelope aborts, or endpoint
position/velocity abort. An explicit `--execute` is still required to fly.
The ordinary command and active YAML profile are unchanged. This mode is prepared
for after the pending normal-profile test; it has not been flown.

The trial retains finite/fresh Vicon, estimator and status telemetry, matching
frames, startup body-frame alignment and ground position/velocity checks,
firmware identity/controller/deck checks, firmware self-test/supervisor readiness,
and observed-ground confirmation before disarming. Tracking loss requests the
same bounded landing. Firmware battery, supervisor and crash protections are
unchanged. Removing the host voltage threshold does not override firmware faults
or make a depleted battery supply more power.

The trial uses the same four-second takeoff, configured settling interval,
two-second hold, and bounded eight-second Land. It never replaces landing with
an indefinite GoTo. The normal read-only `check_hardware.py` still enforces its
ordinary 3.8 V ground diagnostic threshold; the trial's own explicit preflight
omits host voltage thresholds. Do not chain that ordinary diagnostic as a trial
voltage gate. CLI previews and mission preflight logs identify trial mode.

## Completed controller trial and larger PDG preparation

The explicit controller trial at 09:41:53 UTC completed the full takeoff, two-second
hold, bounded landing and confirmed disarming. Hold height was 1.079–1.113 m above
the pad, with maximum hold lateral deviation 0.0289 m. The operator reported that
it looked great. Minimum logged loaded battery voltage was 3.362 V. This proves
the PID baseline sequence, not Mellinger or the longer PDG maneuver.

At the operator's request, preparation now targets a 2.5 m altitude and +1 m X
start relative to the landing origin, with takeoff near the origin and sideways
staging. The operator confirmed 2.8 m overhead clearance plus 0.5 m around the
route. The accepted seven-second descent is saved under
`output/pdg-2p5m-1m-prepared/`; the candidate configuration is
`config/pdg-2p5m-1m-candidate.yaml`. See `docs/pdg-trial.md` for coordinates,
provisional model/feedforward values, staging, saved-plan reuse and the remaining
Mellinger/endurance commissioning steps. No PDG flight or tracker parameter write
was performed during this preparation. The complete suite passed 108 tests.
