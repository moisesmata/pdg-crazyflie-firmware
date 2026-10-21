# Reproduce the current Crazyflie PDG experiment in simulation

Read this file alongside `docs/pdg-simulation-parameters.json`. All paths below
are relative to the `pdg-crazyflie-firmware` repository.

## What has and has not flown

The hardware is a brushed CF 2.1+ with five mocap markers, Flow deck removed.
The active PDG tracker is the stock firmware Mellinger controller behind an
out-of-tree wrapper, not a new onboard PDG optimizer. The host computes the
trajectory offline. Kalman fuses IMU and Vicon full pose onboard.

Neither full hardware attempt reached the PDG descent stream: both stopped
while staging at height. The newest -X attempt developed a real yaw spin, had
motor saturation and descended/flipped. Its raw external pose was continuous.
An earlier +X attempt had large received-pose orientation discontinuities, whose
upstream source was not recorded. Do not treat these as one proven failure mode.

Choose the experiment you want to recreate:
1. **Planner feasibility:** ideal rigid-body nonlinear rollout of the saved plan.
2. **Intended closed-loop PDG:** the same reference tracked by firmware Mellinger.
3. **Observed hardware failure:** the takeoff/staging sequence with motor voltage,
   allocation/saturation, battery and estimator dynamics. Those plant parameters
   are only partly known; no exact identified failure model currently exists.

## Authoritative input files

| Purpose | File |
|---|---|
| Current requested -X mission/model/limits | `config/pdg-2p5m-minus1m-candidate.yaml` |
| Exact accepted trajectory (times, 13-state nodes, 4-controls) | `output/pdg-2p5m-minus1m-prepared/plan.json` |
| Earlier +X mission and trajectory | `config/pdg-2p5m-1m-candidate.yaml`, `output/pdg-2p5m-1m-prepared/plan.json` |
| SCvx optimizer and objective | `host/pdg/planners/scvx.py` |
| Plant equations/allocation/independent integration | `host/pdg/model.py` |
| FOH interpolation and full-state reference sampling | `host/pdg/trajectory.py` |
| Mission timing, staging and landing | `host/pdg/execution.py` |
| ROS message conversion and telemetry | `host/pdg/ros_mission.py` |
| Active onboard tracker wrapper | `apps/pdg_controller/src/pdg_controller.c` |
| Actual Mellinger controller equations and gains | `crazyflie-firmware/src/modules/src/controller/controller_mellinger.c` |
| Motor mixing and saturation | `crazyflie-firmware/src/modules/src/power_distribution_quadrotor.c` |
| Voltage compensation | `crazyflie-firmware/src/drivers/src/motors.c` |
| Brushed CF 2.1+ thrust curve | `crazyflie-firmware/src/platform/interface/platform_defaults_cf2.h` |
| Kalman external-pose update | `crazyflie-firmware/src/modules/src/kalman_core/mm_pose.c` |
| Sensor inputs / external-pose noise settings | `config/crazyflies.yaml`, `launch/pdg.launch.py` |

## Frames and state/control conventions

World is ENU, gravity `[0,0,-9.81]` m/s². Body +X is the physical nose, +Z is up.
Quaternion is body-to-world rotation, stored **[w,x,y,z]** in planner JSON.
ROS fields must be assigned by name; many libraries accept [x,y,z,w] instead.

State: `[px,py,pz, vx,vy,vz, qw,qx,qy,qz, wx,wy,wz]`, SI units, body rates rad/s.
Control: `[T/m, tau_x/Jx, tau_y/Jy, tau_z/Jz]`, units m/s² and rad/s².
The last three controls are normalized torque, not rotor speeds or angular
acceleration after the rigid-body gyroscopic cross term has been included.

```
p_dot = v
v_dot = R(q) * [0,0,u0] + [0,0,-g]
q_dot = 0.5 * q ⊗ [0,omega]
omega_dot = u[1:4] - cross(omega, J*omega)/J
```

## Exact planner settings

Use these to reproduce the accepted solve, not as measured plant parameters:

| Parameter | Value |
|---|---:|
| Constant planning mass | 0.048 kg (assumed upper case) |
| Diagonal inertia | [1.66e-5, 1.67e-5, 2.93e-5] kg m² (illustrative) |
| Center-to-rotor arm radius | 0.046 m |
| Reaction torque / thrust coefficient | 0.006 m |
| Maximum force per rotor | 0.12 N |
| Gravity | 9.81 m/s² |
| Start world position | [-1, 0, 2.5143266757950187] m |
| Goal world position | [0, 0, 0.014326675795018673] m |
| Initial/terminal velocity | [0,0,0] m/s |
| Initial/terminal quaternion | [1,0,0,0] |
| Initial/terminal body rate | [0,0,0] rad/s |
| Nodes | 21 |
| Duration min / initial guess / max | 7 / 8 / 12 s |
| Accepted duration | approximately 7.000000000046 s |
| Maximum speed | 0.55 m/s |
| Maximum tilt | 8 degrees |
| Maximum body rate norm | 1 rad/s |
| Specific thrust interval | [7.5,10.5] m/s² |
| XY workspace | abs(x), abs(y) <= 1.25 m (box) |
| World ceiling | 2.8143266757950185 m |
| Monotonic descent | true; vz <= 0 |
| Normalized control slew penalty | 1.0 |

The solver applies 0.95 margins to node speed and body-rate bounds. It also
constrains intermediate vertical velocity. See the code for the full objective,
trust penalties, convergence thresholds and numerical validation tolerance.
Controls are **first-order held**, not piecewise constant. Optimizer integration
is RK4, four substeps per node interval. Independent validation uses DOP853,
rtol=1e-10, atol=1e-12. The numerical check is not a continuous-time certificate.

Landing-frame start is [-1,0,2.5], goal [0,0,0]. The world-Z offset is the tracked
body origin on the pad; it is not a desired penetration into the floor. Use one
consistent frame throughout and account for body/contact geometry.

The assumed planner maximum total thrust is 0.48 N versus 0.47088 N weight at
48 g: only about 1.9% static margin. This is a tight planning assumption, not an
identified statement of available thrust on our drone.

## Actual tracker and closed-loop interface

Runtime settings applied before Mellinger flights:
```
stabilizer.controller = 0  # auto selects compiled OOT controller; identity = 6
stabilizer.estimator = 2   # Kalman
motion.disable = 1
pdg.capEnable = 0
ctrlMel.mass = 0.044
ctrlMel.massThrust = 135456.8069962816
```

44 g is an effective hover/feedforward calibration, not a scale measurement.
The 48 g planner model and 44 g tracker setting have different roles. For an
initial ideal closed-loop study, sweep physical plant mass over 0.044–0.048 kg
and label these as assumed scenarios. Do not claim exact hardware replication.

Compiled Mellinger defaults (not an independent current onboard gain readback):
```
XY: kp=0.4, kd=0.2, ki=0.05, integral clamp=2.0
 Z: kp=1.25, kd=0.4, ki=0.05, integral clamp=0.4
roll/pitch: kR=70000, kw=20000, ki_m=0, integral clamp=1
       yaw: kR=60000, kw=12000, ki_m=500, integral clamp=1500
roll/pitch rate-error derivative gain: kd_omega_rp=200
```
Mellinger runs at **500 Hz**, on a 1 kHz firmware main loop. Its attitude outputs
are legacy PWM-related commands, **not SI moments**. Port the firmware controller
or use its bindings; copying these gains into a physical torque controller will
not reproduce it. The firmware includes pitch/yaw sign conventions and a
factor convention in its attitude error; reuse the actual implementation.

The host samples the plan at **50 Hz** and sends position, velocity, world
kinematic acceleration, quaternion and body rates. Acceleration **excludes
gravity**; Mellinger adds gravity itself. With absolute position control, the
firmware reconstructs desired roll/pitch from force and extracts heading from
the quaternion; it does not blindly track every supplied quaternion axis.
The planner control vector is not sent as motor commands.

Legacy motor mixing, before battery compensation, for collective c, roll r,
pitch p and yaw y (r and p internally divided by two, with integer conversion):
```
M1 = c - r/2 + p/2 + y
M2 = c - r/2 - p/2 - y
M3 = c + r/2 - p/2 + y
M4 = c + r/2 + p/2 - y
```
After voltage compensation, the firmware saturation logic subtracts the excess
of the largest command over 65535 from **all four motors**, then applies the
lower/idle limit. It is not merely four independent upper clamps. This preserves
differentials at the expense of collective thrust and matters when recreating
loss of height during large attitude/yaw demand.

The compiled brushed CF 2.1+ voltage-to-force polynomial (per rotor) is:
```
V_motor = V_battery * PWM / 65535
F_N = -0.02476537915958403
      +0.06523793527519485 * V_motor
      -0.026792504967750107 * V_motor**2
      +0.006776789303971145 * V_motor**3
```
Firmware voltage compensation inverts this nominal polynomial to obtain PWM
from a normalized thrust command, using THRUST_MAX=0.12 N and
THRUST_MIN=0.012817578393224994 N. Supply below 2 V returns zero.
The profile also defines THRUST2TORQUE=0.0069928948992470565 m; this differs from
the planner's assumed 0.006 m. These are firmware model coefficients, not an
identification of our five-marker aircraft. A battery internal-resistance/capacity
model, motor lag and individual efficiency remain unidentified.

## Mission sequence to reproduce

Start on the pad at world XY approximately [0.0142974351,0.0032781411].
1. Vertical high-level takeoff to world Z=2.5143266758 over 8 s; settle 2 s.
2. High-level GoTo [-1,0,2.5143266758] over 6 s with heading 0 rad; settle 2 s.
3. Begin saved PDG descent only when the start position, velocity and attitude
   checks pass. Stream its 7 s reference at 50 Hz.
4. Queue bounded high-level Land over 8 s, release low-level priority, monitor
   touchdown and disarm after the settled ground-height proxy.

Nominal duration is about 33 s plus final confirmation. Takeoff preserves the
initial heading; staging targets zero heading. Use the firmware high-level
polynomial planner for takeoff/staging if reproducing their detailed transients;
a linear ramp is an approximation. See `host/pdg/execution.py` for checks and
`host/pdg/ros_mission.py` for the pinned server's radians convention.

Full hardware attempts used explicit PDG controller-trial mode, omitting host
voltage and descent tracking-error aborts, retaining telemetry and staging
readiness checks. The latest 60 cm/30 s hover was a different test: ascent 4 s,
settle 2 s, requested hold 30 s, Land 8 s, launch threshold 4.00 V and flight
landing trigger 3.50 V. It landed early at the voltage trigger, not after 30 s.

## Estimator and mocap

Use Kalman + accelerometer/gyro + external position/quaternion; no optical flow
or range deck. Ground pose tracking was approximately 120–126 Hz. Configured
`locSrv.extPosStdDev=0.001` and `locSrv.extQuatStdDev=0.05`; the latter is noise on
the firmware small-angle orientation residual, not independent quaternion
component noise. Kalman prediction is 100 Hz. IMU sampling/main loop is 1 kHz.
Use the firmware estimator implementation for faithful process/noise behavior;
other persistent Kalman parameter values were not independently dumped here.

The pose update uses residual `inv(q_est)*q_mocap` and correction on the right.
Do not apply the archived marker-frame software correction: current setup uses
native Tracker nose +X/up +Z alignment. A perfect-state simulator bypasses this
estimator and cannot reproduce its failure modes. Radio's reported unicast RTT
(usually a few milliseconds) is not an identified camera-to-estimator delay.

## Existing simulator and recommended first steps

`pdg simulate` currently runs the same ideal optimization/nonlinear rollout as
`pdg plan`; it is **not** closed-loop SIL. For exact reference reuse, avoid
re-solving with default smoothing=0:
```
source scripts/local_env.sh
pdg mission --config config/pdg-2p5m-minus1m-candidate.yaml \
  --plan-file output/pdg-2p5m-minus1m-prepared/plan.json \
  --output output/sim-reference-review
```
This is a dry run: no ROS connection and no flight commands. Re-solve with:
```
pdg plan --config config/pdg-2p5m-minus1m-candidate.yaml \
  --control-smoothing 1 --output output/sim-resolve-minusx
```

A useful existing closed-loop starting point is
`ros_ws/src/crazyswarm2/crazyflie_sim/crazyflie_sim/crazyflie_sil.py`, paired with
`backend/np.py`. It calls firmware Mellinger bindings, but its defaults are not
our aircraft: the NumPy plant is 34 g, and stock Mellinger defaults are 29 g / massThrust
132000. Override the controller instance mass/gain and plant mass/inertia, and
match timing. Its PWM→RPM/force fits are a different nominal motor model and its
wrapper does not reproduce our voltage-compensation path. It supplies plant
state rather than the full hardware mocap/IMU Kalman pipeline. Backend `none`
tracks desired state perfectly and is unsuitable for control validation.

Start with the exact saved reference + ideal state + firmware Mellinger; then
add the estimator, delivery/noise, motor voltage compensation, saturation, battery
and per-motor effects individually. Keep ideal-planner and hardware-failure
results separate. Changing state-estimation or propulsion assumptions merely to
match a crash does not identify the real cause.

## Hardware evidence for comparison

- Failed -X staging: `output/pdg-minusx-trial-20261021T103743Z/mission.jsonl`
  and `result.json`. External pose was continuous; gyro confirmed spin;
  motor 3 was saturated in ~66% of logged samples from t=2–18 s. No descent started.
- Latest attempted 30 s hover: `output/hover-20261021T104518Z/mission.jsonl`
  and `result.json`. Launch 4.154 V; 3.499 V trigger at t=8.57 s, 2.56 s into
  hold. Height ~0.588–0.595 m, horizontal error <=0.0211 m during hold.
  Mean logged motor PWM [62514,45483,63659,43664]. Loaded voltage falling through
  3.50 V is not proof that the battery was exhausted.
- Telemetry includes upstream pose, firmware-received pose, estimated pose,
  actual OOT output, four motor commands, gyro yaw rate and battery status.
  PID-specific internal logs are stale when Mellinger runs; do not use those
  PID targets or outputs as active Mellinger control quantities.
