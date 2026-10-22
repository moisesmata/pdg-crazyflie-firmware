# Virtual rocket PDG on a Crazyflie translational emulator

The default planner now optimizes a **gimbaled, mass-depleting rocket**, then
converts its powered-phase translation into Crazyflie full-state references.
The existing onboard Mellinger controller tracks those references. Planning is
on the host; this change does not move the SCvx solver into firmware.

## Target model and paper relationship

Reference: M. Szmuk, T. P. Reynolds, and B. Açıkmeşe, *Successive Convexification
for Real-Time Six-Degree-of-Freedom Powered Descent Guidance with State-Triggered
Constraints*, JGCD 43(8), 2020, [doi:10.2514/1.G004549](https://doi.org/10.2514/1.G004549).

`host/pdg/rocket/paper_model.py` implements the paper's Eqs. (1)–(3):

```
x = [m, r_UEN(3), v_UEN(3), q(4), omega_B(3)]
u = T_B(3)
m_dot     = -||T_B|| / (Isp g0) - beta
r_dot     = v
v_dot     = C_IB(q) (T_B + A_B) / m + g_target
q_dot     = 0.5 Omega(omega_B) q
J omega_dot = r_T × T_B + r_cp × A_B - omega_B × J omega_B
```

UEN means **up, east, north**. Body **x** is the rocket longitudinal axis.
The scalar-first paper quaternion defines `C_BI` with the **minus-skew** DCM;
use the provided conversion rather than a drone quaternion helper.
The model includes Eq. (14)'s ellipsoidal aerodynamics. The solver supports the
velocity-triggered angle-of-attack condition and optional ignition coast time.
Those options are disabled in the initial vacuum, fixed-ignition example.

`paper_scvx.py` uses free burn time, integrated variational equations, first-order
hold (FOH) engine thrust, virtual dynamics controls, and a soft trust penalty.
The objective maximizes terminal rocket mass; this is distinct from the archived
quadrotor planner's minimum-time objective. Constraints include dry mass,
glide slope, tilt, angular rate, gimbal cone, thrust bounds, and terminal landing
state. Initial attitude is free; terminal attitude is upright.

The implementation adapts the existing local `powered_descent/model.py` and
`powered_descent/scvx.py` beside the supplied paper under
`RESEARCH/PDG/AI-Directed-Research/PoweredDescent/papers/successive-convexification-2020/`.
It is not the authors' released software or an exact reproduction of their
reported timings/results. Jacobians use central finite differences and CLARABEL
solves CVXPY subproblems. Total runtime includes substantially more work than
subproblem solve times printed in iteration logs.

Two additional conservative discretization choices are enabled by default:

- `continuous_thrust_inner_bound`: impose `T_B,x >= T_min`. Because FOH preserves
  this affine inequality, thrust magnitude cannot dip below `T_min` between
  nodes. This restricts the paper's feasible set; the paper's node-wise tangent
  lower bound alone does not establish an intersample lower bound.
- `path_margin_fraction: 0.15`: tighten nodal tilt and body-rate limits to reduce
  between-node excursions. This is a numerical margin, not a certificate.

An independent, continuous DOP853 replay never resets to optimized nodes or
renormalizes its quaternion. It checks terminal errors, quaternion norm, replay
defects, and densely sampled original path constraints before a reference is
accepted. A small optimizer step that fails replay does not stop the solve; it
continues up to the configured iteration budget. The example uses the source
implementation's soft-trust weight 0.5 and step threshold 5e-4. These checks are
numerical evidence, not a continuous-time proof.

## Units and physical interpretation

`config/rocket-emulation.yaml` is an **illustrative normalized rocket**, not an
identified Mars/lunar vehicle. Its `g0: 1.0` is explicitly in target units;
`Isp`, force, mass, time, and gravity must use a consistent normalization.
The aero coefficients are illustrative. No exact numerical reproduction of
the paper's tables is claimed. To use SI, declare `rocket.units: SI` and supply
consistent SI parameters, including standard-reference `g0` and target gravity.

Physical Crazyflie parameters are separate, under `emulation.drone`. Target
rocket mass depletion and gravity never replace the drone's actual mass or
Earth gravity in the tracker.

## Rocket-to-lab map

For length scale `sL`, time scale `sT`, fixed proper rotation `P`, and landing
body-origin offset `pL`:

```
t_lab = sT * t_target
p_lab = pL + sL * P r_target
v_lab = (sL/sT) * P v_target
a_lab = (sL/sT^2) * P v_dot_target
F_drone = m_drone * (a_lab + g_Earth * e_z)
```

`P` maps UEN into ENU followed by the configured world yaw. Acceleration is
computed from the independently replayed rocket equations, including target
gravity and optional aerodynamics. Earth gravity is added exactly once to obtain
the drone's required force. Drone body-z aligns with this force; the configured
heading completes its orientation. Rates and feedforward motor-force checks
come from finite differences of this attitude history.

**The experiment emulates scaled translation.** A conventional quadrotor cannot
generally reproduce both a gimbaled rocket's translation and its attitude.
Virtual rocket attitude, rates, mass, and engine thrust are logged target states,
not measurements of those quantities on the physical vehicle. Thrust depletion,
gimbal motion, and the target aerodynamic environment are not physically
reproduced. Tracking error must be measured before interpreting a hardware run
as an accurate translational emulation.

The adapter checks mapped workspace, ceiling, landing plane, speed, tilt, rates,
specific thrust, and allocated rotor forces with reserve. Checks use the declared
nominal drone model and dense samples, not a tracking or robustness guarantee.
Rejected plans do not enter the ROS mission path. Changing scale changes the lab
reference and feasibility; it does not repair an infeasible target solution.

Only the **powered phase** is replayed. An optimized ignition coast remains in
target results and is not flown. The existing takeoff/settle entry requires near
zero initial lab velocity; a moving ignition state needs a separately designed
entry maneuver. The default starts from rest. The final reference reaches the
configured pad body-origin plane with a small descending velocity; the existing
bounded landing/confirmation sequence then completes the mission. Reference
validation alone is not a completed hardware landing.

## Use

From the repository root with Python dependencies installed:

```bash
# Default: rocket SCvx + independent target replay + lab feasibility; NO flight
pdg plan --output output/rocket-emulation

# Mission preparation is also a dry run without --execute
pdg mission --config config/rocket-emulation.yaml --output output/rocket-mission

# Reuse an accepted rocket target; its model/units/solver must match the config
pdg mission --config config/rocket-emulation.yaml \
  --plan-file output/rocket-emulation/target-plan.json \
  --output output/rocket-replay

# Preserve the old quadrotor experiment explicitly
pdg plan --config config/nominal.yaml --output output/legacy-quadrotor
```

`--height` changes only lab length scale, leaving the virtual rocket problem
unchanged. With free ignition time, set the scale explicitly instead.
`simulate` currently performs rocket nonlinear replay and mapping validation;
it does **not** simulate closed-loop firmware tracking for this backend.

Outputs separate the systems:

- `target-plan.json`: schema 2, 14-state UEN rocket, engine thrust, model, units,
  solver settings, history, independent validation. Not interchangeable with
  legacy 13-state quadrotor plan files.
- `target-validation.json`: original target dynamics/path residuals and tolerances.
- `flight-reference.json`: schema 2, mapped ENU/SI drone setpoints.
- `emulation-validation.json`: mapped nominal drone feasibility and peaks.
- `target-trajectory.png`: rocket position, virtual mass, thrust, and tilt.
- `lab-reference.png`: mapped lab path, speed, drone tilt, and specific thrust.
- `execution.json`: existing tracker/mission configuration.

The ROS mission uses the existing Crazyswarm2 full-state interface and existing
Vicon/firmware tracking path. During execution it logs inverse-mapped measured
position/velocity alongside the virtual reference. Existing stale-pose, start
state, tracking, deadline, and landing checks remain in place. No firmware
flashing, arming, or flight was performed as part of this software change.

Archived successful 2 m hardware runs used the legacy quadrotor planner. They
are not evidence validating this newly added rocket-emulation backend.

## Versioned hardware evidence

The completed 2.75 m rocket-model run and the full-reference retarget run are
now archived in `experiments/README.md`, with their distinct landing outcomes.
Use those measured records rather than the historical preflight-only status
above when assessing what has actually flown.
