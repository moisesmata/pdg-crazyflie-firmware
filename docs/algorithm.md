# SCvx baseline and relationship to the paper

Reference: M. Szmuk and B. Acikmese, *Successive Convexification for 6-DoF Mars Rocket Powered Landing with Free-Final-Time*, AIAA 2018, https://arxiv.org/abs/1802.03827.

This implements a **quadrotor adaptation**, not a reproduction of the gimbaled-rocket results. The initial task is a nominal takeoff followed by one precomputed descent. No recovery library, capability estimator, online replanner, or robustness guarantee is claimed here.

## State, control, and physical model

ENU metres; z up; body-z thrust. `x = [position(3), velocity(3), quaternion_wxyz(4), body_rates(3)]`.
`u = [T/m, tau_x/J_x, tau_y/J_y, tau_z/J_z]` in m/s² and rad/s².

The constant-mass rigid body evolves according to:

- p_dot = v
- v_dot = R(q) e_z (T/m) - g e_z
- q_dot = 0.5 q ⊗ [0, omega]
- omega_dot = J^-1(tau - omega × J omega)

A fixed X-configuration allocation maps the optimization control into four nonnegative rotor forces, each bounded by a configured maximum. This checks wrench feasibility for the declared model. It is not a calibrated Crazyflie actuator model. Rotor geometry, mass, inertia, force limits, and drag/yaw coefficient need identification before flight.

## Successive convexification

Free flight time sigma makes the normalized-time dynamics nonlinear. On each interval, first-order-held controls are integrated with four RK4 substeps. Complex-step derivatives linearize this discrete nonlinear map with respect to the initial state, both endpoint controls, and sigma. This substitutes a differentiated RK4 map for the paper's integrated variational equations. A small-step cross-check against independently integrated DOP853 verifies numerical consistency.

Each convex subproblem contains initial/final state equalities, rotor-force limits, scalar thrust bounds, tilt/body-rate/speed limits, an altitude/workspace envelope, virtual dynamics controls penalized in L1, soft trust terms, and a bounded time change. A small control-effort regularizer removes degeneracy; the primary cost remains flight time. CLARABEL solves the subproblems through CVXPY.

The paper's gimbaled-engine mass depletion, gimbal-angle limit, and rocket-specific inertia/force coupling are **not** used. Quadrotor body-z thrust and differential-motor torques replace those mechanisms. Lower total thrust is a scalar bound, so it does not require the paper's nonconvex thrust-vector-norm linearization. The current example does not impose a glide-slope constraint.

Speed/body-rate constraints are tightened by 5% at nodes to reduce intersample excursions observed in early testing. This is a numerical margin, not a continuous-time proof. All accepted plans also pass an independent dense nonlinear rollout checking terminal state, quaternion norm, workspace, speed, tilt, rate, thrust and rotor forces. Nonconvergence, excessive virtual control, or failed checks prevent execution.

## Tracking is a separate problem

The host sends position, velocity, world kinematic acceleration (gravity excluded), quaternion and body rate through Crazyswarm2 `FullState`. The stock Mellinger tracker reconstructs its desired attitude from acceleration and yaw; it does not execute our optimized motor forces or full attitude history verbatim. Thus the optimization rollout alone does not validate the tracked physical trajectory or actuator limits after feedback corrections. A Crazyswarm2 SIL run and restrained hardware tests remain necessary.

Current `simulate` means independent continuous-dynamics integration of the planned FOH control, not firmware-in-the-loop or hardware validation. No online computation deadline is promised: planning happens before arming.

## Extension interfaces

A new algorithm implements `Planner.solve(LandingProblem) -> Plan`. A new vehicle supplies dynamics, discretization and allocation; the current solver's quaternion/constraint transcription remains quadrotor-specific and would also need adaptation. Execution consumes the common trajectory contract and `FlightTransport` interface. Landing-site selection, fault scenarios, estimator outputs and online refinement belong in later host modules, not in the motor driver.

The sample configuration intentionally sets a 1.8 s lower bound on descent duration. This slows initial test trajectories; the returned time optimum may simply activate this bound. No claim is made that this is the unconstrained fastest landing.

The nominal task also imposes nonpositive vertical velocity at the nodes and at three intermediate points per interval (`monotonic_descent`). Independent rollout checks this too. It prevents solutions that wait near the ground or reverse their descent to fill the configured duration; node and dense-sample checks are still not continuous-time certification.
