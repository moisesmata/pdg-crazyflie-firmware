"""Successive convexification solver for the paper's 6-DoF landing problem."""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from time import perf_counter

import cvxpy as cp
import numpy as np
from numpy.typing import NDArray
from scipy.integrate import solve_ivp

from .paper_model import (
    MASS,
    NU,
    NX,
    OMEGA,
    POS,
    QUAT,
    VEL,
    VehicleParameters,
    angle_of_attack,
    dynamics,
    normalize_quaternion,
    numerical_jacobians,
    state_trigger_value,
    tilt_angle,
)

Array = NDArray[np.float64]


@dataclass(frozen=True)
class SCvxConfig:
    nodes: int = 20
    max_iterations: int = 15
    initial_burn_time: float = 5.0
    min_burn_time: float = 1.0
    max_burn_time: float = 8.0
    max_coast_time: float = 2.0
    virtual_control_weight: float = 1.0e4
    trust_region_weight: float = 0.5
    virtual_control_tolerance: float = 2.0e-5
    trust_region_tolerance: float = 5.0e-4
    integrator_rtol: float = 2.0e-7
    integrator_atol: float = 1.0e-9
    substeps_per_interval: int = 8
    solver: str = "CLARABEL"
    use_aerodynamics: bool = True
    use_state_trigger: bool = True
    free_ignition_time: bool = False
    continuous_thrust_inner_bound: bool = True
    path_margin_fraction: float = 0.15
    initial_quaternion: Array | None = None
    initial_body_rate: Array | None = None
    initial_thrust: Array | None = None
    initial_thrust_rate: Array | None = None
    initial_position: Array = field(
        default_factory=lambda: np.array([5.33, 4.5, 0.0])
    )
    initial_velocity: Array = field(
        default_factory=lambda: np.array([-0.5, -2.5, 0.0])
    )


@dataclass
class IterationRecord:
    iteration: int
    objective: float
    burn_time: float
    coast_time: float
    virtual_control: float
    trust_region: float
    solve_time: float
    status: str


@dataclass
class SCvxResult:
    states: Array
    controls: Array
    burn_time: float
    coast_time: float
    history: list[IterationRecord]
    converged: bool
    parameters: VehicleParameters
    config: SCvxConfig
    propagated_states: Array | None = None
    max_position_defect: float = np.nan
    max_attitude_defect_deg: float = np.nan

    @property
    def time(self) -> Array:
        return np.linspace(0.0, self.burn_time, self.states.shape[1])

    @property
    def fuel_consumed_percent(self) -> float:
        p = self.parameters
        return 100.0 * (p.ignition_mass - self.states[MASS, -1]) / p.ignition_mass

    def node_constraint_violations(self) -> dict[str, float]:
        """Maximum positive violation of each node constraint in native units."""
        x, u, p = self.states, self.controls, self.parameters
        speed = np.linalg.norm(x[VEL], axis=0)
        aoa_deg = np.rad2deg([angle_of_attack(x[:, k]) for k in range(x.shape[1])])
        active = speed > p.trigger_speed
        thrust = np.linalg.norm(u, axis=0)
        gimbal_deg = np.rad2deg(
            np.arccos(np.clip(u[0] / np.maximum(thrust, 1e-12), -1.0, 1.0))
        )
        glide = (
            np.linalg.norm(x[2:4], axis=0)
            / np.tan(np.deg2rad(p.glide_slope_deg))
            - x[1]
        )
        return {
            "dry_mass": max(0.0, float(p.dry_mass - np.min(x[MASS]))),
            "glide_slope": max(0.0, float(np.max(glide))),
            "tilt_deg": max(
                0.0,
                float(
                    np.max(
                        [np.rad2deg(tilt_angle(x[:, k])) for k in range(x.shape[1])]
                    )
                    - p.max_tilt_deg
                ),
            ),
            "angular_rate_deg": max(
                0.0,
                float(
                    np.max(np.rad2deg(np.linalg.norm(x[OMEGA], axis=0)))
                    - p.max_angular_rate_deg
                ),
            ),
            "thrust_min": max(0.0, float(p.thrust_min - np.min(thrust))),
            "thrust_max": max(0.0, float(np.max(thrust) - p.thrust_max)),
            "gimbal_deg": max(0.0, float(np.max(gimbal_deg) - p.max_gimbal_deg)),
            "active_aoa_deg": (
                max(0.0, float(np.max(aoa_deg[active]) - p.max_angle_of_attack_deg))
                if np.any(active) and self.config.use_state_trigger
                else 0.0
            ),
            "quaternion_norm": float(np.max(np.abs(np.linalg.norm(x[QUAT], axis=0) - 1.0))),
        }

    def save(self, path: str | Path) -> None:
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        np.savez_compressed(
            path,
            format_version=np.array(2, dtype=int),
            states=self.states,
            controls=self.controls,
            propagated_states=self.propagated_states,
            time=self.time,
            burn_time=self.burn_time,
            coast_time=self.coast_time,
            converged=self.converged,
            ignition_mass=self.parameters.ignition_mass,
            use_aerodynamics=self.config.use_aerodynamics,
            use_state_trigger=self.config.use_state_trigger,
            free_ignition_time=self.config.free_ignition_time,
            max_position_defect=self.max_position_defect,
            max_attitude_defect_deg=self.max_attitude_defect_deg,
            history=np.array(
                [
                    [h.iteration, h.objective, h.burn_time, h.coast_time,
                     h.virtual_control, h.trust_region, h.solve_time]
                    for h in self.history
                ]
            ),
        )


def _effective_parameters(p: VehicleParameters, use_aerodynamics: bool) -> VehicleParameters:
    if use_aerodynamics:
        return p
    values = dict(p.__dict__)
    values["rho"] = 0.0
    return VehicleParameters(**values)


def _straight_line_initialization(
    p: VehicleParameters, cfg: SCvxConfig
) -> tuple[Array, Array, float, float]:
    k_count = cfg.nodes
    x0 = np.zeros(NX)
    x0[MASS] = p.ignition_mass
    x0[POS] = cfg.initial_position
    x0[VEL] = cfg.initial_velocity
    x0[QUAT] = np.array([1.0, 0.0, 0.0, 0.0]) if cfg.initial_quaternion is None else cfg.initial_quaternion
    if cfg.initial_body_rate is not None:x0[OMEGA] = cfg.initial_body_rate

    xf = np.zeros(NX)
    xf[MASS] = p.dry_mass
    xf[VEL] = -p.terminal_descent_speed * np.array([1.0, 0.0, 0.0])
    xf[QUAT] = np.array([1.0, 0.0, 0.0, 0.0])

    alpha = np.linspace(0.0, 1.0, k_count)
    states = x0[:, None] * (1.0 - alpha) + xf[:, None] * alpha
    controls = np.empty((NU, k_count))
    for k in range(k_count):
        controls[:, k] = -states[MASS, k] * p.gravity
    if cfg.initial_thrust is not None:controls[:,0] = cfg.initial_thrust
    return states, controls, cfg.initial_burn_time, 0.0


def _propagate_interval_matrices(
    x0: Array,
    u0: Array,
    u1: Array,
    burn_time: float,
    interval_length: float,
    p: VehicleParameters,
    cfg: SCvxConfig,
) -> tuple[Array, Array, Array, Array, Array]:
    """Integrate Eqs. (24)-(31) for one FOH multiple-shooting interval."""
    n_aug = NX + NX * NX + NX * NU * 2 + NX * 2
    y0 = np.zeros(n_aug)
    y0[:NX] = x0
    cursor = NX
    y0[cursor : cursor + NX * NX] = np.eye(NX).ravel()

    def unpack(y: Array):
        c = NX
        phi = y[c : c + NX * NX].reshape(NX, NX)
        c += NX * NX
        bm = y[c : c + NX * NU].reshape(NX, NU)
        c += NX * NU
        bp = y[c : c + NX * NU].reshape(NX, NU)
        c += NX * NU
        ss = y[c : c + NX]
        c += NX
        ww = y[c : c + NX]
        return phi, bm, bp, ss, ww

    def ode(local_tau: float, y: Array) -> Array:
        lam = local_tau / interval_length
        u = (1.0 - lam) * u0 + lam * u1
        x = y[:NX]
        f = dynamics(x, u, p)
        fx, fu = numerical_jacobians(x, u, p)
        a = burn_time * fx
        b = burn_time * fu
        s_col = f
        w = -a @ x - b @ u
        phi, bm, bp, ss, ww = unpack(y)
        dy = np.empty_like(y)
        dy[:NX] = burn_time * f
        c = NX
        dy[c : c + NX * NX] = (a @ phi).ravel()
        c += NX * NX
        dy[c : c + NX * NU] = (a @ bm + (1.0 - lam) * b).ravel()
        c += NX * NU
        dy[c : c + NX * NU] = (a @ bp + lam * b).ravel()
        c += NX * NU
        dy[c : c + NX] = a @ ss + s_col
        c += NX
        dy[c : c + NX] = a @ ww + w
        return dy

    max_step = interval_length / cfg.substeps_per_interval
    sol = solve_ivp(
        ode,
        (0.0, interval_length),
        y0,
        method="RK45",
        rtol=cfg.integrator_rtol,
        atol=cfg.integrator_atol,
        max_step=max_step,
    )
    if not sol.success:
        raise RuntimeError(f"Linearization propagation failed: {sol.message}")
    return unpack(sol.y[:, -1])


def _linearize_state_trigger(xbar: Array, p: VehicleParameters) -> tuple[float, Array]:
    h0 = state_trigger_value(xbar, p)
    grad = np.zeros(NX)
    if np.linalg.norm(xbar[VEL]) <= p.trigger_speed:
        return 0.0, grad
    for j in list(range(VEL.start, VEL.stop)) + list(range(QUAT.start, QUAT.stop)):
        step = 2e-6 * max(1.0, abs(float(xbar[j])))
        xp, xm = xbar.copy(), xbar.copy()
        xp[j] += step
        xm[j] -= step
        grad[j] = (state_trigger_value(xp, p) - state_trigger_value(xm, p)) / (2.0 * step)
    return h0, grad


def _build_and_solve_subproblem(
    xbar: Array,
    ubar: Array,
    sbar: float,
    tcbar: float,
    matrices: list[tuple[Array, Array, Array, Array, Array]],
    p: VehicleParameters,
    cfg: SCvxConfig,
) -> tuple[Array, Array, float, float, Array, float, str]:
    k_count = cfg.nodes
    x = cp.Variable((NX, k_count))
    u = cp.Variable((NU, k_count))
    nu = cp.Variable((NX, k_count - 1))
    s = cp.Variable()
    tc = cp.Variable()
    constraints: list[cp.Constraint] = []

    constraints += [s >= cfg.min_burn_time, s <= cfg.max_burn_time]
    if cfg.free_ignition_time:
        constraints += [tc >= 0.0, tc <= cfg.max_coast_time]
    else:
        constraints += [tc == 0.0]

    # Free-ignition coast curve, Eq. (13), first-order expansion for position.
    r_coast = cfg.initial_position + cfg.initial_velocity * tcbar + 0.5 * p.gravity * tcbar**2
    dr_dtc = cfg.initial_velocity + p.gravity * tcbar
    constraints += [x[MASS, 0] == p.ignition_mass]
    constraints += [x[POS, 0] == r_coast + dr_dtc * (tc - tcbar)]
    constraints += [x[VEL, 0] == cfg.initial_velocity + p.gravity * tc]
    constraints += [x[OMEGA, 0] == (np.zeros(3) if cfg.initial_body_rate is None else cfg.initial_body_rate)]
    if cfg.initial_quaternion is not None:constraints += [x[QUAT, 0] == cfg.initial_quaternion]
    if cfg.initial_thrust is not None:constraints += [u[:, 0] == cfg.initial_thrust]
    if cfg.initial_thrust_rate is not None:
        constraints += [u[:,1]-u[:,0] == cfg.initial_thrust_rate*s/(cfg.nodes-1)]

    constraints += [x[POS, -1] == 0.0]
    constraints += [x[VEL, -1] == np.array([-p.terminal_descent_speed, 0.0, 0.0])]
    constraints += [x[QUAT, -1] == np.array([1.0, 0.0, 0.0, 0.0])]
    constraints += [x[OMEGA, -1] == 0.0]

    for k, (ak, bmk, bpk, sk, wk) in enumerate(matrices):
        constraints += [
            x[:, k + 1]
            == ak @ x[:, k] + bmk @ u[:, k] + bpk @ u[:, k + 1] + sk * s + wk + nu[:, k]
        ]

    cot_glide = 1.0 / np.tan(np.deg2rad(p.glide_slope_deg))
    tilt_radius = np.sqrt((1.0 - np.cos(np.deg2rad(p.max_tilt_deg*(1-cfg.path_margin_fraction)))) / 2.0)
    gimbal_cos = np.cos(np.deg2rad(p.max_gimbal_deg))
    omega_max = np.deg2rad(p.max_angular_rate_deg)*(1-cfg.path_margin_fraction)

    for k in range(k_count):
        constraints += [x[MASS, k] >= p.dry_mass]
        constraints += [cot_glide * cp.norm(x[2:4, k], 2) <= x[1, k]]
        constraints += [cp.norm(x[9:11, k], 2) <= tilt_radius]
        constraints += [cp.norm(x[OMEGA, k], 2) <= omega_max]
        constraints += [cp.norm(u[:, k], 2) <= p.thrust_max]
        constraints += [gimbal_cos * cp.norm(u[:, k], 2) <= u[0, k]]

        u_norm = np.linalg.norm(ubar[:, k])
        direction = ubar[:, k] / max(u_norm, 1e-9)
        constraints += [direction @ u[:, k] >= p.thrust_min]
        if cfg.continuous_thrust_inner_bound:
            # Conservative convex inner bound: u_x >= Tmin guarantees the
            # nonconvex lower norm bound for every FOH control interpolation.
            constraints += [u[0, k] >= p.thrust_min]

        if cfg.use_state_trigger:
            h0, grad = _linearize_state_trigger(xbar[:, k], p)
            if np.any(grad):
                constraints += [h0 + grad @ (x[:, k] - xbar[:, k]) <= 0.0]

    trust = (
        cp.sum_squares(x - xbar)
        + cp.sum_squares(u - ubar)
        + cp.square(s - sbar)
        + cp.square(tc - tcbar)
    )
    virtual = cp.norm1(nu)
    objective = cp.Minimize(
        -x[MASS, -1]
        + cfg.virtual_control_weight * virtual
        + cfg.trust_region_weight * trust
    )
    problem = cp.Problem(objective, constraints)
    problem.solve(solver=cfg.solver, canon_backend="SCIPY", verbose=False)
    if problem.status not in (cp.OPTIMAL, cp.OPTIMAL_INACCURATE):
        raise RuntimeError(f"Convex subproblem failed with status {problem.status}")
    return (
        np.asarray(x.value),
        np.asarray(u.value),
        float(s.value),
        float(tc.value),
        np.asarray(nu.value),
        float(problem.value),
        str(problem.status),
    )


def propagate_nonlinear(
    states: Array,
    controls: Array,
    burn_time: float,
    p: VehicleParameters,
    cfg: SCvxConfig,
) -> Array:
    """Validate a solution by integrating the original nonlinear dynamics."""
    k_count = states.shape[1]
    dt = burn_time / (k_count - 1)
    out = np.empty_like(states)
    out[:, 0] = states[:, 0]
    for k in range(k_count - 1):
        def ode(t: float, x: Array) -> Array:
            lam = t / dt
            u = (1.0 - lam) * controls[:, k] + lam * controls[:, k + 1]
            return dynamics(x, u, p)

        sol = solve_ivp(
            ode,
            (0.0, dt),
            out[:, k],
            t_eval=[dt],
            rtol=cfg.integrator_rtol,
            atol=cfg.integrator_atol,
            max_step=dt / (2 * cfg.substeps_per_interval),
        )
        if not sol.success:
            raise RuntimeError(f"Nonlinear validation failed: {sol.message}")
        out[:, k + 1] = sol.y[:, -1]
        out[QUAT, k + 1] = normalize_quaternion(out[QUAT, k + 1])
    return out


def _attitude_error_deg(q1: Array, q2: Array) -> float:
    dot = abs(float(np.dot(normalize_quaternion(q1), normalize_quaternion(q2))))
    return float(np.rad2deg(2.0 * np.arccos(np.clip(dot, -1.0, 1.0))))


def solve(
    parameters: VehicleParameters | None = None,
    config: SCvxConfig | None = None,
    *,
    verbose: bool = True,
    initial_guess: SCvxResult | None = None,
) -> SCvxResult:
    """Run Algorithm 1 and return the converged trajectory."""
    p0 = parameters or VehicleParameters()
    cfg = config or SCvxConfig()
    from .validation import validate_inputs
    validate_inputs(p0, cfg)
    if cfg.nodes < 3:
        raise ValueError("At least three temporal nodes are required")
    p = _effective_parameters(p0, cfg.use_aerodynamics)
    xbar, ubar, sbar, tcbar = _straight_line_initialization(p, cfg)
    if initial_guess is not None:
        if (initial_guess.states.shape[0]!=NX or initial_guess.controls.shape!=(NU,initial_guess.states.shape[1])
                or not np.isfinite(initial_guess.states).all() or not np.isfinite(initial_guess.controls).all()
                or not cfg.min_burn_time<=initial_guess.burn_time<=cfg.max_burn_time):
            raise ValueError('Invalid rocket optimization initial guess')
        old=np.linspace(0.,1.,initial_guess.states.shape[1]);new=np.linspace(0.,1.,cfg.nodes)
        xbar=np.array([np.interp(new,old,row) for row in initial_guess.states])
        ubar=np.array([np.interp(new,old,row) for row in initial_guess.controls])
        sbar=float(initial_guess.burn_time);tcbar=float(initial_guess.coast_time)
    history: list[IterationRecord] = []
    converged = False
    d_tau = 1.0 / (cfg.nodes - 1)

    for iteration in range(1, cfg.max_iterations + 1):
        matrices = [
            _propagate_interval_matrices(
                xbar[:, k], ubar[:, k], ubar[:, k + 1], sbar, d_tau, p, cfg
            )
            for k in range(cfg.nodes - 1)
        ]
        start = perf_counter()
        x, u, s, tc, nu, objective, status = _build_and_solve_subproblem(
            xbar, ubar, sbar, tcbar, matrices, p, cfg
        )
        solve_time = perf_counter() - start
        virtual = float(np.sum(np.abs(nu)))
        trust = float(
            np.sum((x - xbar) ** 2)
            + np.sum((u - ubar) ** 2)
            + (s - sbar) ** 2
            + (tc - tcbar) ** 2
        )
        record = IterationRecord(
            iteration, objective, s, tc, virtual, trust, solve_time, status
        )
        history.append(record)
        if verbose:
            print(
                f"iter {iteration:02d} | J={objective:10.4f} | tb={s:6.3f} "
                f"| tc={tc:6.3f} | ||nu||1={virtual:9.2e} "
                f"| trust={trust:9.2e} | {solve_time:5.2f}s"
            )
        xbar, ubar, sbar, tcbar = x, u, s, tc
        if virtual < cfg.virtual_control_tolerance and trust < cfg.trust_region_tolerance:
            # Small optimizer steps are necessary but not sufficient. A
            # continuously propagated trajectory can violate path/terminal
            # constraints between nodes, so keep iterating if replay fails.
            from .validation import validate_result
            candidate = SCvxResult(xbar, ubar, sbar, tcbar, history, True, p, cfg)
            replay_report = validate_result(candidate)
            if replay_report['passed']:
                converged = True
                break
            if verbose:
                print('  independent replay failures:',{k:v for k,v in replay_report['residuals'].items() if v>replay_report['tolerances'][k]},flush=True)

    propagated = propagate_nonlinear(xbar, ubar, sbar, p, cfg)
    position_errors = np.linalg.norm(propagated[POS] - xbar[POS], axis=0)
    attitude_errors = [
        _attitude_error_deg(propagated[QUAT, k], xbar[QUAT, k])
        for k in range(cfg.nodes)
    ]
    return SCvxResult(
        states=xbar,
        controls=ubar,
        burn_time=sbar,
        coast_time=tcbar,
        history=history,
        converged=converged,
        parameters=p,
        config=cfg,
        propagated_states=propagated,
        max_position_defect=float(np.max(position_errors)),
        max_attitude_defect_deg=float(np.max(attitude_errors)),
    )
