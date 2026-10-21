"""Free-final-time SCvx adapted from Szmuk & Acikmese (arXiv:1802.03827).

Quadrotor adaptation: constant mass, fixed body-z thrust plus independent torques,
ENU frame, and explicit rotor-force allocation constraints. Not a rocket replica.
"""
import time
import cvxpy as cp
import numpy as np
from ..contracts import Plan
from ..model import Quadrotor


class SuccessiveConvexification:
    def __init__(self, vehicle=None, max_iterations=40, tolerance=5e-4):
        self.vehicle = vehicle or Quadrotor()
        self.max_iterations = max_iterations
        self.tolerance = tolerance

    def solve(self, p):
        start = time.perf_counter()
        n, m = p.nodes, self.vehicle
        xr = np.repeat(np.asarray(p.initial_state)[None, :], n, axis=0)
        xr[:, :3] = np.linspace(p.initial_state[:3], p.target_position, n)
        xr[:, 3:6] = np.linspace(p.initial_state[3:6], p.terminal_velocity, n)
        ur = np.zeros((n, 4)); ur[:, 0] = m.gravity
        tr = p.initial_time
        target = np.zeros(13); target[:3] = p.target_position
        target[3:6] = p.terminal_velocity; target[6] = 1
        history = []
        x = cp.Variable((n, 13)); u = cp.Variable((n, 4)); tf = cp.Variable()
        nu = cp.Variable((n-1, 13))
        scale_u = np.array([10., 20., 20., 20.])
        converged = False
        for iteration in range(self.max_iterations):
            constraints = [x[0] == p.initial_state, x[-1] == target,
                           u[-1, 1:] == 0, tf >= p.min_time, tf <= p.max_time,
                           cp.abs(tf-tr) <= 0.6,
                           x[:, 2] >= p.target_position[2], x[:, 2] <= p.ceiling,
                           cp.abs(x[:, :2]) <= p.workspace_xy,
                           cp.abs(x[:, 6:10]) <= 1.1,
                           u[:, 0] >= p.min_specific_thrust,
                           u[:, 0] <= p.max_specific_thrust,
                           u @ m.allocation.T >= 0,
                           u @ m.allocation.T <= m.max_motor_force]
            if p.monotonic_descent:
                constraints += [x[:, 5] <= 0]
            for k in range(n):
                constraints += [cp.norm(x[k, 3:6]) <= 0.95*p.max_speed,
                                cp.norm(x[k, 10:13]) <= 0.95*p.max_body_rate,
                                cp.norm(x[k, 7:9]) <= np.sin(np.deg2rad(p.max_tilt_deg)/2)]
            for k in range(n-1):
                phi, A, B, C, S = m.linearize_step(xr[k], ur[k], ur[k+1], tr, n-1)
                constraints += [x[k+1] == phi + A@(x[k]-xr[k]) + B@(u[k]-ur[k])
                                + C@(u[k+1]-ur[k+1]) + S*(tf-tr) + nu[k]]
                if p.monotonic_descent:
                    # Constrain intermediate velocities too: node-only bounds can
                    # hide short climbs under first-order-hold thrust commands.
                    for fraction in (0.25, 0.5, 0.75):
                        uf = (1-fraction)*ur[k] + fraction*ur[k+1]
                        ph, Ah, Bh, Ch, Sh = m.linearize_step(
                            xr[k], ur[k], uf, tr, (n-1)/fraction)
                        uh = (1-fraction)*u[k] + fraction*u[k+1]
                        vh = (ph + Ah@(x[k]-xr[k]) + Bh@(u[k]-ur[k])
                              + Ch@(uh-uf) + Sh*(tf-tr))
                        constraints += [vh[5] <= 0]
            # Soft trust regions and an exact L1 virtual-control penalty.
            cost = tf + 2e4*cp.norm1(nu) + 0.002*cp.sum_squares(x-xr)
            cost += 0.002*cp.sum_squares(cp.multiply(1/scale_u, u-ur))
            cost += 0.3*cp.square(tf-tr) + 0.01*cp.sum_squares(cp.multiply(1/scale_u, u))
            prob = cp.Problem(cp.Minimize(cost), constraints)
            prob.solve(solver="CLARABEL", canon_backend="SCIPY", max_iter=200,
                       tol_gap_abs=1e-8, tol_feas=1e-8, tol_gap_rel=1e-8)
            if prob.status not in (cp.OPTIMAL, cp.OPTIMAL_INACCURATE):
                raise RuntimeError(f"SCvx subproblem {iteration}: {prob.status}")
            xn, un, tn = np.asarray(x.value), np.asarray(u.value), float(tf.value)
            step = max(np.max(np.abs(xn-xr)), np.max(np.abs((un-ur)/scale_u)), abs(tn-tr))
            vc = float(np.max(np.abs(nu.value)))
            defects = [np.max(np.abs(xn[k+1]-m.step(xn[k], un[k], un[k+1], tn/(n-1))))
                       for k in range(n-1)]
            history.append(dict(iteration=iteration+1, time=tn, step=float(step),
                                virtual_control=vc, dynamic_defect=float(max(defects))))
            xr, ur, tr = xn, un, tn
            if step < self.tolerance and vc < 1e-6 and max(defects) < 1e-5:
                converged = True
                break
        from ..validation import validate_plan
        plan = Plan(np.linspace(0, tr, n), xr, ur, converged,
                    dict(model="quadrotor_6dof_constant_mass", algorithm="scvx_free_time_foh",
                         history=history, solve_seconds=time.perf_counter()-start))
        plan.diagnostics['validation'] = validate_plan(m, p, plan)
        plan.converged = plan.converged and plan.diagnostics['validation']['passed']
        return plan
