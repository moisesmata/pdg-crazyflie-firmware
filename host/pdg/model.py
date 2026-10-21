"""Constant-mass quadrotor rigid body, ENU world, body z thrust, q=[w,x,y,z]."""
from dataclasses import dataclass
import numpy as np
from scipy.integrate import solve_ivp


@dataclass(frozen=True)
class Quadrotor:
    mass: float = 0.032
    inertia: tuple = (1.66e-5, 1.67e-5, 2.93e-5)
    arm: float = 0.046  # center-to-rotor radius [m]
    yaw_coefficient: float = 0.006  # torque / force [m]
    max_motor_force: float = 0.15  # illustrative, MUST be identified on hardware [N]
    gravity: float = 9.81

    def __post_init__(self):
        if not all(np.isfinite(v) and v > 0 for v in
                   (self.mass, *self.inertia, self.arm, self.yaw_coefficient,
                    self.max_motor_force, self.gravity)):
            raise ValueError("Vehicle parameters must be finite and positive")

    @property
    def allocation(self):
        # Maps u=[T/m, tau_x/J_x, tau_y/J_y, tau_z/J_z] to rotor forces.
        a, k = self.arm / np.sqrt(2), self.yaw_coefficient
        jx, jy, jz = self.inertia
        return np.array([[self.mass, -jx/a, -jy/a, -jz/k],
                         [self.mass, -jx/a, jy/a, jz/k],
                         [self.mass, jx/a, jy/a, -jz/k],
                         [self.mass, jx/a, -jy/a, jz/k]]) / 4

    def dynamics(self, x, u):
        q, w = x[6:10], x[10:13]
        qw, qx, qy, qz = q
        body_z = np.array([2*(qx*qz+qw*qy), 2*(qy*qz-qw*qx),
                           1-2*(qx*qx+qy*qy)])
        qdot = 0.5*np.concatenate(([-np.dot(q[1:], w)], qw*w+np.cross(q[1:], w)))
        wdot = u[1:] - np.cross(w, np.asarray(self.inertia)*w)/self.inertia
        return np.concatenate((x[3:6], body_z*u[0]+[0, 0, -self.gravity], qdot, wdot))

    def step(self, x, ua, ub, duration, substeps=4):
        """Differentiable RK4 FOH map (no quaternion projection inside optimizer)."""
        h = duration / substeps
        z = x.copy()
        for i in range(substeps):
            def f(z, a):
                return self.dynamics(z, ua*(1-a)+ub*a)
            a = i/substeps
            k1 = f(z, a)
            k2 = f(z+h*k1/2, a+0.5/substeps)
            k3 = f(z+h*k2/2, a+0.5/substeps)
            k4 = f(z+h*k3, a+1/substeps)
            z = z+h*(k1+2*k2+2*k3+k4)/6
        return z

    def linearize_step(self, x, ua, ub, tf, intervals):
        """Complex-step derivatives of the discretized free-time FOH map."""
        args = np.concatenate((x, ua, ub, [tf]))
        def phi(z):
            return self.step(z[:13], z[13:17], z[17:21], z[21]/intervals)
        value = phi(args)
        jac = np.empty((13, 22))
        for j in range(22):
            perturbed = args.astype(complex)
            perturbed[j] += 1e-25j
            jac[:, j] = np.imag(phi(perturbed))/1e-25
        return value, jac[:, :13], jac[:, 13:17], jac[:, 17:21], jac[:, 21]

    def rollout(self, x0, controls, tf, samples_per_interval=8):
        """Independent DOP853 integration, used to reject optimization artifacts."""
        n = len(controls)-1
        dt = tf/n
        times, states = [0.0], [np.array(x0, dtype=float)]
        for k in range(n):
            def f(t, x):
                alpha = t/dt
                return self.dynamics(x, (1-alpha)*controls[k]+alpha*controls[k+1])
            ts = np.linspace(0, dt, samples_per_interval+1)
            sol = solve_ivp(f, (0, dt), states[-1], t_eval=ts,
                            method="DOP853", rtol=1e-10, atol=1e-12)
            if not sol.success:
                raise RuntimeError(sol.message)
            times.extend((k*dt+ts[1:]).tolist())
            states.extend(sol.y[:, 1:].T)
        return np.asarray(times), np.asarray(states)


def rest_state(position):
    x = np.zeros(13)
    x[:3] = position
    x[6] = 1
    return x
