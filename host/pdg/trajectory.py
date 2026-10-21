"""FOH control interpolation and integrated reference-state sampling."""
from dataclasses import dataclass
import numpy as np
from .model import Quadrotor


@dataclass(frozen=True)
class Setpoint:
    position: np.ndarray
    velocity: np.ndarray
    acceleration: np.ndarray  # world kinematic acceleration, gravity excluded
    quaternion_wxyz: np.ndarray
    body_rates: np.ndarray


class Trajectory:
    def __init__(self, plan, vehicle):
        if not plan.converged:
            raise ValueError('Refusing to execute an unconverged or invalid plan')
        self.plan, self.vehicle = plan, vehicle

    def sample(self, t):
        times = self.plan.times
        t = float(np.clip(t, 0, times[-1]))
        k = min(max(np.searchsorted(times, t, side='right')-1, 0), len(times)-2)
        dt = times[k+1]-times[k]
        a = (t-times[k])/dt
        u0, u1 = self.plan.controls[k:k+2]
        u = (1-a)*u0+a*u1
        x = self.vehicle.step(self.plan.states[k], u0, u, t-times[k])
        q = x[6:10]/np.linalg.norm(x[6:10])
        return Setpoint(x[:3], x[3:6], self.vehicle.dynamics(x,u)[3:6], q, x[10:13])
