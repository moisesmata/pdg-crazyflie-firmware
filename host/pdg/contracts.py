from dataclasses import dataclass, field
from typing import Protocol
import numpy as np


@dataclass(frozen=True)
class LandingProblem:
    initial_state: np.ndarray
    target_position: np.ndarray
    terminal_velocity: tuple = (0., 0., -0.10)
    min_time: float = 0.8
    max_time: float = 8.
    initial_time: float = 3.
    max_tilt_deg: float = 20.
    max_speed: float = 2.
    max_body_rate: float = 2.
    min_specific_thrust: float = 1.
    max_specific_thrust: float = 18.
    workspace_xy: float = 2.
    ceiling: float = 2.5
    nodes: int = 25
    monotonic_descent: bool = True

    def __post_init__(self):
        arrays = (np.asarray(self.initial_state), np.asarray(self.target_position),
                  np.asarray(self.terminal_velocity))
        if [a.shape for a in arrays] != [(13,), (3,), (3,)] or not all(np.isfinite(a).all() for a in arrays):
            raise ValueError("Expected finite state(13), target(3), and velocity(3)")
        if self.monotonic_descent and arrays[0][5] > 1e-9:
            raise ValueError('Monotonic descent requires nonpositive initial vertical speed')
        if abs(np.linalg.norm(arrays[0][6:10])-1) > 1e-6:
            raise ValueError("Initial quaternion must have unit norm")
        vals = (self.min_time, self.max_time, self.initial_time, self.max_tilt_deg,
                self.max_speed, self.max_body_rate, self.max_specific_thrust,
                self.workspace_xy, self.ceiling)
        if not all(np.isfinite(v) and v > 0 for v in vals):
            raise ValueError("Limits must be finite and positive")
        if not (0 < self.min_time <= self.initial_time <= self.max_time):
            raise ValueError("Invalid flight time bounds")
        if not 0 < self.max_tilt_deg < 80 or not isinstance(self.nodes, int) or self.nodes < 5:
            raise ValueError("Invalid tilt or node count")
        if not 0 <= self.min_specific_thrust < self.max_specific_thrust:
            raise ValueError("Invalid thrust interval")
        if not 0 <= arrays[1][2] <= arrays[0][2] <= self.ceiling:
            raise ValueError("Expected descent with ground <= target <= initial <= ceiling")


@dataclass
class Plan:
    times: np.ndarray
    states: np.ndarray
    controls: np.ndarray
    converged: bool
    diagnostics: dict = field(default_factory=dict)


class Planner(Protocol):
    def solve(self, problem: LandingProblem) -> Plan: ...
