"""Six-degree-of-freedom model from Szmuk, Reynolds & Acikmese (2020).

The inertial frame is up-east-north and the body x axis is the vehicle's
longitudinal (up) axis. Quaternions are scalar-first and rotate inertial
coordinates into body coordinates.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np
from numpy.typing import NDArray

Array = NDArray[np.float64]

MASS = 0
POS = slice(1, 4)
VEL = slice(4, 7)
QUAT = slice(7, 11)
OMEGA = slice(11, 14)
NX = 14
NU = 3


def skew(v: Array) -> Array:
    """Return the cross-product matrix [v]x."""
    x, y, z = v
    return np.array([[0.0, -z, y], [z, 0.0, -x], [-y, x, 0.0]])


def normalize_quaternion(q: Array) -> Array:
    n = np.linalg.norm(q)
    if n < 1e-12:
        raise ValueError("A zero quaternion cannot be normalized")
    return q / n


def dcm_body_from_inertial(q: Array) -> Array:
    """Direction cosine matrix C_B<-I for scalar-first quaternion q."""
    q0 = q[0]
    qv = q[1:]
    return (
        (q0 * q0 - qv @ qv) * np.eye(3)
        + 2.0 * np.outer(qv, qv)
        - 2.0 * q0 * skew(qv)
    )


def omega_matrix(w: Array) -> Array:
    """Quaternion kinematics matrix for C_B<-I and body-frame rates."""
    out = np.zeros((4, 4))
    out[0, 1:] = -w
    out[1:, 0] = w
    out[1:, 1:] = -skew(w)
    return out


@dataclass(frozen=True)
class VehicleParameters:
    """Rocket parameters; defaults inspired by Table 2 with illustrative aero.

    Supply mutually consistent SI or normalized units, including g0. These
    defaults do not define an exact numerical reproduction of the paper.
    """

    gravity: Array = field(default_factory=lambda: np.array([-1.0, 0.0, 0.0]))
    rho: float = 1.0
    reference_area: float = 1.0
    aero_coefficients: Array = field(
        default_factory=lambda: np.diag([0.05, 0.5, 0.5])
    )
    inertia: Array = field(
        default_factory=lambda: 0.168 * np.diag([2.0e-2, 1.0, 1.0])
    )
    thrust_arm: Array = field(default_factory=lambda: np.array([-0.25, 0.0, 0.0]))
    center_of_pressure: Array = field(
        default_factory=lambda: np.array([0.05, 0.0, 0.0])
    )
    isp: float = 30.0
    g0: float = 9.80665
    ambient_pressure: float = 0.0
    nozzle_area: float = 0.0
    dry_mass: float = 1.0
    ignition_mass: float = 2.0
    thrust_min: float = 1.5
    thrust_max: float = 6.5
    max_gimbal_deg: float = 20.0
    max_tilt_deg: float = 90.0
    max_angular_rate_deg: float = 28.6
    glide_slope_deg: float = 75.0
    trigger_speed: float = 2.0
    max_angle_of_attack_deg: float = 3.0
    terminal_descent_speed: float = 0.1

    @property
    def alpha_mass(self) -> float:
        return 1.0 / (self.isp * self.g0)

    @property
    def beta_mass(self) -> float:
        return self.alpha_mass * self.ambient_pressure * self.nozzle_area


def aerodynamic_force_body(x: Array, p: VehicleParameters) -> Array:
    """Equation (14): ellipsoidal lift/drag force in body coordinates."""
    v_i = x[VEL]
    speed = np.linalg.norm(v_i)
    if speed < 1e-12 or p.rho == 0.0:
        return np.zeros(3)
    c_bi = dcm_body_from_inertial(x[QUAT])
    return -0.5 * p.rho * speed * p.reference_area * p.aero_coefficients @ (c_bi @ v_i)


def dynamics(x: Array, u: Array, p: VehicleParameters) -> Array:
    """Equations (1)-(3), with body-frame thrust control u."""
    m = max(float(x[MASS]), 1e-6)
    q = x[QUAT]
    w = x[OMEGA]
    c_bi = dcm_body_from_inertial(q)
    c_ib = c_bi.T
    a_b = aerodynamic_force_body(x, p)

    dx = np.zeros(NX)
    dx[MASS] = -p.alpha_mass * np.linalg.norm(u) - p.beta_mass
    dx[POS] = x[VEL]
    dx[VEL] = (c_ib @ (u + a_b)) / m + p.gravity
    dx[QUAT] = 0.5 * omega_matrix(w) @ q
    moment = np.cross(p.thrust_arm, u) + np.cross(p.center_of_pressure, a_b)
    dx[OMEGA] = np.linalg.solve(
        p.inertia, moment - np.cross(w, p.inertia @ w)
    )
    return dx


def numerical_jacobians(
    x: Array, u: Array, p: VehicleParameters, relative_step: float = 2e-6
) -> tuple[Array, Array]:
    """Central finite-difference Jacobians of the nonlinear dynamics."""
    a = np.empty((NX, NX))
    b = np.empty((NX, NU))
    for j in range(NX):
        h = relative_step * max(1.0, abs(float(x[j])))
        xp, xm = x.copy(), x.copy()
        xp[j] += h
        xm[j] -= h
        a[:, j] = (dynamics(xp, u, p) - dynamics(xm, u, p)) / (2.0 * h)
    for j in range(NU):
        h = relative_step * max(1.0, abs(float(u[j])))
        up, um = u.copy(), u.copy()
        up[j] += h
        um[j] -= h
        b[:, j] = (dynamics(x, up, p) - dynamics(x, um, p)) / (2.0 * h)
    return a, b


def angle_of_attack(x: Array) -> float:
    """Angle between body +x and the negative inertial velocity, in radians."""
    v = x[VEL]
    speed = np.linalg.norm(v)
    if speed < 1e-10:
        return 0.0
    v_b = dcm_body_from_inertial(x[QUAT]) @ v
    cosine = np.clip(-v_b[0] / speed, -1.0, 1.0)
    return float(np.arccos(cosine))


def tilt_angle(x: Array) -> float:
    body_x_i = dcm_body_from_inertial(x[QUAT]).T[:, 0]
    return float(np.arccos(np.clip(body_x_i[0], -1.0, 1.0)))


def state_trigger_value(x: Array, p: VehicleParameters) -> float:
    """Equation (21): h_alpha <= 0 is the projected continuous STC."""
    v = x[VEL]
    speed = np.linalg.norm(v)
    trigger = max(speed - p.trigger_speed, 0.0)
    if trigger == 0.0:
        return 0.0
    v_b = dcm_body_from_inertial(x[QUAT]) @ v
    constraint = np.cos(np.deg2rad(p.max_angle_of_attack_deg)) * speed + v_b[0]
    return float(trigger * constraint)

