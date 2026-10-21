"""Independent dense integration checks; these are numerical checks, not certificates."""
import numpy as np


def validate_plan(vehicle, problem, plan):
    t, states = vehicle.rollout(problem.initial_state, plan.controls, plan.times[-1])
    target = np.r_[problem.target_position, problem.terminal_velocity, [1,0,0,0], [0,0,0]]
    controls = np.array([np.interp(t, plan.times, plan.controls[:, j]) for j in range(4)]).T
    forces = controls @ vehicle.allocation.T
    norms = np.linalg.norm(states[:, 6:10], axis=1)
    tilt_cos = (1-2*(states[:, 7]**2+states[:, 8]**2)).clip(-1, 1)
    errors = {
        'terminal_state_error': float(np.max(np.abs(states[-1]-target))),
        'quaternion_norm_error': float(np.max(np.abs(norms-1))),
        'altitude_violation': float(max(0, problem.target_position[2]-states[:, 2].min(), states[:, 2].max()-problem.ceiling)),
        'workspace_violation': float(max(0, np.abs(states[:, :2]).max()-problem.workspace_xy)),
        'speed_violation': float(max(0, np.linalg.norm(states[:, 3:6], axis=1).max()-problem.max_speed)),
        'body_rate_violation': float(max(0, np.linalg.norm(states[:, 10:13], axis=1).max()-problem.max_body_rate)),
        'tilt_violation_rad': float(max(0, np.arccos(tilt_cos).max()-np.deg2rad(problem.max_tilt_deg))),
        'rotor_force_violation_N': float(max(0, -forces.min(), forces.max()-vehicle.max_motor_force)),
        'specific_thrust_violation': float(max(0, problem.min_specific_thrust-controls[:, 0].min(), controls[:, 0].max()-problem.max_specific_thrust)),
        'upward_speed_violation': float(max(0, states[:, 5].max())) if problem.monotonic_descent else 0.,
        'time_violation': float(max(0, problem.min_time-plan.times[-1], plan.times[-1]-problem.max_time)),
    }
    errors['passed'] = bool(all(np.isfinite(v) and v <= 1e-3 for v in errors.values()))
    errors['scope'] = 'dense numerical FOH rollout; no interval or continuous-time certificate'
    return errors
