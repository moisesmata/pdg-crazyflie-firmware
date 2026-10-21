from dataclasses import asdict
from pathlib import Path
import json
import numpy as np
import yaml
from .model import Quadrotor, rest_state
from .contracts import LandingProblem, Plan


def load_problem(path, height=None):
    config = yaml.safe_load(Path(path).read_text())
    v = Quadrotor(**config['vehicle'])
    fields = dict(config['problem'])
    position = fields.pop('initial_position')
    velocity = fields.pop('initial_velocity', [0, 0, 0])
    if height is not None:
        if not np.isfinite(height) or height <= 0:
            raise ValueError('Takeoff height must be finite and positive')
        position[2] = config['execution']['ground_body_height']+height
        fields['ceiling'] = max(fields['ceiling'], position[2]+0.25)
    x0 = rest_state(position); x0[3:6] = velocity
    fields['target_position'] = np.asarray(fields['target_position'])
    return v, LandingProblem(initial_state=x0, **fields), config


def save_plan(path, plan, vehicle, problem):
    path = Path(path); path.parent.mkdir(parents=True, exist_ok=True)
    payload = dict(schema_version=1, frame='ENU', quaternion_order='wxyz',
                   control_units=['m/s^2', 'rad/s^2', 'rad/s^2', 'rad/s^2'],
                   times=plan.times.tolist(), states=plan.states.tolist(),
                   controls=plan.controls.tolist(), converged=plan.converged,
                   diagnostics=plan.diagnostics, vehicle=asdict(vehicle),
                   problem={k: np.asarray(v).tolist() if isinstance(v, np.ndarray) else v
                            for k, v in asdict(problem).items()})
    path.write_text(json.dumps(payload, indent=2, allow_nan=False)+'\n')


def plot_plan(path, plan, vehicle):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    t, x = vehicle.rollout(plan.states[0], plan.controls, plan.times[-1])
    fig, ax = plt.subplots(2, 2, figsize=(10, 7))
    ax[0,0].plot(x[:,0], x[:,2]); ax[0,0].set(xlabel='East [m]', ylabel='Height [m]', title='Independent nonlinear rollout')
    ax[0,1].plot(t, x[:,3:6]); ax[0,1].set(xlabel='Time [s]', ylabel='Velocity [m/s]'); ax[0,1].legend(['x','y','z'])
    ax[1,0].plot(plan.times, plan.controls[:,0]); ax[1,0].set(xlabel='Time [s]', ylabel='Specific thrust [m/s²]')
    ax[1,1].plot(plan.times, plan.controls@vehicle.allocation.T); ax[1,1].set(xlabel='Time [s]', ylabel='Rotor force [N]'); ax[1,1].legend(['M1','M2','M3','M4'])
    fig.tight_layout(); fig.savefig(path, dpi=150); plt.close(fig)
