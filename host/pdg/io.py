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


def load_validated_plan(path, vehicle, problem):
    """Reuse an offline plan only for its exact model/problem, with fresh checks."""
    from .validation import validate_plan
    data=json.loads(Path(path).read_text())
    expected_problem={k:np.asarray(v).tolist() if isinstance(v,np.ndarray) else v
                      for k,v in asdict(problem).items()}
    # Normalize tuples into JSON arrays before comparing provenance.
    expected_vehicle=json.loads(json.dumps(asdict(vehicle)))
    expected_problem=json.loads(json.dumps(expected_problem))
    if (data.get('schema_version')!=1 or data.get('frame')!='ENU' or
        data.get('quaternion_order')!='wxyz' or
        data.get('control_units')!=['m/s^2','rad/s^2','rad/s^2','rad/s^2'] or
        data.get('vehicle')!=expected_vehicle or data.get('problem')!=expected_problem):
        raise ValueError('Saved plan does not match the requested model, coordinates, or problem')
    times=np.asarray(data['times'],dtype=float);states=np.asarray(data['states'],dtype=float)
    controls=np.asarray(data['controls'],dtype=float);n=len(times)
    if (n!=problem.nodes or states.shape!=(n,13) or controls.shape!=(n,4) or
        not all(np.isfinite(x).all() for x in (times,states,controls)) or
        times.ndim!=1 or times[0]!=0 or np.any(np.diff(times)<=0) or
        not np.allclose(times,np.linspace(0,times[-1],n),rtol=0,atol=1e-8) or
        not data.get('converged')):
        raise ValueError('Invalid saved plan arrays or convergence flag')
    if np.max(np.abs(states[0]-problem.initial_state))>1e-6:
        raise ValueError('Saved plan initial state differs from the problem')
    defects=[np.max(np.abs(states[k+1]-vehicle.step(states[k],controls[k],controls[k+1],times[k+1]-times[k])))
             for k in range(n-1)]
    if max(defects)>1e-4:raise ValueError('Saved plan has inconsistent trajectory nodes')
    plan=Plan(times,states,controls,True,dict(data.get('diagnostics',{})))
    plan.diagnostics['validation']=validate_plan(vehicle,problem,plan)
    if not plan.diagnostics['validation']['passed']:
        raise ValueError('Saved plan failed independent nonlinear validation')
    plan.diagnostics['loaded_from']=str(Path(path).resolve())
    return plan


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
