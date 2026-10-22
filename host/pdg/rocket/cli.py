"""Rocket plan -> independent replay -> drone emulation -> existing mission runner."""
from pathlib import Path
import json
import numpy as np
from .io import load_config,save_result,load_result,plain
from .paper_scvx import solve
from .paper_model import MASS,POS,VEL,QUAT,dcm_body_from_inertial
from .validation import validate_result
from .emulation import RocketEmulation


def plot_target(path,result):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    x,u=result.states,result.controls;t=result.time
    fig,ax=plt.subplots(2,2,figsize=(10,7))
    ax[0,0].plot(x[2],x[1]);ax[0,0].set(xlabel='Target east [target length]',ylabel='Target up [target length]',title='Rocket PDG — target dynamics')
    ax[0,1].plot(t,x[MASS]);ax[0,1].set(xlabel='Target burn time',ylabel='Virtual rocket mass')
    ax[1,0].plot(t,np.linalg.norm(u,axis=0));ax[1,0].set(xlabel='Target burn time',ylabel='Body thrust magnitude [target force]')
    tilt=[np.rad2deg(np.arccos(np.clip(dcm_body_from_inertial(q).T[0,0],-1,1))) for q in x[QUAT].T]
    ax[1,1].plot(t,tilt);ax[1,1].set(xlabel='Target burn time',ylabel='Virtual rocket tilt [deg]')
    fig.tight_layout();fig.savefig(path,dpi=160);plt.close(fig)


def plot_mapping(path,adapter):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    t=adapter.grid;values=adapter._core_cache
    p=np.array([v[0] for v in values]);vel=np.array([v[1] for v in values])
    force=np.array([v[5] for v in values]);R=np.array([v[4] for v in values])
    tilt=np.rad2deg(np.arccos(np.clip(R[:,2,2],-1,1)))
    fig,ax=plt.subplots(2,2,figsize=(10,7))
    ax[0,0].plot(p[:,0],p[:,2]);ax[0,0].set(xlabel='Lab east [m]',ylabel='Lab altitude [m]',title='Mapped rocket translation — reference only')
    ax[0,1].plot(t,np.linalg.norm(vel,axis=1));ax[0,1].axhline(adapter.limits['max_speed_m_s'],ls='--',c='gray');ax[0,1].set(xlabel='Lab time [s]',ylabel='Speed [m/s]')
    ax[1,0].plot(t,tilt);ax[1,0].axhline(adapter.limits['max_tilt_deg'],ls='--',c='gray');ax[1,0].set(xlabel='Lab time [s]',ylabel='Drone tilt [deg]')
    ax[1,1].plot(t,force);ax[1,1].axhline(adapter.drone.gravity,ls=':',c='gray');ax[1,1].axhline(adapter.limits['max_specific_thrust_m_s2'],ls='--',c='gray');ax[1,1].set(xlabel='Lab time [s]',ylabel='Drone thrust / mass [m/s²]')
    fig.tight_layout();fig.savefig(path,dpi=160);plt.close(fig)


def save_flight_reference(path,adapter,rate):
    ts=np.linspace(0,adapter.duration,int(np.ceil(adapter.duration*rate))+1)
    points=[adapter.sample(t) for t in ts]
    data=dict(schema_version=2,kind='mapped_drone_reference',frame='ENU',quaternion_order='wxyz',
              units='SI',times_s=ts,position_m=np.array([s.position for s in points]),
              velocity_m_s=np.array([s.velocity for s in points]),acceleration_m_s2=np.array([s.acceleration for s in points]),
              quaternion_wxyz=np.array([s.quaternion_wxyz for s in points]),body_rates_rad_s=np.array([s.body_rates for s in points]),
              emulation=adapter.config,target_reference='target-plan.json',
              scope='scaled rocket translation; drone attitude from Earth-gravity force balance, not rocket attitude')
    Path(path).write_text(json.dumps(plain(data),indent=2,allow_nan=False)+'\n')


def run(args):
    p,cfg,c=load_config(args.config)
    if args.control_smoothing:
        raise ValueError('--control-smoothing is a legacy quadrotor-planner option; rocket SCvx uses its declared trust/virtual-control objective')
    if args.height is not None:
        if not np.isfinite(args.height) or args.height<=0:raise ValueError('Invalid lab takeoff height')
        if cfg.free_ignition_time:raise ValueError('--height is ambiguous with optimized ignition; set the emulation scale explicitly')
        # Change only the lab mapping, never silently change target dynamics.
        c['emulation']['length_scale_m_per_target_unit']=args.height/cfg.initial_position[0]
    directory=Path(args.output);directory.mkdir(parents=True,exist_ok=True)
    result=load_result(args.plan_file,p,cfg,c['rocket']['units']) if args.plan_file else solve(p,cfg,verbose=True)
    report=validate_result(result)
    save_result(directory/'target-plan.json',result,c['rocket']['units'],report)
    (directory/'target-validation.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(dict(target_converged=result.converged,target_validation=report,
                         target_burn_time=result.burn_time,virtual_fuel_used=report['fuel_used']),indent=2))
    if not report['passed']:raise SystemExit('Rocket target rejected. No flight commands sent.')
    adapter=RocketEmulation(result,c['emulation'])
    (directory/'emulation-validation.json').write_text(json.dumps(adapter.validation,indent=2)+'\n')
    print(json.dumps(dict(emulation_validation=adapter.validation),indent=2))
    if not adapter.validation['passed']:raise SystemExit('Rocket trajectory cannot be replayed within declared drone limits; adjust target/mapping. No flight commands sent.')
    save_flight_reference(directory/'flight-reference.json',adapter,c['execution']['rate_hz'])
    plot_target(directory/'target-trajectory.png',result)
    plot_mapping(directory/'lab-reference.png',adapter)
    execution=dict(c['execution']);execution['operator_controller_trial']=args.controller_trial
    if not np.isclose(adapter.offset[2],execution['ground_body_height'],atol=1e-9,rtol=0):
        raise ValueError('Mapped landing plane must match the execution ground-body height')
    from ..ground_health import validate_controller_trial_mode
    validate_controller_trial_mode(execution,hover=False)
    (directory/'execution.json').write_text(json.dumps(execution,indent=2)+'\n')
    if args.command=='mission' and args.execute:
        from ..ros_mission import run_mission
        run_mission(adapter.plan,adapter.drone,execution,directory,trajectory=adapter)
    else:
        print('DRY RUN: virtual rocket plan and mapped drone reference validated. No ROS connection or flight commands.')
