"""Independent, dense checks of the original rocket equations (not a certificate)."""
import numpy as np
from scipy.integrate import solve_ivp
from .paper_model import (MASS,POS,VEL,QUAT,OMEGA,dynamics,
    dcm_body_from_inertial,angle_of_attack)


def validate_inputs(p,cfg):
    for name in ['gravity','thrust_arm','center_of_pressure']:
        a=np.asarray(getattr(p,name))
        if a.shape!=(3,) or not np.isfinite(a).all():raise ValueError(f'Invalid rocket {name}')
    if np.linalg.norm(p.gravity)<=0:raise ValueError('Rocket gravity must be nonzero')
    for name in ['inertia','aero_coefficients']:
        a=np.asarray(getattr(p,name))
        if (a.shape!=(3,3) or not np.isfinite(a).all() or
            not np.allclose(a,a.T) or np.linalg.eigvalsh(a).min()<=0):
            raise ValueError(f'Rocket {name} must be symmetric positive definite')
    positive=['reference_area','isp','g0','dry_mass','ignition_mass','thrust_min','thrust_max',
              'max_gimbal_deg','max_tilt_deg','max_angular_rate_deg','glide_slope_deg',
              'trigger_speed','max_angle_of_attack_deg']
    if any(not np.isfinite(getattr(p,k)) or getattr(p,k)<=0 for k in positive):
        raise ValueError('Rocket parameters must be finite and positive')
    for name in ['rho','ambient_pressure','nozzle_area','terminal_descent_speed']:
        if not np.isfinite(getattr(p,name)) or getattr(p,name)<0:raise ValueError(f'Invalid rocket {name}')
    if not p.ignition_mass>p.dry_mass or not p.thrust_max>p.thrust_min:
        raise ValueError('Invalid rocket mass/thrust bounds')
    if not 0<p.max_gimbal_deg<90 or not 0<p.max_tilt_deg<=90 or not 0<p.glide_slope_deg<90 or not 0<p.max_angle_of_attack_deg<90:
        raise ValueError('Invalid rocket angular bounds')
    if not isinstance(cfg.nodes,int) or cfg.nodes<5 or not isinstance(cfg.max_iterations,int) or cfg.max_iterations<1:
        raise ValueError('Invalid rocket solver dimensions')
    if (not np.isfinite([cfg.min_burn_time,cfg.initial_burn_time,cfg.max_burn_time]).all() or
        not 0<cfg.min_burn_time<=cfg.initial_burn_time<=cfg.max_burn_time):
        raise ValueError('Invalid rocket burn-time bounds')
    for name in ['initial_position','initial_velocity']:
        a=np.asarray(getattr(cfg,name))
        if a.shape!=(3,) or not np.isfinite(a).all():raise ValueError(f'Invalid rocket {name}')
    for name,size in [('initial_quaternion',4),('initial_body_rate',3),('initial_thrust',3),('initial_thrust_rate',3)]:
        value=getattr(cfg,name)
        if value is not None:
            a=np.asarray(value)
            if a.shape!=(size,) or not np.isfinite(a).all():raise ValueError(f'Invalid rocket {name}')
    if cfg.initial_quaternion is not None and abs(np.linalg.norm(cfg.initial_quaternion)-1)>1e-4:
        raise ValueError('Initial rocket quaternion must have unit norm')
    if cfg.initial_thrust_rate is not None and cfg.initial_thrust is None:
        raise ValueError('Initial thrust rate requires initial thrust')
    if cfg.free_ignition_time and any(getattr(cfg,k) is not None for k in ['initial_quaternion','initial_body_rate','initial_thrust','initial_thrust_rate']):
        raise ValueError('Continuation boundary conditions require fixed ignition time')
    if not np.isfinite(cfg.path_margin_fraction) or not 0<=cfg.path_margin_fraction<.5:
        raise ValueError('Invalid rocket path margin fraction')
    if cfg.initial_position[0]<=0:raise ValueError('Rocket starts above target plane in UEN coordinates')
    for name in ['use_aerodynamics','use_state_trigger','free_ignition_time','continuous_thrust_inner_bound']:
        if not isinstance(getattr(cfg,name),bool):raise ValueError(f'{name} must be boolean')
    for name in ['virtual_control_weight','trust_region_weight','virtual_control_tolerance',
                 'trust_region_tolerance','integrator_rtol','integrator_atol']:
        if not np.isfinite(getattr(cfg,name)) or getattr(cfg,name)<=0:raise ValueError(f'Invalid {name}')
    if (not np.isfinite(cfg.max_coast_time) or cfg.max_coast_time<0 or
        not isinstance(cfg.substeps_per_interval,int) or cfg.substeps_per_interval<1):raise ValueError('Invalid rocket integration/coast settings')


class RocketReplay:
    """Single continuous powered-phase rollout; never reset to optimized nodes."""
    def __init__(self,result):
        self.result=result;p=result.parameters
        self.times=np.linspace(0,result.burn_time,result.states.shape[1])
        self.solutions=[];state=np.asarray(result.states[:,0],float)
        for k in range(len(self.times)-1):
            a,b=self.times[k:k+2];ua,ub=result.controls[:,k:k+2].T
            def ode(t,x,a=a,b=b,ua=ua,ub=ub):
                u=ua+(ub-ua)*(t-a)/(b-a)
                return dynamics(x,u,p)
            sol=solve_ivp(ode,(a,b),state,method='DOP853',rtol=1e-10,atol=1e-12,
                          max_step=(b-a)/8,dense_output=True)
            if not sol.success:raise RuntimeError('Independent rocket propagation failed: '+sol.message)
            self.solutions.append(sol);state=sol.y[:,-1]
    def at(self,time):
        time=float(np.clip(time,0,self.times[-1]));k=min(np.searchsorted(self.times,time,side='right')-1,len(self.solutions)-1)
        k=max(0,k);a,b=self.times[k:k+2]
        u=self.result.controls[:,k]+(self.result.controls[:,k+1]-self.result.controls[:,k])*(time-a)/(b-a)
        return self.solutions[k].sol(time),u


def validate_result(result,replay=None):
    p,cfg=result.parameters,result.config;validate_inputs(p,cfg)
    if (result.states.shape!=(14,cfg.nodes) or result.controls.shape!=(3,cfg.nodes) or
        not np.isfinite(result.states).all() or not np.isfinite(result.controls).all() or
        not np.isfinite([result.burn_time,result.coast_time]).all()):raise ValueError('Invalid rocket plan arrays')
    if result.burn_time<=0:raise ValueError('Rocket burn time must be positive before replay')
    replay=replay or RocketReplay(result)
    times=np.linspace(0,result.burn_time,32*(cfg.nodes-1)+1)
    pairs=[replay.at(t) for t in times];x=np.array([a for a,_ in pairs]);u=np.array([b for _,b in pairs])
    thrust=np.linalg.norm(u,axis=1);q=x[:,QUAT];qn=np.linalg.norm(q,axis=1)
    tilt=np.rad2deg(np.arccos(np.clip(1-2*(q[:,2]**2+q[:,3]**2),-1,1)))
    speed=np.linalg.norm(x[:,VEL],axis=1)
    aoa=np.array([np.rad2deg(angle_of_attack(z)) for z in x])
    active=(speed>p.trigger_speed+1e-8) if cfg.use_state_trigger else np.zeros(len(x),bool)
    coast=result.coast_time;initial_position=cfg.initial_position+cfg.initial_velocity*coast+.5*p.gravity*coast**2
    initial_velocity=cfg.initial_velocity+p.gravity*coast
    end=x[-1];terminal_q=end[QUAT]/np.linalg.norm(end[QUAT])
    scale_x=np.r_[max(1,p.ignition_mass),np.full(3,max(1,np.linalg.norm(cfg.initial_position))),np.full(3,max(1,np.linalg.norm(cfg.initial_velocity))),np.ones(7)]
    defects=[]
    for k,t in enumerate(replay.times):defects.append(np.max(np.abs((replay.at(t)[0]-result.states[:,k])/scale_x)))
    residuals={
      'initial_mass':abs(float(x[0,MASS]-p.ignition_mass)),
      'initial_position':float(np.linalg.norm(x[0,POS]-initial_position)),
      'initial_velocity':float(np.linalg.norm(x[0,VEL]-initial_velocity)),
      'initial_body_rate':float(np.linalg.norm(x[0,OMEGA]-(np.zeros(3) if cfg.initial_body_rate is None else cfg.initial_body_rate))),
      'dry_mass':max(0.,float(p.dry_mass-x[:,MASS].min())),
      'glide_slope':max(0.,float((np.linalg.norm(x[:,2:4],axis=1)/np.tan(np.deg2rad(p.glide_slope_deg))-x[:,1]).max())),
      'tilt_deg':max(0.,float(tilt.max()-p.max_tilt_deg)),
      'body_rate_rad_s':max(0.,float(np.linalg.norm(x[:,OMEGA],axis=1).max()-np.deg2rad(p.max_angular_rate_deg))),
      'thrust_lower':max(0.,float(p.thrust_min-thrust.min())),
      'thrust_upper':max(0.,float(thrust.max()-p.thrust_max)),
      'gimbal_cone':max(0.,float((np.cos(np.deg2rad(p.max_gimbal_deg))*thrust-u[:,0]).max())),
      'active_aoa_deg':max(0.,float(aoa[active].max()-p.max_angle_of_attack_deg)) if active.any() else 0.,
      'quaternion_norm':float(np.max(abs(qn-1))),
      'terminal_position':float(np.linalg.norm(end[POS])),
      'terminal_velocity':float(np.linalg.norm(end[VEL]-[-p.terminal_descent_speed,0,0])),
      'terminal_attitude_rad':float(2*np.arccos(np.clip(abs(terminal_q[0]),0,1))),
      'terminal_body_rate':float(np.linalg.norm(end[OMEGA])),
      'normalized_node_replay_error':float(max(defects)),
      'burn_time_bounds':max(0.,float(cfg.min_burn_time-result.burn_time),float(result.burn_time-cfg.max_burn_time)),
      'coast_time_bounds':max(0.,float(-coast),float(coast-cfg.max_coast_time)) if cfg.free_ignition_time else abs(float(coast)),
    }
    if cfg.initial_quaternion is not None:
        residuals['initial_quaternion']=float(np.linalg.norm(x[0,QUAT]-cfg.initial_quaternion))
    if cfg.initial_thrust is not None:
        residuals['initial_thrust']=float(np.linalg.norm(u[0]-cfg.initial_thrust))
    if cfg.initial_thrust_rate is not None:
        rate=(result.controls[:,1]-result.controls[:,0])/(result.burn_time/(cfg.nodes-1))
        residuals['initial_thrust_rate']=float(np.linalg.norm(rate-cfg.initial_thrust_rate))
    tolerances={k:1e-3 for k in residuals}
    for k in ['quaternion_norm','normalized_node_replay_error']:tolerances[k]=1e-4
    terminal_ok=all(np.isfinite(v) and v<=tolerances[k] for k,v in residuals.items())
    return dict(passed=bool(result.converged and terminal_ok),optimizer_converged=bool(result.converged),
                residuals=residuals,tolerances=tolerances,samples=len(times),
                scope='independent dense powered-phase nonlinear replay; numerical tolerances in declared target units; no continuous-time certificate',
                fuel_used=float(x[0,MASS]-x[-1,MASS]),final_mass=float(x[-1,MASS]))
