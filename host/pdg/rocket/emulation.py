"""Scale virtual rocket translation into feasible Earth-gravity drone references.

Virtual rocket attitude/mass/thrust stay in the target model. The quadrotor's
attitude follows the force required to realize the mapped translational motion.
"""
from dataclasses import dataclass
import numpy as np
from scipy.spatial.transform import Rotation
from ..model import Quadrotor
from ..trajectory import Setpoint
from .paper_model import POS,VEL,QUAT,dynamics
from .validation import RocketReplay,validate_result

# A proper rotation: [up,east,north] -> [east,north,up].
UEN_TO_ENU=np.array([[0.,1.,0.],[0.,0.,1.],[1.,0.,0.]])


@dataclass
class EmulatedFlightPlan:
    """Reference geometry for the mission runner, NOT a quadrotor optimal plan."""
    times: np.ndarray
    states: np.ndarray
    converged: bool
    diagnostics: dict


def _positive(config,key):
    x=float(config[key])
    if not np.isfinite(x) or x<=0:raise ValueError(f'Invalid emulation {key}')
    return x


class RocketEmulation:
    def __init__(self,result,config,*,require_settled_start=True):
        self.require_settled_start=require_settled_start
        self.result=result;self.config=config
        self.length_scale=_positive(config,'length_scale_m_per_target_unit')
        self.time_scale=_positive(config,'time_scale_s_per_target_unit')
        self.rate=_positive(config,'validation_rate_hz')
        if not 50<=self.rate<=1000:raise ValueError('Emulation validation rate must be between 50 and 1000 Hz')
        self.offset=np.asarray(config['landing_position_enu_m'],float)
        if self.offset.shape!=(3,) or not np.isfinite(self.offset).all():raise ValueError('Invalid landing position')
        self.yaw=float(config.get('drone_heading_deg',0));world_yaw=float(config.get('world_yaw_deg',0))
        if not np.isfinite([self.yaw,world_yaw]).all():raise ValueError('Invalid emulation headings')
        self.rotation=Rotation.from_euler('z',world_yaw,degrees=True).as_matrix()@UEN_TO_ENU
        self.drone=Quadrotor(**config['drone']);self.limits=dict(config['limits'])
        for k in ['max_speed_m_s','max_tilt_deg','max_body_rate_rad_s','max_specific_thrust_m_s2',
                  'min_specific_thrust_m_s2','workspace_xy_m','ceiling_m']:_positive(self.limits,k)
        reserve=float(self.limits['motor_reserve_fraction'])
        if not np.isfinite(reserve) or not 0<=reserve<.5:raise ValueError('Invalid motor reserve fraction')
        if not 0<self.limits['max_tilt_deg']<80 or self.limits['min_specific_thrust_m_s2']>=self.limits['max_specific_thrust_m_s2']:
            raise ValueError('Invalid mapped tilt/thrust limits')
        self.replay=RocketReplay(result);self.target_validation=validate_result(result,self.replay)
        if not self.target_validation['passed']:raise ValueError('Rocket target failed independent validation')
        self.duration=float(result.burn_time*self.time_scale)
        if self.duration*self.rate>200000:raise ValueError('Mapped duration exceeds validation sample budget')
        # Include every FOH knot as well as a regular grid. This exposes fast
        # changes in requested attitude/rates at thrust-slope transitions.
        grid=np.sort(np.r_[np.linspace(0,self.duration,int(np.ceil(self.duration*self.rate))+1),
                           self.replay.times*self.time_scale])
        self.grid=grid[np.r_[True,np.diff(grid)>1e-7]]
        self.grid[-1]=self.duration
        self._core_cache=[self.core_at(t) for t in self.grid]
        rotations=np.array([a[4] for a in self._core_cache])
        derivative=np.gradient(rotations,self.grid,axis=0,edge_order=2)
        skew=np.einsum('nji,njk->nik',rotations,derivative)
        self.rates=.5*np.column_stack((skew[:,2,1]-skew[:,1,2],skew[:,0,2]-skew[:,2,0],skew[:,1,0]-skew[:,0,1]))
        self.angular_acceleration=np.gradient(self.rates,self.grid,axis=0,edge_order=2)
        self.validation=self._validate()
        mapped_times=self.replay.times*self.time_scale
        mapped_states=np.array([self._state_at(t) for t in mapped_times])
        self.plan=EmulatedFlightPlan(mapped_times,mapped_states,self.validation['passed'],
                                    dict(kind='rocket_translation_emulation',emulation_validation=self.validation,
                                         target_validation=self.target_validation))

    def core_at(self,time):
        """Exact chain-rule map of independently replayed target translation."""
        virtual_time=float(np.clip(time/self.time_scale,0,self.result.burn_time))
        x,u=self.replay.at(virtual_time)
        position=self.offset+self.length_scale*(self.rotation@x[POS])
        velocity=self.length_scale/self.time_scale*(self.rotation@x[VEL])
        acceleration=self.length_scale/self.time_scale**2*(self.rotation@dynamics(x,u,self.result.parameters)[VEL])
        specific_force=acceleration+np.array([0.,0.,self.drone.gravity])
        magnitude=np.linalg.norm(specific_force)
        if not np.isfinite(magnitude) or magnitude<1e-8:raise ValueError('Mapped motion requires zero/invalid drone thrust')
        z=specific_force/magnitude
        heading=np.array([np.cos(np.deg2rad(self.yaw)),np.sin(np.deg2rad(self.yaw)),0.])
        y=np.cross(z,heading);norm=np.linalg.norm(y)
        if norm<1e-6:raise ValueError('Mapped thrust parallel to desired heading')
        y/=norm;rotation=np.column_stack((np.cross(y,z),y,z))
        q=Rotation.from_matrix(rotation).as_quat()[[3,0,1,2]]
        if q[0]<0:q=-q
        return position,velocity,acceleration,q,rotation,magnitude

    def _rates_at(self,time):return np.array([np.interp(time,self.grid,self.rates[:,i]) for i in range(3)])
    def _state_at(self,time):
        p,v,a,q,_,_=self.core_at(time)
        return np.r_[p,v,q,self._rates_at(time)]

    def _validate(self):
        p=np.array([a[0] for a in self._core_cache]);v=np.array([a[1] for a in self._core_cache])
        force=np.array([a[5] for a in self._core_cache]);R=np.array([a[4] for a in self._core_cache])
        tilt=np.rad2deg(np.arccos(np.clip(R[:,2,2],-1,1)))
        J=np.asarray(self.drone.inertia);torque=self.angular_acceleration*J+np.cross(self.rates,self.rates*J)
        control=np.column_stack((force,torque/J));motors=control@self.drone.allocation.T
        margin=self.limits['motor_reserve_fraction']*self.drone.max_motor_force
        error={
          'workspace_m':max(0.,float(np.abs(p[:,:2]).max()-self.limits['workspace_xy_m'])),
          'ceiling_m':max(0.,float(p[:,2].max()-self.limits['ceiling_m'])),
          'below_landing_plane_m':max(0.,float(self.offset[2]-p[:,2].min())),
          'speed_m_s':max(0.,float(np.linalg.norm(v,axis=1).max()-self.limits['max_speed_m_s'])),
          'tilt_deg':max(0.,float(tilt.max()-self.limits['max_tilt_deg'])),
          'body_rate_rad_s':max(0.,float(np.linalg.norm(self.rates,axis=1).max()-self.limits['max_body_rate_rad_s'])),
          'specific_thrust_lower_m_s2':max(0.,float(self.limits['min_specific_thrust_m_s2']-force.min())),
          'specific_thrust_upper_m_s2':max(0.,float(force.max()-self.limits['max_specific_thrust_m_s2'])),
          'motor_force_lower_N':max(0.,float(margin-motors.min())),
          'motor_force_upper_N':max(0.,float(motors.max()-(self.drone.max_motor_force-margin))),
        }
        # The existing takeoff/settle runner starts from rest. It cannot reproduce
        # a nonzero-speed ignition condition without a separately designed entry.
        error['settled_start_speed_m_s']=max(0.,float(np.linalg.norm(v[0])-.15)) if self.require_settled_start else 0.
        finite=all(np.isfinite(a).all() for a in [p,v,force,R,self.rates,self.angular_acceleration,motors])
        return dict(passed=bool(finite and all(z<=1e-6 for z in error.values())),violations=error,
                    peaks=dict(speed_m_s=float(np.linalg.norm(v,axis=1).max()),tilt_deg=float(tilt.max()),
                               body_rate_rad_s=float(np.linalg.norm(self.rates,axis=1).max()),
                               specific_thrust_min_m_s2=float(force.min()),specific_thrust_max_m_s2=float(force.max()),
                               motor_force_min_N=float(motors.min()),motor_force_max_N=float(motors.max())),
                    samples=len(self.grid),duration_s=self.duration,phase='powered descent; target coast is not replayed',
                    scope='sampled feedforward feasibility for assumed drone model, including finite-difference attitude rates/torques; not a tracking, timing, intersample or robustness guarantee',
                    emulated_quantities=['scaled position','scaled velocity','scaled kinematic acceleration'],
                    non_emulated_quantities=['rocket mass depletion on physical vehicle','gimbaled thrust direction relative to rocket body','rocket attitude/body rates','rocket aerodynamic environment'])

    def sample(self,time):
        if not self.validation['passed']:raise ValueError('Mapped rocket reference failed drone feasibility checks')
        t=float(np.clip(time,0,self.duration));p,v,a,q,_,_=self.core_at(t)
        return Setpoint(p,v,a,q,self._rates_at(t))

    def diagnostic_sample(self,time,observation):
        target_time=float(np.clip(time/self.time_scale,0,self.result.burn_time));x,u=self.replay.at(target_time)
        observed_target_position=self.rotation.T@(observation.position-self.offset)/self.length_scale
        observed_target_velocity=self.rotation.T@observation.velocity*self.time_scale/self.length_scale
        return dict(event='rocket_emulation_sample',target_time=target_time,target_state_reference=x.tolist(),
                    target_thrust_reference=u.tolist(),observed_position_target_UEN=observed_target_position.tolist(),
                    observed_velocity_target_UEN=observed_target_velocity.tolist(),
                    scope='inverse-mapped drone translation only; virtual mass/attitude are reference, not physical observations')
