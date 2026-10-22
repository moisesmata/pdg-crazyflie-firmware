"""Transport-independent nominal mission state machine. No planner runs airborne."""
from dataclasses import dataclass
from typing import Protocol
import math
import numpy as np
from .trajectory import Trajectory


@dataclass(frozen=True)
class Observation:
    position: np.ndarray
    velocity: np.ndarray
    quaternion_wxyz: np.ndarray
    stamp: float
    received_at: float


class FlightTransport(Protocol):
    def now(self) -> float: ...
    def wait(self, duration: float): ...
    def observe(self) -> Observation: ...
    def preflight(self): ...
    def arm(self, value: bool): ...
    def takeoff(self, height: float, duration: float): ...
    def go_to(self, position, quaternion_wxyz, duration: float): ...
    def send(self, setpoint): ...
    def release_stream(self): ...
    def land(self, height: float, duration: float): ...
    def record(self, event: dict): ...


class Mission:
    def __init__(self, transport, config):
        self.t, self.c = transport, config
        for key in ('rate_hz', 'takeoff_duration', 'settle_duration', 'max_pose_age',
                    'max_tracking_error', 'max_start_error', 'touchdown_height_tolerance',
                    'touchdown_speed_tolerance'):
            if key == 'max_tracking_error' and config[key] is None:
                continue  # Explicitly disabled by the mission configuration.
            if not math.isfinite(config[key]) or config[key] <= 0:
                raise ValueError(f'Invalid execution setting: {key}')
        if not math.isfinite(config['ground_body_height']) or config['ground_body_height'] < 0:
            raise ValueError('Invalid ground-body height')

    def check_observation(self):
        o = self.t.observe()
        if self.t.now()-o.received_at > self.c['max_pose_age']:
            raise RuntimeError('Motion capture is stale')
        if not all(np.isfinite(v).all() for v in (o.position, o.velocity, o.quaternion_wxyz)):
            raise RuntimeError('Non-finite motion capture')
        return o

    def monitored_wait(self, seconds):
        end = self.t.now()+seconds
        while self.t.now() < end:
            self.t.wait(min(0.02, end-self.t.now()))
            self.check_observation()

    def run(self, plan, vehicle, *, trajectory=None):
        from .ground_health import validate_controller_trial_mode
        controller_trial=validate_controller_trial_mode(self.c,hover=False)
        ignore_heading=self.c.get('ignore_start_heading',False)
        if not isinstance(ignore_heading,bool) or (ignore_heading and not controller_trial):
            raise ValueError('Ignoring start heading requires explicit calibrated controller trial mode')
        tilt_limit=float(self.c.get('max_start_tilt_deg',10.))
        if not math.isfinite(tilt_limit) or not 0<tilt_limit<=30:
            raise ValueError('Invalid descent-start tilt limit')
        if trajectory is None:
            trajectory = Trajectory(plan, vehicle)
        elif not plan.converged or not getattr(trajectory, 'validation', {}).get('passed', False):
            raise ValueError('External reference requires passed independent emulation validation')
        airborne = False
        landing_requested=False
        landing_duration=float(self.c.get('landing_duration',2.))
        recovery_duration=float(self.c.get('recovery_landing_duration',3.))
        staging=bool(self.c.get('stage_to_start',False))
        staging_duration=float(self.c.get('staging_duration',5.))
        if not all(math.isfinite(v) and v>0 for v in (landing_duration,recovery_duration,staging_duration)):
            raise ValueError('Invalid landing/staging duration')
        try:
            self.t.preflight()
            o = self.check_observation()
            if abs(o.position[2]-self.c['ground_body_height']) > self.c['touchdown_height_tolerance']:
                raise RuntimeError('Initial position is not at configured ground-body height')
            ground_xy=np.asarray(self.c.get('ground_start_xy',plan.states[0,:2]),dtype=float) if staging else plan.states[0,:2]
            if ground_xy.shape!=(2,) or not np.isfinite(ground_xy).all():raise ValueError('Invalid staging ground position')
            if staging and not hasattr(self.t,'go_to'):raise RuntimeError('Transport cannot stage to the descent start')
            if np.linalg.norm(o.position[:2]-ground_xy) > self.c['max_start_error']:
                raise RuntimeError('Initial horizontal position differs from plan')
            if np.linalg.norm(o.velocity) > self.c['touchdown_speed_tolerance']:
                raise RuntimeError('Vehicle is moving before takeoff')
            self.t.arm(True)
            # Mark possible flight before requesting takeoff, including a timeout
            # after the server may already have delivered the command.
            airborne = True
            self.t.record({'event': 'takeoff'})
            self.t.takeoff(float(plan.states[0,2]), self.c['takeoff_duration'])
            self.monitored_wait(self.c['takeoff_duration']+self.c['settle_duration'])
            if staging:
                o=self.check_observation()
                if abs(o.position[2]-plan.states[0,2])>self.c['max_start_error'] or np.linalg.norm(o.velocity)>.15:
                    raise RuntimeError('Vertical takeoff did not settle before staging')
                self.t.record({'event':'staging_to_start','position':plan.states[0,:3].tolist()})
                self.t.go_to(plan.states[0,:3],plan.states[0,6:10],staging_duration)
                self.monitored_wait(staging_duration+self.c['settle_duration'])
            o = self.check_observation()
            from .ground_health import start_attitude_errors
            angle,tilt_error=start_attitude_errors(o.quaternion_wxyz,plan.states[0,6:10])
            self.t.record({'event':'start_attitude_check','attitude_error_deg':angle,
                           'tilt_error_deg':tilt_error,'heading_check_disabled':ignore_heading,
                           'tilt_limit_deg':tilt_limit})
            attitude_bad=tilt_error>tilt_limit if ignore_heading else angle>10.
            if (np.linalg.norm(o.position-plan.states[0,:3]) > self.c['max_start_error']
                    or np.linalg.norm(o.velocity-plan.states[0,3:6]) > 0.15 or attitude_bad):
                self.t.record({'event':'start_state_mismatch',
                               'phase':'staging' if staging else 'takeoff',
                               'position':o.position.tolist(),
                               'velocity':o.velocity.tolist(),
                               'quaternion_wxyz':o.quaternion_wxyz.tolist(),
                               'position_error_m':float(np.linalg.norm(o.position-plan.states[0,:3])),
                               'attitude_error_deg':angle,'tilt_error_deg':tilt_error,
                               'heading_check_disabled':ignore_heading})
                raise RuntimeError(('Staging' if staging else 'Takeoff')+' did not reach planned initial state')
            self.t.record({'event': 'descent'})
            start, tick = self.t.now(), 0
            while True:
                elapsed = self.t.now()-start
                target = trajectory.sample(elapsed)
                o = self.check_observation()
                if (not controller_trial and self.c['max_tracking_error'] is not None
                        and np.linalg.norm(o.position-target.position) > self.c['max_tracking_error']):
                    raise RuntimeError('Tracking-error limit exceeded')
                self.t.send(target)
                self.t.record({'event':'sample','time':elapsed,'position':o.position.tolist(),
                               'reference':target.position.tolist()})
                if hasattr(trajectory, 'diagnostic_sample'):
                    self.t.record(trajectory.diagnostic_sample(elapsed, o))
                if elapsed >= plan.times[-1]: break
                tick += 1
                remaining = tick/self.c['rate_hz']-(self.t.now()-start)
                if remaining < -1/self.c['rate_hz']:
                    raise RuntimeError('Missed streaming deadline')
                self.t.wait(max(0, remaining))
            # Queue the bounded landing before releasing low-level priority.
            # Releasing first permits an idle high-level motor-off setpoint.
            self.t.land(max(0., self.c['ground_body_height']-0.02), landing_duration)
            landing_requested=True
            self.t.release_stream()
            # Pose-based settled-contact proxy, NOT a physical contact sensor.
            deadline, settled = self.t.now()+landing_duration+3., None
            while self.t.now() < deadline:
                self.t.wait(0.02)
                o = self.check_observation()
                good = (abs(o.position[2]-self.c['ground_body_height']) <= self.c['touchdown_height_tolerance']
                        and np.linalg.norm(o.position[:2]-plan.states[-1,:2]) <= self.c['max_start_error']
                        and np.linalg.norm(o.velocity) <= self.c['touchdown_speed_tolerance'])
                if good:
                    settled = self.t.now() if settled is None else settled
                    if self.t.now()-settled >= 0.5:
                        self.t.arm(False)
                        airborne = False
                        self.t.record({'event':'landed_pose_proxy'})
                        return
                else: settled = None
            raise RuntimeError('Touchdown proxy not satisfied; operator intervention required')
        except BaseException as error:
            self.t.record({'event':'abort','reason':str(error)})
            if airborne:
                # Best-effort nominal landing; never turn motors off on stale pose.
                # This is not a verified recovery policy or a radio-loss guarantee.
                try:
                    if not landing_requested:
                        self.t.land(max(0., self.c['ground_body_height']-0.02), recovery_duration)
                        landing_requested=True
                    self.t.release_stream()
                    deadline=self.t.now()+max(landing_duration,recovery_duration)+3.;settled=None
                    while self.t.now()<deadline:
                        self.t.wait(.02)
                        try:o=getattr(self.t,'observe_recovery',self.t.observe)()
                        except RuntimeError:
                            settled=None;continue
                        good=(self.t.now()-o.received_at<=self.c['max_pose_age']
                              and np.isfinite(o.position).all() and np.isfinite(o.velocity).all()
                              and abs(o.position[2]-self.c['ground_body_height'])<=self.c['touchdown_height_tolerance']
                              and np.linalg.norm(o.velocity)<=self.c['touchdown_speed_tolerance'])
                        settled=(self.t.now() if settled is None else settled) if good else None
                        if settled is not None and self.t.now()-settled>=.5:
                            self.t.arm(False);airborne=False
                            self.t.record({'event':'recovered_landed_pose_proxy'});break
                    if airborne:self.t.record({'event':'recovery_landing_unconfirmed'})
                except Exception as fallback_error:
                    self.t.record({'event':'landing_request_failed','reason':str(fallback_error)})
            raise
