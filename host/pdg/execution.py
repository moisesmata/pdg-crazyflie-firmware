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

    def run(self, plan, vehicle):
        trajectory = Trajectory(plan, vehicle)
        airborne = False
        try:
            self.t.preflight()
            o = self.check_observation()
            if abs(o.position[2]-self.c['ground_body_height']) > self.c['touchdown_height_tolerance']:
                raise RuntimeError('Initial position is not at configured ground-body height')
            if np.linalg.norm(o.position[:2]-plan.states[0,:2]) > self.c['max_start_error']:
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
            o = self.check_observation()
            angle = 2*np.arccos(np.clip(abs(np.dot(o.quaternion_wxyz,plan.states[0,6:10])),0,1))
            if (np.linalg.norm(o.position-plan.states[0,:3]) > self.c['max_start_error']
                    or np.linalg.norm(o.velocity-plan.states[0,3:6]) > 0.15 or angle > np.deg2rad(10)):
                raise RuntimeError('Takeoff did not reach planned initial state')
            self.t.record({'event': 'descent'})
            start, tick = self.t.now(), 0
            while True:
                elapsed = self.t.now()-start
                target = trajectory.sample(elapsed)
                o = self.check_observation()
                if np.linalg.norm(o.position-target.position) > self.c['max_tracking_error']:
                    raise RuntimeError('Tracking-error limit exceeded')
                self.t.send(target)
                self.t.record({'event':'sample','time':elapsed,'position':o.position.tolist(),
                               'reference':target.position.tolist()})
                if elapsed >= plan.times[-1]: break
                tick += 1
                remaining = tick/self.c['rate_hz']-(self.t.now()-start)
                if remaining < -1/self.c['rate_hz']:
                    raise RuntimeError('Missed streaming deadline')
                self.t.wait(max(0, remaining))
            self.t.release_stream()
            self.t.land(max(0., self.c['ground_body_height']-0.02), 2.)
            # Pose-based settled-contact proxy, NOT a physical contact sensor.
            deadline, settled = self.t.now()+5., None
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
                    self.t.release_stream()
                    self.t.land(max(0., self.c['ground_body_height']-0.02), 3.)
                except Exception as fallback_error:
                    self.t.record({'event':'landing_request_failed','reason':str(fallback_error)})
            raise
