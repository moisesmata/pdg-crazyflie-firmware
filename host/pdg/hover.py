"""Short high-level hover check, independent of the PDG force/mass model."""
import numpy as np
from .execution import Mission
from .ground_health import validate_hover_alignment, hover_ground_angles,validate_calibrated_start_attitude


class HoverMission(Mission):
    def run(self, plan, vehicle=None):
        target=plan.states[0,:3]
        ground=float(self.c['ground_body_height'])
        if target.shape!=(3,) or not np.isfinite(target).all() or not np.isfinite(ground) or ground<0 or not .1-1e-9<=target[2]-ground<=1.0+1e-9:
            raise ValueError('Hover requires a finite ground and rise between 0.10 and 1.00 m')
        airborne=False
        initial_q=None
        trial=bool(self.c.get('operator_controller_trial',False))
        if trial and not (self.c.get('baseline_pid') and self.c.get('vicon_only')):
            raise ValueError('Controller trial requires Vicon-only PID mode')
        lateral=float(self.c.get('hover_max_lateral_deviation',.08))
        overshoot=float(self.c.get('hover_max_vertical_overshoot',.08))
        tilt=float(self.c.get('hover_max_tilt_deg',15.))
        attitude=self.c.get('hover_max_attitude_change_deg',15.)
        endpoint=float(self.c.get('hover_endpoint_tolerance',.05))
        settle=float(self.c.get('hover_settle_duration',1.))
        hold=float(self.c.get('hover_hold_duration',2.))
        landing_duration=float(self.c.get('hover_landing_duration',4.))
        if not np.isfinite(hold) or not 1<=hold<=30:
            raise ValueError('Hover hold duration must be between 1 and 30 seconds')
        if not (np.isfinite([lateral,overshoot,tilt,endpoint,settle,landing_duration]).all()
                and .025<=lateral<=.30 and .025<=overshoot<=.20 and 5<=tilt<=25 and .025<=endpoint<=.20
                and 1<=settle<=4 and 4<=landing_duration<=8):
            raise ValueError('Invalid hover envelope or timing')
        if attitude is not None and (not np.isfinite(attitude) or not 5<=attitude<=90):
            raise ValueError('Invalid hover attitude-change limit')
        landing_started=None
        def begin_landing():
            nonlocal landing_started
            # Firmware ignores another Land while already landing. Keep the
            # original timer if nominal landing subsequently enters recovery.
            if landing_started is None:
                self.t.land(max(0.,ground-.01),landing_duration)
                landing_started=self.t.now()
        def observe():
            o=self.check_observation()
            if not trial and np.linalg.norm(o.position[:2]-target[:2])>lateral:
                raise RuntimeError(f'Hover lateral deviation exceeds {lateral*100:g} cm')
            if not trial and not ground-.04 <= o.position[2] <= target[2]+overshoot:
                raise RuntimeError('Hover vertical envelope exceeded')
            if not trial and hover_ground_angles(o.quaternion_wxyz)[0]>tilt:
                raise RuntimeError(f'Hover tilt exceeds {tilt:g} degrees')
            if not trial and attitude is not None and initial_q is not None and 2*np.arccos(np.clip(abs(np.dot(o.quaternion_wxyz,initial_q)),0,1))>np.deg2rad(attitude):
                raise RuntimeError(f'Hover attitude changes more than {attitude:g} degrees from its initial alignment')
            self.t.record({'event':'hover_sample','position':o.position.tolist(),
                           'velocity':o.velocity.tolist(),'quaternion_wxyz':o.quaternion_wxyz.tolist()})
            return o
        def wait(seconds):
            end=self.t.now()+seconds
            while self.t.now()<end:
                self.t.wait(min(.02,end-self.t.now()))
                observe()
        try:
            self.t.preflight()
            o=self.check_observation()
            if abs(o.position[2]-ground)>.02 or np.linalg.norm(o.position[:2]-target[:2])>.025:
                raise RuntimeError('Hover ground start differs from measured pad')
            if np.linalg.norm(o.velocity)>.05:
                raise RuntimeError('Vehicle moving before hover')
            validate_hover_alignment(o.quaternion_wxyz,preserve_heading=self.c.get('preserve_heading',False))
            if 'start_quaternion_wxyz' in self.c:
                validate_calibrated_start_attitude(o.quaternion_wxyz,self.c['start_quaternion_wxyz'])
            initial_q=o.quaternion_wxyz.copy()
            self.t.arm(True)
            airborne=True
            self.t.record({'event':'hover_takeoff'})
            self.t.takeoff(float(target[2]),4.)
            wait(4.+settle)
            o=observe()
            if not trial and (np.linalg.norm(o.position-target)>endpoint or np.linalg.norm(o.velocity)>.15):
                raise RuntimeError('Hover takeoff endpoint not reached')
            self.t.record({'event':'hover_hold'})
            wait(hold)
            begin_landing()
            self.t.record({'event':'hover_landing'})
            deadline=self.t.now()+landing_duration+3.;settled=None
            while self.t.now()<deadline:
                self.t.wait(.02)
                o=observe()
                good=abs(o.position[2]-ground)<.02 and np.linalg.norm(o.velocity)<.05
                settled=(self.t.now() if settled is None else settled) if good else None
                if settled is not None and self.t.now()-settled>=.75:
                    self.t.arm(False);airborne=False
                    self.t.record({'event':'hover_landed_pose_proxy'})
                    return
            raise RuntimeError('Hover touchdown not confirmed; operator intervention required')
        except BaseException as exc:
            self.t.record({'event':'hover_abort','reason':str(exc)})
            if airborne:
                try:
                    begin_landing()
                    self.t.record({'event':'hover_recovery_landing'})
                    # Continue supervision after a fault instead of exiting
                    # immediately after merely queuing the landing request.
                    deadline=self.t.now()+landing_duration+3.;settled=None;last_recovery_error=None
                    while self.t.now()<deadline:
                        self.t.wait(.02)
                        try:
                            o=getattr(self.t,'observe_recovery',self.t.observe)()
                        except RuntimeError as telemetry_error:
                            settled=None
                            if str(telemetry_error)!=last_recovery_error:
                                self.t.record({'event':'hover_recovery_waiting_for_telemetry','reason':str(telemetry_error)})
                                last_recovery_error=str(telemetry_error)
                            continue
                        good=(self.t.now()-o.received_at<=self.c['max_pose_age'] and
                              abs(o.position[2]-ground)<.02 and np.linalg.norm(o.velocity)<.05)
                        settled=(self.t.now() if settled is None else settled) if good else None
                        if settled is not None and self.t.now()-settled>=.75:
                            self.t.arm(False);airborne=False
                            self.t.record({'event':'hover_recovered_landed_pose_proxy'})
                            break
                    if airborne:self.t.record({'event':'hover_recovery_unconfirmed'})
                except Exception as fallback:self.t.record({'event':'hover_land_failed','reason':str(fallback)})
            raise
