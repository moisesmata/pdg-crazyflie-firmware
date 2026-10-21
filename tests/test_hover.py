import numpy as np
import pytest
from pdg.contracts import Plan
from pdg.hover import HoverMission
from test_execution import FakeTransport, fixture


def hover_plan():
    config,plan=fixture()
    plan.states[0,2]=.25
    return config,plan


def test_hover_takeoff_hold_land_disarm():
    config,plan=hover_plan()
    t=FakeTransport()
    HoverMission(t,config).run(plan)
    assert t.events.index(('hover_hold',))<t.events.index(('land',))
    assert t.events[-1]==('hover_landed_pose_proxy',)
    assert ('arm',False) in t.events


def test_thirty_second_hold_is_monitored_and_then_lands():
    config,plan=hover_plan();config['hover_hold_duration']=30.
    class TimedHold(FakeTransport):
        def record(self,event):
            if event['event']=='hover_hold':self.hold_started=self.now()
            super().record(event)
        def land(self,height,duration):
            assert 30<=self.now()-self.hold_started<=30.03
            super().land(height,duration)
    t=TimedHold();HoverMission(t,config).run(plan)
    assert ('arm',False) in t.events


def test_long_hold_responds_to_health_failure_without_waiting_for_timer():
    config,plan=hover_plan();config['hover_hold_duration']=30.
    class FailedHold(FakeTransport):
        def record(self,event):
            if event['event']=='hover_hold':self.hold_started=self.now()
            super().record(event)
        def observe(self):
            if hasattr(self,'hold_started') and self.now()-self.hold_started>.4:
                raise RuntimeError('Battery health lost')
            return super().observe()
        def observe_recovery(self):return super().observe()
        def land(self,height,duration):
            assert self.now()-self.hold_started<.5
            super().land(height,duration)
    t=FailedHold()
    with pytest.raises(RuntimeError,match='Battery'):HoverMission(t,config).run(plan)
    assert ('hover_recovered_landed_pose_proxy',) in t.events
    assert ('arm',False) in t.events


@pytest.mark.parametrize('duration',[float('nan'),0.,-1.,31.])
def test_invalid_hold_duration_never_arms(duration):
    config,plan=hover_plan();config['hover_hold_duration']=duration;t=FakeTransport()
    with pytest.raises(ValueError,match='hold duration'):HoverMission(t,config).run(plan)
    assert ('arm',True) not in t.events


def test_hover_stale_pose_never_arms():
    config,plan=hover_plan();t=FakeTransport(stale=True)
    with pytest.raises(RuntimeError,match='stale'):HoverMission(t,config).run(plan)
    assert ('arm',True) not in t.events


def test_hover_lateral_excursion_requests_landing_without_disarm():
    config,plan=hover_plan()
    class Drifting(FakeTransport):
        def takeoff(self,height,duration):
            super().takeoff(height,duration);self.position[0]=.1
        def land(self,height,duration):
            super().land(height,duration);self.position[2]=.15
    t=Drifting()
    with pytest.raises(RuntimeError,match='lateral'):HoverMission(t,config).run(plan)
    assert ('land',) in t.events
    assert ('arm',False) not in t.events


def test_hover_bad_ground_start_never_arms():
    config,plan=hover_plan();t=FakeTransport();t.position[2]=.10
    with pytest.raises(RuntimeError,match='ground start'):HoverMission(t,config).run(plan)
    assert ('arm',True) not in t.events


def test_heading_preserving_hover_accepts_nonzero_heading():
    config,plan=hover_plan();config['preserve_heading']=True
    class Heading(FakeTransport):
        def observe(self):
            from dataclasses import replace
            return replace(super().observe(),quaternion_wxyz=np.array([0.,0.,0.,1.]))
    t=Heading();HoverMission(t,config).run(plan)
    assert t.events[-1]==('hover_landed_pose_proxy',)


def test_heading_jump_requests_landing():
    config,plan=hover_plan();config['preserve_heading']=True
    class Jumping(FakeTransport):
        def observe(self):
            from dataclasses import replace
            q=np.array([0.,0.,0.,1.]) if self.position[2]>.1 else np.array([1.,0.,0.,0.])
            return replace(super().observe(),quaternion_wxyz=q)
    t=Jumping()
    with pytest.raises(RuntimeError,match='attitude'):HoverMission(t,config).run(plan)
    assert ('land',) in t.events
    assert ('arm',False) in t.events
    assert ('hover_recovered_landed_pose_proxy',) in t.events


def test_battery_abort_continues_landing_supervision():
    config,plan=hover_plan()
    class LowBattery(FakeTransport):
        def takeoff(self,height,duration):
            super().takeoff(height,duration);self.flight=True
        def observe(self):
            if self.flight:raise RuntimeError('Battery health lost')
            return super().observe()
        def observe_recovery(self):return super().observe()
    t=LowBattery()
    with pytest.raises(RuntimeError,match='Battery'):HoverMission(t,config).run(plan)
    assert ('arm',False) in t.events
    assert ('hover_recovered_landed_pose_proxy',) in t.events


def test_marker_heading_change_between_calibration_and_launch_never_arms():
    config,plan=hover_plan();config['preserve_heading']=True
    config['start_quaternion_wxyz']=[0.,0.,0.,1.]
    t=FakeTransport()
    with pytest.raises(RuntimeError,match='Start attitude differs'):HoverMission(t,config).run(plan)
    assert ('arm',True) not in t.events


def test_one_meter_hover_keeps_hold_landing_and_disarm():
    config,plan=hover_plan()
    plan.states[0,2]=config['ground_body_height']+1.
    t=FakeTransport()
    HoverMission(t,config).run(plan)
    assert ('hover_hold',) in t.events
    assert ('land',) in t.events
    assert ('arm',False) in t.events


@pytest.mark.parametrize('rise',[float('nan'),.05,1.1])
def test_unsupported_hover_height_never_arms(rise):
    config,plan=hover_plan();plan.states[0,2]=config['ground_body_height']+rise
    t=FakeTransport()
    with pytest.raises(ValueError,match='rise'):
        HoverMission(t,config).run(plan)
    assert ('arm',True) not in t.events


def test_recovery_waits_for_fresh_tracking_before_disarm():
    config,plan=hover_plan()
    class TemporaryLoss(FakeTransport):
        def takeoff(self,height,duration):
            super().takeoff(height,duration);self.flight=True
        def observe(self):
            if self.flight:raise RuntimeError('Motion capture is stale')
            return super().observe()
        def land(self,height,duration):
            super().land(height,duration);self.landed_at=self.time
        def observe_recovery(self):
            if self.time-self.landed_at<.4:raise RuntimeError('Recovery pose is stale')
            return super().observe()
    t=TemporaryLoss()
    with pytest.raises(RuntimeError,match='stale'):HoverMission(t,config).run(plan)
    assert ('hover_recovery_waiting_for_telemetry',) in t.events
    assert ('hover_recovered_landed_pose_proxy',) in t.events
    assert ('arm',False) in t.events


def test_persistent_recovery_tracking_loss_does_not_disarm_blindly():
    config,plan=hover_plan()
    class PersistentLoss(FakeTransport):
        def takeoff(self,height,duration):
            super().takeoff(height,duration);self.flight=True
        def observe(self):
            if self.flight:raise RuntimeError('Motion capture is stale')
            return super().observe()
        def observe_recovery(self):raise RuntimeError('Recovery pose is stale')
    t=PersistentLoss()
    with pytest.raises(RuntimeError,match='stale'):HoverMission(t,config).run(plan)
    assert ('land',) in t.events
    assert ('hover_recovery_unconfirmed',) in t.events
    assert ('arm',False) not in t.events


def test_commissioning_profile_allows_settling_drift_and_yaw_then_bounded_land():
    config,plan=hover_plan()
    config.update(hover_max_lateral_deviation=.30,hover_max_tilt_deg=25.,
                  hover_max_attitude_change_deg=None,hover_endpoint_tolerance=.20,
                  hover_settle_duration=4.,hover_landing_duration=8.)
    class Settling(FakeTransport):
        def takeoff(self,height,duration):
            super().takeoff(height,duration);self.position[0]=.12;self.flight=True
        def observe(self):
            from dataclasses import replace
            o=super().observe()
            if self.flight:
                a=np.deg2rad(30)/2
                o=replace(o,quaternion_wxyz=np.array([np.cos(a),0.,0.,np.sin(a)]))
            return o
        def land(self,height,duration):
            assert self.time>=10.  # Four seconds ascent + four settle + two hold.
            assert duration==8.
            super().land(height,duration)
    t=Settling();HoverMission(t,config).run(plan)
    assert ('hover_hold',) in t.events
    assert ('arm',False) in t.events


def test_larger_envelope_still_rejects_excursion_outside_confirmed_space():
    config,plan=hover_plan();config['hover_max_lateral_deviation']=.30
    class Excursion(FakeTransport):
        def takeoff(self,height,duration):
            super().takeoff(height,duration);self.position[0]=.31
    t=Excursion()
    with pytest.raises(RuntimeError,match='30 cm'):HoverMission(t,config).run(plan)
    assert ('land',) in t.events


def test_bounded_landing_timeout_does_not_disarm_without_touchdown():
    config,plan=hover_plan();config['hover_landing_duration']=8.
    class NoTouchdown(FakeTransport):
        def land(self,height,duration):
            self.events.append(('land',));self.position[2]=.15
    t=NoTouchdown()
    with pytest.raises(RuntimeError,match='touchdown'):HoverMission(t,config).run(plan)
    assert t.events.count(('land',))==1  # No ignored duplicate Land or indefinite GoTo.
    assert ('arm',False) not in t.events


@pytest.mark.parametrize('overshoot,allowed',[(.15,True),(.21,False)])
def test_vertical_settling_allowance_remains_bounded(overshoot,allowed):
    config,plan=hover_plan()
    plan.states[0,2]=config['ground_body_height']+1.
    config.update(hover_max_vertical_overshoot=.20,hover_endpoint_tolerance=.20)
    class Overshoot(FakeTransport):
        def takeoff(self,height,duration):
            self.position[2]=height+overshoot
    t=Overshoot()
    if allowed:
        HoverMission(t,config).run(plan)
        assert ('hover_hold',) in t.events
    else:
        with pytest.raises(RuntimeError,match='vertical envelope'):HoverMission(t,config).run(plan)
        assert ('hover_hold',) not in t.events


def test_explicit_controller_trial_omits_flight_envelope_and_endpoint_abort():
    config,plan=hover_plan()
    config.update(operator_controller_trial=True,baseline_pid=True,vicon_only=True)
    class ControllerTrial(FakeTransport):
        def takeoff(self,height,duration):
            super().takeoff(height,duration)
            self.position=np.array([.4,0.,height+.3]);self.flight=True
        def observe(self):
            from dataclasses import replace
            o=super().observe()
            if self.flight:
                a=np.deg2rad(35)/2
                o=replace(o,quaternion_wxyz=np.array([np.cos(a),np.sin(a),0.,0.]))
            return o
        def land(self,height,duration):
            self.flight=False
            super().land(height,duration)
    t=ControllerTrial();HoverMission(t,config).run(plan)
    assert ('hover_hold',) in t.events
    assert ('land',) in t.events
    assert ('arm',False) in t.events


def test_controller_trial_still_rejects_stale_ground_pose_before_arming():
    config,plan=hover_plan()
    config.update(operator_controller_trial=True,baseline_pid=True,vicon_only=True)
    t=FakeTransport(stale=True)
    with pytest.raises(RuntimeError,match='stale'):HoverMission(t,config).run(plan)
    assert ('arm',True) not in t.events


def test_controller_trial_still_requests_landing_on_tracking_loss():
    config,plan=hover_plan()
    config.update(operator_controller_trial=True,baseline_pid=True,vicon_only=True)
    class Loss(FakeTransport):
        def takeoff(self,height,duration):
            super().takeoff(height,duration);self.flight=True
        def observe(self):
            if self.flight:raise RuntimeError('Motion capture is stale')
            return super().observe()
    t=Loss()
    with pytest.raises(RuntimeError,match='stale'):HoverMission(t,config).run(plan)
    assert ('land',) in t.events
    assert ('arm',False) not in t.events


def test_controller_trial_cannot_enable_for_other_controller_modes():
    config,plan=hover_plan();config['operator_controller_trial']=True
    t=FakeTransport()
    with pytest.raises(ValueError,match='Vicon-only PID'):HoverMission(t,config).run(plan)
    assert ('arm',True) not in t.events
