from pathlib import Path
from types import SimpleNamespace
import numpy as np
import pytest
from pdg.contracts import Plan
from pdg.execution import Mission, Observation
from pdg.io import load_problem
from pdg.model import rest_state, Quadrotor
from pdg.ros_mission import fill_full_state

ROOT=Path(__file__).resolve().parents[1]


class FakeTransport:
    def __init__(self,stale=False,wrong_start=False,stall=False):
        self.time=0.;self.stale=stale;self.wrong_start=wrong_start;self.stall=stall
        self.position=np.array([0.,0.,.05]);self.events=[];self.flight=False
    def now(self):return self.time
    def wait(self,t):self.time+=t
    def observe(self):return Observation(self.position,np.zeros(3),np.array([1.,0,0,0]),self.time,self.time-1 if self.stale else self.time)
    def preflight(self):pass
    def arm(self,value):self.events.append(('arm',value))
    def takeoff(self,height,duration):
        self.position[2]=height+(1 if self.wrong_start else 0)
    def go_to(self,position,quaternion_wxyz,duration):
        self.events.append(('stage',));self.position=np.asarray(position).copy()
    def send(self,s):
        self.position=s.position.copy()
        if self.stall:self.time+=.1
    def release_stream(self):self.events.append(('release',))
    def land(self,height,duration):self.events.append(('land',));self.position=np.array([0.,0.,.05])
    def record(self,e):self.events.append((e['event'],))


def fixture():
    _,_,c=load_problem(ROOT/'config/nominal.yaml')
    x0=rest_state([0,0,1.05]);xf=rest_state([0,0,.05]);xf[5]=-2
    plan=Plan(np.array([0.,1.]),np.array([x0,xf]),np.array([[7.81,0,0,0],[7.81,0,0,0]]),True)
    return c['execution'],plan


def test_nominal_sequence_queues_landing_before_releasing_stream():
    cfg,p=fixture();t=FakeTransport();Mission(t,cfg).run(p,Quadrotor())
    assert t.events.index(('land',)) < t.events.index(('release',))
    assert t.events[-1] == ('landed_pose_proxy',)
    assert ('arm',False) in t.events


def test_stale_pose_prevents_arming():
    cfg,p=fixture();t=FakeTransport(stale=True)
    with pytest.raises(RuntimeError,match='stale'):Mission(t,cfg).run(p,Quadrotor())
    assert ('arm',True) not in t.events


def test_bad_start_requests_landing_and_disarms_only_after_fresh_ground():
    cfg,p=fixture();t=FakeTransport(wrong_start=True)
    with pytest.raises(RuntimeError,match='initial state'):Mission(t,cfg).run(p,Quadrotor())
    assert ('land',) in t.events
    assert ('recovered_landed_pose_proxy',) in t.events
    assert ('arm',False) in t.events


def test_streaming_deadline_abort():
    cfg,p=fixture();t=FakeTransport(stall=True)
    with pytest.raises(RuntimeError,match='deadline'):Mission(t,cfg).run(p,Quadrotor())
    assert ('land',) in t.events
    assert ('recovered_landed_pose_proxy',) in t.events


def test_pdg_trial_omits_tracking_abort_but_completes_bounded_landing():
    cfg,p=fixture();cfg.update(operator_controller_trial=True,vicon_only=True,
                              expected_tracker_parameters={'ctrlMel.mass':.044},landing_duration=8.)
    class TrackingOffset(FakeTransport):
        def send(self,s):
            super().send(s);self.position[0]+=.3
        def land(self,height,duration):
            assert duration==8.;super().land(height,duration)
    t=TrackingOffset();Mission(t,cfg).run(p,Quadrotor())
    assert ('descent',) in t.events and ('landed_pose_proxy',) in t.events


def test_pdg_trial_still_rejects_unsettled_descent_start():
    cfg,p=fixture();cfg.update(operator_controller_trial=True,vicon_only=True,
                              expected_tracker_parameters={'ctrlMel.mass':.044})
    t=FakeTransport(wrong_start=True)
    with pytest.raises(RuntimeError,match='planned initial state'):Mission(t,cfg).run(p,Quadrotor())
    assert ('descent',) not in t.events


def test_sideways_staging_precedes_descent_from_actual_ground_pad():
    cfg,p=fixture();cfg.update(stage_to_start=True,staging_duration=6.,ground_start_xy=[0.,0.])
    p.states[0,0]=1.
    t=FakeTransport();Mission(t,cfg).run(p,Quadrotor())
    assert t.events.index(('takeoff',))<t.events.index(('stage',))<t.events.index(('descent',))
    assert ('arm',False) in t.events


def test_staging_waits_for_vertical_takeoff_before_sideways_command():
    cfg,p=fixture();cfg.update(stage_to_start=True,ground_start_xy=[0.,0.])
    p.states[0,0]=1.
    t=FakeTransport(wrong_start=True)
    with pytest.raises(RuntimeError,match='before staging'):Mission(t,cfg).run(p,Quadrotor())
    assert ('stage',) not in t.events
    assert ('descent',) not in t.events
    assert ('land',) in t.events


def test_staging_failure_does_not_start_streamed_descent():
    cfg,p=fixture();cfg.update(stage_to_start=True,ground_start_xy=[0.,0.])
    p.states[0,0]=1.
    class MissedStage(FakeTransport):
        def go_to(self,position,quaternion_wxyz,duration):self.events.append(('stage',))
    t=MissedStage()
    with pytest.raises(RuntimeError,match='planned initial state'):Mission(t,cfg).run(p,Quadrotor())
    assert ('descent',) not in t.events
    assert ('land',) in t.events


def test_pdg_recovery_does_not_disarm_on_persistent_tracking_loss():
    cfg,p=fixture()
    class Loss(FakeTransport):
        def takeoff(self,height,duration):
            super().takeoff(height,duration);self.flight=True
        def observe(self):
            if self.flight:raise RuntimeError('Motion capture is stale')
            return super().observe()
    t=Loss()
    with pytest.raises(RuntimeError,match='stale'):Mission(t,cfg).run(p,Quadrotor())
    assert ('land',) in t.events
    assert ('recovery_landing_unconfirmed',) in t.events
    assert ('arm',False) not in t.events


def test_staging_yaw_uses_cpp_wire_radians_despite_srv_comment():
    from pdg.ros_mission import fill_go_to_request
    q=np.array([np.sqrt(.5),0.,0.,np.sqrt(.5)])
    req=fill_go_to_request(SimpleNamespace(goal=SimpleNamespace()),[1,0,2.5],q)
    assert req.yaw==pytest.approx(np.pi/2)
    assert [req.goal.x,req.goal.y,req.goal.z]==[1.,0.,2.5]
    assert not req.relative


def test_ros_message_units_and_quaternion_order():
    def vec():return SimpleNamespace(x=0.,y=0.,z=0.)
    msg=SimpleNamespace(header=SimpleNamespace(),pose=SimpleNamespace(position=vec(),orientation=SimpleNamespace()),
                        twist=SimpleNamespace(linear=vec(),angular=vec()),acc=vec())
    s=SimpleNamespace(position=[1,2,3],velocity=[4,5,6],acceleration=[7,8,9],
                      quaternion_wxyz=[.5,.5,.5,.5],body_rates=[.1,.2,.3])
    fill_full_state(msg,s,123,'world')
    assert msg.pose.orientation.w == .5 and msg.twist.angular.z == .3
    assert msg.acc.z == 9 and msg.header.frame_id == 'world'


def test_direct_takeoff_from_offset_start_never_sends_sideways_stage():
    cfg,p=fixture();cfg.update(stage_to_start=False,ground_start_xy=[0.,0.])
    p.states[0,0]=-1.
    t=FakeTransport();t.position[0]=-1.
    Mission(t,cfg).run(p,Quadrotor())
    assert ('stage',) not in t.events
    assert ('descent',) in t.events
    assert ('landed_pose_proxy',) in t.events


def test_direct_offset_start_rejects_drone_still_at_old_origin_pad():
    cfg,p=fixture();cfg.update(stage_to_start=False,ground_start_xy=[0.,0.])
    p.states[0,0]=-1.
    t=FakeTransport()
    with pytest.raises(RuntimeError,match='horizontal position'):
        Mission(t,cfg).run(p,Quadrotor())
    assert ('arm',True) not in t.events


@pytest.mark.parametrize('tilted,allowed',[(False,True),(True,False)])
def test_trial_can_ignore_heading_but_keeps_start_tilt_check(tilted,allowed):
    from dataclasses import replace
    cfg,p=fixture();cfg.update(operator_controller_trial=True,vicon_only=True,
        expected_tracker_parameters={'ctrlMel.mass':.044},ignore_start_heading=True,max_start_tilt_deg=10.)
    class OffsetAttitude(FakeTransport):
        def observe(self):
            o=super().observe()
            if self.position[2]>.1:
                a=np.deg2rad(20)/2
                q=np.array([np.cos(a),np.sin(a),0.,0.]) if tilted else np.array([np.cos(a),0.,0.,np.sin(a)])
                return replace(o,quaternion_wxyz=q)
            return o
    t=OffsetAttitude()
    if allowed:
        Mission(t,cfg).run(p,Quadrotor())
        assert ('descent',) in t.events
        assert ('landed_pose_proxy',) in t.events
    else:
        with pytest.raises(RuntimeError,match='initial state'):Mission(t,cfg).run(p,Quadrotor())
        assert ('descent',) not in t.events
        assert ('land',) in t.events


def test_heading_gate_override_cannot_silently_enable_outside_trial():
    cfg,p=fixture();cfg['ignore_start_heading']=True;t=FakeTransport()
    with pytest.raises(ValueError,match='explicit calibrated controller trial'):
        Mission(t,cfg).run(p,Quadrotor())
    assert ('arm',True) not in t.events
