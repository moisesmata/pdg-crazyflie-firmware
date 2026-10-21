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


def test_nominal_sequence_releases_stream_before_landing():
    cfg,p=fixture();t=FakeTransport();Mission(t,cfg).run(p,Quadrotor())
    assert t.events.index(('release',)) < t.events.index(('land',))
    assert t.events[-1] == ('landed_pose_proxy',)
    assert ('arm',False) in t.events


def test_stale_pose_prevents_arming():
    cfg,p=fixture();t=FakeTransport(stale=True)
    with pytest.raises(RuntimeError,match='stale'):Mission(t,cfg).run(p,Quadrotor())
    assert ('arm',True) not in t.events


def test_bad_start_requests_landing_without_airborne_disarm():
    cfg,p=fixture();t=FakeTransport(wrong_start=True)
    with pytest.raises(RuntimeError,match='initial state'):Mission(t,cfg).run(p,Quadrotor())
    assert ('land',) in t.events
    assert ('arm',False) not in t.events


def test_streaming_deadline_abort():
    cfg,p=fixture();t=FakeTransport(stall=True)
    with pytest.raises(RuntimeError,match='deadline'):Mission(t,cfg).run(p,Quadrotor())
    assert ('land',) in t.events
    assert ('arm',False) not in t.events


def test_ros_message_units_and_quaternion_order():
    def vec():return SimpleNamespace(x=0.,y=0.,z=0.)
    msg=SimpleNamespace(header=SimpleNamespace(),pose=SimpleNamespace(position=vec(),orientation=SimpleNamespace()),
                        twist=SimpleNamespace(linear=vec(),angular=vec()),acc=vec())
    s=SimpleNamespace(position=[1,2,3],velocity=[4,5,6],acceleration=[7,8,9],
                      quaternion_wxyz=[.5,.5,.5,.5],body_rates=[.1,.2,.3])
    fill_full_state(msg,s,123,'world')
    assert msg.pose.orientation.w == .5 and msg.twist.angular.z == .3
    assert msg.acc.z == 9 and msg.header.frame_id == 'world'
