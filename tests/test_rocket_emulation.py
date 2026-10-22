"""Contracts separating a virtual gimbaled lander from its quadrotor emulator."""
from copy import deepcopy
from dataclasses import replace
import json
from pathlib import Path
import numpy as np
import pytest
from pdg.rocket.paper_model import VehicleParameters, dynamics, aerodynamic_force_body, dcm_body_from_inertial
from pdg.rocket.paper_scvx import SCvxConfig, SCvxResult
from pdg.rocket.validation import RocketReplay, validate_result
from pdg.rocket.emulation import RocketEmulation, UEN_TO_ENU
from pdg.rocket.io import load_config, save_result, load_result

ROOT=Path(__file__).resolve().parents[1]

@pytest.fixture
def rocket():
    # Nearly exact constant-speed descent: FOH thrust follows exponentially
    # decreasing mass. Independent replay still has to verify the approximation.
    p=VehicleParameters(rho=0.,isp=1000.,g0=1.,thrust_min=.1)
    cfg=SCvxConfig(nodes=5,initial_position=np.array([.1,0.,0.]),
                   initial_velocity=np.array([-.1,0.,0.]),min_burn_time=.5,
                   initial_burn_time=1.,max_burn_time=2.,use_aerodynamics=False,
                   use_state_trigger=False)
    t=np.linspace(0,1,5);x=np.zeros((14,5));x[0]=2*np.exp(-t/1000)
    x[1]=.1*(1-t);x[4]=-.1;x[7]=1
    u=np.zeros((3,5));u[0]=x[0]
    r=SCvxResult(x,u,1.,0.,[],True,p,cfg)
    replay=RocketReplay(r)
    r.states=np.column_stack([replay.at(z)[0] for z in t])
    assert validate_result(r)['passed']
    return r

@pytest.fixture
def mapping():
    _,_,c=load_config(ROOT/'config/rocket-emulation.yaml')
    c=deepcopy(c['emulation'])
    c.update(length_scale_m_per_target_unit=1.,time_scale_s_per_target_unit=2.)
    return c


def test_rocket_mass_gimbal_moment_and_frame():
    p=VehicleParameters(rho=0.,g0=1.)
    x=np.zeros(14);x[0]=2;x[7]=1;u=np.array([2.,.2,0.])
    d=dynamics(x,u,p)
    assert d[0]==pytest.approx(-np.linalg.norm(u)/(p.isp*p.g0))
    np.testing.assert_allclose(d[4:7],u/2+p.gravity)
    np.testing.assert_allclose(p.inertia@d[11:14],np.cross(p.thrust_arm,u))
    angle=.3;q=np.array([np.cos(angle/2),0,0,np.sin(angle/2)])
    np.testing.assert_allclose(dcm_body_from_inertial(q).T@np.array([1.,0,0]),[np.cos(angle),np.sin(angle),0])


def test_drag_opposes_velocity():
    p=VehicleParameters();x=np.zeros(14);x[0]=2;x[7]=1;x[4:7]=[-1,2,3]
    assert aerodynamic_force_body(x,p)@x[4:7]<0


def test_mapping_chain_rule_and_earth_gravity(rocket,mapping):
    a=RocketEmulation(rocket,mapping)
    assert a.validation['passed']
    assert np.linalg.det(UEN_TO_ENU)==pytest.approx(1)
    t=.6;x,u=a.replay.at(t/2);p,v,acc,q,R,f=a.core_at(t)
    np.testing.assert_allclose(p,a.offset+a.rotation@x[1:4])
    np.testing.assert_allclose(v,a.rotation@x[4:7]/2)
    np.testing.assert_allclose(acc,a.rotation@dynamics(x,u,rocket.parameters)[4:7]/4)
    np.testing.assert_allclose(R[:,2]*f,acc+[0,0,9.81])
    # Rocket body x is longitudinal. Drone body z is thrust-aligned. Identical
    # scalar-first quaternion layouts do not make these the same physical frame.
    assert a.plan.states.shape==(5,13)
    assert a.sample(t).velocity[2]==pytest.approx(-.05,abs=1e-7)


def test_infeasible_scale_cannot_be_sampled(rocket,mapping):
    mapping['length_scale_m_per_target_unit']=100.
    a=RocketEmulation(rocket,mapping)
    assert not a.validation['passed']
    assert a.validation['violations']['speed_m_s']>0
    with pytest.raises(ValueError,match='feasibility'):a.sample(.1)


def test_replay_rejects_tampered_node_and_thrust(rocket):
    r=deepcopy(rocket);r.states[1,2]+=.1
    assert not validate_result(r)['passed']
    r=deepcopy(rocket);r.controls[0,1]=0
    assert not validate_result(r)['passed']


def test_plan_roundtrip_requires_model_provenance(rocket,tmp_path):
    f=tmp_path/'target.json';save_result(f,rocket,'normalized',validate_result(rocket))
    restored=load_result(f,rocket.parameters,rocket.config,'normalized')
    np.testing.assert_allclose(restored.controls,rocket.controls)
    with pytest.raises(ValueError,match='match'):load_result(f,rocket.parameters,rocket.config,'SI')
    with pytest.raises(ValueError,match='match'):load_result(f,replace(rocket.parameters,isp=500.),rocket.config,'normalized')
    d=json.loads(f.read_text());d['states'][1][2]+=.1;f.write_text(json.dumps(d))
    with pytest.raises(ValueError,match='validation'):load_result(f,rocket.parameters,rocket.config,'normalized')


def test_diagnostics_are_inverse_mapped_translation(rocket,mapping):
    from types import SimpleNamespace
    a=RocketEmulation(rocket,mapping);s=a.sample(.5)
    d=a.diagnostic_sample(.5,SimpleNamespace(position=s.position,velocity=s.velocity))
    x,_=a.replay.at(.25)
    np.testing.assert_allclose(d['observed_position_target_UEN'],x[1:4])
    np.testing.assert_allclose(d['observed_velocity_target_UEN'],x[4:7])


def test_mapped_reference_uses_existing_mission_protocol(rocket,mapping):
    from pdg.execution import Mission, Observation
    from pdg.io import load_problem
    _,_,c=load_problem(ROOT/'config/nominal.yaml')
    a=RocketEmulation(rocket,mapping);cfg=c['execution']
    cfg.update(ground_body_height=a.offset[2],expected_tracker_parameters={},vicon_only=False)
    class Transport:
        def __init__(self):
            self.time=0.;self.position=np.r_[a.plan.states[0,:2],a.offset[2]]
            self.events=[]
        def now(self):return self.time
        def wait(self,t):self.time+=t
        def observe(self):return Observation(self.position,np.zeros(3),np.array([1.,0,0,0]),self.time,self.time)
        def preflight(self):pass
        def arm(self,v):self.events.append(('arm',v))
        def takeoff(self,h,d):self.position[2]=h
        def send(self,s):self.position=s.position.copy()
        def land(self,h,d):self.position[2]=a.offset[2];self.events.append(('land',))
        def release_stream(self):self.events.append(('release',))
        def record(self,e):self.events.append((e['event'],))
    t=Transport();Mission(t,cfg).run(a.plan,a.drone,trajectory=a)
    assert ('rocket_emulation_sample',) in t.events
    assert t.events.index(('land',))<t.events.index(('release',))
    assert ('landed_pose_proxy',) in t.events
    a.validation['passed']=False
    with pytest.raises(ValueError,match='independent emulation'):Mission(Transport(),cfg).run(a.plan,a.drone,trajectory=a)


def test_rocket_attitude_is_not_copied_into_drone_attitude(rocket,mapping):
    from scipy.spatial.transform import Rotation
    a=RocketEmulation(rocket,mapping)
    x=rocket.states[:,0].copy();theta=.4
    x[7:11]=[np.cos(theta/2),0,0,np.sin(theta/2)]
    u=np.array([2.,.1,0.])
    a.replay.at=lambda t:(x,u)
    _,_,acc,q,R,f=a.core_at(.2)
    np.testing.assert_allclose(R[:,2]*f,acc+[0,0,9.81])
    assert not np.allclose(q,x[7:11])
    np.testing.assert_allclose(Rotation.from_quat(q[[1,2,3,0]]).as_matrix(),R)


def test_invalid_time_rejected_before_integrating(rocket):
    r=deepcopy(rocket);r.burn_time=0
    with pytest.raises(ValueError,match='positive before replay'):validate_result(r)
