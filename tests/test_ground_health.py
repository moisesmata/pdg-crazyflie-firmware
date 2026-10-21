from types import SimpleNamespace
import numpy as np
import pytest
from pdg.ground_health import validate_ground_health


def test_loaded_voltage_and_stale_status_are_distinct():
    from pdg.ground_health import validate_flight_status
    validate_flight_status(3.679,.003,3.5)
    for voltage,age in [(3.49,.003),(3.8,2.6),(float('nan'),.003)]:
        with pytest.raises(RuntimeError,match='health lost'):
            validate_flight_status(voltage,age,3.5)


def test_explicit_no_voltage_threshold_still_requires_valid_fresh_status():
    from pdg.ground_health import validate_flight_status
    validate_flight_status(3.3,.01,None)
    for voltage,age in [(3.3,2.6),(float('nan'),.01)]:
        with pytest.raises(RuntimeError):validate_flight_status(voltage,age,None)


def test_explicit_ground_voltage_bypass_does_not_bypass_supervisor():
    o=SimpleNamespace(position=np.zeros(3),quaternion_wxyz=np.array([1.,0,0,0]))
    s=SimpleNamespace(pm_state=0,battery_voltage=3.4,supervisor_info=1)
    validate_ground_health(s,[0,0,0],[1,0,0,0],o,minimum_voltage=None)
    with pytest.raises(RuntimeError):validate_ground_health(s,[0,0,0],[1,0,0,0],o)
    s.supervisor_info=65
    with pytest.raises(RuntimeError,match='Supervisor'):
        validate_ground_health(s,[0,0,0],[1,0,0,0],o,minimum_voltage=None)


def test_arm_waits_for_fresh_supervisor_ready_not_just_service_ack():
    from pdg.ground_health import wait_for_supervisor_arm
    clock=[1.]
    def now():return clock[0]
    def wait(t):clock[0]+=t
    def status():
        # Arm bit alone must not release takeoff while supervisor is spinning up.
        return SimpleNamespace(supervisor_info=2 if clock[0]<1.2 else 10),clock[0]
    wait_for_supervisor_arm(now,wait,status,True)
    assert clock[0]>=1.2


def test_arm_rejects_cached_ready_state_and_times_out():
    from pdg.ground_health import wait_for_supervisor_arm
    clock=[1.]
    with pytest.raises(RuntimeError,match='did not confirm'):
        wait_for_supervisor_arm(lambda:clock[0],lambda t:clock.__setitem__(0,clock[0]+t),
                                lambda:(SimpleNamespace(supervisor_info=10),0.),True,timeout=.1)


def test_disarm_waits_until_arm_flag_clears():
    from pdg.ground_health import wait_for_supervisor_arm
    clock=[1.]
    def status():return SimpleNamespace(supervisor_info=10 if clock[0]<1.2 else 1),clock[0]
    wait_for_supervisor_arm(lambda:clock[0],lambda t:clock.__setitem__(0,clock[0]+t),status,False)
    assert clock[0]>=1.2


def test_healthy_ground_estimator_and_supervisor():
    o=SimpleNamespace(position=np.zeros(3),quaternion_wxyz=np.array([1.,0,0,0]))
    validate_ground_health(SimpleNamespace(pm_state=0,battery_voltage=4.1,supervisor_info=1),[0,0,0],[1,0,0,0],o)


@pytest.mark.parametrize('voltage,flags,position,reason',[
    (3.3,1,[0,0,0],'voltage'),
    (float('nan'),1,[0,0,0],'voltage'),
    (4.1,1,[float('nan'),0,0],'position'),
    (4.1,3,[0,0,0],'Supervisor'),
    (4.1,65,[0,0,0],'Supervisor'),
    (4.1,1,[.1,0,0],'position'),
])
def test_unready_ground_state_is_rejected(voltage,flags,position,reason):
    o=SimpleNamespace(position=np.zeros(3),quaternion_wxyz=np.array([1.,0,0,0]))
    with pytest.raises(RuntimeError,match=reason):
        validate_ground_health(SimpleNamespace(pm_state=0,battery_voltage=voltage,supervisor_info=flags),position,[1,0,0,0],o)


def test_hover_tilt_and_heading_are_checked_separately():
    from scipy.spatial.transform import Rotation
    from pdg.ground_health import validate_hover_alignment
    q=Rotation.from_euler('xyz',[0,4,4],degrees=True).as_quat()
    validate_hover_alignment(q[[3,0,1,2]])
    for angles in ([0,6,0],[0,0,6]):
        q=Rotation.from_euler('xyz',angles,degrees=True).as_quat()
        with pytest.raises(RuntimeError,match='tilt and heading'):
            validate_hover_alignment(q[[3,0,1,2]])


@pytest.mark.parametrize('controller', [0,6])
def test_normal_mission_accepts_only_project_controller(controller):
    from pdg.ground_health import validate_controller_parameters
    validate_controller_parameters(1,controller,6,0)
    with pytest.raises(RuntimeError):
        validate_controller_parameters(1,1,6,0)


@pytest.mark.parametrize('api,controller,oot,cap,baseline', [
    (1,0,6,0,True), (1,6,6,0,True), (0,1,6,0,True),
    (1,1,5,0,True), (1,1,6,1,True), (1,6,5,0,False),
])
def test_baseline_does_not_bypass_identity_or_controller_gates(api,controller,oot,cap,baseline):
    from pdg.ground_health import validate_controller_parameters
    with pytest.raises(RuntimeError):
        validate_controller_parameters(api,controller,oot,cap,baseline_pid=baseline)


def test_explicit_pid_baseline_accepts_pid():
    from pdg.ground_health import validate_controller_parameters
    validate_controller_parameters(1,1,6,0,baseline_pid=True)


@pytest.mark.parametrize('age',[.251,float('inf'),float('nan'),-.1])
def test_lost_estimator_stream_requires_landing(age):
    from pdg.ground_health import validate_estimate_freshness
    with pytest.raises(RuntimeError,match='estimator telemetry is stale'):
        validate_estimate_freshness(age)


def test_fresh_estimator_stream():
    from pdg.ground_health import validate_estimate_freshness
    validate_estimate_freshness(.05)


def test_mellinger_calibration_must_match_the_prepared_candidate():
    from pdg.ground_health import validate_tracker_calibration
    expected={'ctrlMel.mass':.044,'ctrlMel.massThrust':135456.8}
    validate_tracker_calibration(dict(expected),expected)
    for values in ({'ctrlMel.mass':.029,'ctrlMel.massThrust':132000.},
                   {'ctrlMel.mass':.044},
                   {'ctrlMel.mass':float('nan'),'ctrlMel.massThrust':135456.8}):
        with pytest.raises(RuntimeError,match='calibration mismatch'):
            validate_tracker_calibration(values,expected)


def test_pdg_trial_requires_vicon_and_explicit_tracker_calibration():
    from pdg.ground_health import validate_controller_trial_mode
    c=dict(operator_controller_trial=True,vicon_only=True,expected_tracker_parameters={'ctrlMel.mass':.044})
    assert validate_controller_trial_mode(c,hover=False)
    for changes in ({'vicon_only':False},{'expected_tracker_parameters':{}},{'baseline_pid':True}):
        with pytest.raises(ValueError):validate_controller_trial_mode(dict(c,**changes),hover=False)


@pytest.mark.parametrize('values',[(2,0,0,0,0),(2,1,0,1,1),(2,1,1,0,0),(2,1,0,0,1),(1,1,0,0,0)])
def test_vicon_only_rejects_other_estimator_or_deck_inputs(values):
    from pdg.ground_health import validate_vicon_only_inputs
    with pytest.raises(RuntimeError):validate_vicon_only_inputs(*values)


def test_vicon_only_with_removed_deck():
    from pdg.ground_health import validate_vicon_only_inputs
    validate_vicon_only_inputs(2,1,0,0,0)
