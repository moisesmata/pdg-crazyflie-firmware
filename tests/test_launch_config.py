"""Checks the project configuration needed by the actual ROS preflight."""
import importlib.util
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('pdg_launch', ROOT/'launch/pdg.launch.py')
launch = importlib.util.module_from_spec(spec)
spec.loader.exec_module(launch)


def test_preflight_readonly_parameters_are_queried():
    defaults = {'firmware_params': {'query_all_values_on_connect': False}}
    server, mocap = launch.load_parameters(server_defaults=defaults)
    assert server['firmware_params']['query_all_values_on_connect'] is True
    assert defaults['firmware_params']['query_all_values_on_connect'] is False
    assert server['poses_qos_deadline'] == mocap['topics']['poses']['qos']['deadline']


def test_vendor_tracking_matches_mission_robot_and_frame():
    import yaml
    execution = yaml.safe_load((ROOT/'config/nominal.yaml').read_text())['execution']
    server, mocap = launch.load_parameters()
    assert server['robots'][execution['robot']]['enabled']
    assert mocap['topics']['frame_id'] == execution['frame_id']
    assert server['all']['reference_frame'] == execution['frame_id']
    assert mocap['rigid_bodies'] == {}  # vendor supplies bodies; no second marker tracker


def test_configured_firmware_values_override_initial_ros_snapshot(tmp_path):
    import shutil,yaml
    shutil.copytree(ROOT/'config',tmp_path/'config')
    p=tmp_path/'config/crazyflies.yaml';cfg=yaml.safe_load(p.read_text())
    cfg['robot_types']['cf21']['firmware_params']={'locSrv':{'extQuatStdDev':.1}}
    cfg['robots']['Crazyflie']['firmware_params']={'locSrv':{'extQuatStdDev':.2}}
    p.write_text(yaml.safe_dump(cfg))
    server,_=launch.load_parameters(root=tmp_path)
    assert server['Crazyflie.params.locSrv.extQuatStdDev']==.2
    assert server['Crazyflie.params.locSrv.extPosStdDev']==.001
    assert server['Crazyflie.params.pdg.capEnable']==0
    assert server['Crazyflie.params.stabilizer.controller']==0


def test_shorthand_radio_uri_is_expanded_for_cpp_broadcast(tmp_path):
    import shutil
    import yaml
    shutil.copytree(ROOT/'config',tmp_path/'config')
    path=tmp_path/'config/crazyflies.yaml'
    cfg=yaml.safe_load(path.read_text())
    robot=next(iter(cfg['robots'].values()))
    robot['uri']='radio://0/80/2M'
    path.write_text(yaml.safe_dump(cfg))
    server,_=launch.load_parameters(root=tmp_path)
    assert next(iter(server['robots'].values()))['uri']=='radio://0/80/2M/E7E7E7E7E7'
    assert server['robot_types'][robot['type']]['motion_capture']['enabled'] is True


def test_pid_override_reaches_hardware_and_ros_mirror(tmp_path):
    import shutil,yaml
    shutil.copytree(ROOT/'config',tmp_path/'config')
    p=tmp_path/'config/crazyflies.yaml';cfg=yaml.safe_load(p.read_text())
    cfg['robots']['Crazyflie']['firmware_params']={'stabilizer':{'controller':6}}
    p.write_text(yaml.safe_dump(cfg))
    server,_=launch.load_parameters(root=tmp_path,controller_override=1)
    assert server['robots']['Crazyflie']['firmware_params']['stabilizer']['controller']==1
    assert server['Crazyflie.params.stabilizer.controller']==1
    normal,_=launch.load_parameters(root=tmp_path,controller_override=0)
    assert normal['Crazyflie.params.stabilizer.controller']==0
    assert normal['Crazyflie.params.stabilizer.estimator']==2


def test_unknown_controller_override_rejected():
    import pytest
    with pytest.raises(ValueError,match='Controller override'):
        launch.load_parameters(controller_override=99)


def test_radio_probe_period_has_margin_inside_firmware_timeout():
    server,_=launch.load_parameters()
    assert 1/server['warnings.frequency']<=.1


def test_vicon_only_selects_pid_kalman_and_disables_flow():
    server,_=launch.load_parameters(controller_override=0,vicon_only=True)
    assert server['Crazyflie.params.stabilizer.controller']==1
    assert server['Crazyflie.params.stabilizer.estimator']==2
    assert server['Crazyflie.params.motion.disable']==1
    assert server['robots']['Crazyflie']['firmware_params']['motion']['disable']==1
