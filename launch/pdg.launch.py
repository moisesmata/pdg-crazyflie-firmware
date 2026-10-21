"""Project launch: Vicon and Crazyflie server, with PDG preflight parameters read."""
from pathlib import Path
import yaml
import re
import math
import os

ROOT = Path(__file__).resolve().parents[1]


def load_body_frame(root=ROOT):
    path=root/'config/body-frame.yaml'
    if not path.exists():return None
    config=yaml.safe_load(path.read_text())
    if not config.get('enabled'):return None
    # ros2 launch uses the system interpreter, not the project venv. Keep launch
    # validation free of project-package imports; the relay validates it again.
    validation=config.get('validation',{})
    error=validation.get('corrected_max_error_deg',float('inf'))
    q=config.get('right_multiply_marker_orientation_xyzw',[])
    if (not config.get('robot') or not config.get('frame_id') or not validation.get('passed') or
        validation.get('samples',0)<20 or not math.isfinite(error) or not 0<=error<5 or
        len(q)!=4 or not all(math.isfinite(v) for v in q) or abs(math.sqrt(sum(v*v for v in q))-1)>.02):
        raise ValueError('Body frame correction needs a unit quaternion and independent validation below 5 degrees')
    return config


def load_parameters(root=ROOT, server_defaults=None, controller_override=None, vicon_only=False):
    from copy import deepcopy
    robots = yaml.safe_load((root/'config/crazyflies.yaml').read_text())
    mocap = yaml.safe_load((root/'config/motion_capture.yaml').read_text())['/motion_capture_tracking']['ros__parameters']
    if 'CHANGE_ME' in mocap['hostname']:
        raise ValueError('Set the Vicon hostname in config/motion_capture.yaml')
    # cflib accepts a shorthand URI; the pinned C++ broadcaster requires its address.
    for robot in robots['robots'].values():
        uri=robot['uri']
        if re.fullmatch(r'radio://\d+/\d+/(250K|1M|2M)',uri):
            uri += '/E7E7E7E7E7'
        if not re.fullmatch(r'radio://\d+/\d+/(250K|1M|2M)/[a-fA-F0-9]{10}',uri):
            raise ValueError(f'C++ server requires a full radio URI including its 5-byte address: {uri!r}')
        robot['uri']=uri
    if controller_override is not None:
        if controller_override not in (0, 1, 6):
            raise ValueError('Controller override must be 0 (auto), 1 (PID), or 6 (OOT)')
        for robot in robots['robots'].values():
            robot.setdefault('firmware_params',{}).setdefault('stabilizer',{})['controller']=controller_override
    if vicon_only:
        for robot in robots['robots'].values():
            params=robot.setdefault('firmware_params',{})
            params.setdefault('stabilizer',{}).update(controller=1,estimator=2)
            params.setdefault('motion',{})['disable']=1
    for kind in robots['robot_types'].values():
        kind['motion_capture']['enabled']=kind['motion_capture']['tracking'] in ('vendor','librigidbodytracker')
    server = deepcopy(server_defaults or {})
    server.setdefault('firmware_params', {})['query_all_values_on_connect'] = True
    server.update(robots)
    # Send latency probes at 10 Hz, well inside the firmware's 1000 ms radio
    # activity timeout. The upstream 1 Hz probe leaves no scheduling margin.
    # This parameter also controls diagnostics frequency. No flight command.
    server['warnings.frequency']=10.0
    # Tracker streams near 126 Hz here; upstream's 120 Hz upper bound
    # produced a warning on almost every check. Flight freshness is unchanged.
    server['warnings.motion_capture.warning_if_rate_outside']=[80.0,180.0]
    # The pinned C++ server declares ROS parameter values from the initial TOC
    # snapshot, then writes configured firmware values without updating that ROS
    # snapshot. Supply declaration overrides so ROS reports the desired settings.
    # These remain configured values, not acknowledgements read from hardware.
    for name, robot in robots['robots'].items():
        merged={}
        for scope in (robots.get('all',{}),robots['robot_types'][robot['type']],robot):
            for group, values in scope.get('firmware_params',{}).items():
                merged.setdefault(group,{}).update(values)
        for group, values in merged.items():
            for key, value in values.items():
                server[f'{name}.params.{group}.{key}']=value
    server['poses_qos_deadline'] = mocap['topics']['poses']['qos']['deadline']
    mocap['rigid_bodies'] = {}
    for name, robot in robots['robots'].items():
        kind = robots['robot_types'][robot['type']]['motion_capture']
        if robot['enabled'] and kind['tracking'] == 'librigidbodytracker':
            mocap['rigid_bodies'][name] = dict(initial_position=robot['initial_position'],
                                              marker=kind['marker'], dynamics=kind['dynamics'])
    return server, mocap


def generate_launch_description():
    from ament_index_python.packages import get_package_share_directory
    from launch import LaunchDescription
    from launch.actions import ExecuteProcess
    from launch_ros.actions import Node
    share = Path(get_package_share_directory('crazyflie'))
    defaults = yaml.safe_load((share/'config/server.yaml').read_text())['/crazyflie_server']['ros__parameters']
    server, mocap = load_parameters(server_defaults=defaults, controller_override=int(os.environ.get("PDG_CONTROLLER_ID", "0")), vicon_only=os.environ.get("PDG_VICON_ONLY", "0")=="1")
    description = Path(get_package_share_directory('crazyflie_description'))/'urdf/crazyflie_description.urdf'
    server['robot_description'] = description.read_text()
    correction=load_body_frame()
    if correction:
        mocap['topics']['tf']['child_frame_id']='{}_marker'
    nodes=[
        Node(package='motion_capture_tracking', executable='motion_capture_tracking_node',
             name='motion_capture_tracking', output='screen', parameters=[mocap],
             remappings=[('/poses','/poses_raw')] if correction else []),
        Node(package='crazyflie_server_cpp', executable='crazyflie_server',
             name='crazyflie_server', output='screen', parameters=[server]),
    ]
    if correction:
        nodes.append(ExecuteProcess(cmd=[str(ROOT/'.venv/bin/python'),str(ROOT/'scripts/relay_body_pose.py'),
                     '--deadline-hz',str(server['poses_qos_deadline'])],output='screen'))
    return LaunchDescription(nodes)
