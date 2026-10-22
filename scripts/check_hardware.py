#!/usr/bin/env python3
"""Read pose and firmware identity from a running project launch; never arm."""
from pathlib import Path
import json
import argparse
import time
from types import SimpleNamespace
import numpy as np
import yaml
import rclpy
from rcl_interfaces.srv import GetParameters
from rclpy.qos import qos_profile_sensor_data
from motion_capture_tracking_interfaces.msg import NamedPoseArray
from geometry_msgs.msg import PoseStamped
from crazyflie_interfaces.msg import Status
from pdg.ground_health import validate_ground_health, validate_controller_parameters

parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('--wait',type=float,default=5.,help='Seconds to wait for server startup and telemetry')
parser.add_argument('--controller-id',type=int,choices=(1,6),default=None,help='Expected PID baseline (1) or project Mellinger (6/auto)')
parser.add_argument('--vicon-only',action='store_true',help='Also require Kalman, disabled flow and Flow/Z-ranger deck absent')
args=parser.parse_args()
if args.controller_id is None:args.controller_id=1 if args.vicon_only else 6
root = Path(__file__).resolve().parents[1]
config = yaml.safe_load((root/'config/nominal.yaml').read_text())['execution']
rclpy.init()
node = rclpy.create_node('pdg_hardware_check')
poses = []
health = {}
def callback(msg):
    for item in msg.poses:
        if item.name == config['robot']:
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec*1e-9
            poses.append(dict(frame=msg.header.frame_id, timestamp_s=stamp,
                              age_s=node.get_clock().now().nanoseconds*1e-9-stamp,
                              position_m=[item.pose.position.x,item.pose.position.y,item.pose.position.z],
                              quaternion_wxyz=[item.pose.orientation.w,item.pose.orientation.x,item.pose.orientation.y,item.pose.orientation.z]))
sub = node.create_subscription(NamedPoseArray, '/poses', callback, qos_profile_sensor_data)
def on_status(msg): health['status'] = (msg,time.monotonic())
def on_estimate(msg): health['estimate'] = (msg,time.monotonic())
status_sub=node.create_subscription(Status, f"/{config['robot']}/status", on_status,10)
estimate_sub=node.create_subscription(PoseStamped, f"/{config['robot']}/pose",on_estimate,10)
client = node.create_client(GetParameters, '/crazyflie_server/get_parameters')
report = dict(robot=config['robot'], passed=False)
try:
    if not client.wait_for_service(timeout_sec=args.wait):
        raise RuntimeError('Start scripts/launch_crazyswarm.sh first; parameter service unavailable')
    request = GetParameters.Request()
    names = ['pdg.apiVersion','stabilizer.controller','pdg.controllerId','pdg.capEnable']
    if args.vicon_only:names+=['stabilizer.estimator','motion.disable','deck.bcFlow','deck.bcFlow2','deck.bcZRanger2']
    request.names = [config['robot']+'.params.'+name for name in names]
    future = client.call_async(request)
    deadline = time.monotonic()+args.wait
    while time.monotonic()<deadline and (not future.done() or not poses or 'status' not in health or 'estimate' not in health):
        rclpy.spin_once(node, timeout_sec=.05)
    if not future.done() or future.result() is None:
        raise RuntimeError('Firmware parameter query did not complete')
    values = future.result().values
    report['parameters'] = {name:dict(type=v.type,value=v.integer_value) for name,v in zip(names,values)}
    if poses:
        poses[-1]['age_s'] = node.get_clock().now().nanoseconds*1e-9-poses[-1]['timestamp_s']
    report['pose'] = poses[-1] if poses else None
    if len(values)!=len(names) or any(v.type!=2 for v in values):
        raise RuntimeError('Missing/uninitialized PDG parameters; verify firmware and project launch')
    api,controller,oot,cap = [v.integer_value for v in values[:4]]
    if args.vicon_only:
        from pdg.ground_health import validate_vicon_only_inputs
        validate_vicon_only_inputs(*[v.integer_value for v in values[4:]])
    validate_controller_parameters(api,controller,oot,cap,baseline_pid=args.controller_id==1)
    if not poses:
        raise RuntimeError('No Crazyflie pose received; check Vicon tracking')
    pose=poses[-1]
    if pose['frame'].lstrip('/') != config['frame_id'].lstrip('/') or not -.05 <= pose['age_s'] <= config['max_pose_age']:
        raise RuntimeError('Pose frame or freshness fails mission preflight')
    if 'status' not in health or 'estimate' not in health:
        raise RuntimeError('Battery/supervisor or onboard pose telemetry is missing')
    status,received=health['status']
    estimate,estimated_at=health['estimate']
    if time.monotonic()-received>2.5 or time.monotonic()-estimated_at>.15:
        raise RuntimeError('Onboard pose/status telemetry is stale')
    if estimate.header.frame_id.lstrip('/')!=config['frame_id'].lstrip('/'):
        raise RuntimeError('Onboard pose frame differs from configured frame')
    p,q=estimate.pose.position,estimate.pose.orientation
    report['battery_voltage']=status.battery_voltage
    report['supervisor_info']=status.supervisor_info
    report['estimator_position_m']=[p.x,p.y,p.z]
    report['estimator_quaternion_wxyz']=[q.w,q.x,q.y,q.z]
    observation=SimpleNamespace(position=np.array(pose['position_m']),quaternion_wxyz=np.array(pose['quaternion_wxyz']))
    validate_ground_health(status,[p.x,p.y,p.z],[q.w,q.x,q.y,q.z],observation)
    report['pose_publishers']=node.count_publishers('/poses')
    if report['pose_publishers']!=1:
        raise RuntimeError('Require exactly one external-pose publisher')
    report['passed'] = True
    report['scope'] = 'Read-only connectivity check; vehicle calibration and flight behavior remain unverified'
except Exception as exc:
    report['error'] = str(exc)
finally:
    output = root/'output/setup/hardware-check.json'
    output.parent.mkdir(parents=True,exist_ok=True)
    output.write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report,indent=2))
    node.destroy_node()
    rclpy.shutdown()
raise SystemExit(0 if report['passed'] else 1)
