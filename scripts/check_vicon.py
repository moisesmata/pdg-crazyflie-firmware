#!/usr/bin/env python3
"""Observe Vicon via ROS for a bounded interval. Never connects to a radio."""
import argparse
import json
from pathlib import Path
import signal
import os
import subprocess
import time
import rclpy
from rclpy.qos import qos_profile_sensor_data
from ament_index_python.packages import get_package_prefix
from motion_capture_tracking_interfaces.msg import NamedPoseArray

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--seconds', type=float, default=15)
parser.add_argument('--robot', default='Crazyflie')
parser.add_argument('--receiver', action='store_true', help='Start a temporary Vicon receiver; omit if already running')
parser.add_argument('--mock', action='store_true', help='Test ROS transport with synthetic poses; requires --receiver')
parser.add_argument('--output-dir', type=Path, help='Directory for diagnostic reports')
args = parser.parse_args()
if args.mock and not args.receiver:
    parser.error('--mock requires --receiver')
prefix = 'ros-mock' if args.mock else 'vicon'
root = Path(__file__).resolve().parents[1]
output = args.output_dir or root/'output/setup'
output.mkdir(parents=True, exist_ok=True)
rclpy.init()
node = rclpy.create_node('pdg_vicon_check')
seen = {}
frames = 0
first = last = None
ages = []
def callback(msg):
    global frames, first, last
    now = time.monotonic()
    first = now if first is None else first
    last = now
    frames += 1
    stamp = msg.header.stamp.sec + msg.header.stamp.nanosec*1e-9
    ages.append(node.get_clock().now().nanoseconds*1e-9-stamp)
    for item in msg.poses:
        p, q = item.pose.position, item.pose.orientation
        seen[item.name] = dict(frame=msg.header.frame_id, position_m=[p.x,p.y,p.z], quaternion_xyzw=[q.x,q.y,q.z,q.w])
sub = node.create_subscription(NamedPoseArray, '/poses', callback, qos_profile_sensor_data)
child = None
log = None
try:
    if args.receiver:
        log = (output/f'{prefix}-receiver.log').open('w')
        executable = Path(get_package_prefix('motion_capture_tracking'))/'lib/motion_capture_tracking/motion_capture_tracking_node'
        command = [str(executable),'--ros-args','-r','__node:=motion_capture_tracking','--params-file',str(root/'config/motion_capture.yaml')]
        if args.mock:
            command += ['-p', 'type:=mock', '-p', f'rigid_bodies.{args.robot}.initial_position:=[0.0,0.0,0.05]']
        child = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    deadline = time.monotonic()+args.seconds
    while time.monotonic()<deadline:
        rclpy.spin_once(node,timeout_sec=.1)
    report = dict(synthetic=args.mock, frames=frames,rate_hz=(frames-1)/(last-first) if frames>1 else 0,
                  max_age_s=max(ages) if ages else None, bodies=seen,
                  expected_robot=args.robot, expected_robot_seen=args.robot in seen,
                  ros_nodes=node.get_node_names_and_namespaces(),
                  pose_publishers=node.count_publishers('/poses'),
                  receiver_executable=str(executable) if child else None,
                  receiver_exit_code=child.poll() if child else None)
    (output/f'{prefix}-check.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report,indent=2))
finally:
    if child:
        if child.poll() is None:
            os.killpg(child.pid, signal.SIGINT)
            try: child.wait(timeout=3)
            except subprocess.TimeoutExpired:
                os.killpg(child.pid, signal.SIGKILL); child.wait()
    if log: log.close()
    node.destroy_node()
    rclpy.shutdown()
raise SystemExit(0 if args.robot in seen else 1)
