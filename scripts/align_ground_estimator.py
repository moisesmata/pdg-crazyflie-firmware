#!/usr/bin/env python3
"""Initialize Kalman position/yaw from a stationary tracked pad; never arm or fly."""
import argparse
import json
import time
from pathlib import Path
import numpy as np
import rclpy
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from rcl_interfaces.srv import SetParameters
from crazyflie_interfaces.msg import Status
from motion_capture_tracking_interfaces.msg import NamedPoseArray
from pdg.ground_health import hover_ground_angles
import yaml

ROOT=Path(__file__).resolve().parents[1]
p=argparse.ArgumentParser(description=__doc__)
p.add_argument('--apply',action='store_true',help='Write ground initialization parameters and reset the estimator')
a=p.parse_args()
cfg=yaml.safe_load((ROOT/'config/hover-today.yaml').read_text())
robot=cfg['execution']['robot'];pad=np.array(cfg['calibration']['ground_position_m'])
rclpy.init();node=rclpy.create_node('pdg_ground_estimator_alignment')
rows=[];status=None;fault=None;last_stamp=None
def pose(msg):
    global fault,last_stamp
    stamp=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
    age=node.get_clock().now().nanoseconds*1e-9-stamp
    if msg.header.frame_id.lstrip('/')!=cfg['execution']['frame_id'].lstrip('/') or not -.05<=age<=.15:
        fault='Invalid pose frame or timestamp';return
    if last_stamp is not None and stamp<=last_stamp:return
    for item in msg.poses:
        if item.name!=robot:continue
        v=item.pose.position;q=item.pose.orientation
        xyz=np.array([v.x,v.y,v.z]);quat=np.array([q.w,q.x,q.y,q.z])
        if not np.isfinite(xyz).all():fault='Invalid position';return
        tilt,yaw=hover_ground_angles(quat)
        if tilt>5:fault='Tracked body is tilted more than 5 degrees';return
        rows.append((time.monotonic(),xyz,quat,yaw));last_stamp=stamp
def health(msg):
    global status
    status=(msg,time.monotonic())
s1=node.create_subscription(NamedPoseArray,'/poses',pose,qos_profile_sensor_data)
s2=node.create_subscription(Status,f'/{robot}/status',health,10)
client=node.create_client(SetParameters,'/crazyflie_server/set_parameters')
report={'applied':False,'flight_commands_sent':False}
def set_values(values):
    req=SetParameters.Request()
    req.parameters=[Parameter(robot+'.params.'+name,value=value).to_parameter_msg() for name,value in values.items()]
    f=client.call_async(req);end=time.monotonic()+5
    while not f.done() and time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.02)
    if not f.done() or f.result() is None:raise RuntimeError('Estimator parameter write timed out')
    if len(f.result().results)!=len(values) or not all(r.successful for r in f.result().results):
        raise RuntimeError('Estimator parameter write rejected: '+str(f.result()))
try:
    end=time.monotonic()+4
    while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.02)
    if fault:raise RuntimeError(fault)
    if len(rows)<100 or time.monotonic()-rows[-1][0]>.15:raise RuntimeError('Insufficient fresh tracking')
    if node.count_publishers('/poses')!=1:raise RuntimeError('Require exactly one pose publisher')
    if status is None or time.monotonic()-status[1]>2.5:raise RuntimeError('Missing fresh supervisor status')
    st=status[0]
    if st.pm_state!=0 or not np.isfinite(st.battery_voltage) or st.battery_voltage<3.8:raise RuntimeError('Battery must be at least 3.8 V')
    if not st.supervisor_info&1 or st.supervisor_info&(2|32|64):raise RuntimeError('Drone must be unarmed and ready on the ground')
    xyz=np.array([r[1] for r in rows]);q=np.array([r[2] for r in rows])
    center=np.median(xyz,axis=0)
    if np.linalg.norm(center[:2]-pad[:2])>.025 or abs(center[2]-pad[2])>.02:raise RuntimeError('Drone is not at the calibrated pad')
    if max(np.ptp(xyz,axis=0))>.005 or max(2*np.arccos(np.clip(abs(q@q[0]),0,1)))>np.deg2rad(2):
        raise RuntimeError('Drone must remain stationary; tracking changed during capture')
    yaw=float(rows[-1][3])
    values=dict(zip(['kalman.initialX','kalman.initialY','kalman.initialZ'],map(float,center)))
    values['kalman.initialYaw']=float(np.deg2rad(yaw))
    report.update(position_m=center.tolist(),heading_deg=yaw,battery_voltage=st.battery_voltage,parameters=values)
    if a.apply:
        if not client.wait_for_service(timeout_sec=3):raise RuntimeError('Radio parameter service unavailable')
        set_values(values)
        set_values({'kalman.resetEstimation':1})
        end=time.monotonic()+.3
        while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.02)
        set_values({'kalman.resetEstimation':0})
        report['applied']=True
    print(json.dumps(report,indent=2))
finally:
    (ROOT/'output/setup/estimator-alignment.json').write_text(json.dumps(report,indent=2)+'\n')
    node.destroy_node();rclpy.shutdown()
