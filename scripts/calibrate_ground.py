#!/usr/bin/env python3
"""Measure a stationary takeoff pose; write only a candidate flight configuration."""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time
import numpy as np
from scipy.spatial.transform import Rotation
from pdg.ground_health import hover_ground_angles
import yaml
import rclpy
from ament_index_python.packages import get_package_prefix
from motion_capture_tracking_interfaces.msg import NamedPoseArray
from rclpy.qos import qos_profile_sensor_data

ROOT=Path(__file__).resolve().parents[1]
p=argparse.ArgumentParser(description=__doc__)
p.add_argument('--seconds',type=float,default=10)
p.add_argument('--receiver',action='store_true')
p.add_argument('--preserve-heading',action='store_true',help='Capture any stable yaw for the patched heading-preserving hover')
p.add_argument('--mass-grams',type=float)
p.add_argument('--write-candidate',action='store_true')
a=p.parse_args()
if a.seconds<3 or not math.isfinite(a.seconds):p.error('--seconds must be finite and at least 3')
if a.write_candidate and (a.mass_grams is None or not math.isfinite(a.mass_grams) or a.mass_grams<=0):p.error('Measured --mass-grams required for a candidate')
config=yaml.safe_load((ROOT/'config/nominal.yaml').read_text())
robot=config['execution']['robot'];frame=config['execution']['frame_id']
output=ROOT/'output/setup';output.mkdir(parents=True,exist_ok=True)
rclpy.init();node=rclpy.create_node('pdg_ground_calibration')
rows=[];rejected=0;last_stamp=None
warmup_end=None
def callback(msg):
    global rejected,last_stamp,warmup_end
    if warmup_end is None:
        warmup_end=time.monotonic()+1.0
        return
    if time.monotonic()<warmup_end:return
    stamp=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
    age=node.get_clock().now().nanoseconds*1e-9-stamp
    if msg.header.frame_id.lstrip('/')!=frame.lstrip('/') or not -.05<=age<=.15:
        rejected+=1;return
    if last_stamp is not None and stamp<=last_stamp:return
    for item in msg.poses:
        if item.name!=robot:continue
        pos=item.pose.position;q=item.pose.orientation
        value=[stamp,pos.x,pos.y,pos.z,q.w,q.x,q.y,q.z]
        if not np.isfinite(value).all() or abs(np.linalg.norm(value[4:])-1)>.02:
            rejected+=1;return
        rows.append(value);last_stamp=stamp
sub=node.create_subscription(NamedPoseArray,'/poses',callback,qos_profile_sensor_data)
child=None;log=None
try:
    if a.receiver:
        executable=Path(get_package_prefix('motion_capture_tracking'))/'lib/motion_capture_tracking/motion_capture_tracking_node'
        log=(output/'ground-receiver.log').open('w')
        child=subprocess.Popen([str(executable),'--ros-args','-r','__node:=motion_capture_tracking','--params-file',str(ROOT/'config/motion_capture.yaml')],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    deadline=time.monotonic()+a.seconds
    while time.monotonic()<deadline:rclpy.spin_once(node,timeout_sec=.05)
    report=dict(robot=robot,frame=frame,samples=len(rows),rejected_frames=rejected,passed=False,ground_placement_confirmed_by_script=False)
    if len(rows)<100:report['error']='Fewer than 100 valid tracked poses'
    else:
        data=np.asarray(rows);xyz=data[:,1:4];q=data[:,4:]
        center=np.median(xyz,axis=0);spread=np.ptp(xyz,axis=0)
        angle=np.rad2deg(2*np.arccos(np.clip(np.abs(q[:,0]),0,1)))
        attitude=Rotation.from_quat(q[:,[1,2,3,0]]).mean().as_euler('xyz',degrees=True)
        report.update(position_m=center.tolist(),position_range_m=spread.tolist(),roll_pitch_yaw_deg=attitude.tolist(),
                      max_orientation_angle_from_world_deg=float(max(angle)),duration_s=float(data[-1,0]-data[0,0]))
        angles=np.asarray([hover_ground_angles(value) for value in q])
        report['max_tilt_deg']=float(max(angles[:,0]))
        report['max_abs_heading_deg']=float(max(abs(angles[:,1])))
        relative_angle=np.rad2deg(2*np.arccos(np.clip(np.abs(q @ q[0]),0,1)))
        report['preserve_heading']=a.preserve_heading
        report['max_attitude_change_deg']=float(max(relative_angle))
        heading_ok=max(relative_angle)<2 if a.preserve_heading else max(abs(angles[:,1]))<5
        report['passed']=bool(max(spread)<.01 and max(angles[:,0])<5 and heading_ok and center[2]>=0)
        if not report['passed']:report['error']='Requires stationary body, tilt and heading each within 5 degrees, and nonnegative ground-origin height'
        if a.write_candidate and report['passed']:
            config['vehicle']['mass']=a.mass_grams/1000
            config['execution']['ground_body_height']=float(center[2])
            config['execution']['takeoff_duration']=4.0
            config['execution']['settle_duration']=2.0
            problem=config['problem']
            problem.update(initial_position=[float(center[0]),float(center[1]),float(center[2]+.3)],
                           target_position=center.tolist(),terminal_velocity=[0.,0.,-.05],
                           min_time=3.0,initial_time=4.0,max_speed=.5,max_tilt_deg=10.,
                           workspace_xy=float(max(2.,max(abs(center[:2]))+.5)),ceiling=float(center[2]+.6))
            candidate=ROOT/'config/flight-today.yaml'
            candidate.write_text('# Candidate: stationary ground capture; room clearance and body-frame alignment still require operator verification.\n'+yaml.safe_dump(config,sort_keys=False))
            report['candidate']=str(candidate)
    (output/'ground-samples.json').write_text(json.dumps(rows)+'\n')
    (output/'ground-calibration.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report,indent=2))
finally:
    if child and child.poll() is None:
        os.killpg(child.pid,signal.SIGINT)
        try:child.wait(timeout=3)
        except subprocess.TimeoutExpired:os.killpg(child.pid,signal.SIGKILL);child.wait()
    if log:log.close()
    node.destroy_node();rclpy.shutdown()
raise SystemExit(0 if report['passed'] else 1)
