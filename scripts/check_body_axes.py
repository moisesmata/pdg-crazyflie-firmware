#!/usr/bin/env python3
"""Props-off stationary tilt check: compare raw IMU gravity with Vicon body axes."""
import json
import argparse
import time
from pathlib import Path
import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from motion_capture_tracking_interfaces.msg import NamedPoseArray
from crazyflie_interfaces.msg import LogDataGeneric,Status
from scipy.spatial.transform import Rotation

ROOT=Path(__file__).resolve().parents[1]
p=argparse.ArgumentParser(description=__doc__)
p.add_argument('--label',choices=['level','nose-up','right-side-up','nose-down','corrected-level','corrected-nose-up','corrected-nose-down'],help='Preserve this capture separately for comparison')
p.add_argument('--level',action='store_true',help='Check a stationary level pose instead of a tilted pose')
args=p.parse_args()
rclpy.init();n=rclpy.create_node('pdg_body_axes_check');latest=None;status=None;rows=[]
counts=dict(pose_arrays=0,body_poses=0,imu=0,status=0)
def poses(msg):
    global latest
    counts['pose_arrays']+=1
    age=n.get_clock().now().nanoseconds*1e-9-msg.header.stamp.sec-msg.header.stamp.nanosec*1e-9
    if msg.header.frame_id.lstrip('/')!='world' or not -.05<=age<=.15:return
    for p in msg.poses:
        if p.name=='Crazyflie':
            counts['body_poses']+=1
            q=p.pose.orientation;latest=(time.monotonic(),np.array([q.x,q.y,q.z,q.w]))
def health(msg):
    global status
    counts['status']+=1
    status=(msg,time.monotonic())
def imu(msg):
    counts['imu']+=1
    if latest is None or status is None:return
    if time.monotonic()-latest[0]>.05 or time.monotonic()-status[1]>2.5:return
    if status[0].supervisor_info&2:raise RuntimeError('Axis check requires unarmed drone with props removed')
    a=np.array(msg.values,dtype=float);q=latest[1]
    if a.shape!=(3,) or not np.isfinite(a).all() or not .9<np.linalg.norm(a)<1.1:return
    if not np.isfinite(q).all() or abs(np.linalg.norm(q)-1)>.02:return
    expected=Rotation.from_quat(q).inv().apply([0,0,1])
    rows.append(dict(acceleration_g=a.tolist(),quaternion_xyzw=q.tolist(),
                     expected_gravity=expected.tolist(),
                     disagreement_deg=float(np.rad2deg(np.arccos(np.clip(np.dot(a/np.linalg.norm(a),expected),-1,1)))),
                     tilt_deg=float(np.rad2deg(np.arccos(np.clip(expected[2],-1,1))))))
s1=n.create_subscription(NamedPoseArray,'/poses',poses,qos_profile_sensor_data)
s2=n.create_subscription(Status,'/Crazyflie/status',health,10)
s3=n.create_subscription(LogDataGeneric,'/Crazyflie/imu',imu,10)
report={'passed':False,'flight_commands_sent':False,'scope':'One stationary tilted pose; does not certify a complete calibration'}
try:
    print('Read-only: keep the props-off drone stationary in the requested '+('level' if args.level else 'tilted')+' pose.',flush=True)
    end=time.monotonic()+6
    while time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.02)
    if len(rows)<20:raise RuntimeError('Need fresh tracking, raw IMU and unarmed supervisor telemetry; restart the server after enabling imu logging')
    errors=np.array([r['disagreement_deg'] for r in rows]);tilts=np.array([r['tilt_deg'] for r in rows])
    acc=np.array([r['acceleration_g'] for r in rows]);q=np.array([r['quaternion_xyzw'] for r in rows])
    motion=float(np.rad2deg(max(2*np.arccos(np.clip(abs(q@q[0]),0,1)))))
    report.update(samples=len(rows),median_disagreement_deg=float(np.median(errors)),max_disagreement_deg=float(max(errors)),
                  median_tilt_deg=float(np.median(tilts)),attitude_range_deg=motion,acceleration_std_g=np.std(acc,axis=0).tolist())
    tilt_ok=max(tilts)<5 if args.level else min(tilts)>8 and max(tilts)<25
    report['passed']=bool(max(errors)<5 and tilt_ok and motion<2 and max(np.std(acc,axis=0))<.03)
    if not report['passed']:report['error']='Requires a stable '+('level pose below 5 degrees' if args.level else 'tilt of 8–25 degrees')+' and IMU/Vicon gravity disagreement below 5 degrees'
except Exception as exc:report['error']=str(exc)
finally:
    report['stream_counts']=counts
    (ROOT/'output/setup/body-axes-check.json').write_text(json.dumps(report,indent=2)+'\n')
    (ROOT/'output/setup/body-axes-samples.json').write_text(json.dumps(rows)+'\n')
    if args.label:
        (ROOT/f'output/setup/body-axes-{args.label}.json').write_text(json.dumps(report,indent=2)+'\n')
        (ROOT/f'output/setup/body-axes-{args.label}-samples.json').write_text(json.dumps(rows)+'\n')
    print(json.dumps(report,indent=2));n.destroy_node();rclpy.shutdown()
raise SystemExit(0 if report['passed'] else 1)
