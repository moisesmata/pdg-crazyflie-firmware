#!/usr/bin/env python3
"""Read-only ground comparison: ROS mocap -> received firmware pose -> Kalman state."""
import argparse
import json
import time
from pathlib import Path
import numpy as np
import rclpy
from rclpy.qos import QoSProfile,ReliabilityPolicy
from motion_capture_tracking_interfaces.msg import NamedPoseArray
from crazyflie_interfaces.msg import LogDataGeneric,Status
from geometry_msgs.msg import PoseStamped

p=argparse.ArgumentParser(description=__doc__)
p.add_argument('--seconds',type=float,default=8.)
p.add_argument('--output',type=Path,help='Preserve this audit and source pose samples in a separate directory')
p.add_argument('--moving',action='store_true',help='Capture a props-off hand movement; do not apply stationary alignment pass criteria')
a=p.parse_args()
if not 5<=a.seconds<=120:p.error('Use 5 to 120 seconds')
rclpy.init();n=rclpy.create_node('pdg_mocap_path_audit');latest={};rows=[];faults=[];source_samples=[]
previous_quaternions={};orientation_jumps=[]
def capture(stream,position,q,**metadata):
 now=time.monotonic()
 source_samples.append(dict(stream=stream,monotonic_s=now,position_m=position,
                            quaternion_wxyz=q.tolist(),**metadata))
 if stream in previous_quaternions:
  then,previous=previous_quaternions[stream]
  try:
   change=angle(previous,q)
   if 0<now-then<=.15 and change>30:
    orientation_jumps.append(dict(stream=stream,monotonic_s=now,interval_s=now-then,change_deg=change))
  except ValueError as e:faults.append(str(e))
 previous_quaternions[stream]=(now,q.copy())
def parts(p):
 q=p.orientation;v=p.position
 return np.array([v.x,v.y,v.z]),np.array([q.w,q.x,q.y,q.z])
def ext(m):
 stamp=m.header.stamp.sec+m.header.stamp.nanosec*1e-9
 for x in m.poses:
  if x.name=='Crazyflie':
   v,q=parts(x.pose)
   capture('external',v.tolist(),q,source_timestamp_s=stamp,frame_id=m.header.frame_id)
 if m.header.frame_id.lstrip('/')!='world':return
 if not -.05<=n.get_clock().now().nanoseconds*1e-9-stamp<=.15:return
 for x in m.poses:
  if x.name=='Crazyflie':latest['external']=(time.monotonic(),*parts(x.pose))
def onboard(m):
 v,q=parts(m.pose)
 capture('estimator',v.tolist(),q)
 latest['onboard']=(time.monotonic(),v,q)
def status(m):
 latest['status']=(time.monotonic(),m)
 if m.supervisor_info&2:faults.append('Ground diagnostic requires unarmed drone')
def rxq(m):
 if len(m.values)==4:
  x,y,z,w=map(float,m.values);q=np.array([w,x,y,z])
  capture('firmware_received',None,q,timestamp_ms=int(m.timestamp))
  latest['rxq']=(time.monotonic(),q)
def angle(q,r):
 if not np.isfinite(q).all() or not np.isfinite(r).all() or abs(np.linalg.norm(q)-1)>.02 or abs(np.linalg.norm(r)-1)>.02:
  raise ValueError('Invalid quaternion')
 return float(np.rad2deg(2*np.arccos(np.clip(abs(np.dot(q/np.linalg.norm(q),r/np.linalg.norm(r))),0,1))))
def rxp(m):
 if len(m.values)!=4 or not all(k in latest for k in ('external','onboard','rxq','status')):return
 now=time.monotonic()
 if any(now-latest[k][0]>.25 for k in ('external','onboard','rxq')) or now-latest['status'][0]>2.5:return
 _,p,q=latest['external'];_,ep,eq=latest['onboard'];_,rq=latest['rxq'];rp=np.array(m.values[:3],dtype=float)
 # Firmware uses a 1 kHz RTOS tick. Both clocks wrap; tick is uint16.
 # Log timestamp is sampled just before variables, so a 1–3 tick snapshot
 # race may put the receive tick slightly ahead. This is diagnostic only.
 age=(int(m.timestamp)-int(m.values[3]))&0xffff
 if age>=65533:age=0
 try:
  rows.append(dict(received_position_m=rp.tolist(),received_tick_age_ms=age,
   host_to_received_position_error_m=float(np.linalg.norm(p-rp)),
   host_to_received_orientation_error_deg=angle(q,rq),
   host_to_estimator_position_error_m=float(np.linalg.norm(p-ep)),
   host_to_estimator_orientation_error_deg=angle(q,eq)))
 except ValueError as e:faults.append(str(e))
subs=[n.create_subscription(NamedPoseArray,'/poses',ext,QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT)),
 n.create_subscription(PoseStamped,'/Crazyflie/pose',onboard,10),n.create_subscription(Status,'/Crazyflie/status',status,10),
 n.create_subscription(LogDataGeneric,'/Crazyflie/mocap_rx_orientation',rxq,10),
 n.create_subscription(LogDataGeneric,'/Crazyflie/mocap_rx_position',rxp,10)]
report=dict(flight_commands_sent=False,passed=False)
print('Read-only ground audit: waiting for firmware received-pose logs',flush=True)
try:
 deadline=time.monotonic()+15
 while not rows and not faults and time.monotonic()<deadline:rclpy.spin_once(n,timeout_sec=.05)
 if not rows:raise RuntimeError('No paired received-pose samples; inspect topic delivery and source-sample counts before restarting')
 end=time.monotonic()+a.seconds
 while time.monotonic()<end and not faults:rclpy.spin_once(n,timeout_sec=.02)
 if faults:raise RuntimeError('; '.join(sorted(set(faults))))
 if len(rows)<20:raise RuntimeError('Insufficient paired received-pose samples')
 maxima={k:max(r[k] for r in rows) for k in rows[0] if k!='received_position_m'}
 spread=float(np.max(np.ptp([r['received_position_m'] for r in rows],axis=0)))
 report.update(samples=len(rows),maxima=maxima,position_range_m=spread)
 report['passed']=bool(spread<.005 and maxima['received_tick_age_ms']<50 and
  maxima['host_to_received_position_error_m']<.01 and maxima['host_to_received_orientation_error_deg']<2 and
  maxima['host_to_estimator_position_error_m']<.04 and maxima['host_to_estimator_orientation_error_deg']<5)
 report['scope']='Stationary ground path check; receive tick is firmware input age, not total camera-to-drone latency'
 if a.moving:
  # Asynchronous 10 Hz receive logs cannot establish alignment while moving.
  # Preserve their evidence without treating delivery skew as a frame error.
  report['passed']=None
  report['capture_completed']=True
  report['scope']='Unarmed hand-movement capture; inspect timestamped source/received/estimated streams. No stationary alignment verdict.'
except Exception as e:report['error']=str(e)
finally:
 report['orientation_jumps']=orientation_jumps
 if orientation_jumps:report['passed']=False
 report['source_sample_counts']={stream:sum(r['stream']==stream for r in source_samples)
                                 for stream in ('external','firmware_received','estimator')}
 out=a.output or Path(__file__).resolve().parents[1]/'output/setup'
 out.mkdir(parents=True,exist_ok=True)
 (out/'mocap-path-audit.json').write_text(json.dumps(report,indent=2)+'\n')
 (out/'mocap-path-samples.json').write_text(json.dumps(rows)+'\n')
 (out/'mocap-source-samples.json').write_text(json.dumps(source_samples)+'\n')
 print(json.dumps(report,indent=2));n.destroy_node();rclpy.shutdown()
raise SystemExit(0 if report['passed'] or (a.moving and report.get('capture_completed') and not faults) else 1)
