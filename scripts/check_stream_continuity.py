#!/usr/bin/env python3
"""Observe ground telemetry continuity; send no parameters or flight commands."""
import argparse
import json
import time
from pathlib import Path
import rclpy
from rclpy.qos import qos_profile_sensor_data
from crazyflie_interfaces.msg import Status,LogDataGeneric
from geometry_msgs.msg import PoseStamped
from motion_capture_tracking_interfaces.msg import NamedPoseArray

p=argparse.ArgumentParser(description=__doc__)
p.add_argument('--seconds',type=float,default=30)
a=p.parse_args()
if not 5<=a.seconds<=120:p.error('Use 5 to 120 seconds')
rclpy.init();n=rclpy.create_node('pdg_ground_stream_continuity')
root=Path(__file__).resolve().parents[1]
rows=[];received={k:[] for k in ('status','onboard','vicon','actuators')};faults=[]
start=time.monotonic()
def sample(kind,values):
 t=time.monotonic()-start;received[kind].append(t);rows.append(dict(t=t,stream=kind,**values))
def status(m):
 sample('status',dict(flags=m.supervisor_info,voltage=m.battery_voltage))
 if m.supervisor_info&2:faults.append('Drone must be unarmed for this ground check')
def onboard(m):
 q=m.pose.orientation;p=m.pose.position
 sample('onboard',dict(p=[p.x,p.y,p.z],q_wxyz=[q.w,q.x,q.y,q.z]))
def vicon(m):
 for x in m.poses:
  if x.name=='Crazyflie':
   p=x.pose.position;q=x.pose.orientation
   sample('vicon',dict(p=[p.x,p.y,p.z],q_wxyz=[q.w,q.x,q.y,q.z]))
def actuators(m):
 sample('actuators',dict(timestamp_ms=int(m.timestamp),values=list(m.values)))
 if any(float(v)!=0 for v in m.values[:4]):faults.append('Nonzero motor commands during unarmed ground check')
subs=[n.create_subscription(Status,'/Crazyflie/status',status,10),
      n.create_subscription(PoseStamped,'/Crazyflie/pose',onboard,10),
      n.create_subscription(NamedPoseArray,'/poses',vicon,qos_profile_sensor_data),
      n.create_subscription(LogDataGeneric,'/Crazyflie/actuators',actuators,10)]
print('Waiting for all streams; no flight commands sent',flush=True)
try:
 discovery_deadline=time.monotonic()+15
 while not all(received.values()) and time.monotonic()<discovery_deadline and not faults:
  rclpy.spin_once(n,timeout_sec=.02)
 if not all(received.values()):faults.append('Not all streams appeared within 15 seconds')
 # Start the timed interval only after ROS discovery and server startup.
 measurement_start=time.monotonic()-start
 print(f'Observing {a.seconds:g} seconds after discovery',flush=True)
 while time.monotonic()-start-measurement_start<a.seconds and not faults:
  rclpy.spin_once(n,timeout_sec=.02)
 end=time.monotonic()-start

 streams={}
 for k,times in received.items():
  active=[t for t in times if t>=measurement_start]
  previous=[t for t in times if t<measurement_start]
  bounds=([previous[-1]] if previous else [measurement_start])+active+[end]
  gap=max((b-a for a,b in zip(bounds,bounds[1:])),default=end)
  limit=2.5 if k=='status' else .25
  streams[k]=dict(samples=len(times),max_gap_s=gap,last_age_s=end-times[-1] if times else None,passed=bool(active) and gap<=limit)
 report=dict(flight_commands_sent=False,seconds=end-measurement_start,discovery_seconds=measurement_start,passed=not faults and all(s['passed'] for s in streams.values()),streams=streams,faults=sorted(set(faults)))
 out=root/'output/setup';out.mkdir(exist_ok=True,parents=True)
 (out/'stream-continuity.json').write_text(json.dumps(report,indent=2)+'\n')
 (out/'stream-continuity-samples.json').write_text(json.dumps(rows)+'\n')
 print(json.dumps(report,indent=2))
finally:
 n.destroy_node();rclpy.shutdown()
raise SystemExit(0 if report['passed'] else 1)
