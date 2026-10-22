#!/usr/bin/env python3
"""Preview or apply provisional Mellinger settings while unarmed and stopped."""
import argparse
import json
import time
from pathlib import Path
import numpy as np
import yaml

ROOT=Path(__file__).resolve().parents[1]
p=argparse.ArgumentParser(description=__doc__)
p.add_argument('--apply',action='store_true')
p.add_argument('--candidate',type=Path,help='Explicit reviewed tracker settings; default local candidate, then versioned config')
a=p.parse_args()
candidate_path=a.candidate or ROOT/'output/setup/pdg-tracker-candidate.json'
if a.candidate is None and not candidate_path.exists():candidate_path=ROOT/'config/pdg-tracker.json'
candidate=json.loads(candidate_path.read_text())
desired=candidate['parameters']
ground=float(yaml.safe_load((ROOT/'config/hover-today.yaml').read_text())['execution']['ground_body_height'])
print(json.dumps({'apply':a.apply,'parameters':desired,'mass_scale_measured':False,'flight_commands_sent':False},indent=2),flush=True)
if not a.apply:raise SystemExit(0)

import rclpy
from rclpy.parameter import Parameter
from rcl_interfaces.srv import GetParameters,SetParameters
from crazyflie_interfaces.msg import Status,LogDataGeneric
from geometry_msgs.msg import PoseStamped
from motion_capture_tracking_interfaces.msg import NamedPoseArray
from rclpy.qos import qos_profile_sensor_data
from types import SimpleNamespace
from pdg.ground_health import validate_ground_health

rclpy.init();n=rclpy.create_node('pdg_configure_mellinger');latest={};report={'flight_commands_sent':False,'applied':False,'desired':desired}
def status(m):latest['status']=(time.monotonic(),m)
def motors(m):latest['motors']=(time.monotonic(),list(m.values[:4]))
def estimate(m):latest['estimate']=(time.monotonic(),m)
def external(m):
    stamp=m.header.stamp.sec+m.header.stamp.nanosec*1e-9
    if m.header.frame_id.lstrip('/')!='world' or not -.05<=n.get_clock().now().nanoseconds*1e-9-stamp<=.15:return
    for x in m.poses:
        if x.name=='Crazyflie':latest['external']=(time.monotonic(),x.pose)
subs=[n.create_subscription(Status,'/Crazyflie/status',status,10),
      n.create_subscription(LogDataGeneric,'/Crazyflie/actuators',motors,10),
      n.create_subscription(PoseStamped,'/Crazyflie/pose',estimate,10),
      n.create_subscription(NamedPoseArray,'/poses',external,qos_profile_sensor_data)]
get=n.create_client(GetParameters,'/crazyflie_server/get_parameters')
setp=n.create_client(SetParameters,'/crazyflie_server/set_parameters')
def wait_ground():
    end=time.monotonic()+10;steady=None
    while time.monotonic()<end:
        rclpy.spin_once(n,timeout_sec=.02);now=time.monotonic()
        if not all(k in latest for k in ['status','motors','estimate','external']):continue
        if now-latest['status'][0]>2.5 or any(now-latest[k][0]>.25 for k in ['motors','estimate','external']):
            steady=None;continue
        s=latest['status'][1];ep=latest['estimate'][1].pose;xp=latest['external'][1]
        obs=SimpleNamespace(position=np.array([xp.position.x,xp.position.y,xp.position.z]),
                            quaternion_wxyz=np.array([xp.orientation.w,xp.orientation.x,xp.orientation.y,xp.orientation.z]))
        validate_ground_health(s,[ep.position.x,ep.position.y,ep.position.z],
                               [ep.orientation.w,ep.orientation.x,ep.orientation.y,ep.orientation.z],obs)
        if len(latest['motors'][1])!=4 or any(v!=0 for v in latest['motors'][1]):
            raise RuntimeError('Controller configuration requires four zero motor commands')
        if abs(xp.position.z-ground)>.02:raise RuntimeError('Drone is not on the measured pad')
        steady=now if steady is None else steady
        if now-steady>=1.:return
    raise RuntimeError('No fresh, unarmed, stopped ground state')
def call(client,req):
    if not client.wait_for_service(timeout_sec=3):raise RuntimeError('Parameter service unavailable')
    f=client.call_async(req);rclpy.spin_until_future_complete(n,f,timeout_sec=3)
    if not f.done() or f.result() is None:raise RuntimeError('Parameter request timed out')
    return f.result()
def write(values):
    req=SetParameters.Request();req.parameters=[Parameter('Crazyflie.params.'+k,value=v).to_parameter_msg() for k,v in values.items()]
    answer=call(setp,req)
    if len(answer.results)!=len(values) or not all(x.successful for x in answer.results):
        raise RuntimeError('Parameter write rejected: '+str(answer))
before={}
try:
    wait_ground()
    q=GetParameters.Request();q.names=['Crazyflie.params.'+k for k in desired]
    values=call(get,q).values
    if len(values)!=len(desired) or any(v.type not in (2,3) for v in values):raise RuntimeError('Missing tracker parameters')
    before={k:(v.double_value if v.type==3 else v.integer_value) for k,v in zip(desired,values)};report['before']=before
    # Set feedforward and sensor configuration before selecting the tracker.
    write({k:v for k,v in desired.items() if k!='stabilizer.controller'})
    wait_ground()
    write({'stabilizer.controller':desired['stabilizer.controller']})
    wait_ground();report['applied']=True
    report['scope']='Server acknowledged writes and fresh stopped/unarmed state; mass remains provisional, not scale measured'
except Exception as e:
    report['error']=str(e)
    if before:
        try:
            wait_ground()
            write({k:v for k,v in before.items() if k!='stabilizer.controller'})
            write({'stabilizer.controller':before['stabilizer.controller']});report['rollback_acknowledged']=True
        except Exception as rollback:report['rollback_error']=str(rollback)
finally:
    (ROOT/'output/setup/pdg-tracker-configuration.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report,indent=2));n.destroy_node();rclpy.shutdown()
raise SystemExit(0 if report['applied'] else 1)
