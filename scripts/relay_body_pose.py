#!/usr/bin/env python3
"""Correct marker orientation to airframe orientation; never command the drone."""
import argparse
import math
from copy import deepcopy
from pathlib import Path
import yaml
import rclpy
from rclpy.duration import Duration
from rclpy.qos import QoSProfile,HistoryPolicy,ReliabilityPolicy,DurabilityPolicy,qos_profile_sensor_data
from motion_capture_tracking_interfaces.msg import NamedPoseArray
from pdg.body_frame import correct_quaternion_xyzw,validate_body_frame_config

ROOT=Path(__file__).resolve().parents[1]
p=argparse.ArgumentParser(description=__doc__)
p.add_argument('--config',type=Path,default=ROOT/'config/body-frame.yaml')
p.add_argument('--input-topic',default='/poses_raw')
p.add_argument('--output-topic',default='/poses')
p.add_argument('--deadline-hz',type=float,default=100.)
a=p.parse_args()
config=yaml.safe_load(a.config.read_text())
correction=validate_body_frame_config(config)
if not 0<a.deadline_hz<=1000:raise SystemExit('Invalid pose deadline rate')
rclpy.init();node=rclpy.create_node('pdg_body_pose_relay')
qos=QoSProfile(history=HistoryPolicy.KEEP_LAST,depth=1,reliability=ReliabilityPolicy.BEST_EFFORT,
               durability=DurabilityPolicy.VOLATILE,deadline=Duration(seconds=1/a.deadline_hz))
pub=node.create_publisher(NamedPoseArray,a.output_topic,qos)
last_stamp=None
def callback(msg):
    global last_stamp
    if node.count_publishers(a.input_topic)!=1:return
    if msg.header.frame_id.lstrip('/')!=config['frame_id'].lstrip('/'):
        node.get_logger().error('Rejecting pose in unexpected world frame');return
    stamp=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
    age=node.get_clock().now().nanoseconds*1e-9-stamp
    if not -.05<=age<=.15 or (last_stamp is not None and stamp<=last_stamp):return
    output=deepcopy(msg)
    try:
        for item in output.poses:
            if item.name!=config['robot']:continue
            position=item.pose.position
            if not all(math.isfinite(v) for v in (position.x,position.y,position.z)):
                raise ValueError('Rejecting nonfinite tracked position')
            q=item.pose.orientation
            values=correct_quaternion_xyzw([q.x,q.y,q.z,q.w],correction)
            q.x,q.y,q.z,q.w=map(float,values)
    except ValueError as exc:
        node.get_logger().error(str(exc));return
    pub.publish(output)
    last_stamp=stamp
sub=node.create_subscription(NamedPoseArray,a.input_topic,callback,qos_profile_sensor_data)
node.get_logger().info('Validated marker-to-airframe orientation relay active; world positions/timestamps unchanged')
try:rclpy.spin(node)
except KeyboardInterrupt:pass
finally:node.destroy_node();rclpy.try_shutdown()
