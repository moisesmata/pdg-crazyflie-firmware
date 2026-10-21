"""Crazyswarm2 ROS 2 transport. Imported only by an explicit --execute command."""
import json
import time
import numpy as np
from .execution import Mission, Observation


def fill_full_state(msg, setpoint, stamp, frame):
    msg.header.stamp, msg.header.frame_id = stamp, frame
    for name, value in zip(('x','y','z'), setpoint.position): setattr(msg.pose.position, name, float(value))
    for name, value in zip(('x','y','z'), setpoint.velocity): setattr(msg.twist.linear, name, float(value))
    for name, value in zip(('x','y','z'), setpoint.acceleration): setattr(msg.acc, name, float(value))
    for name, value in zip(('w','x','y','z'), setpoint.quaternion_wxyz): setattr(msg.pose.orientation, name, float(value))
    for name, value in zip(('x','y','z'), setpoint.body_rates): setattr(msg.twist.angular, name, float(value))
    return msg


def run_mission(plan, vehicle, config, directory):
    try:
        import rclpy
        from rclpy.qos import qos_profile_sensor_data
        from crazyflie_interfaces.msg import FullState
        from crazyflie_interfaces.srv import Arm, Takeoff, Land, NotifySetpointsStop
        from motion_capture_tracking_interfaces.msg import NamedPoseArray
        from rcl_interfaces.srv import GetParameters
    except ImportError as exc:
        raise RuntimeError('Source a ROS 2 / Crazyswarm2 workspace before --execute. See docs/hardware.md') from exc

    class Transport:
        def __init__(self):
            self.node = rclpy.create_node('pdg_mission')
            self.last = None
            self.last_ros_stamp = None
            self.fault = None
            self.stream = self.node.create_publisher(FullState, f"/{config['robot']}/cmd_full_state", 1)
            self.sub = self.node.create_subscription(NamedPoseArray, '/poses', self.on_pose, qos_profile_sensor_data)
            self.clients = {key:self.node.create_client(typ,f"/{config['robot']}/{key}")
                            for key,typ in [('arm',Arm),('takeoff',Takeoff),('land',Land),('notify_setpoints_stop',NotifySetpointsStop)]}
            self.param = self.node.create_client(GetParameters,'/crazyflie_server/get_parameters')
            self.log = open(directory/'mission.jsonl','w',buffering=1)

        def now(self): return time.monotonic()
        def wait(self, duration):
            end = self.now()+duration
            while rclpy.ok() and self.now() < end:
                rclpy.spin_once(self.node, timeout_sec=min(0.01,end-self.now()))
            if not rclpy.ok(): raise RuntimeError('ROS shutdown')
        def on_pose(self, msg):
            if msg.header.frame_id.lstrip('/') != config['frame_id'].lstrip('/'):
                self.fault = f'Pose frame {msg.header.frame_id!r} differs from expected frame'
                return
            stamp = msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
            rosnow = self.node.get_clock().now().nanoseconds*1e-9
            if not -0.05 <= rosnow-stamp <= config['max_pose_age']:
                self.fault = 'Pose source timestamp is stale or clocks are unsynchronized'
                return
            for item in msg.poses:
                if item.name != config['robot']: continue
                if self.last_ros_stamp is not None and stamp <= self.last_ros_stamp:
                    return  # duplicate source frames must not refresh the watchdog
                pose = item.pose
                position = np.array([pose.position.x,pose.position.y,pose.position.z])
                q = np.array([pose.orientation.w,pose.orientation.x,pose.orientation.y,pose.orientation.z])
                if not np.isfinite(position).all() or not np.isfinite(q).all() or abs(np.linalg.norm(q)-1) > .02:
                    self.fault = 'Invalid tracked rigid body pose'; return
                velocity = np.zeros(3) if self.last is None else (position-self.last.position)/(stamp-self.last_ros_stamp)
                self.last = Observation(position,velocity,q,stamp,self.now())
                self.last_ros_stamp = stamp
        def observe(self):
            if self.fault: raise RuntimeError(self.fault)
            if self.last is None: raise RuntimeError('No tracked rigid body pose')
            return self.last
        def call(self, client, request):
            if not client.wait_for_service(timeout_sec=3.): raise RuntimeError('Crazyswarm2 service unavailable')
            future = client.call_async(request)
            end = self.now()+3.
            while not future.done() and self.now() < end: self.wait(.01)
            if not future.done(): raise RuntimeError('Crazyswarm2 service timed out')
            result = future.result()
            if result is None: raise RuntimeError('Crazyswarm2 service failed')
            return result
        def preflight(self):
            deadline = self.now()+3.
            while self.last is None and self.now() < deadline: self.wait(.02)
            self.observe()
            req = GetParameters.Request()
            prefix = config['robot']+'.params.'
            req.names = [prefix+n for n in ['pdg.apiVersion','stabilizer.controller','pdg.controllerId','pdg.capEnable']]
            values = self.call(self.param,req).values
            if len(values) != 4 or any(v.type != 2 for v in values):
                raise RuntimeError('Required firmware parameters are missing or mistyped')
            api, ctrl, oot, cap = [v.integer_value for v in values]
            if api != 1 or ctrl not in (0,oot) or cap != 0:
                raise RuntimeError('Expected PDG firmware, OOT controller, and disabled capability injection')
        def arm(self, value):
            req=Arm.Request();req.arm=value;self.call(self.clients['arm'],req)
        def _duration(self, req, seconds):
            req.duration.sec=int(seconds);req.duration.nanosec=int((seconds-int(seconds))*1e9)
        def takeoff(self,height,duration):
            req=Takeoff.Request();req.height=height;self._duration(req,duration)
            self.call(self.clients['takeoff'],req)
        def land(self,height,duration):
            req=Land.Request();req.height=height;self._duration(req,duration)
            self.call(self.clients['land'],req)
        def release_stream(self):
            req=NotifySetpointsStop.Request();req.remain_valid_millisecs=0
            self.call(self.clients['notify_setpoints_stop'],req)
            self.wait(.10)
        def send(self,setpoint):
            self.stream.publish(fill_full_state(FullState(),setpoint,self.node.get_clock().now().to_msg(),config['frame_id']))
        def record(self,event):
            self.log.write(json.dumps(dict(wall_monotonic=self.now(),**event))+'\n')

    rclpy.init()
    transport = None
    try:
        transport=Transport()
        Mission(transport, config).run(plan, vehicle)
    finally:
        if transport:
            transport.log.close();transport.node.destroy_node()
        rclpy.shutdown()
