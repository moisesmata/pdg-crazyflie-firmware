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


def fill_go_to_request(req, position, quaternion_wxyz):
    # Pinned C++ server and firmware use radians, despite the .srv's deg comment.
    w,x,y,z=quaternion_wxyz
    req.relative=False
    req.goal.x,req.goal.y,req.goal.z=map(float,position)
    req.yaw=float(np.arctan2(2*(w*z+x*y),1-2*(y*y+z*z)))
    return req


def run_mission(plan, vehicle, config, directory, *, hover=False, trajectory=None):
    from .ground_health import validate_controller_trial_mode
    controller_trial=validate_controller_trial_mode(config,hover=hover)
    try:
        import rclpy
        from rclpy.signals import SignalHandlerOptions
        from rclpy.qos import QoSProfile, ReliabilityPolicy
        from crazyflie_interfaces.msg import FullState, Status, LogDataGeneric
        from geometry_msgs.msg import PoseStamped
        from crazyflie_interfaces.srv import Arm, Takeoff, Land, GoTo, NotifySetpointsStop
        from motion_capture_tracking_interfaces.msg import NamedPoseArray
        from rcl_interfaces.srv import GetParameters
    except ImportError as exc:
        raise RuntimeError('Source a ROS 2 / Crazyswarm2 workspace before --execute. See docs/hardware.md') from exc

    class Transport:
        def __init__(self):
            self.node = rclpy.create_node('pdg_mission')
            from .pose_stream import PoseStream
            self.pose_stream=PoseStream(config['robot'],config['frame_id'],config['max_pose_age'],self.record)
            self.last = None
            self.last_ros_stamp = None
            self.fault = None
            self.status = None
            self.estimate = None
            self.status_sub = self.node.create_subscription(Status, f"/{config['robot']}/status", self.on_status, 10)
            self.estimate_sub = self.node.create_subscription(PoseStamped, f"/{config['robot']}/pose", self.on_estimate, 10)
            self.pdg_sub = self.node.create_subscription(LogDataGeneric, f"/{config['robot']}/pdg", self.on_pdg, 10)
            self.actuator_sub = self.node.create_subscription(LogDataGeneric, f"/{config['robot']}/actuators", self.on_actuators, 10)
            self.yaw_sub = self.node.create_subscription(LogDataGeneric, f"/{config['robot']}/yaw_control", self.on_yaw_control, 10)
            self.vertical_sub = self.node.create_subscription(LogDataGeneric, f"/{config['robot']}/vertical_control", self.on_vertical_control, 10)
            self.mellinger_sub = self.node.create_subscription(LogDataGeneric, f"/{config['robot']}/mellinger_control", self.on_mellinger_control, 10)
            self.mocap_rx_position_sub = self.node.create_subscription(LogDataGeneric, f"/{config['robot']}/mocap_rx_position", self.on_mocap_rx_position, 10)
            self.mocap_rx_orientation_sub = self.node.create_subscription(LogDataGeneric, f"/{config['robot']}/mocap_rx_orientation", self.on_mocap_rx_orientation, 10)
            self.stream = self.node.create_publisher(FullState, f"/{config['robot']}/cmd_full_state", 1)
            self.sub = self.node.create_subscription(NamedPoseArray, '/poses', self.on_pose, QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT))
            self.clients = {key:self.node.create_client(typ,f"/{config['robot']}/{key}")
                            for key,typ in [('arm',Arm),('takeoff',Takeoff),('land',Land),('go_to',GoTo),('notify_setpoints_stop',NotifySetpointsStop)]}
            self.param = self.node.create_client(GetParameters,'/crazyflie_server/get_parameters')
            self.log = open(directory/'mission.jsonl','w',buffering=1)

        def on_status(self, msg):
            self.status = (msg, self.now())
            self.record({'event':'status','battery_voltage':msg.battery_voltage,
                         'supervisor_info':msg.supervisor_info,'pm_state':msg.pm_state,
                         'num_rx_unicast':msg.num_rx_unicast,'num_tx_unicast':msg.num_tx_unicast,
                         'num_rx_broadcast':msg.num_rx_broadcast,'num_tx_broadcast':msg.num_tx_broadcast,
                         'latency_unicast_ms':msg.latency_unicast})
        def on_estimate(self, msg):
            self.estimate = (msg, self.now())
            p,q=msg.pose.position,msg.pose.orientation
            self.record({'event':'onboard_pose','position':[p.x,p.y,p.z],
                         'quaternion_wxyz':[q.w,q.x,q.y,q.z]})
        def on_pdg(self, msg):
            if len(msg.values)==4:
                self.record({'event':'controller_output','timestamp_ms':int(msg.timestamp),
                             'values':dict(zip(['updates','capActive','requested_pwm','commanded_pwm'],map(float,msg.values)))})
        def on_actuators(self, msg):
            if len(msg.values)==6:
                self.record({'event':'actuator_output','timestamp_ms':int(msg.timestamp),'values':dict(zip(
                    ['motor1','motor2','motor3','motor4','gyro_z_deg_s','onboard_yaw_deg'],map(float,msg.values)))})
        def on_mellinger_control(self, msg):
            if len(msg.values)==6:
                self.record({'event':'mellinger_control','timestamp_ms':int(msg.timestamp),
                             'values':dict(zip(['yaw_command_legacy','gyro_yaw_rad_s',
                                               'roll_command_legacy','pitch_command_legacy',
                                               'collective_command_legacy','integrated_z_error_m_s'],
                                              map(float,msg.values)))})
        def on_yaw_control(self, msg):
            if len(msg.values)==6:
                self.record({'event':'yaw_control','timestamp_ms':int(msg.timestamp),'values':dict(zip(
                    ['desired_yaw_deg','desired_yaw_rate_deg_s','attitude_yaw_p','rate_yaw_p','onboard_yaw_deg','gyro_z_deg_s'],map(float,msg.values)))})
        def on_vertical_control(self, msg):
            if len(msg.values)==6:
                self.record({'event':'vertical_control','timestamp_ms':int(msg.timestamp),'values':dict(zip(
                    ['collective_before_battery_compensation','target_z_m','target_vz_m_s','z_m','vz_m_s','battery_voltage'],map(float,msg.values)))})
        def on_mocap_rx_position(self, msg):
            if len(msg.values)==4:
                self.record({'event':'mocap_received_position','timestamp_ms':int(msg.timestamp),
                             'position_m':list(map(float,msg.values[:3])),
                             'last_received_tick_u16':int(msg.values[3])})
        def on_mocap_rx_orientation(self, msg):
            if len(msg.values)==4:
                q=list(map(float,msg.values))
                self.record({'event':'mocap_received_orientation','timestamp_ms':int(msg.timestamp),
                             'quaternion_wxyz':[q[3],q[0],q[1],q[2]]})
        def now(self): return time.monotonic()
        def wait(self, duration):
            end = self.now()+duration
            while rclpy.ok() and self.now() < end:
                rclpy.spin_once(self.node, timeout_sec=min(0.01,end-self.now()))
            if not rclpy.ok(): raise RuntimeError('ROS shutdown')
        def on_pose(self, msg):
            self.pose_stream.ingest(msg,self.node.get_clock().now().nanoseconds*1e-9,self.now())
            self.last=self.pose_stream.last
            self.last_ros_stamp=self.pose_stream.last_stamp
            self.fault=self.pose_stream.fault
        def observe(self):
            from .ground_health import validate_estimate_freshness
            validate_estimate_freshness(float('inf') if self.estimate is None else self.now()-self.estimate[1])
            if self.fault: raise RuntimeError(self.fault)
            if self.last is None: raise RuntimeError('No tracked rigid body pose')
            if self.status is not None:
                voltage=self.status[0].battery_voltage;age=self.now()-self.status[1]
                from .ground_health import validate_flight_status
                validate_flight_status(voltage,age,None if controller_trial else config.get('minimum_flight_voltage',3.7))
            return self.last
        def observe_recovery(self):
            # Battery-triggered landing still needs pose monitoring. Do not hide
            # invalid tracking, stale radio status, or the source watchdog.
            if self.fault:raise RuntimeError(self.fault)
            if self.last is None or self.now()-self.last.received_at>config['max_pose_age']:
                raise RuntimeError('Recovery pose is stale')
            if self.status is None or self.now()-self.status[1]>2.5:
                raise RuntimeError('Recovery supervisor status is stale')
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
            if config.get('stage_to_start',False) and not self.clients['go_to'].wait_for_service(timeout_sec=3.):
                raise RuntimeError('Staging GoTo service unavailable before arming')
            deadline = self.now()+5.
            while (self.last is None or self.status is None or self.estimate is None) and self.now() < deadline:
                self.wait(.02)
            observation = self.observe()
            if self.node.count_publishers('/poses') != 1:
                raise RuntimeError('Require exactly one external-pose publisher')
            if self.status is None or self.estimate is None:
                raise RuntimeError('Missing battery/supervisor or onboard pose telemetry')
            if self.now()-self.status[1]>2.5 or self.now()-self.estimate[1]>.15:
                raise RuntimeError('Stale battery/supervisor or onboard pose telemetry')
            estimate = self.estimate[0]
            if estimate.header.frame_id.lstrip('/') != config['frame_id'].lstrip('/'):
                raise RuntimeError('Onboard estimator frame differs from configured frame')
            p, q = estimate.pose.position, estimate.pose.orientation
            from .ground_health import validate_ground_health
            validate_ground_health(self.status[0], [p.x,p.y,p.z], [q.w,q.x,q.y,q.z], observation,
                                   minimum_voltage=None if controller_trial or config.get('min_launch_voltage',3.8) is None else 3.8)
            minimum=config.get('min_launch_voltage',3.8)
            if not controller_trial and minimum is not None and self.status[0].battery_voltage<minimum:
                raise RuntimeError(f'Launch requires a charged battery: at least {minimum:.2f} V, measured {self.status[0].battery_voltage:.3f} V')
            req = GetParameters.Request()
            prefix = config['robot']+'.params.'
            req.names = [prefix+n for n in ['pdg.apiVersion','stabilizer.controller','pdg.controllerId','pdg.capEnable']]
            values = self.call(self.param,req).values
            if len(values) != 4 or any(v.type != 2 for v in values):
                raise RuntimeError('Required firmware parameters are missing or mistyped')
            api, ctrl, oot, cap = [v.integer_value for v in values]
            from .ground_health import validate_controller_parameters
            validate_controller_parameters(api, ctrl, oot, cap, baseline_pid=hover and config.get('baseline_pid',False))
            self.record({'event':'controller_preflight','controller':ctrl,'baseline_pid':bool(hover and config.get('baseline_pid',False)),
                         'operator_controller_trial':controller_trial})
            expected_tracker=config.get('expected_tracker_parameters',{})
            if expected_tracker:
                req=GetParameters.Request();req.names=[prefix+name for name in expected_tracker]
                answer=self.call(self.param,req).values
                configured={name:(v.double_value if v.type==3 else v.integer_value)
                            for name,v in zip(expected_tracker,answer) if v.type in (2,3)}
                from .ground_health import validate_tracker_calibration
                validate_tracker_calibration(configured,expected_tracker)
                self.record({'event':'tracker_calibration_preflight','configured_values':configured,
                             'scope':'ROS configured values; not independent hardware readback'})
            if config.get('vicon_only',False):
                req=GetParameters.Request()
                sensor_names=['stabilizer.estimator','motion.disable','deck.bcFlow','deck.bcFlow2','deck.bcZRanger2']
                req.names=[prefix+name for name in sensor_names]
                sensor_values=self.call(self.param,req).values
                if len(sensor_values)!=5 or any(v.type!=2 for v in sensor_values):
                    raise RuntimeError('Missing Vicon-only sensor configuration')
                from .ground_health import validate_vicon_only_inputs
                validate_vicon_only_inputs(*[v.integer_value for v in sensor_values])
                self.record({'event':'vicon_only_preflight','values':dict(zip(sensor_names,[v.integer_value for v in sensor_values]))})
            req=GetParameters.Request();req.names=[prefix+'system.selftestPassed']
            result=self.call(self.param,req).values
            if len(result)!=1 or result[0].type!=2 or result[0].integer_value!=1:
                raise RuntimeError('Drone startup self-test did not pass; power-cycle and investigate before flight')
            if hover and config.get('preserve_heading',False):
                req=GetParameters.Request();req.names=['high_level.preserve_yaw']
                flag=self.call(self.param,req).values
                if len(flag)!=1 or flag[0].type!=1 or not flag[0].bool_value:
                    raise RuntimeError('Restart the rebuilt heading-preserving radio server before hover')
        def arm(self, value):
            req=Arm.Request();req.arm=value;self.call(self.clients['arm'],req)
            from .ground_health import wait_for_supervisor_arm
            try:
                wait_for_supervisor_arm(self.now,self.wait,lambda:self.status,value)
                self.record({'event':'supervisor_arm_confirmed','armed':value,'flags':self.status[0].supervisor_info})
            except Exception:
                # No takeoff has been requested by this call. Disarm on a fresh,
                # stationary pad pose; otherwise request a nominal landing.
                if value:
                    o=self.last
                    if (o is not None and self.now()-o.received_at<.15 and
                        abs(o.position[2]-config['ground_body_height'])<.02 and np.linalg.norm(o.velocity)<.05):
                        req=Arm.Request();req.arm=False;self.call(self.clients['arm'],req)
                    else:
                        self.land(max(0.,config['ground_body_height']-.01),4.)
                raise
        def _duration(self, req, seconds):
            req.duration.sec=int(seconds);req.duration.nanosec=int((seconds-int(seconds))*1e9)
        def takeoff(self,height,duration):
            req=Takeoff.Request();req.height=height;self._duration(req,duration)
            self.call(self.clients['takeoff'],req)
        def land(self,height,duration):
            req=Land.Request();req.height=height;self._duration(req,duration)
            self.call(self.clients['land'],req)
        def go_to(self,position,quaternion_wxyz,duration):
            req=fill_go_to_request(GoTo.Request(),position,quaternion_wxyz)
            self._duration(req,duration)
            self.call(self.clients['go_to'],req)
        def release_stream(self):
            req=NotifySetpointsStop.Request();req.remain_valid_millisecs=0
            self.call(self.clients['notify_setpoints_stop'],req)
            self.wait(.10)
        def send(self,setpoint):
            self.stream.publish(fill_full_state(FullState(),setpoint,self.node.get_clock().now().to_msg(),config['frame_id']))
        def record(self,event):
            self.log.write(json.dumps(dict(wall_monotonic=self.now(),**event))+'\n')

    # Keep ROS alive during KeyboardInterrupt so the mission can request landing.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    transport = None
    try:
        transport=Transport()
        if hover:
            from .hover import HoverMission
            HoverMission(transport, config).run(plan, vehicle)
        else:
            Mission(transport, config).run(plan, vehicle, trajectory=trajectory)
    finally:
        if transport:
            transport.log.close();transport.node.destroy_node()
        rclpy.shutdown()
