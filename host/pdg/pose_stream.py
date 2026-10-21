"""Track fresh external poses without letting rejected packets refresh the watchdog."""
import numpy as np
from .execution import Observation


class PoseStream:
    def __init__(self, robot, frame, maximum_age, record=lambda event:None):
        self.robot,self.frame,self.maximum_age=robot,frame.lstrip('/'),maximum_age
        self.record=record
        self.last=None
        self.last_stamp=None
        self.fault=None

    def ingest(self, msg, ros_now, monotonic_now):
        # Capture the actual host input throughout takeoff, staging and descent,
        # including rejected packets. Firmware receive logs alone cannot identify
        # whether an orientation discontinuity originated upstream of the radio.
        stamp=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
        for item in msg.poses:
            if item.name==self.robot:
                p,q=item.pose.position,item.pose.orientation
                self.record({'event':'external_pose_input','frame_id':msg.header.frame_id,
                             'source_timestamp_s':stamp,'source_age_s':ros_now-stamp,
                             'position':[p.x,p.y,p.z],
                             'quaternion_wxyz':[q.w,q.x,q.y,q.z]})
        if msg.header.frame_id.lstrip('/')!=self.frame:
            self.fault=f'Pose frame {msg.header.frame_id!r} differs from expected frame'
            return False
        age=ros_now-stamp
        if not np.isfinite(age) or not -.05<=age<=self.maximum_age:
            self.record({'event':'pose_rejected','reason':'source timestamp outside freshness window',
                         'source_age_s':age,'source_timestamp_s':stamp})
            return False
        if self.last_stamp is not None and stamp<=self.last_stamp:return False
        for item in msg.poses:
            if item.name!=self.robot:continue
            p,q=item.pose.position,item.pose.orientation
            position=np.array([p.x,p.y,p.z]);quat=np.array([q.w,q.x,q.y,q.z])
            if not np.isfinite(position).all() or not np.isfinite(quat).all() or abs(np.linalg.norm(quat)-1)>.02:
                self.fault='Invalid tracked rigid body pose'
                return False
            velocity=np.zeros(3) if self.last is None else (position-self.last.position)/(stamp-self.last_stamp)
            # Include source age, so recently received but delayed frames do not
            # buy another complete watchdog interval.
            self.last=Observation(position,velocity,quat,stamp,monotonic_now-max(0.,age))
            self.last_stamp=stamp
            return True
        return False
