from types import SimpleNamespace as N
import numpy as np
import pytest
from pdg.pose_stream import PoseStream


def msg(stamp,frame='world',x=0.,q=(1.,0.,0.,0.)):
    sec=int(stamp)
    return N(header=N(frame_id=frame,stamp=N(sec=sec,nanosec=round((stamp-sec)*1e9))),poses=[N(name='Crazyflie',pose=N(position=N(x=x,y=0.,z=.014),orientation=N(w=q[0],x=q[1],y=q[2],z=q[3])))])


def test_delayed_packet_does_not_latch_fault_or_refresh_watchdog():
    rejected=[];s=PoseStream('Crazyflie','world',.15,rejected.append)
    assert s.ingest(msg(100.),100.,10.)
    first=s.last
    assert not s.ingest(msg(99.5),100.02,10.02)
    assert s.last is first and s.fault is None
    assert rejected[-1]['source_age_s']>.5
    assert s.ingest(msg(100.03,x=.001),100.03,10.03)
    assert s.last.received_at==10.03


def test_duplicate_packet_cannot_extend_pose_lifetime():
    s=PoseStream('Crazyflie','world',.15)
    s.ingest(msg(100.),100.,10.)
    assert not s.ingest(msg(100.),100.1,10.1)
    assert s.last.received_at==10.


def test_source_age_counts_toward_watchdog_limit():
    s=PoseStream('Crazyflie','world',.15)
    s.ingest(msg(100.),100.14,10.)
    assert 10.-s.last.received_at==pytest.approx(.14)
    assert 10.02-s.last.received_at>.15


def test_tracking_starvation_does_not_appear_fresh():
    s=PoseStream('Crazyflie','world',.15)
    s.ingest(msg(100.),100.,10.)
    s.ingest(msg(100.01),100.3,10.3)
    assert 10.3-s.last.received_at>.15


def test_wrong_frame_and_invalid_quaternion_remain_fatal():
    s=PoseStream('Crazyflie','world',.15)
    assert not s.ingest(msg(100.,frame='other'),100.,10.)
    assert 'frame' in s.fault
    s=PoseStream('Crazyflie','world',.15)
    assert not s.ingest(msg(100.,q=(2.,0.,0.,0.)),100.,10.)
    assert 'Invalid' in s.fault


def test_original_quaternions_are_recorded_even_when_packet_is_rejected():
    events=[]
    s=PoseStream('Crazyflie','world',.15,events.append)
    assert s.ingest(msg(100.),100.,10.)
    assert not s.ingest(msg(99.5,q=(0.,1.,0.,0.)),100.02,10.02)
    inputs=[e for e in events if e['event']=='external_pose_input']
    assert [e['quaternion_wxyz'] for e in inputs]==[[1.,0.,0.,0.],[0.,1.,0.,0.]]
    assert inputs[-1]['source_timestamp_s']==99.5
    assert s.last.quaternion_wxyz.tolist()==[1.,0.,0.,0.]
