#!/usr/bin/env python3
"""Measure radio status on the ground; never send parameters or flight commands."""
import argparse
import json
import time
from pathlib import Path
import numpy as np
import rclpy
from crazyflie_interfaces.msg import Status

p=argparse.ArgumentParser(description=__doc__)
p.add_argument('--seconds',type=float,default=10.)
a=p.parse_args()
if not 5<=a.seconds<=60:p.error('Use 5 to 60 seconds')
rclpy.init();n=rclpy.create_node('pdg_radio_link_audit');rows=[];start=time.monotonic()
def receive(m):
 stamp=m.header.stamp.sec+m.header.stamp.nanosec*1e-9
 rows.append(dict(t=time.monotonic()-start,flags=m.supervisor_info,voltage=m.battery_voltage,
  rssi=m.rssi,latency_ms=m.latency_unicast,rx_unicast=m.num_rx_unicast,tx_unicast=m.num_tx_unicast,
  rx_broadcast=m.num_rx_broadcast,tx_broadcast=m.num_tx_broadcast,
  ros_delivery_age_s=n.get_clock().now().nanoseconds*1e-9-stamp))
s=n.create_subscription(Status,'/Crazyflie/status',receive,10)
print('Waiting for ground radio status; no flight commands sent',flush=True)
report={'flight_commands_sent':False,'passed':False}
try:
 deadline=time.monotonic()+15
 while not rows and time.monotonic()<deadline:rclpy.spin_once(n,timeout_sec=.05)
 if not rows:raise RuntimeError('No radio status; restart server telemetry')
 end=time.monotonic()+a.seconds
 while time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.05)
 if len(rows)<5:raise RuntimeError('Insufficient radio status samples')
 if any(x['flags']&2 for x in rows):raise RuntimeError('This diagnostic requires an unarmed drone')
 ages=[x['latency_ms'] for x in rows]
 gaps=np.diff([x['t'] for x in rows])
 totals={key:sum(x[key] for x in rows) for key in ['rx_unicast','tx_unicast','rx_broadcast','tx_broadcast']}
 report.update(samples=len(rows),latency_ms=dict(min=min(ages),median=float(np.median(ages)),p95=float(np.percentile(ages,95)),max=max(ages)),
  max_status_gap_s=float(max(gaps)),radio_packet_totals=totals,
  max_ros_delivery_age_s=max(x['ros_delivery_age_s'] for x in rows),
  scope='Ground measurements; packet counters cover approximately matched intervals, not an exact loss measurement')
 report['passed']=max(gaps)<=2.5 and time.monotonic()-start-rows[-1]['t']<=2.5
except Exception as exc:report['error']=str(exc)
finally:
 out=Path(__file__).resolve().parents[1]/'output/setup'
 (out/'radio-link-audit.json').write_text(json.dumps(dict(report,raw_samples=rows),indent=2)+'\n')
 print(json.dumps(report,indent=2));n.destroy_node();rclpy.shutdown()
raise SystemExit(0 if report['passed'] else 1)
