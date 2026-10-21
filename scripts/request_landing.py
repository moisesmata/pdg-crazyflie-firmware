#!/usr/bin/env python3
"""Request a high-level landing at the measured pad height; never arms."""
from pathlib import Path
import yaml
import rclpy
from crazyflie_interfaces.srv import Land

root=Path(__file__).resolve().parents[1]
c=yaml.safe_load((root/'config/hover-today.yaml').read_text())['execution']
rclpy.init()
node=rclpy.create_node('pdg_operator_land')
try:
    client=node.create_client(Land,f"/{c['robot']}/land")
    if not client.wait_for_service(timeout_sec=2.):
        raise RuntimeError('Landing service unavailable; no request sent')
    request=Land.Request()
    request.group_mask=0
    request.height=max(0.,float(c['ground_body_height'])-.01)
    request.duration.sec=3
    future=client.call_async(request)
    rclpy.spin_until_future_complete(node,future,timeout_sec=2.)
    if not future.done() or future.result() is None:
        raise RuntimeError('Landing request was not acknowledged')
    print('Landing request acknowledged; touchdown has not been independently confirmed.')
finally:
    node.destroy_node();rclpy.shutdown()
