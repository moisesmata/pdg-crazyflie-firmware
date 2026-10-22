#!/usr/bin/env python3
"""Prepare a hover at the measured rocket start; never send flight commands."""
import json
from pathlib import Path
import numpy as np
import yaml

root = Path(__file__).resolve().parents[1]
mission = yaml.safe_load((root/'config/rocket-lab.yaml').read_text())
report = json.loads((root/'output/setup/ground-calibration.json').read_text())
reference = json.loads((root/'output/rocket-setup-review/flight-reference.json').read_text())
# Read the validated mapped start, rather than duplicating the rocket coordinate map.
start = np.asarray(reference['position_m'][0])
ground = np.asarray(report.get('position_m', []))
if (not report.get('passed') or ground.shape != (3,) or not np.isfinite(ground).all()
        or np.linalg.norm(ground[:2]-start[:2]) > mission['execution']['max_start_error']
        or abs(ground[2]-mission['execution']['ground_body_height']) > .01):
    raise SystemExit('Require accepted stationary ground capture below the rocket start, within 8 cm XY and 1 cm of the existing pad-height calibration. Do not re-zero Vicon.')
config = yaml.safe_load((root/'config/pdg-mellinger-hover.yaml').read_text())
config['calibration']['ground_position_m'] = ground.tolist()
config['calibration']['purpose'] = 'Fresh rocket-start capture; existing tracker calibration retained'
config['execution']['ground_body_height'] = float(ground[2])
if 'roll_pitch_yaw_deg' in report:
    from scipy.spatial.transform import Rotation
    q = Rotation.from_euler('xyz', report['roll_pitch_yaw_deg'], degrees=True).as_quat()
    config['execution']['start_quaternion_wxyz'] = q[[3, 0, 1, 2]].tolist()
config['execution']['expected_tracker_parameters'] = mission['execution']['expected_tracker_parameters']
path = root/'output/setup/rocket-hover.yaml'
path.write_text(yaml.safe_dump(config, sort_keys=False))
print('Prepared hover configuration:', path)
