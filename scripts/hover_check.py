#!/usr/bin/env python3
"""Preview or explicitly execute a configurable high-level hover, hold, and landing."""
import argparse
import json
from pathlib import Path
from types import SimpleNamespace
from datetime import datetime, timezone
import numpy as np
import yaml

ROOT=Path(__file__).resolve().parents[1]
p=argparse.ArgumentParser(description=__doc__)
p.add_argument('--execute',action='store_true',help='Arm, take off, hold briefly and land on the real drone')
p.add_argument('--controller',choices=('mellinger','pid'),help='Expected tracker; default PID with --vicon-only, otherwise Mellinger')
p.add_argument('--vicon-only',action='store_true',help='Use native Tracker body axes, PID and Kalman; require Flow deck removed')
p.add_argument('--controller-trial',action='store_true',help='Explicit Vicon-only PID trial: omit host flight-envelope/endpoint and voltage-threshold aborts; retain tracking, startup and firmware checks')
p.add_argument('--height',type=float,default=.20,help='Height above measured pad, 0.10–1.00 m (default 0.20)')
p.add_argument('--hold',type=float,help='Hold after settling, 1–30 seconds (default: config value, otherwise 2)')
p.add_argument('--config',type=Path,default=ROOT/'config/hover-today.yaml',help='Hover configuration, including optional expected tracker calibration')
a=p.parse_args()
if not np.isfinite(a.height) or not .10<=a.height<=1.00:p.error('--height must be between 0.10 and 1.00 m')
if a.hold is not None and (not np.isfinite(a.hold) or not 1<=a.hold<=30):p.error('--hold must be between 1 and 30 seconds')
a.controller=a.controller or ('pid' if a.vicon_only else 'mellinger')
if a.controller_trial and not (a.vicon_only and a.controller=='pid'):p.error('--controller-trial requires Vicon-only PID')
config=yaml.safe_load(a.config.read_text())
c=config['execution']
if a.hold is not None:c['hover_hold_duration']=a.hold
c['baseline_pid']=a.controller=='pid'
c['vicon_only']=a.vicon_only
c['operator_controller_trial']=a.controller_trial
ground=np.array(config['calibration']['ground_position_m'],dtype=float)
target=ground+[0,0,a.height]
preview=dict(height_above_pad_m=a.height,controller=a.controller,vicon_only=a.vicon_only,robot=c['robot'],frame=c['frame_id'],ground_position_m=ground.tolist(),hover_position_m=target.tolist(),
             preserve_heading=c.get('preserve_heading',False),
             takeoff_duration_s=4.,settle_duration_s=c.get('hover_settle_duration',1.),hold_duration_s=c.get('hover_hold_duration',2.),landing_duration_s=c.get('hover_landing_duration',4.),
             max_lateral_deviation_m=c.get('hover_max_lateral_deviation',.08),
             max_height_above_pad_m=a.height+c.get('hover_max_vertical_overshoot',.08),
             max_tilt_deg=c.get('hover_max_tilt_deg',15.),
             max_attitude_change_deg=c.get('hover_max_attitude_change_deg',15.),
             endpoint_tolerance_m=c.get('hover_endpoint_tolerance',.05),
             mode='high-level takeoff/hover/land; no optimized PDG force trajectory',execute=a.execute)
preview['operator_controller_trial']=a.controller_trial
preview['config_file']=str(a.config.resolve())
preview['expected_tracker_parameters']=c.get('expected_tracker_parameters',{})
if a.controller_trial:
    for key in ('max_lateral_deviation_m','max_height_above_pad_m','max_tilt_deg','max_attitude_change_deg','endpoint_tolerance_m'):
        preview[key]=None
preview['minimum_launch_voltage']=None if a.controller_trial else c.get('min_launch_voltage',3.8)
preview['minimum_flight_voltage']=None if a.controller_trial else c.get('minimum_flight_voltage',3.7)
print(json.dumps(preview,indent=2),flush=True)
if not a.execute:raise SystemExit(0)
from pdg.body_frame import validate_body_frame_config
body_frame=yaml.safe_load((ROOT/'config/body-frame.yaml').read_text())
if a.vicon_only:
    tracker=yaml.safe_load((ROOT/'config/tracker-frame.yaml').read_text())
    if body_frame.get('enabled') or not tracker.get('operator_confirmed_nose_x_up_z'):
        raise SystemExit('Vicon-only mode requires native Tracker axes aligned to nose +X/up +Z and the old correction disabled')
else:
    validate_body_frame_config(body_frame)
report=json.loads((ROOT/'output/setup/ground-calibration.json').read_text())
if not report.get('passed'):raise SystemExit('Ground calibration is not accepted')
if np.linalg.norm(np.asarray(report['position_m'])-ground)>.01:raise SystemExit('Hover config no longer matches ground capture')
flash=json.loads((ROOT/'output/setup/flash-result.json').read_text())
if not flash.get('identity_verified'):raise SystemExit('PDG firmware flash identity has not been verified')
from pdg.ros_mission import run_mission
out=ROOT/'output'/('hover-'+datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ'))
out.mkdir(parents=True,exist_ok=False)
(out/'command.json').write_text(json.dumps(preview,indent=2)+'\n')
run_mission(SimpleNamespace(states=np.array([target])),None,c,out,hover=True)
print('Hover and pose-confirmed landing completed. Log:',out,flush=True)
