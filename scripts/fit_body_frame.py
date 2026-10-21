#!/usr/bin/env python3
"""Fit saved props-off gravity poses and independently validate a body correction."""
import argparse
import json
from pathlib import Path
import numpy as np
import yaml
from scipy.spatial.transform import Rotation

ROOT=Path(__file__).resolve().parents[1]
p=argparse.ArgumentParser(description=__doc__)
p.add_argument('--validate',help='Independent saved capture label, e.g. nose-down')
p.add_argument('--enable',action='store_true',help='Enable only after independent saved validation passes')
a=p.parse_args()
out=ROOT/'output/setup'
labels=['level','nose-up','right-side-up']
def capture(label):
 rows=json.loads((out/f'body-axes-{label}-samples.json').read_text())
 if len(rows)<20:raise ValueError(f'{label}: need at least 20 samples')
 acc=np.array([r['acceleration_g'] for r in rows]);q=np.array([r['quaternion_xyzw'] for r in rows])
 if not np.isfinite(acc).all() or not np.isfinite(q).all():raise ValueError('Nonfinite calibration')
 if np.max(np.abs(np.linalg.norm(q,axis=1)-1))>.02:raise ValueError('Non-unit marker orientation')
 if max(np.std(acc,axis=0))>.03:raise ValueError(f'{label}: accelerometer was not stationary')
 change=max(2*np.arccos(np.clip(abs(q@q[0]),0,1)))
 if change>np.deg2rad(2):raise ValueError(f'{label}: orientation was not stationary')
 norm=np.linalg.norm(acc,axis=1)
 if min(norm)<.9 or max(norm)>1.1:raise ValueError('Capture requires stationary gravity')
 marker_g=Rotation.from_quat(q).inv().apply([0,0,1])
 return rows,acc/norm[:,None],marker_g

def disagreement(C,acc,g):
 dot=np.sum(C.apply(g)*acc,axis=1)
 return np.rad2deg(np.arccos(np.clip(dot,-1,1)))

captures=[capture(label) for label in labels]
body=np.array([np.mean(x[1],axis=0) for x in captures]);marker=np.array([np.mean(x[2],axis=0) for x in captures])
body/=np.linalg.norm(body,axis=1)[:,None];marker/=np.linalg.norm(marker,axis=1)[:,None]
singular=np.linalg.svd(marker,compute_uv=False)
if singular[-1]<.05:raise ValueError('Poses do not span independent tilt directions')
C,rssd=Rotation.align_vectors(body,marker)
errors=[disagreement(C,x[1],x[2]) for x in captures]
if max(max(e) for e in errors)>=5:raise ValueError('Fit residual exceeds 5 degrees; repeat stable poses')
fit=dict(labels=labels,marker_to_body_xyzw=C.as_quat().tolist(),
 right_multiply_marker_orientation_xyzw=C.inv().as_quat().tolist(),
 marker_to_body_rpy_deg=C.as_euler('xyz',degrees=True).tolist(),rssd=float(rssd),
 pose_means_singular_values=singular.tolist(),max_fit_error_deg=float(max(max(e) for e in errors)),
 flight_commands_sent=False)
(out/'body-frame-fit.json').write_text(json.dumps(fit,indent=2)+'\n')
print(json.dumps(fit,indent=2))
if a.enable and not a.validate:p.error('--enable requires a new independent --validate capture')
if a.validate:
 if a.validate in labels:p.error('Validation pose must be independent of fit poses')
 rows,acc,g=capture(a.validate);errors=disagreement(C,acc,g)
 validation=dict(independent_pose=a.validate,samples=len(rows),corrected_median_error_deg=float(np.median(errors)),
  corrected_max_error_deg=float(max(errors)),passed=bool(max(errors)<5))
 (out/'body-frame-validation.json').write_text(json.dumps(validation,indent=2)+'\n')
 print(json.dumps(validation,indent=2))
 if not validation['passed']:raise SystemExit('Independent validation failed; correction remains disabled')
 if a.enable:
  config=dict(enabled=True,robot='Crazyflie',frame_id='world',
   right_multiply_marker_orientation_xyzw=fit['right_multiply_marker_orientation_xyzw'],validation=validation,
   source_fit='output/setup/body-frame-fit.json',position_correction_applied=False)
  (ROOT/'config/body-frame.yaml').write_text(yaml.safe_dump(config,sort_keys=False))
  print('Validated correction enabled for next server launch; no flight commands sent')
