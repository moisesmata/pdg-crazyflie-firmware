"""Explicit target-model serialization; never confuse rocket and flight states."""
from dataclasses import asdict
from pathlib import Path
import json
import numpy as np
import yaml
from .paper_model import VehicleParameters
from .paper_scvx import SCvxConfig,SCvxResult,IterationRecord
from .validation import validate_inputs,validate_result


def plain(value):
    if isinstance(value,np.ndarray):return value.tolist()
    if isinstance(value,np.generic):return value.item()
    if isinstance(value,dict):return {k:plain(v) for k,v in value.items()}
    if isinstance(value,(list,tuple)):return [plain(v) for v in value]
    return value


def parameters(values):
    values=dict(values)
    for k in ['gravity','inertia','aero_coefficients','thrust_arm','center_of_pressure']:
        if k in values:values[k]=np.asarray(values[k],float)
    return VehicleParameters(**values)


def solver_config(values):
    values=dict(values)
    for k in ['initial_position','initial_velocity','initial_quaternion','initial_body_rate','initial_thrust','initial_thrust_rate']:
        if k in values and values[k] is not None:values[k]=np.asarray(values[k],float)
    return SCvxConfig(**values)


def load_config(path):
    c=yaml.safe_load(Path(path).read_text())
    if c.get('planner',{}).get('model')!='rocket_6dof':raise ValueError('Expected planner.model: rocket_6dof')
    target=c['rocket'];units=target.get('units')
    if units not in ['normalized','SI']:raise ValueError('Declare rocket.units as normalized or SI')
    if 'g0' not in target['vehicle']:raise ValueError('Declare rocket g0 in the selected target units')
    p=parameters(target['vehicle']);cfg=solver_config(target.get('solver',{}));validate_inputs(p,cfg)
    return p,cfg,c


def save_result(path,result,units,validation):
    d=dict(schema_version=2,kind='rocket_pdg_target',source='Szmuk, Reynolds & Acikmese 2020; doi:10.2514/1.G004549',
           frame='UEN',quaternion='scalar-first paper q_B_from_I; use dcm_body_from_inertial',
           units=units,state_layout=['mass','position_UEN(3)','velocity_UEN(3)','quaternion(4)','body_rates(3)'],
           control_layout='body-frame gimbaled thrust vector (3), in target force units',
           parameters=asdict(result.parameters),solver=asdict(result.config),states=result.states,
           controls=result.controls,burn_time=result.burn_time,coast_time=result.coast_time,
           history=[asdict(r) for r in result.history],converged=bool(result.converged),validation=validation)
    Path(path).write_text(json.dumps(plain(d),indent=2,allow_nan=False)+'\n')


def load_result(path,p,cfg,units):
    d=json.loads(Path(path).read_text())
    expected_p=plain(asdict(p));expected_cfg=plain(asdict(cfg))
    # Aero-disabled effective model is part of the saved trajectory provenance.
    if not cfg.use_aerodynamics:expected_p['rho']=0.
    if (d.get('schema_version')!=2 or d.get('kind')!='rocket_pdg_target' or d.get('frame')!='UEN' or
        d.get('units')!=units or d.get('parameters')!=expected_p or plain(asdict(solver_config(d.get('solver',{}))))!=expected_cfg or not d.get('converged')):
        raise ValueError('Saved rocket plan does not match requested target model, units, or solver configuration')
    effective=parameters(d['parameters'])
    r=SCvxResult(np.asarray(d['states'],float),np.asarray(d['controls'],float),float(d['burn_time']),
                 float(d['coast_time']),[IterationRecord(**v) for v in d['history']],True,effective,cfg)
    report=validate_result(r)
    if not report['passed']:raise ValueError('Saved rocket plan failed independent dynamics/constraint validation')
    return r
