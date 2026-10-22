"""Offline two-stage rocket retargeting with a continuous full-state handoff."""
import argparse
from dataclasses import asdict, replace
from pathlib import Path
import json
import numpy as np
from scipy.optimize import brentq
import yaml
from .io import load_config,load_result,save_result,plain
from .paper_model import MASS,POS,VEL,QUAT,OMEGA
from .paper_scvx import solve,SCvxResult
from .validation import validate_result
from .emulation import RocketEmulation,EmulatedFlightPlan
from .cli import save_flight_reference,plot_mapping,plot_target

ROOT=Path(__file__).resolve().parents[3]
PREPARED=ROOT/'output/rocket-retarget-prepared'
ARCHIVED=ROOT/'experiments/rocket-retarget-20261022T022742Z'

def prepared_directory():
    return PREPARED if (PREPARED/'retarget-plan.json').exists() else ARCHIVED


class RetargetEmulation(RocketEmulation):
    def __init__(self,first,second,switch_time):
        self.first,self.second,self.switch_time=first,second,float(switch_time)
        self.config=dict(first.config,retarget_landing_position_enu_m=second.offset.tolist(),switch_time_s=self.switch_time,
                         switch_height_above_pad_m=1.5)
        self.drone=first.drone;self.limits=first.limits;self.offset=second.offset
        self.duration=self.switch_time+second.duration;self.require_settled_start=True
        a=first.core_at(self.switch_time);b=second.core_at(0.)
        self.join_errors={name:float(np.linalg.norm(a[i]-b[i])) for name,i in [('position_m',0),('velocity_m_s',1),('acceleration_m_s2',2),('rotation_matrix',4)]}
        x,u=first.replay.at(self.switch_time/first.time_scale);y,v=second.replay.at(0.)
        delta=x-y;delta[POS]=0. # The only coordinate change is translation of the target origin.
        self.join_errors['virtual_mass_attitude_velocity_rates']=float(np.linalg.norm(delta))
        self.join_errors['engine_thrust']=float(np.linalg.norm(u-v))
        if max(self.join_errors.values())>1e-4:raise ValueError('Discontinuous rocket retarget handoff: '+str(self.join_errors))
        # Include tightly spaced points around the join to expose acceleration/rate discontinuities.
        self.grid=np.unique(np.r_[first.grid[first.grid<self.switch_time],self.switch_time-1e-4,self.switch_time,
                                  self.switch_time+1e-4,self.switch_time+second.grid])
        self._core_cache=[self.core_at(t) for t in self.grid]
        rotations=np.array([a[4] for a in self._core_cache]);derivative=np.gradient(rotations,self.grid,axis=0,edge_order=2)
        skew=np.einsum('nji,njk->nik',rotations,derivative)
        self.rates=.5*np.column_stack((skew[:,2,1]-skew[:,1,2],skew[:,0,2]-skew[:,2,0],skew[:,1,0]-skew[:,0,1]))
        self.angular_acceleration=np.gradient(self.rates,self.grid,axis=0,edge_order=2)
        self.validation=self._validate();self.validation.update(join_errors=self.join_errors,switch_time_s=self.switch_time,
             switch_reference_height_above_pad_m=1.5,landing_position_enu_m=second.offset.tolist(),
             phase='first rocket descent prefix, then full-state-continuous retargeted rocket descent')
        if not self.validation['passed']:raise ValueError('Combined drone reference rejected: '+str(self.validation))
        self.plan=EmulatedFlightPlan(self.grid,np.array([self._state_at(t) for t in self.grid]),True,dict(kind='two_stage_rocket_retarget',validation=self.validation))

    def core_at(self,time):
        return self.first.core_at(time) if time<self.switch_time else self.second.core_at(time-self.switch_time)

    def diagnostic_sample(self,time,observation):
        second=time>=self.switch_time
        result=(self.second if second else self.first).diagnostic_sample(time-self.switch_time if second else time,observation)
        result.update(rocket_phase=2 if second else 1,switch_time_s=self.switch_time)
        return result


def prepare(directory=PREPARED):
    directory.mkdir(parents=True,exist_ok=True)
    p,cfg,c=load_config(ROOT/'config/rocket-2p75m.yaml')
    target_path=ROOT/'output/rocket-2p75m-prepared/target-plan.json'
    if not target_path.exists():target_path=ROOT/'experiments/rocket-2p75m-20261022T015434Z/target-plan.json'
    result=load_result(target_path,p,cfg,c['rocket']['units'])
    first=RocketEmulation(result,c['emulation'])
    switch=brentq(lambda t:first.core_at(t)[0][2]-first.offset[2]-1.5,0,first.duration,xtol=1e-11)
    target_time=switch/first.time_scale;x,u=first.replay.at(target_time)
    k=min(np.searchsorted(first.replay.times,target_time,side='right')-1,cfg.nodes-2)
    thrust_rate=(result.controls[:,k+1]-result.controls[:,k])/(result.burn_time/(cfg.nodes-1))
    offset=np.array([0.,-1.,float(c['execution']['ground_body_height'])])
    initial_position=first.rotation.T@(first.core_at(switch)[0]-offset)/first.length_scale
    p2=replace(p,ignition_mass=float(x[MASS]))
    cfg2=replace(cfg,initial_position=initial_position,initial_velocity=x[VEL].copy(),initial_quaternion=x[QUAT].copy(),
                 initial_body_rate=x[OMEGA].copy(),initial_thrust=u.copy(),initial_thrust_rate=thrust_rate,
                 min_burn_time=1.,initial_burn_time=4.,max_burn_time=8.,max_iterations=70,
                 nodes=25,path_margin_fraction=.30)
    c2=json.loads(json.dumps(plain(c)));c2['rocket']['vehicle']=plain(asdict(p2));c2['rocket']['solver']=plain(asdict(cfg2))
    c2['emulation']['landing_position_enu_m']=offset.tolist()
    (directory/'stage1-config.yaml').write_text(yaml.safe_dump(plain(c),sort_keys=False))
    (directory/'stage2-config.yaml').write_text(yaml.safe_dump(c2,sort_keys=False))
    save_result(directory/'stage1-target-plan.json',result,c['rocket']['units'],validate_result(result))
    print(json.dumps({'switch_time_s':switch,'switch_position_enu_m':first.core_at(switch)[0].tolist(),
                      'new_landing_position_enu_m':offset.tolist(),'continuation_mass':float(x[MASS])}),flush=True)
    guess=None
    guess_path=directory/'diagnostic-target.json'
    if guess_path.exists():
        d=json.loads(guess_path.read_text())
        guess=SCvxResult(np.asarray(d['states']),np.asarray(d['controls']),float(d['burn_time']),0.,[],False,p2,cfg2)
    r2=solve(p2,cfg2,verbose=True,initial_guess=guess);report=validate_result(r2)
    save_result(directory/'stage2-target-plan.json',r2,c['rocket']['units'],report)
    (directory/'stage2-target-validation.json').write_text(json.dumps(report,indent=2)+'\n')
    if not report['passed']:raise ValueError('Second rocket solve failed validation: '+str(report))
    second=RocketEmulation(r2,c2['emulation'],require_settled_start=False)
    if not second.validation['passed']:raise ValueError('Second mapped trajectory rejected: '+str(second.validation))
    combined=RetargetEmulation(first,second,switch)
    manifest=dict(schema_version=1,kind='two_stage_rocket_retarget',switch_time_s=switch,switch_height_above_pad_m=1.5,
                  stage1='stage1-target-plan.json',stage2='stage2-target-plan.json',
                  switch_trigger='precomputed reference crossing of 1.5 m above pad; not measured-state online replanning')
    (directory/'retarget-plan.json').write_text(json.dumps(manifest,indent=2)+'\n')
    write_outputs(directory,combined,c)
    plot_target(directory/'stage2-rocket.png',r2)
    return combined,c


def load_prepared(directory=None):
    directory=prepared_directory() if directory is None else Path(directory)
    manifest=json.loads((directory/'retarget-plan.json').read_text())
    if manifest.get('kind')!='two_stage_rocket_retarget':raise ValueError('Invalid retarget manifest')
    p,cfg,c=load_config(directory/'stage1-config.yaml');p2,cfg2,c2=load_config(directory/'stage2-config.yaml')
    r=load_result(directory/'stage1-target-plan.json',p,cfg,c['rocket']['units'])
    r2=load_result(directory/'stage2-target-plan.json',p2,cfg2,c2['rocket']['units'])
    first=RocketEmulation(r,c['emulation']);second=RocketEmulation(r2,c2['emulation'],require_settled_start=False)
    if not second.validation['passed']:raise ValueError('Second stage fails mapped feasibility')
    combined=RetargetEmulation(first,second,manifest['switch_time_s'])
    if abs(first.core_at(combined.switch_time)[0][2]-first.offset[2]-1.5)>1e-5:raise ValueError('Wrong switch height')
    return combined,c


def write_outputs(directory,combined,c):
    (directory/'emulation-validation.json').write_text(json.dumps(plain(combined.validation),indent=2)+'\n')
    save_flight_reference(directory/'flight-reference.json',combined,c['execution']['rate_hz'])
    f=json.loads((directory/'flight-reference.json').read_text());f['target_reference']='retarget-plan.json'
    (directory/'flight-reference.json').write_text(json.dumps(f,indent=2)+'\n')
    (directory/'execution.json').write_text(json.dumps(c['execution'],indent=2)+'\n')
    plot_mapping(directory/'lab-reference.png',combined)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--prepare',action='store_true');parser.add_argument('--execute',action='store_true')
    args=parser.parse_args()
    if args.prepare and args.execute:parser.error('Prepare offline before requesting execution')
    combined,c=prepare() if args.prepare else load_prepared()
    print(json.dumps({'validated':True,'duration_s':combined.duration,'switch_time_s':combined.switch_time,
                      'landing_position_enu_m':combined.offset.tolist(),'execution_requested':args.execute}),flush=True)
    if args.execute:
        from datetime import datetime,timezone
        from ..ros_mission import run_mission
        out=ROOT/'output'/('rocket-retarget-flight-'+datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ'))
        out.mkdir(exist_ok=False)
        import shutil
        for file in prepared_directory().iterdir():
            if file.is_file():shutil.copyfile(file,out/file.name)
        run_mission(combined.plan,combined.drone,c['execution'],out,trajectory=combined)
        print('Retarget mission and touchdown confirmation completed:',out,flush=True)


if __name__=='__main__':main()
