import json
import numpy as np
import pytest
from pdg.model import Quadrotor,rest_state
from pdg.contracts import LandingProblem,Plan
from pdg.io import save_plan,load_validated_plan


def saved(tmp_path):
    v=Quadrotor();x=rest_state([0,0,.5])
    p=LandingProblem(initial_state=x,target_position=x[:3],terminal_velocity=(0,0,0),
                     min_time=1,max_time=3,initial_time=2,nodes=5)
    controls=np.zeros((5,4));controls[:,0]=v.gravity
    plan=Plan(np.linspace(0,2,5),np.repeat(x[None,:],5,axis=0),controls,True)
    path=tmp_path/'plan.json';save_plan(path,plan,v,p)
    return path,v,p


def test_saved_plan_is_independently_revalidated(tmp_path):
    path,v,p=saved(tmp_path);plan=load_validated_plan(path,v,p)
    assert plan.converged and plan.diagnostics['validation']['passed']


@pytest.mark.parametrize('damage',['state','controls','clock','units','converged','model'])
def test_saved_plan_rejects_stale_or_inconsistent_data(tmp_path,damage):
    path,v,p=saved(tmp_path);d=json.loads(path.read_text())
    if damage=='state':d['states'][2][2]+=.1
    if damage=='controls':d['controls'][2][0]+=1
    if damage=='clock':d['times'][2]=.5
    if damage=='units':d['quaternion_order']='xyzw'
    if damage=='converged':d['converged']=False
    if damage=='model':d['vehicle']['mass']=.044
    path.write_text(json.dumps(d))
    with pytest.raises(ValueError):load_validated_plan(path,v,p)
