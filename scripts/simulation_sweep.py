"""Small repeatable numerical smoke study, not a flight-readiness campaign."""
from dataclasses import replace
from pathlib import Path
import json
import numpy as np
from pdg.io import load_problem,save_plan
from pdg.model import rest_state
from pdg.planners import SuccessiveConvexification

root=Path(__file__).resolve().parents[1]
m,p,c=load_problem(root/'config/nominal.yaml')
output=root/'output/smoke-study';output.mkdir(parents=True,exist_ok=True)
rows=[]
for name,height,xy in [('vertical',.5,[0.,0.]),('east',1.,[.2,0.]),('west',1.,[-.2,0.]),('diagonal',1.5,[.25,.15])]:
    problem=replace(p,initial_state=rest_state([0,0,height+.05]),target_position=np.r_[xy,.05],ceiling=max(2.,height+.3))
    plan=SuccessiveConvexification(m).solve(problem)
    save_plan(output/f'{name}.json',plan,m,problem)
    row=dict(name=name,converged=plan.converged,duration=float(plan.times[-1]),
             iterations=len(plan.diagnostics['history']),solve_seconds=plan.diagnostics['solve_seconds'],
             validation=plan.diagnostics['validation'])
    rows.append(row);print(name,plan.converged,row['duration'],flush=True)
(output/'summary.json').write_text(json.dumps(rows,indent=2)+'\n')
if not all(r['converged'] for r in rows):raise SystemExit('Some cases failed; inspect retained logs')
