from dataclasses import replace
from pathlib import Path
import numpy as np
import pytest
from pdg.io import load_problem
from pdg.planners import SuccessiveConvexification
from pdg.trajectory import Trajectory

ROOT=Path(__file__).resolve().parents[1]


@pytest.fixture(scope='module')
def solution():
    m,p,_=load_problem(ROOT/'config/nominal.yaml')
    plan=SuccessiveConvexification(m).solve(p)
    return m,p,plan


def test_nominal_scvx_converges_and_independent_rollout_passes(solution):
    _,_,plan=solution
    assert plan.converged,plan.diagnostics
    assert plan.diagnostics['validation']['passed']
    assert len(plan.diagnostics['history']) > 1
    assert plan.diagnostics['history'][-1]['virtual_control'] < 1e-6


def test_reference_uses_world_kinematic_acceleration(solution):
    m,p,plan=solution
    traj=Trajectory(plan,m)
    s=traj.sample(0)
    np.testing.assert_allclose(s.position,p.initial_state[:3],atol=1e-6)
    assert abs(np.linalg.norm(s.quaternion_wxyz)-1) < 1e-10
    # Force / mass includes gravity, the command's acceleration must not.
    np.testing.assert_allclose(s.acceleration,m.dynamics(plan.states[0],plan.controls[0])[3:6],atol=1e-6)


def test_invalid_plan_is_not_executable(solution):
    m,_,plan=solution
    with pytest.raises(ValueError): Trajectory(replace(plan,converged=False),m)


def test_input_guards():
    _,p,_=load_problem(ROOT/'config/nominal.yaml')
    for fields in [dict(nodes=2),dict(min_time=9),dict(max_speed=float('nan')),
                   dict(initial_state=np.zeros(13)),dict(target_position=np.array([0,0,-1]))]:
        with pytest.raises(ValueError): replace(p,**fields)
