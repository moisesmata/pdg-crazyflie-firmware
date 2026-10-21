import numpy as np
from pdg.model import Quadrotor, rest_state


def test_hover_and_allocation():
    m=Quadrotor();x=rest_state([0,0,1]);u=np.array([m.gravity,0,0,0])
    np.testing.assert_allclose(m.dynamics(x,u),0,atol=1e-12)
    f=m.allocation@u
    np.testing.assert_allclose(f,np.repeat(m.mass*m.gravity/4,4))
    np.testing.assert_allclose(m.step(x,u,u,.1),x,atol=1e-12)


def test_foh_map_matches_independent_integrator():
    m=Quadrotor();x=rest_state([.1,0,1]);a=np.array([9.,.8,-.3,.2]);b=np.array([11.,-.2,.4,-.1])
    _, states=m.rollout(x,np.array([a,b]),.08)
    coarse=m.step(x,a,b,.08,substeps=4)
    fine=m.step(x,a,b,.08,substeps=8)
    assert np.max(np.abs(fine-states[-1])) < np.max(np.abs(coarse-states[-1]))/8
    np.testing.assert_allclose(fine,states[-1],atol=3e-9)


def test_free_time_jacobian_matches_finite_difference():
    m=Quadrotor();x=rest_state([0,0,1]);a=np.array([9.,.2,-.3,.1]);b=np.array([10.,-.2,.1,0.])
    val,A,B,C,S=m.linearize_step(x,a,b,2.,24)
    args=np.r_[x,a,b,2.]
    J=np.column_stack([A,B,C,S])
    for j in [0,3,6,7,10,13,14,18,21]:
        plus=args.copy();minus=args.copy();plus[j]+=1e-5;minus[j]-=1e-5
        def f(v):return m.step(v[:13],v[13:17],v[17:21],v[-1]/24)
        np.testing.assert_allclose(J[:,j],(f(plus)-f(minus))/2e-5,atol=2e-8)
