import numpy as np
import pytest
from scipy.spatial.transform import Rotation
from pdg.body_frame import correct_quaternion_xyzw


def test_body_correction_is_right_composed_and_recovers_gravity():
    marker_to_body=Rotation.from_euler('xyz',[-.3,-3.6,178],degrees=True)
    for angles in ([0,0,0],[0,-20,15],[20,0,80],[-10,15,-45]):
        body=Rotation.from_euler('xyz',angles,degrees=True)
        marker=body*marker_to_body
        corrected=Rotation.from_quat(correct_quaternion_xyzw(marker.as_quat(),marker_to_body.inv().as_quat()))
        assert (corrected.inv()*body).magnitude()<1e-12
        assert np.allclose(corrected.inv().apply([0,0,1]),body.inv().apply([0,0,1]))


@pytest.mark.parametrize('q',[[0,0,0,0],[float('nan'),0,0,1],[0,0,1],[0,0,0,2]])
def test_invalid_orientation_is_rejected(q):
    with pytest.raises(ValueError):correct_quaternion_xyzw(q,[0,0,0,1])


def test_enabled_correction_requires_independent_validation():
    from pdg.body_frame import validate_body_frame_config
    c=dict(enabled=True,robot='Crazyflie',frame_id='world',right_multiply_marker_orientation_xyzw=[0,0,0,1])
    with pytest.raises(ValueError):validate_body_frame_config(c)
    c['validation']=dict(passed=True,samples=30,corrected_max_error_deg=3.)
    assert np.allclose(validate_body_frame_config(c),[0,0,0,1])
    c['validation']['corrected_max_error_deg']=float('nan')
    with pytest.raises(ValueError):validate_body_frame_config(c)
