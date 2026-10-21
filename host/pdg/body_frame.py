"""Fixed marker-to-airframe orientation correction; world positions are unchanged."""
import numpy as np


def validate_body_frame_config(config):
    validation=config.get('validation',{})
    error=validation.get('corrected_max_error_deg',float('inf'))
    if (not config.get('enabled') or not config.get('robot') or not config.get('frame_id') or
        not validation.get('passed') or validation.get('samples',0)<20 or
        not np.isfinite(error) or not 0<=error<5):
        raise ValueError('Body frame correction needs an enabled, independent validation below 5 degrees')
    return correct_quaternion_xyzw([0,0,0,1],config['right_multiply_marker_orientation_xyzw'])


def correct_quaternion_xyzw(marker_xyzw, right_multiplier_xyzw):
    """Return q_world_body = q_world_marker * q_marker_body (Hamilton product)."""
    q=np.asarray(marker_xyzw,dtype=float);c=np.asarray(right_multiplier_xyzw,dtype=float)
    for value in (q,c):
        if value.shape!=(4,) or not np.isfinite(value).all() or abs(np.linalg.norm(value)-1)>.02:
            raise ValueError('Body correction requires finite unit quaternions in xyzw order')
    q=q/np.linalg.norm(q);c=c/np.linalg.norm(c)
    xyz=q[3]*c[:3]+c[3]*q[:3]+np.cross(q[:3],c[:3])
    out=np.r_[xyz,q[3]*c[3]-np.dot(q[:3],c[:3])]
    return out/np.linalg.norm(out)
