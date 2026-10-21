"""Read-only ground checks shared by diagnostics and actual ROS preflight."""
import numpy as np


def validate_controller_parameters(api, controller, oot, cap, *, baseline_pid=False):
    """PID is allowed only for an explicitly selected diagnostic hover."""
    allowed = (1,) if baseline_pid else (0, 6)
    if api != 1 or oot != 6 or cap != 0 or controller not in allowed:
        mode = 'stock PID baseline' if baseline_pid else 'project Mellinger/OOT'
        raise RuntimeError(f'Expected verified PDG firmware, {mode}, and disabled capability injection')


def validate_vicon_only_inputs(estimator, motion_disabled, flow1, flow2, zranger2):
    if estimator!=2 or motion_disabled!=1 or any((flow1,flow2,zranger2)):
        raise RuntimeError('Vicon-only hover requires Kalman, disabled flow, and no initialized Flow/Z-ranger deck')


def validate_estimate_freshness(age, maximum_age=.25):
    if not np.isfinite(age) or not 0<=age<=maximum_age:
        raise RuntimeError('Onboard estimator telemetry is stale; requesting landing')


def validate_tracker_calibration(values, expected):
    for name,target in expected.items():
        value=values.get(name)
        if value is None or not np.isfinite([value,target]).all() or not np.isclose(value,target,rtol=1e-4,atol=1e-6):
            raise RuntimeError(f'Tracker calibration mismatch for {name}: configured {value}, expected {target}')


def validate_controller_trial_mode(config, *, hover):
    trial=bool(config.get('operator_controller_trial',False))
    if trial:
        valid=config.get('vicon_only') and (config.get('baseline_pid') if hover else
               not config.get('baseline_pid') and bool(config.get('expected_tracker_parameters')))
        if not valid:raise ValueError('Controller trial requires Vicon-only PID hover or calibrated project PDG tracker')
    return trial


def validate_flight_status(voltage, age, minimum_voltage=3.7):
    if minimum_voltage is not None and (not np.isfinite(minimum_voltage) or not 3.2<=minimum_voltage<=4.2):
        raise ValueError('Invalid airborne battery threshold')
    if not np.isfinite(age) or age<0 or age>2.5 or not np.isfinite(voltage) or (minimum_voltage is not None and voltage<minimum_voltage):
        raise RuntimeError(f'Battery/status health lost: voltage={voltage:.3f} V, status_age={age:.3f} s; requesting landing')


def wait_for_supervisor_arm(now, wait, get_status, armed, timeout=3.):
    """ROS service completion only queues arming; await firmware's actual state."""
    start=now()
    while now()-start<timeout:
        status=get_status()
        if status is not None:
            msg,received=status
            flags=msg.supervisor_info
            if received>=start and now()-received<=2.5:
                if armed and flags&(32|64):
                    raise RuntimeError('Supervisor fault while arming')
                ready=(flags&10)==10 if armed else not flags&2
                if ready:return
        wait(.02)
    raise RuntimeError('Supervisor did not confirm '+('armed and ready to fly' if armed else 'disarmed'))


def validate_ground_health(status, estimate_position, estimate_q, observation, *, minimum_voltage=3.8):
    if minimum_voltage is not None and (not np.isfinite(minimum_voltage) or not 3.2<=minimum_voltage<=4.2):
        raise ValueError('Invalid ground battery threshold')
    if status.pm_state!=0 or not np.isfinite(status.battery_voltage) or (minimum_voltage is not None and not minimum_voltage<=status.battery_voltage<=4.5):
        raise RuntimeError('Ground check requires battery operation and voltage at least 3.8 V')
    flags=status.supervisor_info
    if not flags & 1 or flags & (2|32|64):
        raise RuntimeError('Supervisor is not ready for an unarmed ground start')
    position=np.asarray(estimate_position,dtype=float)
    if position.shape!=(3,) or not np.isfinite(position).all():
        raise RuntimeError('Invalid onboard estimator position')
    if np.linalg.norm(position-observation.position)>.04:
        raise RuntimeError('Onboard estimator position differs from Vicon by more than 4 cm')
    q=np.asarray(estimate_q,dtype=float)
    if not np.isfinite(q).all() or abs(np.linalg.norm(q)-1)>.02:
        raise RuntimeError('Invalid onboard estimator quaternion')
    q=q/np.linalg.norm(q)
    if 2*np.arccos(np.clip(abs(np.dot(q,observation.quaternion_wxyz)),0,1))>np.deg2rad(5):
        raise RuntimeError('Onboard estimator and Vicon attitudes differ by more than 5 degrees')


def hover_ground_angles(quaternion_wxyz):
    """Separate vertical alignment from heading for a yaw-to-zero takeoff."""
    q=np.asarray(quaternion_wxyz,dtype=float)
    if q.shape!=(4,) or not np.isfinite(q).all() or abs(np.linalg.norm(q)-1)>.02:
        raise RuntimeError('Invalid hover ground quaternion')
    w,x,y,z=q/np.linalg.norm(q)
    tilt=float(np.rad2deg(np.arccos(np.clip(1-2*(x*x+y*y),-1,1))))
    heading=float(np.rad2deg(np.arctan2(2*(w*z+x*y),1-2*(y*y+z*z))))
    return tilt,heading


def validate_hover_alignment(quaternion_wxyz, *, preserve_heading=False):
    tilt,heading=hover_ground_angles(quaternion_wxyz)
    if tilt>5 or (not preserve_heading and abs(heading)>5):
        raise RuntimeError(f'Hover requires tilt and heading each within 5 degrees (tilt={tilt:.2f}, heading={heading:.2f})')


def validate_calibrated_start_attitude(quaternion_wxyz, reference_wxyz):
    q=np.asarray(quaternion_wxyz,dtype=float);reference=np.asarray(reference_wxyz,dtype=float)
    for v in (q,reference):
        if v.shape!=(4,) or not np.isfinite(v).all() or abs(np.linalg.norm(v)-1)>.02:
            raise RuntimeError('Invalid calibrated start attitude')
    angle=np.rad2deg(2*np.arccos(np.clip(abs(np.dot(q/np.linalg.norm(q),reference/np.linalg.norm(reference))),0,1)))
    if angle>10:
        raise RuntimeError(f'Start attitude differs from calibrated pad by {angle:.1f} degrees; check physical heading and marker identity')


def start_attitude_errors(quaternion_wxyz, reference_wxyz):
    """Return full attitude and body-Z tilt errors in degrees; yaw does not tilt body Z."""
    normalized=[]
    for value in (quaternion_wxyz, reference_wxyz):
        q=np.asarray(value,dtype=float)
        if q.shape!=(4,) or not np.isfinite(q).all() or abs(np.linalg.norm(q)-1)>.02:
            raise RuntimeError('Invalid descent-start quaternion')
        normalized.append(q/np.linalg.norm(q))
    q,reference=normalized
    angle=np.rad2deg(2*np.arccos(np.clip(abs(np.dot(q,reference)),0,1)))
    def body_z(q):
        w,x,y,z=q
        return np.array([2*(x*z+w*y),2*(y*z-w*x),1-2*(x*x+y*y)])
    tilt=np.rad2deg(np.arccos(np.clip(np.dot(body_z(q),body_z(reference)),-1,1)))
    return float(angle),float(tilt)
