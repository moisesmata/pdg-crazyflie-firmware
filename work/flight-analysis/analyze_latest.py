from pathlib import Path
import json,math
import numpy as np
from scipy.spatial.transform import Rotation
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
ROOT=Path(__file__).resolve().parents[2]
run=ROOT/'output/pdg-full33-yawlogged-20261021T175535Z'
rows=[json.loads(s) for s in (run/'mission.jsonl').read_text().splitlines()]
t0=next(r['wall_monotonic'] for r in rows if r['event']=='takeoff')
def series(kind):return [r for r in rows if r['event']==kind and r['wall_monotonic']>=t0]
def times(a):return np.array([r['wall_monotonic']-t0 for r in a])
external=series('external_pose_input');onboard=series('onboard_pose');motors=series('actuator_output');ctrl=series('mellinger_control');status=series('status')
t=times(external);p=np.array([r['position'] for r in external]);q=np.array([r['quaternion_wxyz'] for r in external]);ypr=Rotation.from_quat(q[:,[1,2,3,0]]).as_euler('xyz',degrees=True)
tm=times(motors);m=np.array([[r['values'][f'motor{i}'] for i in range(1,5)] for r in motors]);tc=times(ctrl)
yawcmd=np.array([r['values']['yaw_command_legacy'] for r in ctrl]);collective=np.array([r['values']['collective_command_legacy'] for r in ctrl]);iz=np.array([r['values']['integrated_z_error_m_s'] for r in ctrl]);ts=times(status);vb=np.array([r['battery_voltage'] for r in status])
angles=np.rad2deg(2*np.arccos(np.clip(np.abs(np.sum(q[1:]*q[:-1],axis=1)/(np.linalg.norm(q[1:],axis=1)*np.linalg.norm(q[:-1],axis=1))),0,1)))
jumps=[{'t_s':float(t[i+1]),'angle_deg':float(a),'interval_s':float(t[i+1]-t[i])} for i,a in enumerate(angles) if a>30]
phase_stats={}
for name,lo,hi in [('climb',0,6),('at_height',6,10),('early_staging',10,14),('loss',14,18),('recovery',18,26)]:
 mm=(tm>=lo)&(tm<hi);cc=(tc>=lo)&(tc<hi);ss=(ts>=lo)&(ts<hi)
 phase_stats[name]={'window_s':[lo,hi],'motor_pwm_mean':m[mm].mean(axis=0).tolist(),'any_motor_at_65535_fraction':float(np.mean(np.max(m[mm],axis=1)>=65535)), 'yaw_at_32000_fraction':float(np.mean(np.abs(yawcmd[cc])>=32000)), 'vertical_integral_at_limit_fraction':float(np.mean(iz[cc]>=.399)), 'voltage_range_V':[float(np.min(vb[ss])),float(np.max(vb[ss]))]}
# Pair each firmware Mellinger record to its nearest external pose only for a timeline;
# this is not a camera-to-controller latency estimate.
timeline=[]
for mark in [0,2,4,7,10,13,14,15,16,18,22,24,26]:
 i=np.argmin(abs(t-mark));j=np.argmin(abs(tc-mark));k=np.argmin(abs(ts-mark));l=np.argmin(abs(tm-mark))
 timeline.append({'t_s':mark,'position_m':p[i].tolist(),'yaw_deg':float(ypr[i,2]),'battery_V':float(vb[k]),'yaw_command_legacy':float(yawcmd[j]),'gyro_yaw_deg_s':float(motors[l]['values']['gyro_z_deg_s']),'collective_command_legacy':float(collective[j]),'integrated_z_error_m_s':float(iz[j]),'motor_pwm':m[l].tolist()})
summary={'run':str(run),'PDG_descent_started':False,'operator_observation':'Drone flipped and turned off','peak_world_height_m':float(p[:,2].max()),'launch_voltage_V':4.148,'minimum_logged_voltage_V':float(vb.min()),'first_external_jump_over_30deg':jumps[0] if jumps else None,'stage_start_s':10.011,'stage_rejected_s':18.023,'disarm_confirmed_s':29.655,'staging_position_error_m':.5435405908110982,'staging_attitude_error_deg':171.18944151649967,'phase_statistics':phase_stats,'timeline':timeline,'interpretation':'Loss of height and yaw control with saturation and falling loaded voltage precedes mocap pose jumps. Exact battery/motor/propeller/calibration cause remains unidentified. Not a completed PDG descent or clean controlled landing.'}
(run/'analysis.json').write_text(json.dumps(summary,indent=2)+'\n')
fig,ax=plt.subplots(3,2,figsize=(13,10),sharex=True)
ax[0,0].plot(t,p[:,2],label='Vicon world Z');ax[0,0].axhline(2.5143266758,ls='--',color='k',label='staging target');ax[0,0].set(ylabel='Height [m]',title='Height fell before the abort');ax[0,0].legend(fontsize=8)
ax[0,1].plot(ts,vb,'o-',ms=3);ax[0,1].axhline(3.5,color='gray',ls='--',label='earlier hover trigger (disabled in this trial)');ax[0,1].set(ylabel='Battery [V]',title='Loaded voltage');ax[0,1].legend(fontsize=7)
ax[1,0].plot(t,ypr[:,2],label='Vicon yaw');ax[1,0].set(ylabel='Wrapped yaw [deg]',title='Real rotation during loss');aa=ax[1,0].twinx();aa.plot(tm,[r['values']['gyro_z_deg_s'] for r in motors],color='tab:orange',alpha=.6,label='gyro yaw');aa.set_ylabel('Gyro yaw [deg/s]',color='tab:orange')
ax[1,1].plot(tm,m/65535*100);ax[1,1].axhline(100,color='k',ls='--');ax[1,1].set(ylabel='Motor command [%]',title='Saturation leaves little control margin');ax[1,1].legend(['M1','M2','M3','M4'],fontsize=8)
ax[2,0].plot(tc,yawcmd);ax[2,0].axhline(32000,color='gray',ls='--');ax[2,0].axhline(-32000,color='gray',ls='--');ax[2,0].set(ylabel='Yaw command [legacy units]',title='Active Mellinger yaw demand')
ax[2,1].plot(tc,collective,label='requested collective');ax[2,1].axhline(65535,color='gray',ls='--',label='normalized full-scale command');ax[2,1].set(ylabel='Collective [legacy units]',title='Requested collective is not achieved motor thrust');aa=ax[2,1].twinx();aa.plot(tc,iz,color='tab:red',alpha=.65);aa.set_ylabel('Z integral [m s]',color='tab:red');ax[2,1].legend(fontsize=7)
for a in ax.flat:
 a.axvline(10.011,color='tab:green',ls=':',lw=1);a.axvline(18.023,color='tab:red',ls=':',lw=1);a.axvline(24.419,color='tab:purple',ls=':',lw=1);a.grid(alpha=.25);a.set_xlim(0,30)
for a in ax[-1]:a.set_xlabel('Seconds after takeoff request')
fig.suptitle('Latest full-height trial: staging began at 10 s; abort at 18 s; first large pose jump at 24.4 s\nPDG descent never started; operator reported a flip',fontsize=12)
fig.tight_layout(rect=[0,0,1,.94]);fig.savefig(run/'analysis.png',dpi=140);plt.close(fig)
print(json.dumps({'phase_statistics':phase_stats,'first_external_jump':jumps[0] if jumps else None,'figure':str(run/'analysis.png')},indent=2))
