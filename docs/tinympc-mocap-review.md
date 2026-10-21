# TinyMPC mocap reference review

Reviewed the requested deprecated repository at commit
`4628c515271465b5f9d58e75580e25058b223f6a` (deprecation notice dated 2026-04-17).
Reference checkout: `work/reference-tinympc/`, ignored by the project Git checkout.
No reference build scripts were executed and no reference firmware was flashed.

## Conclusion

There is no different mocap estimator architecture to import. Both stacks receive
external position/orientation through the Crazyflie localization service and fuse
it with IMU inputs in the onboard Kalman estimator. The reference's TinyMPC/LQR
controllers read the resulting `state_t`; they do not replace the Kalman estimator
with raw mocap state. The useful addition is observing the localization-service
input on the drone, alongside host mocap and fused state.

## Source comparison

| Item | Requested reference | Current stack / implication |
|---|---|---|
| Full external pose | XYZ and quaternion x/y/z/w passed to `estimatorEnqueuePose` | Same path and component convention |
| Broadcast packed pose | Vehicle ID, position in integer millimeters, compressed quaternion | Same decoder; addressed to the drone's ID |
| Orientation residual | `q_ekf^-1 * q_measured` | Same calculation; current file differs only by an explanatory comment |
| Prediction rate | 100 Hz Kalman prediction | Same nominal prediction rate; not the 20 Hz diagnostic log rate |
| External position noise | Firmware default `extPosStdDev=0.01` | Firmware default is also 0.01; our launch config sets 0.001 |
| External orientation noise | Firmware default `extQuatStdDev=0.0045` | Firmware default is also 0.0045; our launch config sets 0.05 |
| Initial heading | `initialYaw=0`, radians; +X is zero | Same convention; stationary-pad seeding already exists locally |
| Exported velocity | Kalman rotates internal body velocity into world velocity | Same. Some example controller comments call it body velocity; the actual exported field is world velocity |
| Attitude representations | Quaternion is standard; legacy Euler pitch has its sign inverted | Same. Do not mix quaternion conventions with legacy Euler pitch signs |
| Input diagnostics | `locSrv.{x,y,z,qx,qy,qz,qw}` and `locSrvZ.tick` | Already compiled into our installed firmware; now exposed in ROS logs |

The reference firmware defaults are not verified settings from its experiments:
the README delegates high-level commands to a separate Crazyswarm fork, and no
complete lab configuration is supplied here. Do not copy those defaults as if
they were a validated tuning set. Our launch values are configured values; the
ROS parameter mirror alone is not an independent live firmware readback.

The reference's `mm_pose.c` and current `mm_pose.c` have identical executable
measurement-update code. Modern Kalman code also contains newer configuration,
persistence, drag and supervisor handling; reverting those is unjustified.
No estimator equations, gains, noise weights, or controller were changed from
this review. The reference's older build flags and app controllers were not adopted.

## Useful diagnostic added

Two 10 Hz log blocks in `config/crazyflies.yaml` expose the firmware's received pose:

- `mocap_rx_position`: `locSrv.x/y/z`, `locSrvZ.tick` (14 bytes).
- `mocap_rx_orientation`: `locSrv.qx/qy/qz/qw` (16 bytes).

Each is below the 26-byte firmware log payload limit. The exact variable names
and types were checked against the installed image's generated log TOC.
The mission transport records both blocks beside external pose, onboard state,
controller targets, motor commands, and radio statistics.

`python scripts/check_mocap_path.py` performs an unarmed stationary ground
comparison of host Vicon pose, firmware-received pose, and Kalman output. It also
records age of the last received mocap packet in the firmware clock. RTOS and log
timestamps use 1 ms ticks in this build; the 16-bit receive tick wraps every
65.536 seconds and the comparison accounts for that wrap and a small snapshot
race. This is firmware input age, not camera-to-estimator latency: the ordinary
CRTP external-pose payload does not carry the original camera timestamp.

The received-pose logs were validated live after the server restart. The unarmed
stationary audit passed across 106 paired samples: maximum received-packet age
17 ms, host/received position difference 1.005 mm and orientation difference
0.097 degrees. Host/Kalman differences stayed below 0.793 mm and 2.216 degrees.
Reports are in `output/setup/mocap-path-audit.json` and
`output/setup/mocap-path-samples.json`. These ground measurements do not prove
tracking behavior during flight. No flight command was sent during the audit.
Existing targeted tests: 64 passed after the received-pose logging additions.

## Primary references

- [Repository README](https://github.com/RoboticExplorationLab/tinympc-crazyflie-firmware/blob/4628c515271465b5f9d58e75580e25058b223f6a/README.md)
- [Localization packet handlers and input logs](https://github.com/RoboticExplorationLab/tinympc-crazyflie-firmware/blob/4628c515271465b5f9d58e75580e25058b223f6a/src/modules/src/crtp_localization_service.c)
- [Kalman external-pose measurement update](https://github.com/RoboticExplorationLab/tinympc-crazyflie-firmware/blob/4628c515271465b5f9d58e75580e25058b223f6a/src/modules/src/kalman_core/mm_pose.c)
- [Kalman prediction, initial parameters and exported frame conventions](https://github.com/RoboticExplorationLab/tinympc-crazyflie-firmware/blob/4628c515271465b5f9d58e75580e25058b223f6a/src/modules/src/kalman_core/kalman_core.c)
- [TinyMPC controller reads fused state](https://github.com/RoboticExplorationLab/tinympc-crazyflie-firmware/blob/4628c515271465b5f9d58e75580e25058b223f6a/examples/controller_tinympc/src/controller_tinympc.c)
