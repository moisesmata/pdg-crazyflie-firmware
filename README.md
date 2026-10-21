# pdg-crazyflie-firmware
Custom firmware for the Bitcraze Crazyflie, designed to be a testing framework for powered descent guidance algorithms. 

## Experiments on this workstation

See [local setup and validation](docs/local-setup.md) for Ubuntu/WSL + ROS 2 Jazzy, the CF 2.1+ brushed target, local Windows Vicon, and remaining hardware checks. See [hardware integration](docs/hardware.md) for frame calibration, firmware flashing and explicit mission execution.

From the repository root, `source scripts/local_env.sh` activates the prepared environment. `pdg mission --height 1.0` is a planning dry run. `./scripts/launch_crazyswarm.sh` starts the Vicon receiver and radio server without a flight mission.
