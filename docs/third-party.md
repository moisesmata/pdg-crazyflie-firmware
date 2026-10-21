# Sources and attribution

- Bitcraze `crazyflie-firmware`: https://github.com/bitcraze/crazyflie-firmware, pinned at `9912584c85db92dbbba14334ffed20ebabba463e`. Preserved as a Git submodule; its source and bundled components retain their own license notices. Our app wraps stock Mellinger, not a copied/reimplemented controller.
- A2R-Lab `tinympc-crazyflie`: https://github.com/A2R-Lab/tinympc-crazyflie. Architectural inspiration for a separate app and upstream firmware dependency. TinyMPC and Eigen are not required or included by our app.
- Szmuk & Acikmese (2018): https://arxiv.org/abs/1802.03827. Algorithmic reference for free-time SCvx, FOH discretization, virtual control and trust terms. See `algorithm.md` for every substantive model/transcription difference.
- Crazyswarm2: https://github.com/IMRCLab/crazyswarm2. ROS interface and flight services. Our ROS transport is a small client of its documented messages/services. `ros.repos` records the inspected revision.
- Motion capture tracking: https://github.com/IMRCLab/motion_capture_tracking/tree/ros2. Native Vicon path. Our repository does not redistribute the proprietary Vicon runtime/SDK or tracking software.
- OPT4SMART: https://github.com/OPT4SMART/ros2-vicon-receiver. Optional alternate lab integration; not in the default runtime and not presently implemented as a bridge here.
- Local development toolchain: xPack Arm GCC 14.2.1-1.1, https://github.com/xpack-dev-tools/arm-none-eabi-gcc-xpack/releases/tag/v14.2.1-1.1. Download checked against its published SHA256. Toolchain binaries live in ignored `.tools/`, outside the distributable source tree.

New project code is GPL-3.0-or-later. The included license text is GPLv3 from the firmware dependency. Source dependencies and development tools retain their upstream licenses.
