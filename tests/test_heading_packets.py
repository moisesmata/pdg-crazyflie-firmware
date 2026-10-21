from pathlib import Path
import subprocess


def test_takeoff_landing_preserve_heading_on_wire(tmp_path):
    root=Path(__file__).resolve().parents[1]
    cpp=root/'ros_ws/src/crazyswarm2/crazyflie_server_cpp/deps/crazyflie_tools/crazyflie_cpp'
    binary=tmp_path/'heading-packets'
    subprocess.run(['c++','-std=c++17','-I'+str(cpp/'include'),
                    '-I'+str(cpp/'crazyflie-link-cpp/include'),
                    str(root/'tests/test_heading_packets.cpp'),'-o',str(binary)],check=True)
    subprocess.run([str(binary)],check=True)
