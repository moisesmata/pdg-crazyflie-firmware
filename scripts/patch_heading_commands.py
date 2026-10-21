#!/usr/bin/env python3
"""Apply the local heading-preserving fix to the pinned Crazyswarm C++ source."""
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
server = ROOT / 'ros_ws/src/crazyswarm2/crazyflie_server_cpp'
header = server / 'deps/crazyflie_tools/crazyflie_cpp/include/crazyflie_cpp/crtp.h'
text = header.read_text()
original = text
for kind, command in [('Takeoff', 7), ('Land', 8)]:
    start = text.index('class crtpCommanderHighLevel' + kind + 'Request')
    end = text.index('\n};', start) + len('\n};')
    old = text[start:end]
    if 'useCurrentYaw' in old:
        continue
    assert ': Packet(8, 0, 10)' in old and 'setPayloadAt<float>(6, duration)' in old
    new = f'''class crtpCommanderHighLevel{kind}Request
    : public bitcraze::crazyflieLinkCpp::Packet
{{
public:
  crtpCommanderHighLevel{kind}Request(uint8_t groupMask, float height, float duration)
      : Packet(8, 0, 15)
  {{
    // Firmware COMMAND_{kind.upper()}_2: preserve measured heading.
    setPayloadAt<uint8_t>(0, {command});
    setPayloadAt<uint8_t>(1, groupMask);
    setPayloadAt<float>(2, height);
    setPayloadAt<float>(6, 0.0f);
    setPayloadAt<uint8_t>(10, 1); // useCurrentYaw
    setPayloadAt<float>(11, duration);
  }}
}};'''
    text = text[:start] + new + text[end:]
if text != original:
    header.write_text(text)
source = server / 'src/crazyflie_server.cpp'
text = source.read_text()
marker = 'this->declare_parameter("high_level.preserve_yaw", true);'
if marker not in text:
    anchor = 'this->declare_parameter("firmware_params.query_all_values_on_connect", false);'
    assert text.count(anchor) == 1
    text = text.replace(anchor, anchor + '\n    // Local COMMAND_TAKEOFF_2/LAND_2 packet fix; checked by hover preflight.\n    ' + marker)
    source.write_text(text)
print('Heading-preserving takeoff/landing packet patch applied.')
