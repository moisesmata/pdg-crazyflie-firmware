#!/usr/bin/env python3
"""Discover radio URIs without sending arming, takeoff, or flight setpoints."""
import argparse
import json
from pathlib import Path
import cflib.crtp

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--address', default='E7E7E7E7E7')
args = parser.parse_args()
address = int(args.address, 16)
if not 0 <= address < 2**40:
    parser.error('Radio address must fit in 5 bytes')
if not Path('/dev/bus/usb').exists():
    raise SystemExit('USB device nodes are unavailable here. Run from a normal WSL terminal with the Crazyradio attached through usbipd.')
cflib.crtp.init_drivers()
links = cflib.crtp.scan_interfaces(address=address)
report = dict(address=f'{address:010X}', links=links)
path = Path(__file__).resolve().parents[1]/'output/setup/radio-scan.json'
path.parent.mkdir(parents=True, exist_ok=True)
path.write_text(json.dumps(report, indent=2)+'\n')
print(json.dumps(report, indent=2))
if not links:
    raise SystemExit('No drone found. Check power, USB permissions/attachment, and the radio address.')
