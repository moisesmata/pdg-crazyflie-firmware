#!/usr/bin/env python3
"""Check the built image, then explicitly flash CF 2.1+ brushed STM32 firmware."""
import argparse
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import time
import yaml

ROOT=Path(__file__).resolve().parents[1]
p=argparse.ArgumentParser(description=__doc__)
p.add_argument('--flash',action='store_true',help='Actually reboot and flash the discovered drone')
p.add_argument('--props-removed',action='store_true',help='Operator confirms propellers are removed')
a=p.parse_args()
manifest=json.loads((ROOT/'output/firmware/cf2-flash-manifest.json').read_text())
image=ROOT/'output/firmware/cf2.bin'
elf=ROOT/'output/firmware/cf2.elf'
if manifest['platform']!='cf2' or not manifest.get('bin_matches_elf_with_upstream_objcopy_flags'):
    raise SystemExit('Expected verified brushed cf2 image manifest')
if hashlib.sha256(image.read_bytes()).hexdigest()!=manifest['bin_sha256'] or hashlib.sha256(elf.read_bytes()).hexdigest()!=manifest['elf_sha256']:
    raise SystemExit('Firmware hash mismatch; rebuild and reverify image before flashing')
uri=yaml.safe_load((ROOT/'config/crazyflies.yaml').read_text())['robots']['Crazyflie']['uri']
print(json.dumps(dict(platform='cf2',target='stm32-fw',uri=uri,image=str(image),sha256=manifest['bin_sha256'],flash_requested=a.flash),indent=2),flush=True)
if not a.flash:raise SystemExit(0)
if not a.props_removed:p.error('--props-removed is required for the physical flash step')
if not Path('/dev/bus/usb').exists():raise SystemExit('Run in a normal WSL terminal with the Crazyradio attached')
# Refresh battery/identity before touching firmware; this sends no motor commands.
subprocess.run([sys.executable,str(ROOT/'scripts/probe_drone.py'),'--uri',uri],check=True,timeout=90)
probe=json.loads((ROOT/'output/setup/drone-probe.json').read_text())
voltage=[s['pm.vbat'] for s in probe.get('samples',[]) if 'pm.vbat' in s]
if not voltage or min(voltage)<3.8:raise SystemExit('Charge/swap battery before flashing: require a fresh measured battery voltage >= 3.8 V')
import cflib.crtp
from cflib.bootloader import Bootloader,Target
from cflib.bootloader.boottypes import BootVersion
cflib.crtp.init_drivers()
loader=Bootloader(uri)
status=dict(started_at=datetime.now(timezone.utc).isoformat(),uri=uri,platform='cf2',sha256=manifest['bin_sha256'],flash_complete=False,identity_verified=False)
try:
    print('Rebooting into bootloader...',flush=True)
    if not loader.start_bootloader(warm_boot=True):raise RuntimeError('Warm bootloader connection failed')
    if loader.protocol_version!=BootVersion.CF2_PROTO_VER:raise RuntimeError('Unexpected bootloader protocol; firmware was not written')
    loader.progress_cb=lambda message,percent: print(f'{percent}% {message}',flush=True)
    loader.flash(str(image),[Target('cf2','stm32','fw',[],[])])
    if not loader.reset_to_firmware(boot_delay=5.0):raise RuntimeError('Flash ended but reboot failed; keep props removed')
    status['flash_complete']=True
    time.sleep(6)
    subprocess.run([sys.executable,str(ROOT/'scripts/probe_drone.py'),'--uri',uri],check=True,timeout=90)
    values=json.loads((ROOT/'output/setup/drone-probe.json').read_text())['parameters']
    status['identity_verified']=int(values.get('pdg.apiVersion',-1))==1 and int(values.get('pdg.capEnable',-1))==0 and 'pdg.controllerId' in values
    if not status['identity_verified']:raise RuntimeError('Post-flash PDG identity check failed')
except Exception as exc:
    status['error']=str(exc)
    raise
finally:
    loader.close()
    (ROOT/'output/setup/flash-result.json').write_text(json.dumps(status,indent=2)+'\n')
    print(json.dumps(status,indent=2),flush=True)
