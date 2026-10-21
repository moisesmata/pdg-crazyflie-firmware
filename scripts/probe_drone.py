#!/usr/bin/env python3
"""Read radio, firmware identity, battery and attitude. Never arm or fly."""
import argparse
from datetime import datetime, timezone
import json
import re
from pathlib import Path
import subprocess
import sys
import yaml

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT/'output/setup/drone-probe.json'
parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--uri')
parser.add_argument('--save-uri', action='store_true')
parser.add_argument('--worker', action='store_true', help=argparse.SUPPRESS)
args = parser.parse_args()
if not Path('/dev/bus/usb').exists():
    raise SystemExit('USB nodes unavailable. Run this script in a normal Ubuntu/WSL terminal.')
if not args.worker:
    import cflib.crtp
    if args.uri:
        uri=args.uri
    else:
        cflib.crtp.init_drivers()
        links=cflib.crtp.scan_interfaces(address=0xE7E7E7E7E7)
        print('Discovered:', json.dumps(links), flush=True)
        if len(links)!=1:
            raise SystemExit('Expected exactly one URI. If multiple are listed, rerun with --uri radio://... for this drone.')
        uri=links[0][0]
    if re.fullmatch(r'radio://\d+/\d+/(250K|1M|2M)',uri):
        uri += '/E7E7E7E7E7'
    print('Read-only radio probe: '+uri+' (timeout: 90 seconds)', flush=True)
    try:
        result=subprocess.run([sys.executable,__file__,'--worker','--uri',uri],timeout=90)
    except subprocess.TimeoutExpired:
        raise SystemExit('Radio probe timed out; tracking/server was not started. Check drone power, Crazyradio attachment, and close other radio clients before retrying.')
    if result.returncode: raise SystemExit(result.returncode)
    if args.save_uri:
        config=ROOT/'config/crazyflies.yaml'
        data=yaml.safe_load(config.read_text())
        old=data['robots']['Crazyflie']['uri']
        config.write_text(config.read_text().replace('uri: '+old,'uri: '+uri))
        print('Saved URI:',uri)
    raise SystemExit(0)

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncLogger import SyncLogger
cflib.crtp.init_drivers()
cache=ROOT/'.cache/radio'
cache.mkdir(parents=True,exist_ok=True)
report={'uri':args.uri,'captured_at_utc':datetime.now(timezone.utc).isoformat(),'flight_commands_sent':False}
print('Connecting to drone and downloading parameter/log tables...',flush=True)
with SyncCrazyflie(args.uri, cf=Crazyflie(rw_cache=str(cache))) as scf:
    print('Connected; waiting for parameter values...',flush=True)
    scf.wait_for_params()
    print('Parameters received; sampling battery and attitude...',flush=True)
    names=['system.selftestPassed','firmware.revision0','firmware.revision1','firmware.modified',
           'pdg.apiVersion','pdg.controllerId','pdg.capEnable','stabilizer.controller',
           'stabilizer.estimator','commander.enHighLevel',
           'deck.bcFlow','deck.bcFlow2','deck.bcZRanger2','motion.disable']
    values={}
    for name in names:
        group,key=name.split('.')
        if key in scf.cf.param.toc.toc.get(group,{}):
            values[name]=scf.cf.param.get_value(name)
    report['parameters']=values
    report['pdg_identity_present']=all(n in values for n in ['pdg.apiVersion','pdg.controllerId','pdg.capEnable'])
    log=LogConfig('pdg_readonly_probe',period_in_ms=200)
    for name in ['pm.vbat','stabilizer.roll','stabilizer.pitch','stabilizer.yaw']:
        group,key=name.split('.')
        if key in scf.cf.log.toc.toc.get(group,{}):log.add_variable(name,'float')
    if log.variables:
        with SyncLogger(scf,log) as logger:
            report['samples']=[]
            for timestamp,data,_ in logger:
                report['samples'].append(dict(timestamp_ms=timestamp,**data))
                if len(report['samples'])>=5:break
OUT.parent.mkdir(parents=True,exist_ok=True)
OUT.write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps(report,indent=2),flush=True)
