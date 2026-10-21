from pathlib import Path
import hashlib,json,subprocess,sys
root=Path(__file__).resolve().parents[1]
platform,elf=sys.argv[1:]
if Path(elf).stem != platform:
    raise ValueError("ELF platform name does not match requested platform")
def cmd(*a): return subprocess.check_output(a,cwd=root,text=True).strip()
manifest={'platform':platform,'firmware_commit':cmd('git','-C','crazyflie-firmware','rev-parse','HEAD'),
          'compiler':cmd('arm-none-eabi-gcc','--version').splitlines()[0],
          'elf_sha256':hashlib.sha256(Path(elf).read_bytes()).hexdigest(),
          'app_sources':{str(p.relative_to(root)):hashlib.sha256(p.read_bytes()).hexdigest()
                         for p in sorted((root/'apps/pdg_controller/src').glob('*.[ch]'))},
          'configuration_sha256':hashlib.sha256((root/'apps/pdg_controller/build/.config').read_bytes()).hexdigest()}
(root/'output/firmware'/f'{platform}-manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
