from pathlib import Path
import subprocess
import shutil


def test_native_collective_limit_edge_cases(tmp_path):
    root=Path(__file__).resolve().parents[1]
    src=root/'apps/pdg_controller/src'
    compiler=shutil.which('cc')
    assert compiler,'A native C compiler is required'
    binary=tmp_path/'limits-test'
    subprocess.run([compiler,'-std=c11','-Wall','-Werror','-I',str(src),str(root/'tests/test_limits.c'),str(src/'pdg_limits.c'),'-lm','-o',str(binary)],check=True)
    subprocess.run([str(binary)],check=True)


def test_manifest_rejects_mislabeled_platform(tmp_path):
    import sys
    root=Path(__file__).resolve().parents[1]
    result=subprocess.run([sys.executable,str(root/'scripts/build_manifest.py'),
                           'cf21bl',str(tmp_path/'cf2.elf')],capture_output=True,text=True)
    assert result.returncode != 0
    assert 'ELF platform name does not match' in result.stderr
