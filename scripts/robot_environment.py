"""Checked project-only Linux bootstrap; refuse unsupported systems explicitly."""
import hashlib
import json
import os
from pathlib import Path
import platform
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]


def inspect():
    release = platform.freedesktop_os_release() if sys.platform == 'linux' else {}
    return dict(system=platform.system(), architecture=platform.machine(),
        python=list(sys.version_info[:3]), distribution=release.get('ID'), version=release.get('VERSION_ID'))


def main():
    observed = inspect()
    if observed['system'] != 'Linux' or observed['architecture'] not in ('x86_64','aarch64'):
        raise RuntimeError('requires_Linux_x86_64_or_aarch64: '+json.dumps(observed))
    if sys.version_info[:2] != (3,12) or observed['distribution'] != 'ubuntu' or observed['version'] != '24.04':
        raise RuntimeError('native_stack_requires_Ubuntu_24.04_Python_3.12: '+json.dumps(observed))
    env = os.environ.copy()
    for name in ('PIP_TARGET','PIP_PREFIX','PIP_USER','PYTHONPATH','PYTHONHOME'):
        env.pop(name, None)
    env.update(PIP_CONFIG_FILE=os.devnull, PYTHONNOUSERSITE='1')
    python = ROOT/'.venv/bin/python'
    fingerprint = hashlib.sha256((ROOT/'requirements-linux.txt').read_bytes()+
        (ROOT/'pyproject.toml').read_bytes()).hexdigest()
    receipt = ROOT/'.venv/tg3_environment.json'
    old = json.loads(receipt.read_text()) if receipt.exists() else {}
    if not python.exists():
        subprocess.run([sys.executable,'-m','venv',str(ROOT/'.venv')],check=True,env=env)
    if old.get('fingerprint') != fingerprint:
        base = [str(python),'-s','-m','pip']
        subprocess.run(base+['install','--no-user','-r',str(ROOT/'requirements-linux.txt')],check=True,env=env)
        subprocess.run(base+['install','--no-user','--no-deps','-e',str(ROOT)],check=True,env=env)
    env['LD_LIBRARY_PATH'] = str(ROOT/'.venv/lib/python3.12/site-packages/cmeel.prefix/lib') + ':' + env.get('LD_LIBRARY_PATH','')
    subprocess.run([str(python),'-s','-m','tg3_pico_dualarm','selftest'],check=True,env=env,cwd=ROOT)
    from datetime import datetime, timezone
    receipt.write_text(json.dumps(dict(fingerprint=fingerprint, system=observed,
        real_placo_selftest=True, hardware_commands_sent=False,
        time=datetime.now(timezone.utc).isoformat()),indent=2),encoding='utf-8')
    print(json.dumps(dict(environment_ready=True, native_placo_selftest=True,
        hardware_ready=False, hardware_motion_enabled=False)),flush=True)


if __name__ == '__main__':
    try:
        main()
    except (RuntimeError, subprocess.CalledProcessError, OSError) as exc:
        print('环境未就绪：'+str(exc),file=sys.stderr)
        raise SystemExit(2)
