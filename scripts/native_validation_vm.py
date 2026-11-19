"""Project-owned VirtualBox Ubuntu guest for real Linux/Placo/ROS validation.

Never reuses, snapshots, changes, or boots a user's existing VM. The only VM
this script controls is created underneath artifacts/native_environment and
recorded with its UUID. All SSH keys are generated for this guest only.
Run with the desktop venv (Paramiko); pycdlib wheel is loaded locally.
"""
from __future__ import annotations
import argparse
import hashlib
import io
import json
import os
import logging
from pathlib import Path
import shlex
import socket
import subprocess
import sys
import time
import uuid
import zipfile
import urllib.request

ROOT=Path(__file__).resolve().parents[1]
ENV=ROOT/'artifacts/native_environment'
VBOX=Path('C:/Program Files/Oracle/VirtualBox/VBoxManage.exe')
IMAGE=ENV/'ubuntu-24.04-20261017-amd64.vmdk'
IMAGE_SHA='9a1e3959c9f4ad9d4752101fd4ff9a43191fb7d72e61fa20e033f9fd235ce794'
IMAGE_URL='https://cloud-images.ubuntu.com/noble/20261017/noble-server-cloudimg-amd64.vmdk'
IMAGE_BYTES=590749696
ISO_WHEEL_SHA='a905827335f0066af3fd416c5cf9b1f29dffaf4d0914b714555213d1809f38d4'
STATE=ENV/'state.json'
sys.path.insert(0,str(ROOT/'src'))


def vbox(*args,timeout=120):
    environment=os.environ.copy()
    environment['VBOX_USER_HOME']=str(ENV/'vbox-profile')
    result=subprocess.run([str(VBOX),*map(str,args)],capture_output=True,text=True,
                          encoding='utf-8',errors='replace',timeout=timeout,env=environment)
    if result.returncode:raise RuntimeError(result.stdout+result.stderr)
    return result.stdout


def load():
    value=json.loads(STATE.read_text(encoding='utf-8'))
    if value.get('owner')!='tg3_native_validation':raise RuntimeError('unowned VM state')
    details=vbox('showvminfo',value['uuid'],'--machinereadable')
    config=next(line.split('=',1)[1].strip('"') for line in details.splitlines() if line.startswith('CfgFile='))
    if ENV.resolve() not in Path(config).resolve().parents:
        raise RuntimeError('VM is outside this task environment; refusing mutation')
    return value


def prepare():
    import paramiko
    ENV.mkdir(parents=True,exist_ok=True)
    if not IMAGE.exists() or IMAGE.stat().st_size<IMAGE_BYTES:
        # Resume a truncated transfer only after checking the HTTP range.
        for attempt in range(4):
            start=IMAGE.stat().st_size if IMAGE.exists() else 0
            headers={'Range':f'bytes={start}-'} if start else {}
            with urllib.request.urlopen(urllib.request.Request(IMAGE_URL,headers=headers),timeout=30) as response:
                if start and (response.status!=206 or not response.headers.get('Content-Range','').startswith(f'bytes {start}-')):
                    raise RuntimeError('server refused exact resume range')
                with IMAGE.open('ab' if start else 'wb') as output:
                    while block:=response.read(4*1024*1024):output.write(block)
            if IMAGE.stat().st_size==IMAGE_BYTES:break
        if IMAGE.stat().st_size!=IMAGE_BYTES:raise RuntimeError('Ubuntu image transfer incomplete')
    if hashlib.file_digest(IMAGE.open('rb'),'sha256').hexdigest()!=IMAGE_SHA:
        raise RuntimeError('Ubuntu cloud image hash mismatch')
    if STATE.exists():return load()
    hostdeps=ROOT/'artifacts/native_hostdeps';hostdeps.mkdir(parents=True,exist_ok=True)
    wheel=hostdeps/'pycdlib-1.14.0-py2.py3-none-any.whl'
    if not wheel.exists():
        with urllib.request.urlopen('https://pypi.org/pypi/pycdlib/1.14.0/json',timeout=20) as response:
            metadata=json.load(response)
        item=next(item for item in metadata['urls'] if item['filename']==wheel.name)
        if item['digests']['sha256']!=ISO_WHEEL_SHA:raise RuntimeError('pycdlib source checksum mismatch')
        with urllib.request.urlopen(item['url'],timeout=20) as response:wheel.write_bytes(response.read())
    if hashlib.file_digest(wheel.open('rb'),'sha256').hexdigest()!=ISO_WHEEL_SHA:
        raise RuntimeError('pycdlib wheel checksum mismatch')
    sys.path.insert(0,str(wheel))
    import pycdlib
    key=paramiko.RSAKey.generate(2048)
    key.write_private_key_file(str(ENV/'client_key'))
    public=key.get_name()+' '+key.get_base64()+' tg3-native-validation-only'
    user_data='''#cloud-config
hostname: tg3-native-validation
manage_etc_hosts: true
users:
  - name: tg3
    groups: [sudo]
    shell: /bin/bash
    lock_passwd: true
    sudo: ALL=(ALL) NOPASSWD:ALL
    ssh_authorized_keys:
      - PUBLIC_KEY
ssh_pwauth: false
disable_root: true
package_update: true
packages: [python3-venv, unzip, ca-certificates]
final_message: TG3 isolated native validation guest initialized
'''.replace('PUBLIC_KEY',public)
    iso=pycdlib.PyCdlib();iso.new(interchange_level=3,joliet=3,vol_ident='cidata',rock_ridge='1.09')
    for name,content,iso_name in (
        ('user-data',user_data,'USERDATA.;1'),
        ('meta-data','instance-id: tg3-'+uuid.uuid4().hex+'\nlocal-hostname: tg3-native-validation\n','METADATA.;1')):
        data=content.encode();iso.add_fp(io.BytesIO(data),len(data),iso_path='/'+iso_name,
                                      rr_name=name,joliet_path='/'+name)
    iso.write(str(ENV/'seed.iso'));iso.close()
    name='TG3-Native-Validation-'+uuid.uuid4().hex[:8]
    output=vbox('createvm','--name',name,'--ostype','Ubuntu_64','--basefolder',ENV/'vms','--register')
    identifier=next(line.split(':',1)[1].strip() for line in output.splitlines() if line.startswith('UUID:'))
    state=dict(owner='tg3_native_validation',name=name,uuid=identifier,port=32224,
               cloud_image_sha256=IMAGE_SHA,robot_commands_sent=False)
    STATE.write_text(json.dumps(state,indent=2),encoding='utf-8')
    load() # Verify the resolved config before any VM mutation.
    disk=ENV/'native-system.vdi'
    vbox('clonemedium','disk',IMAGE,disk,'--format','VDI',timeout=180)
    vbox('modifymedium','disk',disk,'--resize','24576')
    # One vCPU avoids demonstrated NEM/KVM virtual-timer jitter on this host.
    # This changes this disposable validation guest only, never host Hyper-V.
    vbox('modifyvm',identifier,'--memory','4096','--cpus','1','--firmware','efi',
         '--nic1','nat','--nictype1','virtio','--natpf1','tg3ssh,tcp,127.0.0.1,32224,,22',
         '--boot1','disk','--boot2','dvd','--boot3','none','--boot4','none',
         '--graphicscontroller','vmsvga','--vram','16','--uart1','0x3f8','4',
         '--uartmode1','file',ENV/'serial.log')
    vbox('storagectl',identifier,'--name','SATA','--add','sata','--controller','IntelAhci')
    vbox('storageattach',identifier,'--storagectl','SATA','--port','0','--device','0','--type','hdd','--medium',disk)
    vbox('storageattach',identifier,'--storagectl','SATA','--port','1','--device','0','--type','dvddrive','--medium',ENV/'seed.iso')
    return state


def connect(wait=180):
    import paramiko
    logging.getLogger('paramiko.transport').setLevel(logging.CRITICAL)
    state=load();deadline=time.monotonic()+wait
    error=None
    while time.monotonic()<deadline:
        client=paramiko.SSHClient()
        # New task-owned localhost NAT guest only. This policy must never be
        # used for a manufacturer robot or an arbitrary network host.
        client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
        try:
            client.connect('127.0.0.1',port=state['port'],username='tg3',
                           key_filename=str(ENV/'client_key'),allow_agent=False,look_for_keys=False,
                           timeout=3,banner_timeout=3,auth_timeout=3)
            return client
        except (OSError,paramiko.SSHException) as exc:
            client.close();error=exc;time.sleep(2)
    raise RuntimeError('new guest not ready: '+str(error))


def command(client,script,*,timeout=1800,log_name='command.log'):
    channel=client.get_transport().open_session();channel.exec_command(script)
    deadline=time.monotonic()+timeout;data=[]
    with (ENV/log_name).open('w',encoding='utf-8') as log:
        try:
            while not channel.exit_status_ready() or channel.recv_ready() or channel.recv_stderr_ready():
                if time.monotonic()>deadline:raise TimeoutError('native command timeout')
                for ready,read in ((channel.recv_ready,channel.recv),(channel.recv_stderr_ready,channel.recv_stderr)):
                    if ready():
                        value=read(8192).decode('utf-8',errors='replace');data.append(value)
                        log.write(value);log.flush();print(value,end='',flush=True)
                time.sleep(.03)
            status=channel.recv_exit_status()
            if status:raise RuntimeError('native command failed: '+str(status)+'; '+log_name)
            return ''.join(data)
        finally:channel.close()


def upload(client):
    from tg3_pico_dualarm.deployment import make_robot_bundle
    # The deployment bundle correctly omits desktop software. Native full-suite
    # validation additionally needs the real desktop modules and already built
    # APK/receipt, never their build tools or local machine state.
    selected={}
    with zipfile.ZipFile(io.BytesIO(make_robot_bundle(ROOT))) as archive:
        selected.update((name,archive.read(name)) for name in archive.namelist())
    # Native tests also inspect headset source/receipts. Upload the complete
    # reviewed public tree so an older guest copy cannot silently participate.
    from release_scope import public_source
    selected.update(public_source(ROOT))
    for path in sorted((ROOT/'desktop').glob('*')):
        if path.is_file() and not path.is_symlink():
            selected[path.relative_to(ROOT).as_posix()]=path.read_bytes()
    for name in ('TG3-Wireless-0.4.0.apk','TG3-Wireless-0.4.0.json'):
        path=ROOT/'dist'/name
        if path.is_file():selected[path.relative_to(ROOT).as_posix()]=path.read_bytes()
    buffer=io.BytesIO()
    with zipfile.ZipFile(buffer,'w',zipfile.ZIP_DEFLATED) as archive:
        for name,content in sorted(selected.items()):
            info=zipfile.ZipInfo(name,date_time=(2026,11,14,0,0,0))
            info.compress_type=zipfile.ZIP_DEFLATED;info.external_attr=0o100644<<16
            archive.writestr(info,content)
    data=buffer.getvalue();digest=hashlib.sha256(data).hexdigest()
    snapshots=ROOT/'artifacts/native_snapshots';snapshots.mkdir(exist_ok=True)
    snapshot=snapshots/(digest+'.zip')
    if snapshot.exists():
        if hashlib.file_digest(snapshot.open('rb'),'sha256').hexdigest()!=digest:
            raise RuntimeError('existing native source snapshot checksum mismatch')
    else:snapshot.write_bytes(data)
    sftp=client.open_sftp()
    with sftp.file('/home/tg3/project.zip','wb') as stream:stream.write(data)
    sftp.close()
    command(client,"python3 -c "+shlex.quote("import hashlib,pathlib,zipfile; p=pathlib.Path('/home/tg3/project.zip'); assert hashlib.sha256(p.read_bytes()).hexdigest()=="+repr(digest)+"; zipfile.ZipFile(p).extractall('/home/tg3/project')"),timeout=30,log_name='upload.log')
    return digest


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('operation',choices=('prepare','start','command','upload','stop'))
    parser.add_argument('--script');parser.add_argument('--script-file',type=Path)
    parser.add_argument('--log',default='command.log');parser.add_argument('--timeout',type=int,default=1800)
    args=parser.parse_args()
    if args.operation=='prepare':print(json.dumps(prepare(),indent=2));return
    state=load()
    if args.operation=='start':
        details=vbox('showvminfo',state['uuid'],'--machinereadable')
        if 'VMState="running"' not in details:vbox('startvm',state['uuid'],'--type','headless')
        client=connect();command(client,'uname -a; python3 --version; cat /etc/os-release',timeout=20,log_name='system.log');client.close();return
    if args.operation=='stop':
        client=connect(10)
        command(client,'sudo shutdown -h +1',timeout=10,log_name='shutdown.log');client.close();return
    client=connect(20)
    try:
        if args.operation=='upload':print(json.dumps(dict(source_sha256=upload(client))));return
        script=args.script_file.read_text(encoding='utf-8') if args.script_file else args.script
        if not script:raise ValueError('provide --script or --script-file')
        command(client,script,timeout=args.timeout,log_name=args.log)
    finally:client.close()


if __name__=='__main__':main()
