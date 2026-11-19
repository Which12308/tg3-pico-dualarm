"""Owned XARM process and reversible, source-preserving factory bridge handoff.

No target, enable, mode, gait velocity, or all-body service is called here.
The root supervisor owns these contexts; ordinary VR admission stays disarmed.
"""
import hashlib
import json
import os
from pathlib import Path
import select
import signal
import subprocess
import time
from .bounded_output import bounded_output

LOG_ROOT=Path('/home/ubuntu/.local/share/tg3-teleop/diagnostics')

HOME=Path('/home/ubuntu')
RELEASE=HOME/'tiangong3_factory_joystick_overlay/releases/20260814_16'
EXECUTABLE=RELEASE/'install/tiangong3_factory_joystick_overlay/lib/tiangong3_factory_joystick_overlay/joystick_bridge_node_tg3'
SHIM=HOME/'xos/joystick/lib/joystick/joystick_bridge_node.py'
LOCK=Path('/run/lock/tiangong3-factory-joystick-overlay-runtime.lock')
REVIEWED_SOURCE='bceecafc8d5989af0afdb3a2a79836d76469d48ac6baf46aac651f4d978107ff'
REVIEWED_SHIM='3456386c5807c3c1344d22a35ac5c81163597c559da305da932b3008293f450f'


def digest(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def bridge_processes():
    rows=[]
    for p in Path('/proc').iterdir():
        if not p.name.isdigit():continue
        try:
            args=[v.decode() for v in (p/'cmdline').read_bytes().split(b'\0') if v]
            if str(EXECUTABLE) not in args:continue
            stat=(p/'stat').read_text().split()
            fields={s.split(':')[0]:s.split(':')[1].strip() for s in (p/'status').read_text().splitlines() if s.startswith(('SigBlk:','SigIgn:'))}
            rows.append(dict(pid=int(p.name),argv=args,start=int(stat[21]),parent=int(stat[3]),signals=fields))
        except (OSError,ValueError,UnicodeError):continue
    return rows


def stop_owned_group(process):
    """Stop only a process group created here, including vendor grandchildren."""
    if process is None:return
    try:os.killpg(process.pid,signal.SIGINT)
    except ProcessLookupError:return
    try:process.wait(timeout=8.)
    except subprocess.TimeoutExpired:
        try:os.killpg(process.pid,signal.SIGTERM)
        except ProcessLookupError:pass
        try:process.wait(timeout=2.)
        except subprocess.TimeoutExpired:pass
    # Even a successful launch-parent exit may leave a vendor thread/process.
    members=[]
    for p in Path('/proc').iterdir():
        if not p.name.isdigit():continue
        try:
            if int((p/'stat').read_text().split()[4])==process.pid:members.append(int(p.name))
        except (OSError,ValueError,IndexError):continue
    if members:
        try:os.killpg(process.pid,signal.SIGKILL)
        except ProcessLookupError:pass
    if process.poll() is None:process.wait(timeout=2.)


class FactoryBridgeHandoff:
    def __init__(self,directory,*,observe,log=print):
        self.directory=Path(directory);self.observe=observe;self.log=log
        self.original=None;self.lock=None;self.child=None;self.changed=False;self.restored=False

    def _save(self,**extra):
        data=dict(original=self.original,changed=self.changed,restored=self.restored,**extra)
        path=self.directory/'bridge_handoff.json';path.write_text(json.dumps(data,indent=2),encoding='utf-8');path.chmod(0o600)

    def start(self):
        import fcntl
        if os.geteuid()!=0:raise RuntimeError('administrator_required_for_factory_bridge_handoff')
        self.observe(require_neutral=True)
        if digest(EXECUTABLE)!=REVIEWED_SOURCE or digest(SHIM)!=REVIEWED_SHIM:
            raise RuntimeError('factory_bridge_or_lock_shim_version_changed_review_required')
        release=(HOME/'tiangong3_factory_joystick_overlay/active_factory_release').read_text().strip()
        if release!=str(RELEASE):raise RuntimeError('factory_active_release_changed')
        rows=bridge_processes()
        if len(rows)!=1:raise RuntimeError('unique_factory_bridge_required')
        self.original=rows[0]
        if any('/tg3/xarm_retired/' in arg for arg in self.original['argv']):
            raise RuntimeError('previous_factory_handoff_requires_recovery')
        if int(self.original['signals'].get('SigBlk','0'),16)&0x4003:
            raise RuntimeError('factory_bridge_shutdown_signals_blocked')
        self.directory.mkdir(parents=True,exist_ok=True);self.directory.chmod(0o700)
        copies=[];args=self.original['argv']
        for i,value in enumerate(args[:-1]):
            if value=='--params-file':
                path=Path(args[i+1]);raw=path.read_bytes();copy=self.directory/('params_'+str(i)+'.yaml')
                copy.write_bytes(raw);copy.chmod(0o600)
                copies.append(dict(source=str(path),copy=str(copy),sha256=hashlib.sha256(raw).hexdigest()))
        self.original['parameter_backups']=copies;self._save()
        descriptor=os.pidfd_open(self.original['pid'])
        try:
            current=bridge_processes()
            if len(current)!=1 or any(current[0][key]!=self.original[key] for key in ('pid','start','argv')):
                raise RuntimeError('factory_bridge_identity_changed_before_handoff')
            signal.pidfd_send_signal(descriptor,signal.SIGTERM);self.changed=True;self._save()
            poll=select.poll();poll.register(descriptor,select.POLLIN)
            if not poll.poll(4000):raise RuntimeError('original_factory_bridge_did_not_exit')
        finally:os.close(descriptor)
        self.lock=LOCK.open('rb');fcntl.flock(self.lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        if bridge_processes():raise RuntimeError('factory_bridge_respawn_conflict')
        from .stationary_guard import bridge_isolation_remaps
        remaps=bridge_isolation_remaps()
        with bounded_output(LOG_ROOT/'remapped_bridge.log') as output:
            self.child=subprocess.Popen(['/bin/bash','-c','source "$1"; shift; exec "$@"','tg3-factory-bridge',
                str(RELEASE/'install/setup.bash'),*args,*remaps],stdout=output,stderr=subprocess.STDOUT,
                pass_fds=(self.lock.fileno(),),start_new_session=True)
        self.observe(require_neutral=True,seconds=3.)
        if self.child.poll() is not None:raise RuntimeError('remapped_factory_bridge_exited')
        self._save(remapped_pid=self.child.pid)
        self.log('Factory bridge handoff ready; body/FSM/head retained; arm/hand/walk/stand-pose outputs isolated.')
        return self

    def close(self):
        stop_owned_group(self.child);self.child=None
        if self.lock:self.lock.close();self.lock=None
        if not self.changed:return
        rows=bridge_processes()
        if not rows:
            args=list(self.original['argv'][2:])
            for item in self.original['parameter_backups']:
                if digest(item['copy'])!=item['sha256']:raise RuntimeError('factory_parameter_backup_changed')
            # Keep original argv when its exact parameter bytes remain present.
            for i,value in enumerate(args[:-1]):
                if value=='--params-file':
                    backup=next(x for x in self.original['parameter_backups'] if x['source']==args[i+1])
                    if not Path(args[i+1]).exists() or digest(args[i+1])!=backup['sha256']:
                        args[i+1]=backup['copy']
            with bounded_output(LOG_ROOT/'restored_bridge.log') as output:
                subprocess.Popen(['/usr/bin/python3',str(SHIM),*args],stdout=output,stderr=subprocess.STDOUT,start_new_session=True)
        deadline=time.monotonic()+5.
        while time.monotonic()<deadline:
            rows=bridge_processes()
            if len(rows)==1 and self._original_identity(rows[0]['argv']):
                if int(rows[0]['signals'].get('SigBlk','0'),16)&0x4003:raise RuntimeError('restored_bridge_signals_blocked')
                self.restored=True;self._save(restored_pid=rows[0]['pid']);return
            time.sleep(.1)
        self._save(error='original_factory_bridge_restore_unconfirmed')
        raise RuntimeError('original_factory_bridge_restore_unconfirmed')

    def _original_identity(self,args):
        expected=list(self.original['argv'])
        if len(args)!=len(expected):return False
        for i,(actual,original) in enumerate(zip(args,expected)):
            if actual==original:continue
            if i==0 and actual=='/usr/bin/python3' and original.endswith('/python3'):continue
            if i and expected[i-1]=='--params-file':
                backup=next((x for x in self.original['parameter_backups'] if x['source']==original),None)
                if backup and actual==backup['copy'] and digest(actual)==backup['sha256']:continue
            return False
        return True


class DisabledFactoryProcess:
    def __init__(self,directory,holding_profile=None):
        self.directory=Path(directory);self.process=None;self.model_sha=None;self.holding_profile=holding_profile

    def start(self):
        from ament_index_python.packages import get_package_prefix,get_package_share_directory
        import xacro
        bringup=Path(get_package_share_directory('tiangong3_bringup'))
        model=xacro.process_file(str(bringup/'config/tiangong3_description.urdf.xacro'),
            mappings={'hardware':'real','initial_positions_file':str(bringup/'config/initial_positions.yaml')}).toxml()
        self.model_sha=hashlib.sha256(model.encode()).hexdigest()
        if self.model_sha!='6825b8f6d709cee8c653aad12d1d63a1301e813a2a851de59d6102cce377ce02':
            raise RuntimeError('installed_factory_model_changed_review_before_hardware_start')
        environment=os.environ.copy();library=HOME/'.local/lib/python3.12/site-packages/cmeel.prefix/lib'
        environment['LD_LIBRARY_PATH']=str(library)+':'+environment.get('LD_LIBRARY_PATH','')
        environment.update(OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1',MKL_NUM_THREADS='1')
        hardware=Path(get_package_prefix('tiangong3_hardware'))/'lib/libtiangong3_hardware.so'
        checked=subprocess.run(['/usr/bin/ldd',str(hardware)],capture_output=True,text=True,env=environment,timeout=10)
        if checked.returncode or 'not found' in checked.stdout:raise RuntimeError('unresolved_factory_native_dependency:'+checked.stdout[-1500:])
        launch='''from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction
def generate_launch_description():
 return LaunchDescription([
  Node(package='robot_state_publisher',executable='robot_state_publisher',name='tg3_xarm_model',parameters=[{'robot_description':MODEL}],remappings=[('/robot_description','/xarm_robot_description'),('/joint_states','/joint_state_broadcaster/joint_states')]),
  Node(package='controller_manager',executable='ros2_control_node',parameters=[CONFIG,{'run_type':'real'},{'robot_type':'tiangong2dex'},{'lock_memory':False}],remappings=[('/arm/cmd','/tg3/xarm/arm_raw'),('~/robot_description','/xarm_robot_description'),('/robot_description','/xarm_robot_description')],output='screen'),
  TimerAction(period=2.,actions=[Node(package='controller_manager',executable='spawner',arguments=['--inactive','empty_L_controller','empty_R_controller','jointspace_ruckig_qp_controller'],output='screen')])])
'''.replace('MODEL',repr(model)).replace('CONFIG',repr(str(bringup/'config/tiangong3_controllers.yaml')))
        if self.holding_profile is None:raise RuntimeError('verified_installed_holding_profile_required')
        # Native EAIHardware declares these parameters itself. Passing kp/kd
        # as launch overrides causes ParameterAlreadyDeclaredException in the
        # installed build. Supervisor sets/readbacks them only after startup,
        # with every arm controller inactive and no real arm publisher.
        self.directory.mkdir(parents=True,exist_ok=True);path=self.directory/'disabled_factory.launch.py';path.write_text(launch,encoding='utf-8')
        with bounded_output(LOG_ROOT/'factory.log') as output:
            self.process=subprocess.Popen(['/opt/ros/jazzy/bin/ros2','launch',str(path)],cwd=bringup/'config',
                env=environment,stdout=output,stderr=subprocess.STDOUT,start_new_session=True)
        return self

    def close(self):stop_owned_group(self.process);self.process=None
