"""Private SSH deployment and forwarding, with explicit host-key verification."""
from contextlib import suppress
import hashlib
import io
import json
import os
from pathlib import Path
import re
import shlex
import socket
import socketserver
import threading
import time
import zipfile
import uuid

EXCLUDE = {'.git','.venv','.desktop','dist','build','artifacts','recordings',
           '__pycache__','generated_client','.test_tmp','manufacturer_research','.headset_tools',
           '.pytest_cache','.mypy_cache','.ruff_cache','usb_dependencies'}


def _config_overrides(overrides):
    result={}
    for name,data in (overrides or {}).items():
        if (not isinstance(name,str) or '\\' in name or '\x00' in name or ':' in name
            or not name.startswith('config/') or any(p in ('','..','.') for p in name.split('/'))):
            raise ValueError('unsafe_config_override_path')
        if not isinstance(data,(str,bytes)):raise ValueError('config_override_must_contain_bytes_or_text')
        result[name]=data.encode('utf-8') if isinstance(data,str) else data
    return result


def make_robot_bundle(root, overrides=None):
    root=Path(root).resolve()
    overrides=_config_overrides(overrides)
    selected={}
    for directory,dirs,names in os.walk(root,followlinks=False):
        base=Path(directory)
        dirs[:]=sorted(name for name in dirs if name not in EXCLUDE and not name.endswith('.egg-info')
            and not (base/name).is_symlink() and not (base==root and name in ('headset','headset_native','desktop')))
        for filename in sorted(names):
            path=base/filename;rel=path.relative_to(root)
            if path.is_symlink() or path.suffix.lower() in ('.pyc','.pyo','.apk','.zip','.log'):continue
            name=rel.as_posix()
            # Local provider files enter only through the explicitly selected
            # descriptor/source overrides, never through a previous local copy.
            if name in ('config/factory.json','config/factory_driver.py'):continue
            selected[name]=path.read_bytes()
    selected.update(overrides)
    buffer=io.BytesIO()
    with zipfile.ZipFile(buffer,'w',zipfile.ZIP_DEFLATED) as archive:
        for name,content in sorted(selected.items()):
            info=zipfile.ZipInfo(name,date_time=(2026,11,14,0,0,0))
            info.compress_type=zipfile.ZIP_DEFLATED
            info.external_attr=0o100644 << 16
            archive.writestr(info,content)
    return buffer.getvalue()


def reviewed_factory_overrides(path,expected_descriptor_sha256=None):
    """Copy only the reviewed descriptor and exact provider source file.

    Parsing never runs the provider. Hashes are checked again at upload time;
    the remote descriptor references the two files inside config only.
    """
    from .manufacturer_session import load_factory_bundle
    descriptor=Path(path).resolve(strict=True)
    raw=descriptor.read_bytes();digest=hashlib.sha256(raw).hexdigest()
    if expected_descriptor_sha256 is not None and digest!=expected_descriptor_sha256:
        raise ValueError('factory_descriptor_changed_reload_reviewed_bundle')
    bundle=load_factory_bundle(descriptor)
    if descriptor.read_bytes()!=raw:raise ValueError('factory_descriptor_changed_during_validation')
    source=bundle.source_path.read_bytes()
    if hashlib.sha256(source).hexdigest()!=bundle.source_sha256:
        raise ValueError('reviewed_factory_source_changed_before_upload')
    rewritten=json.loads(raw.decode('utf-8'))
    rewritten['driver']['path']='factory_driver.py'
    return {'config/factory.json':json.dumps(rewritten,ensure_ascii=False,indent=2,allow_nan=False).encode('utf-8'),
            'config/factory_driver.py':source}


def reviewed_brainco_overrides(path,expected_sha256=None):
    from .brainco_adapter import validate_brainco_config
    raw=Path(path).read_bytes()
    if expected_sha256 is not None and hashlib.sha256(raw).hexdigest()!=expected_sha256:
        raise ValueError('hand_calibration_changed_reload_file')
    value=json.loads(raw.decode('utf-8'))
    validate_brainco_config(value)
    return {'config/brainco.json':json.dumps(value,ensure_ascii=False,allow_nan=False).encode('utf-8')}


def validate_host(host,user):
    if not isinstance(host,str) or not re.fullmatch(r'[a-zA-Z0-9][a-zA-Z0-9.\-]{0,252}',host):
        raise ValueError('invalid_robot_host')
    if not isinstance(user,str) or not re.fullmatch(r'[a-zA-Z_][a-zA-Z0-9_\-]{0,31}',user):
        raise ValueError('invalid_robot_user')


def tcp_nodelay(connection):
    """Only tune an actual TCP socket; never treat Paramiko channels as sockets."""
    if not isinstance(connection,socket.socket) or connection.family not in (socket.AF_INET,socket.AF_INET6):
        return False
    connection.setsockopt(socket.IPPROTO_TCP,socket.TCP_NODELAY,1)
    return True


class Tunnel:
    def __init__(self, transport, remote_port=8878):
        class Handler(socketserver.BaseRequestHandler):
            def handle(self):
                channel=None;reader=None;finished=threading.Event()
                try:
                    tcp_nodelay(self.request)
                    channel=transport.open_channel('direct-tcpip',('127.0.0.1',remote_port),self.request.getpeername(),timeout=2)
                    if channel is None:return
                    self.request.settimeout(.5);channel.settimeout(.5)
                    def receive_remote():
                        try:
                            while not finished.is_set() and not self.server.stopping.is_set():
                                try:data=channel.recv(16384)
                                except socket.timeout:continue
                                if not data:break
                                self.request.sendall(data)
                        except (OSError,EOFError):pass
                        finally:
                            finished.set()
                            with suppress(OSError):self.request.shutdown(socket.SHUT_RDWR)
                    # Channel.recv uses a condition variable. Avoid select on
                    # Channel.fileno: Paramiko implements that on Windows with
                    # an extra TCP socket pair and one-byte wakeup packets.
                    reader=threading.Thread(target=receive_remote,daemon=True);reader.start()
                    while not finished.is_set() and not self.server.stopping.is_set():
                        try:data=self.request.recv(16384)
                        except socket.timeout:continue
                        if not data:break
                        channel.sendall(data)
                except OSError:
                    pass
                finally:
                    finished.set()
                    if channel:channel.close()
                    with suppress(OSError):self.request.shutdown(socket.SHUT_RDWR)
                    if reader:reader.join(1.)
        class Server(socketserver.ThreadingTCPServer):
            daemon_threads=True
            allow_reuse_address=False
        self.server=Server(('127.0.0.1',0),Handler)
        self.server.stopping=threading.Event()
        self.port=self.server.server_address[1]
        self.thread=threading.Thread(target=self.server.serve_forever,daemon=True)
        self.thread.start()

    def close(self):
        self.server.stopping.set()
        self.server.shutdown();self.server.server_close();self.thread.join(2)


class RobotDeployment:
    def __init__(self, root, known_hosts, confirm_host, log, overrides=None,runtime_mode='dry_run',hardware_backend='manufacturer',headset_transport='workstation',network_mode='auto',network_provider=None):
        if runtime_mode not in ('dry_run','hardware'):raise ValueError('invalid_robot_runtime_mode')
        self.root,self.known_hosts=Path(root),Path(known_hosts)
        self.confirm_host,self.log=confirm_host,log
        self.ssh=self.channel=self.tunnel=None
        self.remote=None
        self.overrides=overrides
        self.runtime_mode=runtime_mode
        if hardware_backend not in ('manufacturer','xarm'):raise ValueError('invalid_hardware_backend')
        self.hardware_backend=hardware_backend
        if headset_transport not in ('workstation','robot_usb'):raise ValueError('invalid_headset_transport')
        self.headset_transport=headset_transport;self.headset_pairing_code=None
        self.closing=threading.Event()
        self.station_exit=None
        if network_mode not in ('auto','wired'):raise ValueError('invalid_robot_network_mode')
        self.network_mode=network_mode;self.network_report=None
        self.network_provider=network_provider

    def verify_network(self):
        if self.network_mode!='wired':return None
        if self.network_provider is None:
            from .wired_network import WindowsWiredNetwork
            self.network_provider=WindowsWiredNetwork()
        report=self.network_provider.inspect(self._network_target)
        if not report.get('ready'):raise RuntimeError(report.get('reason','有线检查未通过'))
        if self.network_report is not None and (report['source_ipv4'],report['interface_index'])!=(self.network_report['source_ipv4'],self.network_report['interface_index']):
            raise RuntimeError('有线接口或源地址发生变化，请停止本项目后重新连接')
        self.network_report=dict(self.network_report or {},**report)
        return report

    def connect(self,host,user,password=''):
        validate_host(host,user)
        self._network_target=host
        report=self.verify_network()
        import base64
        import paramiko
        owner=self
        class Confirm(paramiko.MissingHostKeyPolicy):
            def missing_host_key(self,client,hostname,key):
                fingerprint='SHA256:'+base64.b64encode(hashlib.sha256(key.asbytes()).digest()).decode().rstrip('=')
                if not owner.confirm_host(hostname,fingerprint):
                    raise RuntimeError('robot_host_key_not_approved')
                client.get_host_keys().add(hostname,key.get_name(),key)
                owner.known_hosts.parent.mkdir(exist_ok=True,parents=True)
                client.save_host_keys(str(owner.known_hosts))
        client=paramiko.SSHClient()
        if self.known_hosts.exists():client.load_host_keys(str(self.known_hosts))
        client.set_missing_host_key_policy(Confirm())
        pinned=None
        try:
            options={}
            if report is not None:
                pinned=self.network_provider.connect(host,report=report)
                options['sock']=pinned
            client.connect(host,username=user,password=password or None,timeout=8,banner_timeout=8,auth_timeout=12,
                           allow_agent=True,look_for_keys=True,**options)
            if report is not None:
                self.verify_network()
                self.network_report=dict(self.network_report,ssh_source_ipv4=pinned.getsockname()[0],ssh_bound=True)
        except BaseException:
            client.close()
            if pinned:pinned.close()
            raise
        if self.closing.is_set():
            client.close()
            raise RuntimeError('deployment_cancelled')
        try:tcp_nodelay(getattr(client.get_transport(),'sock',None))
        except OSError as exc:
            self.log('SSH TCP_NODELAY 设置未完成：'+str(exc));client.close();raise
        self.ssh=client

    def command(self,command,timeout=900,*,stdin_data=None):
        channel=self.ssh.get_transport().open_session(timeout=8)
        channel.exec_command(command)
        if stdin_data is not None:
            channel.sendall(stdin_data);channel.shutdown_write()
        output=[];deadline=time.monotonic()+timeout
        try:
            while not channel.exit_status_ready() or channel.recv_ready() or channel.recv_stderr_ready():
                if self.closing.is_set():raise RuntimeError('deployment_cancelled')
                if time.monotonic()>deadline:raise TimeoutError('robot_command_timeout')
                for available,read in ((channel.recv_ready,channel.recv),(channel.recv_stderr_ready,channel.recv_stderr)):
                    if available():
                        value=read(8192).decode('utf-8',errors='replace')
                        output.append(value);self.log(value.rstrip())
                time.sleep(.03)
            code=channel.recv_exit_status()
            if code:raise RuntimeError('robot_command_failed:'+str(code))
            return ''.join(output)
        finally:channel.close()

    def deploy(self):
        if self.runtime_mode=='hardware' and self.hardware_backend=='manufacturer' and not {'config/factory.json','config/factory_driver.py'}<=set(self.overrides or {}):
            raise RuntimeError('硬件模式缺少已审查的厂家驱动 JSON 和源码，请先加载厂家配置。')
        system=self.command("python3 -c 'import json,platform,sys; print(json.dumps(dict(system=platform.system(),architecture=platform.machine(),python=list(sys.version_info[:3]))))'",30)
        observed=json.loads(system.strip().splitlines()[-1])
        if observed['system']!='Linux' or observed['architecture'] not in ('x86_64','aarch64') or observed['python'][:2]!=[3,12]:
            raise RuntimeError('需要 Ubuntu 24.04、Python 3.12、x86_64/aarch64；实际：'+json.dumps(observed))
        bundle=make_robot_bundle(self.root,self.overrides)
        digest=hashlib.sha256(bundle).hexdigest()
        sftp=self.ssh.open_sftp()
        try:
            home=sftp.normalize('.')
            if not home.startswith('/') or any(c in home for c in '\n\r\0'):
                raise ValueError('invalid_remote_home')
            relative='.local/share/tg3-teleop/releases/'+digest[:20]
            current=home
            for part in relative.split('/'):
                current+='/'+part
                try:sftp.stat(current)
                except FileNotFoundError:sftp.mkdir(current)
            self.remote=current
            archive=current+'/source.zip'
            with sftp.file(archive,'wb') as output:output.write(bundle)
        finally:sftp.close()
        # The staged path is absolute, version-specific and shell-quoted. The
        # installer checks SHA and rejects traversal/symlinks before extraction.
        script='''import hashlib,pathlib,zipfile
p=pathlib.Path(ARCHIVE)
assert hashlib.sha256(p.read_bytes()).hexdigest()==DIGEST, 'upload hash mismatch'
root=p.parent.resolve()
with zipfile.ZipFile(p) as z:
 for item in z.infolist():
  target=(root/item.filename).resolve()
  assert root in target.parents and not item.filename.startswith(('/', chr(92))) and chr(92) not in item.filename, 'invalid archive path'
  assert (item.external_attr>>16)&0o170000 != 0o120000, 'archive symlink'
 z.extractall(root)
'''.replace('ARCHIVE',repr(archive)).replace('DIGEST',repr(digest))
        self.command('python3 -c '+shlex.quote(script),45)
        self.command('cd '+shlex.quote(current)+' && python3 scripts/robot_environment.py',1800)
        return observed

    def capture_interfaces(self,seconds=8):
        """Observe only; do not import a vendor SDK or invoke control services."""
        if not self.remote or self.ssh is None:raise RuntimeError('deploy_before_interface_capture')
        if type(seconds) not in (int,float) or not 5<=seconds<=30:raise ValueError('invalid_readonly_capture_seconds')
        if not (self.root/'scripts/inspect_factory_readonly.py').is_file():
            raise RuntimeError('readonly_collector_missing_from_deployment_source')
        folder=(self.root/'.desktop/robot_reports').resolve()
        if self.root.resolve() not in folder.parents:raise ValueError('robot_report_directory_outside_project')
        folder.mkdir(parents=True,exist_ok=True)
        name='factory_readonly_'+uuid.uuid4().hex+'.json'
        local=folder/name;partial=folder/(name+'.part');remote=self.remote+'/artifacts/'+name
        shell=('set -e\nsource "$HOME/xos/setup.bash"\ncd '+shlex.quote(self.remote)+'\n'
            'if [ -f "$HOME/XARM/install/setup.bash" ]; then source "$HOME/XARM/install/setup.bash"; fi\n'
            'export PYTHONNOUSERSITE=1\n'
            'export LD_LIBRARY_PATH="$PWD/.venv/lib/python3.12/site-packages/cmeel.prefix/lib:${LD_LIBRARY_PATH:-}"\n'
            'exec .venv/bin/python -s scripts/inspect_factory_readonly.py --seconds '+str(seconds)+
            ' --output '+shlex.quote('artifacts/'+name))
        command_error=None
        try:self.command('bash -c '+shlex.quote(shell),timeout=180)
        except (RuntimeError,TimeoutError,OSError) as exc:command_error=str(exc)
        sftp=self.ssh.open_sftp()
        try:
            sftp.get_channel().settimeout(10)
            size=sftp.stat(remote).st_size
            if not 0<size<=32*1024*1024:raise ValueError('readonly_report_size_invalid')
            sftp.get(remote,str(partial))
            if partial.stat().st_size!=size:raise ValueError('readonly_report_download_incomplete')
            value=json.loads(partial.read_text(encoding='utf-8'))
            if (value.get('passive_only') is not True or value.get('hardware_motion_enabled') is not False
                or value.get('robot_commands_sent') is not False
                or value.get('motion_publishers_created')!=[] or value.get('motion_services_called')!=[]):
                raise ValueError('readonly_report_safety_scope_unconfirmed')
            partial.rename(local)
        finally:
            sftp.close();partial.unlink(missing_ok=True)
        graph=value.get('graph',{})
        summary=dict(passive_only=True,graph_topics=len(graph.get('topic_types',{})),
            arm_consumers=list(graph.get('arm_consumers',{})),
            parsed_feedback_present=value.get('parsed_feedback') is not None,
            error=value.get('error') or command_error,robot_commands_sent=False)
        return dict(path=str(local),ok=summary['error'] is None,summary=summary)

    def prepare_robot_usb(self,administrator_password=None):
        """Only prerequisites and authorization. Never starts factory/arm processes."""
        if self.headset_transport!='robot_usb':return None
        if not self.remote or self.ssh is None:raise RuntimeError('USB检查需要先部署本项目源码')
        cache=self.root/'usb_dependencies'
        if not (cache/'manifest.json').exists():cache=self.root/'.desktop/ubuntu_adb_noble_amd64'
        script='cd '+shlex.quote(self.remote)+' && /usr/bin/python3 scripts/robot_usb_preflight.py --wait 60'
        managed=self.runtime_mode=='hardware' and self.hardware_backend=='xarm'
        data=None
        if managed:
            if not isinstance(administrator_password,str) or not administrator_password or any(c in administrator_password for c in '\r\n\0'):
                raise RuntimeError('USB检查需要本次Ubuntu管理员密码，不会保存')
            if (cache/'manifest.json').exists():
                from .usb_preflight import verified_packages
                raw=(cache/'manifest.json').read_bytes();digest=hashlib.sha256(raw).hexdigest()
                paths,_=verified_packages(cache,digest)
                target=self.remote+'/usb_dependencies';sftp=self.ssh.open_sftp()
                try:
                    try:sftp.stat(target)
                    except FileNotFoundError:sftp.mkdir(target)
                    for path in [cache/'manifest.json',*map(Path,paths)]:
                        with sftp.file(target+'/'+path.name,'wb') as output:output.write(path.read_bytes())
                finally:sftp.close()
                script+=' --install-missing --packages '+shlex.quote(target)+' --manifest-sha '+shlex.quote(digest)
            self.log('正在以真机会话相同的管理员身份检查USB；请在头显允许机器人USB调试授权。')
            command="sudo -k -S -p '' /bin/bash -c "+shlex.quote(script)
            data=(administrator_password+'\n').encode()
        else:command='/bin/bash -c '+shlex.quote(script)
        text=self.command(command,timeout=300,stdin_data=data)
        reports=[]
        for line in text.splitlines():
            try:value=json.loads(line)
            except ValueError:continue
            if isinstance(value,dict) and value.get('event')=='robot_usb_preflight':reports.append(value)
        if not reports or reports[-1].get('ready') is not True:raise RuntimeError('机器人USB预检未确认就绪；厂家尚未接管')
        if managed and reports[-1].get('operator_uid')!=0:raise RuntimeError('USB预检身份与真机启动身份不一致')
        return reports[-1]

    def start(self,key,*,administrator_password=None):
        if self.closing.is_set():raise RuntimeError('deployment_cancelled')
        if not self.remote:raise RuntimeError('deploy_before_start')
        self.verify_network()
        managed=self.runtime_mode=='hardware' and self.hardware_backend=='xarm'
        if managed and (not isinstance(administrator_password,str) or not administrator_password or
                        any(c in administrator_password for c in '\r\n\0')):
            raise RuntimeError('XARM 接管需要本次 Ubuntu 管理员密码；不会保存到文件或命令行。')
        if not isinstance(key,str) or len(key)<32 or any(c in key for c in '\r\n\0'):raise ValueError('invalid_private_relay_key')
        self.channel=self.ssh.get_transport().open_session()
        command='bash '+shlex.quote(self.remote+'/scripts/start_robot_wireless.sh')
        if self.headset_transport=='robot_usb':command+=' --headset-usb-local'
        if self.runtime_mode=='hardware':
            if self.hardware_backend=='xarm':
                command+=' --hardware --xarm'
                if 'config/brainco.json' in (self.overrides or {}):
                    command+=' --hands-config '+shlex.quote(self.remote+'/config/brainco.json')
            else:command+=' --hardware --factory-config '+shlex.quote(self.remote+'/config/factory.json')
        if managed:command="sudo -k -S -p '' /bin/"+command
        self.channel.exec_command(command)
        if managed:self.channel.sendall((administrator_password+'\n').encode())
        self.channel.sendall((key+'\n').encode())
        data=b'';deadline=time.monotonic()+(100 if managed else 60)
        while time.monotonic()<deadline:
            if self.closing.is_set():raise RuntimeError('deployment_cancelled')
            if self.channel.recv_ready():
                data+=self.channel.recv(8192)
                while b'\n' in data:
                    line,data=data.split(b'\n',1)
                    text=line.decode('utf-8',errors='replace')
                    try:value=json.loads(text)
                    except ValueError:self.log(text);continue
                    if not isinstance(value,dict):self.log(text);continue
                    visible={k:v for k,v in value.items() if k!='headset_pairing_code'}
                    self.log(json.dumps(visible,ensure_ascii=False))
                    if value.get('event')=='station_failed':raise RuntimeError('机器人启动失败：'+str(value.get('error')))
                    if value.get('event')=='robot_station_started':
                        from . import __version__
                        if value.get('version')!=__version__ or value.get('protocol')!='tg3-native-1':
                            raise RuntimeError('机器人与电脑协议或版本不一致')
                        if value.get('runtime_mode')!=self.runtime_mode:
                            raise RuntimeError('机器人返回的运行模式与所选模式不一致')
                        if value.get('hardware_motion_enabled') is not False:
                            raise RuntimeError('机器人未确认启动时硬件输出关闭；已拒绝建立控制会话')
                        observed_transport=value.get('headset_transport','workstation')
                        if observed_transport!=self.headset_transport:raise RuntimeError('headset_transport_mismatch')
                        if self.headset_transport=='robot_usb':
                            pin=value.get('headset_pairing_code')
                            if not isinstance(pin,str) or not re.fullmatch('[0-9]{8}',pin):raise RuntimeError('robot_usb_pairing_code_missing')
                            self.headset_pairing_code=pin
                        self.tunnel=Tunnel(self.ssh.get_transport(),value['listen_port'])
                        threading.Thread(target=self._drain,daemon=True).start()
                        return self.tunnel.port
            if self.channel.recv_stderr_ready():self.log(self.channel.recv_stderr(8192).decode(errors='replace'))
            if self.channel.exit_status_ready():raise RuntimeError('机器人接收未启动；请看日志中的具体缺项')
            time.sleep(.05)
        raise RuntimeError('机器人启动等待超时')

    def _drain(self):
        channel=self.channel;tail=''
        while channel and not self.closing.is_set():
            try:
                for available,read in ((channel.recv_ready,channel.recv),(channel.recv_stderr_ready,channel.recv_stderr)):
                    while available():
                        raw=read(8192)
                        if not raw:break
                        value=raw.decode(errors='replace');tail=(tail+value)[-65536:];self.log(value)
                if channel.exit_status_ready():
                    # Preserve the first useful station failure instead of
                    # burying it under repeated HTTP reconnection failures.
                    code=channel.recv_exit_status();reason='station_exit:'+str(code)
                    for line in tail.splitlines():
                        try:value=json.loads(line)
                        except ValueError:continue
                        if isinstance(value,dict) and value.get('event')=='station_failed':
                            reason=str(value.get('error') or reason)
                    transport=self.ssh.get_transport() if self.ssh else None
                    transport_error=getattr(transport,'get_exception',lambda:None)()
                    self.station_exit=dict(exit_code=code,reason=reason,
                        ssh_transport_active=getattr(transport,'is_active',lambda:None)(),
                        ssh_error=str(transport_error)[:256] if transport_error else None,
                        channel_eof_received=getattr(channel,'eof_received',None),channel_closed=bool(channel.closed))
                    self.log('机器人接收程序已退出：'+reason+'；退出码 '+str(code)+'。需要重新启动本项目。')
                    self.log('连接退出诊断：'+json.dumps(self.station_exit,ensure_ascii=False))
                    return
                if channel.closed:
                    self.station_exit=dict(exit_code=None,reason='station_channel_closed_without_exit_status')
                    self.log('机器人接收会话已关闭，未收到进程退出码。');return
                time.sleep(.1)
            except (OSError,EOFError) as exc:
                self.station_exit=dict(exit_code=None,reason='station_channel_lost:'+str(exc))
                self.log('机器人接收会话中断：'+str(exc));return

    def close(self):
        self.closing.set()
        if self.tunnel:self.tunnel.close();self.tunnel=None
        if self.channel:
            with suppress(OSError):self.channel.shutdown_write()
            # EOF gives the station its own graceful cleanup/recording flush.
            deadline=time.monotonic()+(65 if self.runtime_mode=='hardware' and self.hardware_backend=='xarm' else 20)
            while not self.channel.exit_status_ready() and time.monotonic()<deadline:time.sleep(.05)
            self.channel.close();self.channel=None
        if self.ssh:self.ssh.close();self.ssh=None
