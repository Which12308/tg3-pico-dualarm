"""Independent PC recording worker and nonblocking UI supervisor.

Importing/constructing these objects creates neither data files nor a process.
Secrets travel in memory over spawn IPC / SSH stdin, never argv or files.
"""
import json,multiprocessing as mp,queue,shlex,threading,time
from .recording_stream import ByteQueue,read_packet
from .recording_writer import DatasetWriter,benchmark,check_destination

def connect(config):
    import paramiko
    client=paramiko.SSHClient();client.load_host_keys(config['known_hosts'])
    client.set_missing_host_key_policy(paramiko.RejectPolicy())
    try:
        client.connect(config['host'],username=config['user'],password=config.get('password') or None,
            timeout=8,banner_timeout=8,auth_timeout=8,allow_agent=True,look_for_keys=True)
        return client
    except BaseException:client.close();raise

def exporter_command(remote,listing=False):
    from pathlib import PurePosixPath
    path=PurePosixPath(remote)
    if not path.is_absolute() or '..' in path.parts:raise ValueError('invalid_deployed_release_path')
    script=('set -e\nsource /home/ubuntu/xos/setup.bash\n'
            'source /home/ubuntu/XARM/install/setup.bash\ncd '+shlex.quote(str(path))+
            '\nexport PYTHONPATH="$PWD/src${PYTHONPATH:+:$PYTHONPATH}"\n'
            'exec .venv/bin/python -s -m tg3_pico_dualarm.recording_exporter'+(' --list' if listing else ''))
    return '/bin/bash -c '+shlex.quote(script)

def discover(config):
    client=connect(config)
    try:
        stdin,stdout,stderr=client.exec_command(exporter_command(config['remote'],True),timeout=12)
        stdin.close();raw=stdout.read(1024*1024);err=stderr.read(4096)
        if stdout.channel.recv_exit_status():raise RuntimeError('recording_discovery_failed:'+err.decode(errors='replace'))
        value=json.loads(raw)
        if not isinstance(value,list):raise ValueError('invalid_recording_catalogue')
        return value
    finally:client.close()

def worker(config,stop,status):
    client=channel=writer=None;error=None;started=time.monotonic();finished=threading.Event()
    def report(state,**extra):
        value=dict(state=state,duration_s=time.monotonic()-started,**extra)
        try:status.put_nowait(value)
        except queue.Full:pass
    try:
        report('checking');write_rate=benchmark(config['destination'])
        rate=int(config.get('rate',1024*1024))
        if write_rate<rate*2:raise RuntimeError('destination_sustained_write_too_slow')
        client=connect(config);channel=client.get_transport().open_session(timeout=8)
        channel.settimeout(3.);channel.exec_command(exporter_command(config['remote']))
        def drain_stderr():
            try:
                while channel.recv_stderr(4096):pass
            except Exception:pass
        threading.Thread(target=drain_stderr,daemon=True).start()
        channel.sendall((json.dumps(dict(topics=config['topics'],token=config['token'],rate=rate,port=8878))+'\n').encode())
        source=channel.makefile('rb');probe_started=time.monotonic();probe_bytes=0
        while True:
            if stop.is_set():raise RuntimeError('recording_start_cancelled')
            head,data=read_packet(source)
            if head['kind']=='probe':probe_bytes+=len(data)
            elif head['kind']=='probe_end':break
            elif head['kind']=='error':raise RuntimeError(head['reason'])
            else:raise ValueError('invalid_recording_probe')
        network_rate=probe_bytes/max(.001,time.monotonic()-probe_started)
        if network_rate<rate*.85:raise RuntimeError('recording_network_cannot_sustain_selected_limit')
        channel.sendall(b'proceed\n')
        if stop.is_set():raise RuntimeError('recording_start_cancelled')
        writer=DatasetWriter(config['destination'],dict(topics=config['topics'],rate_limit_bytes_s=rate,
            write_probe_bytes_s=write_rate,network_probe_bytes_s=network_rate,
            calibration=config.get('calibration',{}),software=config.get('software'),
            time_basis='ROS source/header and Unix wall receive timestamps; monotonic clocks are host-local'))
        buffer=ByteQueue();reader_error=[];ended=threading.Event()
        def receive():
            try:
                while True:
                    head,data=read_packet(source)
                    if head['kind']=='error':raise RuntimeError(head['reason'])
                    if head['kind']=='end':ended.set();return
                    buffer.put(head,data)
            except Exception as exc:reader_error.append(str(exc))
            finally:finished.set()
        threading.Thread(target=receive,daemon=True).start()
        stopping_at=None;last_report=0.
        while True:
            if stop.is_set() and stopping_at is None:
                report('stopping',path=str(writer.directory));stopping_at=time.monotonic();channel.shutdown_write()
            if reader_error:raise RuntimeError(reader_error[0])
            if finished.is_set() and not buffer.items:
                if not ended.is_set():raise RuntimeError('recording_stream_ended_without_completion')
                break
            if stopping_at and time.monotonic()-stopping_at>6:raise RuntimeError('recording_stop_flush_timeout')
            try:head,data=buffer.get(.1)
            except queue.Empty:
                check_destination(writer.root,writer.device);continue
            if head['kind']=='ready':report('recording',path=str(writer.directory),bytes=writer.total)
            else:writer.write(head,data)
            if time.monotonic()-last_report>.5:
                last_report=time.monotonic();report('stopping' if stopping_at else 'recording',
                    path=str(writer.directory),bytes=writer.size_bytes(),free_bytes=check_destination(writer.root,writer.device))
        writer.close();report('off',path=str(writer.directory),bytes=writer.size_bytes(),complete=True)
    except Exception as exc:
        error=str(exc)[:1200]
        if writer:
            try:writer.close(error)
            except Exception:pass  # Missing USB cannot hold an updated receipt. UI still reports incomplete.
        report('error',reason=error,incomplete=True,path=str(writer.directory) if writer else config['destination'])
    finally:
        finished.set()
        if channel:channel.close()
        if client:client.close()

class ManualRecording:
    def __init__(self,context=None):
        self.context=context or mp.get_context('spawn');self.process=None;self.stop_event=None;self.queue=None
        self.status=dict(state='off');self.stop_at=None;self.last_update=0.
    @property
    def active(self):return bool(self.process and self.process.is_alive())
    def start(self,config):
        if self.active:raise RuntimeError('recording_already_active')
        if self.process:self.process.join(0)
        if self.queue:self.queue.close()
        self.queue=self.context.Queue(maxsize=8);self.stop_event=self.context.Event();self.stop_at=None
        self.process=self.context.Process(target=worker,args=(config,self.stop_event,self.queue),daemon=True)
        self.status=dict(state='checking');self.last_update=time.monotonic();self.process.start()
    def stop(self):
        if self.active:
            self.stop_event.set();self.stop_at=self.stop_at or time.monotonic()
            self.status=dict(self.status,state='stopping')
    def poll(self):
        if self.queue:
            while True:
                try:self.status=self.queue.get_nowait();self.last_update=time.monotonic()
                except queue.Empty:break
        if self.active and (self.stop_at and time.monotonic()-self.stop_at>12 or time.monotonic()-self.last_update>25):
            # Kill ONLY our writer child; control/station/relay have no dependency on it.
            self.process.terminate();self.status=dict(self.status,state='error',incomplete=True,reason='recording_worker_unresponsive')
        elif self.process and not self.active and self.status['state'] not in ('off','error'):
            self.status=dict(self.status,state='error',incomplete=True,reason='recording_worker_exited')
        return self.status
    def close(self):
        self.stop()
        if self.process:
            self.process.join(8)
            if self.process.is_alive():self.process.terminate();self.process.join(2)
        if self.queue:self.queue.close();self.queue=None
