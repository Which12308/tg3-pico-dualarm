"""Direct destination writer. Never chooses or falls back to another volume."""
import json,os,shutil,tempfile,time,uuid
from pathlib import Path

RESERVE=512*1024*1024
SEGMENT=1024**3

def check_destination(directory,device=None,reserve=RESERVE):
    path=Path(directory)
    if not path.is_dir() or path.is_symlink():raise OSError('recording_destination_disappeared')
    if device is not None and path.stat().st_dev!=device:raise OSError('recording_destination_volume_changed')
    free=shutil.disk_usage(path).free
    if free<reserve:raise OSError('recording_destination_space_low')
    return free

def benchmark(directory,size=16*1024*1024):
    """Only called by an explicit Start. The temporary probe stays on that volume."""
    path=Path(directory).resolve(strict=True);check_destination(path)
    start=time.monotonic();name=None
    try:
        with tempfile.NamedTemporaryFile(dir=path,prefix='.tg3_write_probe_',delete=False) as stream:
            name=Path(stream.name)
            block=os.urandom(min(size,1024*1024))
            remaining=size
            while remaining:
                part=block[:remaining];stream.write(part);remaining-=len(part)
            stream.flush();os.fsync(stream.fileno())
        return size/max(.001,time.monotonic()-start)
    finally:
        if name is not None:
            try:name.unlink()
            except FileNotFoundError:pass

class DatasetWriter:
    def __init__(self,directory,metadata,*,segment_bytes=SEGMENT,reserve=RESERVE):
        from mcap.writer import Writer  # Required before creating an episode.
        self.Writer=Writer;self.root=Path(directory).resolve(strict=True);self.device=self.root.stat().st_dev
        self.reserve=reserve;check_destination(self.root,self.device,reserve)
        self.directory=self.root/('tg3_'+time.strftime('%Y%m%d_%H%M%S')+'_'+uuid.uuid4().hex[:8])
        self.directory.mkdir();self.segment_bytes=segment_bytes;self.index=0;self.total=0
        self.schemas={};self.channels={};self.file=None;self.writer=None;self.jsonfile=None;self.metadata=metadata
        self.closed=False;self.started=time.monotonic();self.last_check=0.
        self._manifest('recording')
        self._segment()

    def _manifest(self,state,**extra):
        check_destination(self.root,self.device,self.reserve if state=='recording' else 0)
        value=dict(self.metadata,state=state,incomplete=state!='complete',bytes=self.total,segments=self.index,**extra)
        # Write directly on the chosen destination; never use a system-temp fallback.
        path=self.directory/'session.json';temporary=self.directory/'session.json.tmp'
        temporary.write_text(json.dumps(value,ensure_ascii=False,allow_nan=False,indent=2),encoding='utf-8')
        os.replace(temporary,path)

    def _finish_segment(self):
        if self.writer:self.writer.finish();self.writer=None
        for stream in (self.file,self.jsonfile):
            if stream:stream.flush();os.fsync(stream.fileno());stream.close()
        self.file=self.jsonfile=None

    def _segment(self):
        self._finish_segment();self.index+=1;self.segment_start=self.total
        self.file=(self.directory/f'ros_{self.index:04d}.mcap').open('xb')
        self.writer=self.Writer(self.file);self.writer.start(profile='ros2',library='tg3-manual-recorder')
        self.jsonfile=(self.directory/f'controller_events_{self.index:04d}.jsonl').open('x',encoding='utf-8')
        self.channels={}
        for topic,(typ,schema) in self.schemas.items():self._register(topic,typ,schema)

    def _register(self,topic,typ,schema):
        sid=self.writer.register_schema(typ,'ros2msg',schema)
        self.channels[topic]=self.writer.register_channel(topic,'cdr',sid)

    def write(self,header,payload):
        check_destination(self.root,self.device,self.reserve)
        kind=header['kind']
        if kind=='schema':
            topic=header['topic'];self.schemas[topic]=(header['type'],payload);self._register(topic,header['type'],payload);return
        if kind not in ('ros','sample','event'):raise ValueError('unexpected_recording_packet')
        if self.total-self.segment_start+len(payload)>self.segment_bytes and self.total>self.segment_start:self._segment()
        if kind=='ros':
            stamp=int(header['timestamp_ns']);self.writer.add_message(self.channels[header['topic']],stamp,payload,
                int(header.get('publish_time_ns') or stamp))
        else:
            value=dict(kind=kind,received_time_ns=time.time_ns(),source=json.loads(payload))
            self.jsonfile.write(json.dumps(value,ensure_ascii=False,allow_nan=False)+'\n')
        self.total+=len(payload)
        if time.monotonic()-self.last_check>1.:
            self.last_check=time.monotonic();self.jsonfile.flush();self.file.flush()

    def close(self,error=None):
        if self.closed:return
        self.closed=True
        try:
            self._finish_segment();self._manifest('error' if error else 'complete',reason=error)
        finally:
            for stream in (self.file,self.jsonfile):
                if stream:
                    try:stream.close()
                    except OSError:pass

    def size_bytes(self):
        return sum(p.stat().st_size for p in self.directory.iterdir() if p.is_file())
