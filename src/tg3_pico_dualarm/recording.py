"""Timestamped observations and applied dry-run actions, never implied hardware."""
from dataclasses import asdict,is_dataclass
from datetime import datetime,timezone
import json
from pathlib import Path
import uuid
import threading
import numpy as np


def serializable(value):
    if is_dataclass(value): return serializable(asdict(value))
    if isinstance(value,np.ndarray): return value.tolist()
    if isinstance(value,np.generic): return value.item()
    if isinstance(value,dict): return {str(k):serializable(v) for k,v in value.items()}
    if isinstance(value,(tuple,list)): return [serializable(v) for v in value]
    return value


class SessionRecorder:
    def __init__(self,directory,metadata):
        self.directory=Path(directory)/(datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')+'_'+uuid.uuid4().hex[:8])
        self.directory.mkdir(parents=True,exist_ok=False)
        self.metadata=dict(schema='tg3_pico_dualarm_v1',clock='host_monotonic_seconds',
                           camera_frames_included=False,**metadata)
        (self.directory/'metadata.json').write_text(json.dumps(serializable(self.metadata),ensure_ascii=False,indent=2,allow_nan=False),encoding='utf-8')
        self._file=(self.directory/'steps.jsonl').open('x',encoding='utf-8')
        self._write_lock=threading.RLock()
        self.steps=0
        self.events=0;self.terminal_reason=None;self.recording_failed=False
        self.first_control_failure=None

    def append(self,record):
        line=json.dumps(serializable(record),ensure_ascii=False,separators=(',',':'),allow_nan=False)
        with self._write_lock:
            self._file.write(line+'\n');self.steps+=1
            if self.steps%50==0:self._file.flush()

    def close(self):
        with self._write_lock:
            if self._file.closed:return
            self._file.flush();self._file.close()
            (self.directory/'summary.json').write_text(json.dumps(dict(steps=self.steps,events=self.events,
                closed_cleanly=not self.recording_failed,terminal_reason=self.terminal_reason,
                recording_failed=self.recording_failed,first_control_failure=self.first_control_failure),
                ensure_ascii=False,allow_nan=False),encoding='utf-8')

    def event(self,reason,timestamp,details=None):
        record=dict(event='disarm',reason=str(reason),timestamp=timestamp,robot_commands_sent=False)
        if details is not None:record['details']=details
        self.append(record)
        self.terminal_reason=str(reason)
        if self.first_control_failure is None and str(reason).startswith((
                'control_failed:','control_tick_overrun','command_watchdog_expired','recording_failed')):
            # Snapshot after successful serialization; later cleanup must not
            # hide the actual initiating failure behind "station_stopped".
            self.first_control_failure=serializable(record)
        self.events+=1
