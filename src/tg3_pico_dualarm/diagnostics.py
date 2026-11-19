"""Bounded fault/connection logs. This is not a trajectory recorder."""
import json,logging,logging.handlers
from pathlib import Path
import time,uuid

class DiagnosticsSink:
    """Recorder-compatible no-op for samples; only sparse events go to disk."""
    recording_failed=False
    samples_enabled=False
    steps=0
    def __init__(self,directory,metadata=None,*,budget=4*1024*1024):
        self.directory=Path(directory);self.events=0;self.last_error=None
        self.logger=logging.getLogger('tg3.diag.'+uuid.uuid4().hex)
        self.logger.propagate=False;self.logger.setLevel(logging.INFO)
        self.handler=None
        try:
            self.directory.mkdir(parents=True,exist_ok=True)
            self.handler=logging.handlers.RotatingFileHandler(self.directory/'events.log',
                maxBytes=budget//2,backupCount=1,encoding='utf-8')
            self.logger.addHandler(self.handler)
            self.event('session_started',time.monotonic())
        except OSError as exc:self.last_error=type(exc).__name__

    def append(self,record):
        pass  # No raw poses, image frames, targets, or per-tick runtime dumps.

    def event(self,reason,timestamp,details=None):
        self.events+=1
        value=dict(event=str(reason)[:1500],timestamp=timestamp)
        if isinstance(details,dict):
            value['details']={k:v for k,v in details.items() if k in (
                'reason','error','stage','input_timing','tick_interval_s','wait_s','isolation')}
        try:self.logger.info(json.dumps(value,ensure_ascii=True,allow_nan=False)[:4096])
        except (OSError,ValueError,TypeError) as exc:self.last_error=type(exc).__name__

    def report(self,name,value):
        # Keep only a bounded, sparse diagnostic summary, not a data archive.
        fields={k:value[k] for k in ('error','reason','issues','restart_required','hardware_motion_enabled') if k in value}
        self.event(name,time.monotonic(),dict(reason=json.dumps(fields,default=str)[:2000]))

    def close(self):
        if self.handler:
            self.logger.removeHandler(self.handler);self.handler.close();self.handler=None
