"""Drain child diagnostics to a bounded shared log, surviving its launcher.

Disk failures discard diagnostics and keep draining; no recording/controller API.
"""
import contextlib,os,subprocess,sys
from pathlib import Path

class RingLog:
    def __init__(self,path,budget=1024*1024):
        self.path=Path(path);self.half=budget//2;self.path.parent.mkdir(parents=True,exist_ok=True)
    def write(self,data):
        for offset in range(0,len(data),min(4096,self.half)):
            chunk=data[offset:offset+min(4096,self.half)]
            size=self.path.stat().st_size if self.path.exists() else 0
            if size+len(chunk)>self.half:
                os.replace(self.path,self.path.with_suffix(self.path.suffix+'.1'))
            with self.path.open('ab') as stream:stream.write(chunk)

@contextlib.contextmanager
def bounded_output(path):
    # Standalone stdlib helper has no dependencies on ROS, Placo or a venv.
    process=subprocess.Popen([sys.executable,'-s',str(Path(__file__).resolve()),str(path)],
        stdin=subprocess.PIPE,stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL,start_new_session=True)
    try:yield process.stdin
    finally:process.stdin.close()  # Child's duplicate remains until that child exits.

def main():
    writer=None
    try:writer=RingLog(sys.argv[1])
    except OSError:pass
    pending=b'';discard=False;last=None
    while True:
        data=sys.stdin.buffer.read1(4096)
        if not data:return
        for part in data.splitlines(keepends=True):
            if not discard:pending+=part
            if len(pending)>4096:pending=b'';discard=True
            if not part.endswith((b'\n',b'\r')):continue
            line=pending;pending=b'';discard=False
            # Do not store per-tick targets, joint dumps or ordinary SDK output.
            lowered=line.lower()
            if writer and line!=last and any(tag in lowered for tag in (
                    b'[error]',b'[warn',b'[fatal]',b'error:',b'exception',b'timeout',
                    b'connection lost',b'controller exited',b'handoff ready')):
                try:writer.write(line[:1500]+(b'\n' if not line.endswith(b'\n') else b''))
                except OSError:pass  # No buffering, retries or fallback file.
                last=line

if __name__=='__main__':main()
