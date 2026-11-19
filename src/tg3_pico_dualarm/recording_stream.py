"""Bounded binary recording transport; no robot command interface."""
import collections,json,queue,struct,threading,time

MAX_HEADER=65536
MAX_PAYLOAD=16*1024*1024
MAX_BUFFER=64*1024*1024

def encode_packet(header,payload=b''):
    raw=json.dumps(header,separators=(',',':'),allow_nan=False).encode()
    if not 0<len(raw)<=MAX_HEADER or len(payload)>MAX_PAYLOAD:raise ValueError('recording_packet_too_large')
    return struct.pack('!II',len(raw),len(payload))+raw+payload

def read_exact(stream,length):
    out=bytearray()
    while len(out)<length:
        part=stream.read(length-len(out))
        if not part:raise EOFError('recording_stream_closed')
        out.extend(part)
    return bytes(out)

def read_packet(stream):
    header_size,payload_size=struct.unpack('!II',read_exact(stream,8))
    if not 0<header_size<=MAX_HEADER or payload_size>MAX_PAYLOAD:raise ValueError('recording_packet_too_large')
    header=json.loads(read_exact(stream,header_size))
    if not isinstance(header,dict):raise ValueError('invalid_recording_header')
    return header,read_exact(stream,payload_size)

class ByteQueue:
    def __init__(self,max_bytes=MAX_BUFFER,max_age=1.,clock=time.monotonic):
        self.max_bytes,self.max_age,self.clock=max_bytes,max_age,clock
        self.items=collections.deque();self.size=0;self.lock=threading.Condition();self.closed=False
    def put(self,header,payload):
        cost=len(payload)+len(json.dumps(header))+8
        with self.lock:
            if self.closed:raise RuntimeError('recording_queue_closed')
            if self.size+cost>self.max_bytes or (self.items and self.clock()-self.items[0][0]>self.max_age):
                raise RuntimeError('recording_bandwidth_or_storage_backlog')
            self.items.append((self.clock(),cost,header,payload));self.size+=cost;self.lock.notify()
    def get(self,timeout=.1):
        with self.lock:
            if not self.items:self.lock.wait(timeout)
            if not self.items:raise queue.Empty()
            stamp,cost,header,payload=self.items.popleft();self.size-=cost
            if self.clock()-stamp>self.max_age:raise RuntimeError('recording_backlog_too_old')
            return header,payload
    def close(self):
        with self.lock:self.closed=True;self.lock.notify_all()

class RateLimiter:
    def __init__(self,bytes_per_second,clock=time.monotonic,sleep=time.sleep):
        if not 65536<=bytes_per_second<=4*1024*1024:raise ValueError('invalid_recording_rate_limit')
        self.rate,self.clock,self.sleep=bytes_per_second,clock,sleep;self.next=self.clock()
    def wait(self,size):
        now=self.clock()
        if self.next>now:self.sleep(self.next-now)
        self.next=max(self.next,self.clock())+size/self.rate
