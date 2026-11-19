"""Bounded in-memory latency distributions; never records poses or images."""
import collections,math,time

class TimingStats:
    def __init__(self,capacity=2048,clock=time.monotonic):
        self.values={};self.capacity=capacity;self.clock=clock;self.cached_at=None;self.cached={}
    def add(self,name,value):
        if not math.isfinite(value) or value<0:return
        self.values.setdefault(name,collections.deque(maxlen=self.capacity)).append(float(value))
    def snapshot(self):
        now=self.clock()
        if self.cached_at is not None and now-self.cached_at<1.:return self.cached
        self.cached_at=now;self.cached={}
        for name,values in self.values.items():
            ordered=sorted(values)
            def percentile(p):return ordered[min(len(ordered)-1,math.ceil(len(ordered)*p)-1)]
            self.cached[name]=dict(samples=len(ordered),p50_ms=percentile(.5)*1000,
                p95_ms=percentile(.95)*1000,p99_ms=percentile(.99)*1000,max_ms=ordered[-1]*1000)
        return self.cached
