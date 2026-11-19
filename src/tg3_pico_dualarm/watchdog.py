"""Independent software watchdog for the dry-run command boundary."""
import threading
import time


class CommandWatchdog:
    def __init__(self,on_expired,timeout=.15,monotonic=time.monotonic,*,active=True):
        if not 0<timeout<=1:raise ValueError('invalid watchdog timeout')
        if type(active) is not bool:raise ValueError('watchdog active must be boolean')
        self.clock,self.timeout,self.on_expired=monotonic,timeout,on_expired
        self._stamp=monotonic();self._expired=False;self._stop=threading.Event()
        self._active=active
        self._lock=threading.Lock();self.thread=None

    def activate(self):
        with self._lock:
            if self._expired:raise RuntimeError('expired_watchdog_requires_new_station')
            if not self._active:
                self._stamp=self.clock();self._active=True

    def deactivate(self):
        # Observe a deadline crossed during the final transition before
        # ending the lease. Deactivation can never clear an expired latch.
        if self.check():return False
        with self._lock:self._active=False
        return True

    def beat(self):
        with self._lock:self._stamp=self.clock()

    def check(self):
        with self._lock:
            if not self._active:return self._expired
            expired=self.clock()-self._stamp>self.timeout
            notify=expired and not self._expired
            if expired:self._expired=True
        if notify:self.on_expired()
        return self._expired

    def start(self):
        if self.thread is not None:raise RuntimeError('watchdog already started')
        def worker():
            while not self._stop.wait(min(.02,self.timeout/3)):self.check()
        self.thread=threading.Thread(target=worker,daemon=True);self.thread.start()

    def close(self):
        self._stop.set()
        if self.thread:self.thread.join(timeout=1)
