"""One bounded operator request, with diagnostic IDs; no hardware authority."""
import threading


class MotionRequests:
    def __init__(self,enabled):
        self.enabled=enabled;self.lock=threading.Lock();self.pending=None;self.number=0

    def enqueue(self,value):
        with self.lock:
            if value['operation']=='disable':self.pending=None
            if self.pending is not None:raise ValueError('motion_request_already_pending')
            self.number+=1;self.pending=dict(value,request_number=self.number)
            return dict(accepted=True,queued=True,pending=True,operation=value['operation'],
                request_number=self.number,hardware_motion_enabled=bool(self.enabled()))

    def pop(self):
        with self.lock:
            value,self.pending=self.pending,None
            return value
