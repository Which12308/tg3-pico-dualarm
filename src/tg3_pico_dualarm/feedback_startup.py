"""Bounded startup discovery of every required feedback stream.

This is used only before output. Runtime freshness remains in
ManufacturerFeedback and the actuator guards, with no extended timeout.
"""
import json
import time


class FeedbackStartupGate:
    def __init__(self,feedback,*,require_hands=False,guard=None,clock=time.monotonic,timeout=10.,stable_s=.2):
        if not 0<timeout<=15. or not 0<=stable_s<timeout:raise ValueError('invalid_startup_feedback_wait')
        self.feedback,self.require_hands,self.guard=feedback,require_hands,guard
        self.clock,self.timeout,self.stable_s=clock,timeout,stable_s
        self.started=clock();self.stable_since=None;self.ready=False;self.last=None;self.guard_error=None

    def poll(self):
        now=self.clock();self.last=self.feedback.snapshot(require_hands=self.require_hands)
        self.guard_error=None
        if self.guard:
            try:self.guard()
            except RuntimeError as exc:self.guard_error=str(exc)
        valid=self.last['ready'] and self.guard_error is None
        if valid:
            if self.stable_since is None:self.stable_since=now
        else:self.stable_since=None
        self.ready=bool(valid and now-self.stable_since>=self.stable_s)
        if now-self.started>self.timeout:
            self.ready=False
            raise RuntimeError('startup_feedback_discovery_timeout:'+json.dumps(self.status(),ensure_ascii=False))
        return self.ready

    def status(self):
        return dict(ready=self.ready,elapsed_s=self.clock()-self.started,required_hands=self.require_hands,
            issues=[] if self.last is None else self.last['issues'],guard_error=self.guard_error,
            feedback_streams={} if self.last is None else self.last.get('feedback_streams',{}))
