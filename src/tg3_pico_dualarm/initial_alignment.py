"""Evidence gate before creating an actuator publisher. No robot I/O."""
import math


class InitialOutputAlignment:
    def __init__(self,seed,now):
        self.seed=self._vector(seed)
        self.started=now;self.stable_since=None;self.last_observed=None
        self.raw=None;self.measured=None;self.samples=0
        self.tolerance=.004;self.stable_s=.1;self.timeout_s=.5

    @staticmethod
    def _vector(values):
        values=tuple(values)
        if len(values)!=14 or any(type(v) not in (int,float) or not math.isfinite(v) for v in values):
            raise ValueError('invalid_initial_alignment_vector')
        return values

    def observe(self,raw,measured,now):
        raw,measured=self._vector(raw),self._vector(measured)
        if now<self.started or (self.last_observed is not None and now<=self.last_observed):
            raise ValueError('initial_alignment_clock_not_advancing')
        if self.last_observed is not None and now-self.last_observed>.02:
            self.stable_since=None
        self.raw,self.measured=raw,measured;self.last_observed=now;self.samples+=1
        if max(abs(a-b) for a,b in zip(measured,self.seed))>self.tolerance:
            self.stable_since=None
            raise RuntimeError('initial_alignment_measured_pose_moved')
        if max(abs(a-b) for a,b in zip(raw,self.seed))>self.tolerance:
            self.stable_since=None
        elif self.stable_since is None:self.stable_since=now

    def ready(self,now):
        if now-self.started>self.timeout_s:raise RuntimeError('initial_raw_alignment_timeout')
        return bool(self.stable_since is not None and self.last_observed is not None
            and 0<=now-self.last_observed<=.02 and now-self.stable_since>=self.stable_s)

    def status(self):
        return dict(phase='observing_before_actuator_output',samples=self.samples,
            seed=list(self.seed),last_raw=None if self.raw is None else list(self.raw),
            last_measured=None if self.measured is None else list(self.measured),
            raw_error_rad=None if self.raw is None else max(abs(a-b) for a,b in zip(self.raw,self.seed)),
            measured_error_rad=None if self.measured is None else max(abs(a-b) for a,b in zip(self.measured,self.seed)),
            tolerance_rad=self.tolerance,required_stability_s=self.stable_s,
            started_at=self.started,last_observed_at=self.last_observed)
