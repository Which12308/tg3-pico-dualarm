"""Fresh operator intentions executed through the manufacturer's raw limiter.

Six motor measurements/commands are authoritative. A displayed scalar closure
is only a projection of the commanded motor vector onto the calibration line.
No motor feedback limit is interpreted as contact, force or grasp success.
"""
from . import safety as motion_policy
import time

from .input_validation import validate_frame
from .input_policy import NORMAL
from .manufacturer import ManufacturerAdapter


class ManufacturerGrasp:
    def __init__(self,robot,*,clock=time.monotonic,policy=NORMAL):
        from .xarm_adapter import XArmAdapter
        if not isinstance(robot,(ManufacturerAdapter,XArmAdapter)):raise TypeError('manufacturer_adapter_required')
        self.robot,self.clock=robot,clock
        self.policy=policy
        self.initialized=False;self.session=None;self.sequence=-1;self.source_time=None
        self.ready={side:False for side in robot.profiles}
        self.opening={side:False for side in robot.profiles}
        self.source_ids={side:None for side in robot.profiles}
        self.open_pressed={side:False for side in robot.profiles}
        self.reasons={side:'measured_initialization_required' for side in robot.profiles}

    def _cancel(self,reason):
        for side in self.ready:
            self.ready[side]=False;self.opening[side]=False;self.open_pressed[side]=False
            self.reasons[side]=reason

    def initialize(self):
        if not self.robot.hardware_writes_enabled:raise RuntimeError('hardware_ack_required_before_hand_initialization')
        self.robot.measured_hand_initialization()  # Validates fresh six-channel measurements.
        self.initialized=True;self.session=None;self.sequence=-1;self.source_time=None
        self.source_ids=dict.fromkeys(self.ready)
        self._cancel('fresh_neutral_hand_input_required')
        return self._status()

    def invalidate(self):
        self.initialized=False
        self._cancel('hand_input_invalidated_hold_last_command')
        return self._status()

    def _status(self,sent=(),fault=None):
        with self.robot.lock:
            snapshot=self.robot.feedback.snapshot(require_hands=self.robot.profiles)
            commanded={side:tuple(self.robot.hand_positions.get(side,())) for side in self.ready}
            measured={side:(snapshot['hands'][side]['feedback'] or {}).get('positions') for side in self.ready}
            diagnostics={side:dict(self.robot.hand_diagnostics.get(side,{})) for side in self.ready}
        projection={}
        for side,values in commanded.items():
            profile=self.robot.profiles[side]
            delta=[b-a for a,b in zip(profile.opened,profile.closed)]
            denominator=sum(v*v for v in delta)
            projection[side]=(max(0.,min(1.,sum((v-a)*d for v,a,d in zip(values,profile.opened,delta))/denominator))
                if len(values)==6 and denominator>0 else None)
        return dict(closure=projection,closure_is_projection=True,ready=self.ready.copy(),
            reason=self.reasons.copy(),intent={side:'open' if self.opening[side] else None for side in self.ready},
            commanded_motor_positions=commanded,measured_motor_positions=measured,
            motor_diagnostics=diagnostics,measurement_fresh=snapshot['ready'],
            hardware_hand_commands_sent=bool(sent),hardware_hand_sides=list(sent),fault=fault)

    def step(self,frame):
        if not self.initialized:return self._status()
        if not self.robot.hardware_writes_enabled:
            self._cancel('hardware_output_disabled');self.initialized=False
            return self._status()
        try:validate_frame(frame,self.clock(),self.policy)
        except (ValueError,TypeError):
            self._cancel('invalid_or_stale_hand_input_hold')
            return self._status()
        if not motion_policy.LIMITS_MANAGED_EXTERNALLY and not frame.focused:
            self._cancel('unfocused_hand_input_hold');return self._status()
        if frame.session!=self.session:
            self._cancel('new_session_requires_neutral')
            self.session=frame.session;self.sequence=-1;self.source_time=None
            self.source_ids=dict.fromkeys(self.ready)
        if not motion_policy.LIMITS_MANAGED_EXTERNALLY and frame.sequence==self.sequence:
            return self._status()  # No repeated motor output for the same packet.
        if not motion_policy.LIMITS_MANAGED_EXTERNALLY and (frame.sequence<self.sequence or (self.source_time is not None and frame.source_time_s<=self.source_time)):
            self._cancel('nonadvancing_hand_input_hold');return self._status()
        self.sequence,self.source_time=frame.sequence,frame.source_time_s
        sent=[]
        for side in self.ready:
            item=getattr(frame,side)
            if self.source_ids[side]!=item.source_id:
                self.ready[side]=False;self.opening[side]=False;self.open_pressed[side]=False
                self.source_ids[side]=item.source_id
            if motion_policy.LIMITS_MANAGED_EXTERNALLY:self.ready[side]=True
            if not item.connected or not item.tracked or item.pose is None or (not motion_policy.LIMITS_MANAGED_EXTERNALLY and item.release_required):
                self.ready[side]=False;self.opening[side]=False;self.open_pressed[side]=False
                self.reasons[side]='tracking_lost_fresh_neutral_required';continue
            if item.neutral:self.ready[side]=True
            button=('Y' if side=='left' else 'B') in item.buttons
            rising=button and not self.open_pressed[side]
            self.open_pressed[side]=button
            # A fresh open press is an explicit opening intention, but cannot
            # bypass a lost-tracking/reconnect neutral gate.
            if rising and self.ready[side]:self.opening[side]=True
            if self.opening[side]:
                operation,closure='open',0.
                self.reasons[side]='opening_to_measured_calibration'
            elif self.ready[side] and not button and item.trigger>.1:
                operation,closure='close',(item.trigger-.1)/.9
                self.reasons[side]='closing_with_manufacturer_raw_rate'
            else:
                self.reasons[side]='holding_motor_command' if self.ready[side] else 'fresh_neutral_hand_input_required'
                continue
            result=self.robot.submit_hand(side,closure,intent_stamp=frame.received_at,operation=operation)
            if not result.accepted:
                if result.reason=='no_new_hand_intent':continue
                self._cancel('manufacturer_hand_output_rejected')
                return self._status(sent,fault=result.reason)
            sent.append(side)
            with self.robot.lock:
                diag=dict(self.robot.hand_diagnostics[side])
                if diag['feedback_limited']:self.reasons[side]='limited_by_measured_motor_feedback_corridor'
                if operation=='open':
                    opened=self.robot.profiles[side].opened
                    if (tuple(diag['commanded_positions'])==tuple(opened)
                            and all(abs(v-g)<=.5 for v,g in zip(diag['measured_positions'],opened))):
                        self.opening[side]=False;self.ready[side]=False
                        self.reasons[side]='measured_open_motor_target_reached_release_inputs'
            if button:self.ready[side]=False
        return self._status(sent)
