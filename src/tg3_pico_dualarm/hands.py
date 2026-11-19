"""Independent continuous-grasp state machine and calibrated BrainCo format.

This module never imports the old bottle controller and never creates a ROS
publisher. A six-channel manufacturer's calibration is required for motors.
"""
from dataclasses import dataclass
import math
from .input_validation import validate_frame
from .input_policy import NORMAL

SIDES = ('left', 'right')


class ContinuousHands:
    """Trigger is an absolute closing request with a retained high-water mark.

    Trigger release retains grasp. Y/B request open and win over trigger.
    After loss/reconnect, the trigger must be released before it can close
    again; the retained grasp is not automatically opened or strengthened.
    """
    def __init__(self, closing_rate=.5, opening_rate=.8,*,policy=NORMAL):
        self.policy=policy
        self.rates = closing_rate, opening_rate
        if any(type(v) not in (int, float) or not math.isfinite(v) or not 0 < v <= 2 for v in self.rates):
            raise ValueError('invalid_hand_rate')
        self.closure = dict.fromkeys(SIDES, 0.)
        self.goal = dict.fromkeys(SIDES, 0.)
        self.ready = dict.fromkeys(SIDES, False)
        self.session = None
        self.source_ids = dict.fromkeys(SIDES, None)
        self.opening = dict.fromkeys(SIDES, False)

    def invalidate(self):
        self.ready = dict.fromkeys(SIDES, False)
        # Cancel any not-yet-executed closure ramp. Do not open a held object.
        self.goal = self.closure.copy()
        self.opening = dict.fromkeys(SIDES, False)

    def initialize_from_measured_projection(self,measurements):
        # A scalar is only a fit to the six measured channels. The hardware
        # adapter separately keeps the original six values and raw rate limit.
        for side,item in measurements.items():
            if side not in SIDES or item.get('closure_is_projection') is not True:
                raise ValueError('explicit_measured_hand_projection_required')
            value=item.get('closure_projection')
            if type(value) not in (float,int) or not math.isfinite(value) or not 0<=value<=1:
                raise ValueError('invalid_measured_hand_projection')
            self.closure[side]=float(value)
        self.invalidate()

    def update(self, frame, now, dt):
        if type(dt) not in (int, float) or not math.isfinite(dt) or not 0 < dt <= .1:
            self.invalidate()
            return self.status('invalid_hand_tick')
        valid = bool(frame and frame.focused and 0 <= now-frame.received_at+frame.source_age_s <= self.policy.total_age_s)
        if frame is not None:
            try:validate_frame(frame,now,self.policy)
            except (ValueError,TypeError):valid=False
        if not valid:
            self.invalidate()
            return self.status('input_lost_hold_grasp')
        if self.session != frame.session:
            self.invalidate()
            self.session = frame.session
        reasons = {}
        for side in SIDES:
            item = getattr(frame, side)
            if self.source_ids[side] != item.source_id:
                self.ready[side] = False
                self.goal[side] = self.closure[side]
                self.opening[side] = False
                self.source_ids[side] = item.source_id
            if not item.connected or not item.tracked or item.release_required:
                self.ready[side] = False
                self.goal[side] = self.closure[side]
                self.opening[side] = False
                reasons[side] = 'release_inputs_after_tracking_loss'
                continue
            if not self.ready[side]:
                if item.neutral:
                    self.ready[side] = True
            open_button = 'Y' if side == 'left' else 'B'
            if open_button in item.buttons:
                self.goal[side] = 0.
                self.opening[side] = True
                # Require released trigger after opening to avoid reclosing
                # while the operator is still squeezing it.
                self.ready[side] = False
            elif not self.opening[side] and self.ready[side] and item.trigger > .1:
                request = (item.trigger-.1)/.9
                self.goal[side] = max(self.closure[side], request)
            elif not self.opening[side]:
                self.goal[side] = self.closure[side]
            delta = self.goal[side]-self.closure[side]
            rate = self.rates[0 if delta >= 0 else 1]
            self.closure[side] += max(-rate*dt, min(rate*dt, delta))
            if self.closure[side] <= 0.:self.opening[side] = False
            reasons[side] = 'opening' if delta < 0 else 'closing' if delta > 0 else 'holding'
        return self.status(reasons)

    def status(self, reason):
        return dict(closure=self.closure.copy(), goal=self.goal.copy(),
                    ready=self.ready.copy(), reason=reason, hardware_hand_commands_sent=False)


@dataclass(frozen=True)
class HandCalibration:
    opened: tuple
    closed: tuple
    lower: tuple
    upper: tuple
    speed: tuple
    current_limit: tuple
    evidence: str

    def __post_init__(self):
        for name in ('opened', 'closed', 'lower', 'upper', 'speed', 'current_limit'):
            values = tuple(getattr(self, name))
            if len(values) != 6 or any(type(v) is not int for v in values):
                raise ValueError('six_integer_manufacturer_values_required:'+name)
            object.__setattr__(self, name, values)
        if not isinstance(self.evidence, str) or not self.evidence.strip():
            raise ValueError('manufacturer_hand_calibration_evidence_required')
        for a,b,low,high,speed,current in zip(self.opened,self.closed,self.lower,self.upper,self.speed,self.current_limit):
            # This profile is emitted in position-only mode 1. The official
            # six-channel example omits speed/current, leaving both zero;
            # factory software does likewise. They are not protective limits.
            if not low <= a <= high or not low <= b <= high or speed < 0 or current < 0:
                raise ValueError('invalid_manufacturer_hand_limits')

    def positions(self, closure):
        if type(closure) not in (int,float) or not math.isfinite(closure) or not 0 <= closure <= 1:
            raise ValueError('invalid_hand_closure')
        return tuple(round(a+closure*(b-a)) for a,b in zip(self.opened,self.closed))


class BrainCoAdapter:
    """Format only the checked SetMotorMulti schema; ownership is external.

    Motor order, endpoints, speed and current units must come from this hand's
    manufacturer configuration, independently for each side. No
    current-to-force claims. Zero speed/current are valid unused fields in
    the documented position-only mode 1, not motor-current protection.
    """
    def __init__(self, calibration, message_type):
        if not isinstance(calibration, HandCalibration):
            raise ValueError('hand_calibration_required')
        self.calibration, self.message_type = calibration, message_type

    def format(self, closure):
        msg = self.message_type()
        fields = msg.get_fields_and_field_types()
        if not {'mode','positions','speeds','currents','pwms'} <= fields.keys():
            raise ValueError('incompatible_BrainCo_SetMotorMulti_schema')
        msg.mode = 1
        msg.positions = list(self.calibration.positions(closure))
        msg.speeds = list(self.calibration.speed)
        msg.currents = list(self.calibration.current_limit)
        msg.pwms = [0]*6
        return msg
