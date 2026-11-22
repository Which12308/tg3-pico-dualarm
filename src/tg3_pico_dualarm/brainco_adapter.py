"""Measured BrainCo hands composed with an admitted XARM parent.

No arm controller, ownership request, SDK construction or automatic hand-open
command lives here. SDK 26.06.x documents mode 1 and MotorStatus. Calibration
and software raw-rate/feedback-lead limits must still be supplied per hand.
"""
from . import safety as motion_policy
import json
import math
from numbers import Real
from pathlib import Path
import re
import time

from .hands import HandCalibration
from .manufacturer_feedback import ManufacturerFeedback
from .robot import SubmitResult


def hand_schema(message_type):
    fields={key:str(value).replace(' ','') for key,value in message_type.get_fields_and_field_types().items()}
    pwm_names=set(fields)&{'pwm','pwms'}
    required={'mode','positions','speeds','currents'}
    if len(pwm_names)!=1 or not required<=fields.keys() or fields.keys()-required-pwm_names-{'durations'}:
        raise ValueError('unverified_BrainCo_command_fields')
    if fields['mode'] not in ('uint8','uint16','uint32','int8','int16','int32'):
        raise ValueError('unverified_BrainCo_mode_type')
    for name in fields.keys()-{'mode'}:
        fixed=re.fullmatch(r'((?:u?int)(?:8|16|32|64))\[6\]',fields[name])
        sequence=re.fullmatch(r'sequence<((?:u?int)(?:8|16|32|64))(?:,6)?>',fields[name])
        if not (fixed or sequence):raise ValueError('unsupported_BrainCo_six_channel_type:'+name)
    return fields


def _fits(values,field):
    match=re.search(r'(u?int)(8|16|32|64)',field)
    unsigned=match.group(1).startswith('u');bits=int(match.group(2))
    low=0 if unsigned else -(2**(bits-1));high=2**bits-1 if unsigned else 2**(bits-1)-1
    return all(type(value) is int and low<=value<=high for value in values)


def encode_hand(message_type,fields,profile,positions):
    positions=tuple(positions)
    if len(positions)!=6 or any(type(value) is not int or (not motion_policy.LIMITS_MANAGED_EXTERNALLY and not low<=value<=high)
            for value,low,high in zip(positions,profile.lower,profile.upper)):
        raise ValueError('hand_target_outside_measured_limits')
    # mode 1 controls positions; the historical current_limit profile name
    # fills the current command field, not SDK service mode-7 protection.
    values=dict(positions=positions,speeds=profile.speed,currents=profile.current_limit)
    values['pwm' if 'pwm' in fields else 'pwms']=(0,)*6
    if 'durations' in fields:values['durations']=(0,)*6
    message=message_type();message.mode=1
    for name,value in values.items():
        if not _fits(value,fields[name]):raise ValueError('BrainCo_field_range_mismatch:'+name)
        setattr(message,name,list(value))
    return message


def limit_hand_target(profile,closure,*,operation,previous,floating,measured,raw_rate,raw_lead,dt):
    """The shared six-channel factory limiter; floating residuals accumulate."""
    if operation not in (None,'close','open'):raise ValueError('unknown_hand_operation')
    if motion_policy.LIMITS_MANAGED_EXTERNALLY:
        requested=tuple(profile.positions(closure))
        values=tuple(old if operation=='close' and (goal-old)*(closed-opened)<0 else goal
            for old,goal,opened,closed in zip(previous,requested,profile.opened,profile.closed))
        return values,tuple(map(float,values)),dict(commanded_positions=values,measured_positions=tuple(measured),
            feedback_limited=False,feedback_limited_channels=(False,)*6,limits_owner='external',operation=operation,
            requested_positions=requested,effective_positions=values,command_target_reached=True)
    if not 0<dt<=.05:raise ValueError('nonadvancing_hand_clock')
    target=profile.positions(closure);values=[];next_float=[];limited=[];effective=[]
    for index,(old,goal) in enumerate(zip(floating,target)):
        lead=raw_lead[index]
        if abs(previous[index]-measured[index])>lead+1e-9:raise ValueError('hand_feedback_lead_violation')
        if operation=='close':
            direction=profile.closed[index]-profile.opened[index]
            if direction==0 or (goal-old)*direction<0:goal=old
        effective.append(goal)
        bound=raw_rate[index]*dt
        position=old+max(-bound,min(bound,goal-old))
        low=max(profile.lower[index],measured[index]-lead);high=min(profile.upper[index],measured[index]+lead)
        integer_low,integer_high=math.ceil(low),math.floor(high)
        if integer_low>integer_high:raise ValueError('no_integer_hand_target_in_feedback_corridor')
        clipped=max(low,min(high,position));limited.append(abs(clipped-position)>1e-9)
        values.append(max(integer_low,min(integer_high,round(clipped))));next_float.append(clipped)
    diagnostics=dict(operation=operation,requested_positions=target,effective_positions=tuple(effective),
        commanded_positions=tuple(values),measured_positions=tuple(measured),feedback_limited=any(limited),
        feedback_limited_channels=tuple(limited),limit_interpretation='feedback_corridor_only',
        command_target_reached=all(abs(value-goal)<.5 for value,goal in zip(values,effective)))
    return tuple(values),tuple(next_float),diagnostics


def measured_projection(profile,raw,raw_lead):
    differences=[closed-opened for opened,closed in zip(profile.opened,profile.closed)]
    denominator=sum(value*value for value in differences)
    if denominator==0:raise ValueError('degenerate_hand_open_close_calibration')
    closure=max(0.,min(1.,sum((value-opened)*delta for value,opened,delta
        in zip(raw,profile.opened,differences))/denominator))
    residual=tuple(value-(opened+closure*delta) for value,opened,delta in zip(raw,profile.opened,differences))
    return dict(positions=tuple(raw),closure_projection=closure,closure_is_projection=True,residual_raw=residual,
        fit_within_configured_lead=all(abs(value)<=lead for value,lead in zip(residual,raw_lead)))


def load_brainco_config(path):
    """Load measured profiles and independent raw limits; invent no defaults.

    JSON: {"hands": {"right": <all HandCalibration fields>},
           "hand_raw_rate": {"right": [six positive values]},
           "hand_raw_lead": {"right": [six positive values]}}.
    Missing/invalid side data stays visible to the adapter's disabled report.
    """
    value=json.loads(Path(path).read_text(encoding='utf-8'))
    if not isinstance(value,dict) or set(value)-{'hands','hand_raw_rate','hand_raw_lead'}:
        raise ValueError('invalid_BrainCo_configuration_fields')
    return dict(profiles=value.get('hands'),hand_raw_rate=value.get('hand_raw_rate'),hand_raw_lead=value.get('hand_raw_lead'))


def _positive_six(values,name):
    try:result=tuple(values)
    except TypeError as exc:raise ValueError('missing_or_invalid_'+name) from exc
    if len(result)!=6 or any(isinstance(value,bool) or not isinstance(value,Real) or not math.isfinite(value) or value<=0 for value in result):
        raise ValueError('missing_or_invalid_'+name)
    return tuple(float(value) for value in result)


def validate_brainco_config(value):
    """Strict, side-effect-free validation for selecting a hand JSON file.

    Runtime loading remains permissive per side so missing robot calibration
    can be reported as disabled. A desktop selection must instead be complete
    for every selected side and must not accept unrelated configuration files.
    """
    required={'hands','hand_raw_rate','hand_raw_lead'}
    if not isinstance(value,dict) or set(value)!=required:
        raise ValueError('BrainCo_config_requires_exact_hands_hand_raw_rate_hand_raw_lead_fields')
    if any(not isinstance(value[name],dict) for name in required):
        raise ValueError('BrainCo_configuration_sections_must_be_side_mappings')
    sides=set(value['hands'])
    if not sides or not sides<={'left','right'}:
        raise ValueError('BrainCo_configuration_requires_one_or_two_valid_hand_sides')
    if any(set(value[name])!=sides for name in ('hand_raw_rate','hand_raw_lead')):
        raise ValueError('BrainCo_calibration_rate_and_lead_sides_must_match')
    for side in sorted(sides):
        profile=value['hands'][side]
        if not isinstance(profile,dict) or set(profile)!=set(HandCalibration.__dataclass_fields__):
            raise ValueError(side+'_hand_requires_complete_measured_HandCalibration_fields')
        try:
            measured=HandCalibration(**profile)
            _positive_six(value['hand_raw_rate'][side],'hand_raw_rate')
            lead=_positive_six(value['hand_raw_lead'][side],'hand_raw_lead')
            measured_projection(measured,measured.opened,lead)
        except (ValueError,TypeError,OverflowError) as exc:
            raise ValueError(side+'_hand_configuration_invalid:'+str(exc)) from exc
    return sorted(sides)


class BrainCoHandAdapter:
    """Composition helper; parent must be XARM's admitted, audited adapter.

    The parent owns the lock and calls invalidate on disable/disconnect and
    check_watchdog on its normal tick. Each send also rechecks its actual armed
    state, controller-audit freshness, physical E-stop and joint feedback.
    Message injection is for explicit offline tests only.
    """
    def __init__(self,node,feedback,*,parent,profiles=None,hand_raw_rate=None,hand_raw_lead=None,
                 config_path=None,message_type=None,clock=time.monotonic,command_timeout=.1):
        if not isinstance(feedback,ManufacturerFeedback):raise TypeError('manufacturer_feedback_required')
        if not all(callable(getattr(parent,name,None)) for name in ('_fresh_status','_feedback_ok')):
            raise TypeError('audited_XARM_parent_required')
        if type(command_timeout) not in (int,float) or not math.isfinite(command_timeout) or not 0<command_timeout<=.15:
            raise ValueError('invalid_hand_command_timeout')
        self.node,self.feedback,self.parent,self.clock=node,feedback,parent,clock
        self.lock=parent.lock;self.command_timeout=command_timeout
        self.profiles={};self.hand_raw_rate={};self.hand_raw_lead={};self.disabled_reasons={}
        load_error=None
        if config_path is not None:
            if any(value is not None for value in (profiles,hand_raw_rate,hand_raw_lead)):
                raise ValueError('provide_hand_config_path_or_explicit_profiles')
            try:
                loaded=load_brainco_config(config_path)
                profiles,hand_raw_rate,hand_raw_lead=(loaded[name] for name in ('profiles','hand_raw_rate','hand_raw_lead'))
            except (OSError,ValueError,TypeError) as exc:load_error='hand_configuration_unavailable:'+type(exc).__name__
        for side in ('left','right'):
            try:
                if load_error:raise ValueError(load_error)
                if not isinstance(profiles,dict) or side not in profiles:raise ValueError('measured_hand_calibration_missing')
                profile=profiles[side]
                if isinstance(profile,dict):profile=HandCalibration(**profile)
                if not isinstance(profile,HandCalibration):raise ValueError('measured_hand_calibration_required')
                rate=_positive_six(hand_raw_rate.get(side) if isinstance(hand_raw_rate,dict) else None,'hand_raw_rate')
                lead=_positive_six(hand_raw_lead.get(side) if isinstance(hand_raw_lead,dict) else None,'hand_raw_lead')
                measured_projection(profile,profile.opened,lead)
                self.profiles[side]=profile;self.hand_raw_rate[side]=rate;self.hand_raw_lead[side]=lead
            except (ValueError,TypeError) as exc:self.disabled_reasons[side]=str(exc)
        self.message_type=message_type;self.hand_fields=None;self.hand_publishers={};self._publisher_gids={}
        self.hand_positions={};self.hand_diagnostics={};self._hand_float={};self._last_hand_output={};self._last_hand_intent={}
        self.initialized=False;self.reason='measured_hand_initialization_required' if self.profiles else 'hands_disabled_configuration_required'

    def _parent_guard(self):
        if self.parent.state!='armed' or self.parent.hardware_writes_enabled is not True:
            raise RuntimeError('XARM_parent_not_armed')
        self.parent._fresh_status();self.parent._feedback_ok()

    def _measurements(self):
        self._parent_guard();snapshot=self.feedback.snapshot(require_hands=False)
        for side,profile in self.profiles.items():
            hand=snapshot['hands'][side]
            if not motion_policy.LIMITS_MANAGED_EXTERNALLY and not hand['fresh']:raise RuntimeError(side+'_hand_feedback_missing_or_stale')
            if not motion_policy.LIMITS_MANAGED_EXTERNALLY and not hand['states_safe_for_position_control']:raise RuntimeError(side+'_hand_motor_state_not_safe_for_position_control')
            raw=hand['feedback']['positions']
            if not motion_policy.LIMITS_MANAGED_EXTERNALLY and any(not low<=value<=high for value,low,high in zip(raw,profile.lower,profile.upper)):
                raise ValueError('actual_hand_outside_measured_profile:'+side)
        return snapshot

    def _check_publishers(self,side):
        endpoints=self.node.get_publishers_info_by_topic('/'+side+'_hand/set_motor_multi')
        # Before opening, even another publisher on this same ROS node is a
        # competitor. Humble exposes GIDs through graph endpoints, not through
        # Publisher; pin the first matching local endpoint once discovered.
        publisher=self.hand_publishers.get(side)
        if publisher is None:
            if endpoints:raise RuntimeError(side+'_hand_competing_command_publisher')
            return
        if len(endpoints)>1:raise RuntimeError(side+'_hand_duplicate_command_publisher')
        for endpoint in endpoints:
            if (endpoint.node_name!=self.node.get_name() or endpoint.node_namespace!=self.node.get_namespace()
                    or endpoint.topic_type!='brainco_hand_msgs/msg/SetMotorMulti'):
                raise RuntimeError(side+'_hand_competing_command_publisher')
            gid=bytes(endpoint.endpoint_gid)
            if side in self._publisher_gids and self._publisher_gids[side]!=gid:
                raise RuntimeError(side+'_hand_command_publisher_identity_changed')
            self._publisher_gids[side]=gid

    def measured_hand_initialization(self):
        with self.lock:
            if not self.profiles:return {}
            try:
                snapshot=self._measurements()
                result={side:measured_projection(profile,snapshot['hands'][side]['feedback']['positions'],self.hand_raw_lead[side])
                    for side,profile in self.profiles.items()}
                if self.initialized:return result
                if self.hand_publishers:raise RuntimeError('previous_hand_publisher_cleanup_required')
                if self.message_type is None:
                    from rclpy.type_support import check_for_type_support
                    from brainco_hand_msgs.msg import SetMotorMulti
                    check_for_type_support(SetMotorMulti);self.message_type=SetMotorMulti
                self.hand_fields=hand_schema(self.message_type)
                for side,profile in self.profiles.items():
                    encode_hand(self.message_type,self.hand_fields,profile,profile.opened)
                    encode_hand(self.message_type,self.hand_fields,profile,profile.closed)
                    self._check_publishers(side)
                now=self.clock()
                for side,item in result.items():
                    self._check_publishers(side);self._parent_guard()
                    self.hand_publishers[side]=self.node.create_publisher(self.message_type,'/'+side+'_hand/set_motor_multi',1)
                    self.hand_positions[side]=tuple(round(value) for value in item['positions'])
                    self._hand_float[side]=tuple(float(value) for value in item['positions']);self._last_hand_output[side]=now
                self.initialized=True;self.reason='measured_hands_initialized_no_initial_command'
                return result
            except Exception as exc:
                self.invalidate('hand_initialization_failed:'+str(exc));raise

    def submit_hand(self,side,closure,*,intent_stamp,operation=None):
        with self.lock:
            if side not in self.profiles:return SubmitResult(False,self.disabled_reasons.get(side,'hand_profile_missing'))
            try:
                snapshot=self._measurements();now=self.clock()
                if not self.initialized or side not in self.hand_publishers:raise RuntimeError('measured_hand_initialization_required')
                if type(intent_stamp) not in (int,float) or not math.isfinite(intent_stamp) or (not motion_policy.LIMITS_MANAGED_EXTERNALLY and not 0<=now-intent_stamp<=self.command_timeout):
                    raise ValueError('stale_or_invalid_hand_intent')
                if not motion_policy.LIMITS_MANAGED_EXTERNALLY and side in self._last_hand_intent and intent_stamp<=self._last_hand_intent[side]:return SubmitResult(False,'no_new_hand_intent')
                self._check_publishers(side)
                values,floating,diagnostics=limit_hand_target(self.profiles[side],closure,operation=operation,
                    previous=self.hand_positions[side],floating=self._hand_float[side],measured=snapshot['hands'][side]['feedback']['positions'],
                    raw_rate=self.hand_raw_rate[side],raw_lead=self.hand_raw_lead[side],dt=min(.05,now-self._last_hand_output[side]))
                message=encode_hand(self.message_type,self.hand_fields,self.profiles[side],values)
                self._parent_guard();self._check_publishers(side)
                self.hand_publishers[side].publish(message)
                self.hand_positions[side]=values;self._hand_float[side]=floating;self._last_hand_output[side]=now
                self._last_hand_intent[side]=intent_stamp;self.hand_diagnostics[side]=diagnostics
                return SubmitResult(True,'published_calibrated_hand_from_measured_pose')
            except Exception as exc:
                if motion_policy.LIMITS_MANAGED_EXTERNALLY:
                    self.reason=str(exc)
                    return SubmitResult(False,self.reason)
                self.invalidate('BrainCo_hand_failed:'+str(exc));return SubmitResult(False,self.reason)

    def check_watchdog(self):
        if motion_policy.LIMITS_MANAGED_EXTERNALLY:return False
        with self.lock:
            if not self.initialized:return False
            try:
                self._measurements()
                for side in self.hand_publishers:self._check_publishers(side)
                return False
            except Exception as exc:self.invalidate('BrainCo_hand_invalidated:'+str(exc));return True

    @property
    def hardware_writes_enabled(self):
        with self.lock:
            self.check_watchdog()
            return self.initialized and bool(self.hand_publishers)

    def invalidate(self,reason='hand_input_invalidated_hold_last_command'):
        with self.lock:
            self.initialized=False;self.reason=reason;self._last_hand_intent={};self._publisher_gids={}
            errors=[]
            for side,publisher in list(self.hand_publishers.items()):
                try:
                    if self.node.destroy_publisher(publisher) is False:raise RuntimeError('publisher_not_destroyed')
                    del self.hand_publishers[side]
                except Exception:errors.append(side)
            if errors:self.reason+=';hand_publisher_cleanup_failed:'+','.join(errors)
            return SubmitResult(not errors,self.reason)

    def close(self):return self.invalidate('BrainCo_adapter_closed_no_open_command')

    def status(self):
        with self.lock:
            return dict(enabled=self.hardware_writes_enabled,reason=self.reason,configured_sides=list(self.profiles),
                disabled_sides=dict(self.disabled_reasons),hand_positions=dict(self.hand_positions),
                hand_diagnostics=dict(self.hand_diagnostics),automatic_open_on_close=False)
