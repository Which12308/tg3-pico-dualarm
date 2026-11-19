"""TianGong ROS actuator adapter with observed factory ownership admission.

The output transport really publishes the inspected ArmCtrl/SetMotorMulti
messages. It remains unusable without a separately supplied, verified factory
control driver. The public SDK currently does not provide that driver.
"""
from dataclasses import dataclass
import math
from numbers import Real
import threading
import time

from .hands import HandCalibration
from .manufacturer_contract import FactoryControlDriver, FactoryStatus, FactoryResult
from .manufacturer_feedback import ManufacturerFeedback
from .robot import JointCommand, SubmitResult
from .tiangong_protocol import ArmCommandFormatter
from .brainco_adapter import hand_schema,encode_hand,limit_hand_target,measured_projection


def vector(value,count,name,positive=False):
    result=tuple(value)
    if len(result)!=count or any(isinstance(v,bool) or not isinstance(v,Real) or not math.isfinite(v) for v in result):
        raise ValueError('invalid_'+name)
    if positive and any(v<=0 for v in result):raise ValueError('nonpositive_'+name)
    return tuple(float(v) for v in result)


@dataclass(frozen=True)
class ManufacturerLimits:
    kp: tuple
    kd: tuple
    lower: tuple
    upper: tuple
    velocity: tuple
    feedback_lead: tuple
    hand_raw_rate: dict
    hand_raw_lead: dict
    evidence: str
    command_timeout: float=.1
    ownership_timeout: float=.15
    watchdog_timeout: float=.15
    acquisition_timeout: float=2.
    caller_label: int=0

    def __post_init__(self):
        if type(self.caller_label) is not int or not (self.caller_label==0 or 101<=self.caller_label<=255):
            raise ValueError('invalid_external_caller_label')
        for name in ('kp','kd','lower','upper','velocity','feedback_lead'):
            object.__setattr__(self,name,vector(getattr(self,name),14,name,name in ('velocity','feedback_lead')))
        if any(a>=b for a,b in zip(self.lower,self.upper)) or any(v<0 for v in self.kp+self.kd):
            raise ValueError('invalid_factory_joint_bounds_or_gains')
        if not isinstance(self.evidence,str) or not self.evidence.strip():raise ValueError('factory_limits_evidence_required')
        for name in ('command_timeout','ownership_timeout','watchdog_timeout'):
            value=getattr(self,name)
            if type(value) not in (float,int) or not math.isfinite(value) or not 0<value<=.15:
                raise ValueError('invalid_'+name)
        if type(self.acquisition_timeout) not in (float,int) or not math.isfinite(self.acquisition_timeout) or not 0<self.acquisition_timeout<=5:
            raise ValueError('invalid_acquisition_timeout')
        for name in ('hand_raw_rate','hand_raw_lead'):
            value=getattr(self,name)
            if not isinstance(value,dict) or not set(value)<= {'left','right'}:raise ValueError('invalid_'+name)
            object.__setattr__(self,name,{side:vector(v,6,name,True) for side,v in value.items()})


class ROSFactoryTransport:
    """Own only /arm/cmd and configured BrainCo command publishers.

    Calling code cannot open this transport with an enable flag. An actual
    FactoryControlDriver object and its fresh observed lease are required.
    Message-type injection is for explicit tests, not a runtime CLI option.
    """
    def __init__(self,node,profiles,limits,message_types=None,clock=time.monotonic):
        self.node,self.profiles,self.limits,self.clock=node,profiles,limits,clock
        self.message_types=message_types
        self.arm_publisher=None;self.hand_publishers={};self.formatter=None
        self.hand_type=None;self.hand_fields=None

    def open(self,driver,status):
        if not isinstance(driver,FactoryControlDriver) or not isinstance(status,FactoryStatus):
            raise RuntimeError('verified_factory_control_driver_required_before_publishers')
        now=self.clock();status.validate_estop(now,self.limits.ownership_timeout)
        if status.lease is None:raise RuntimeError('factory_dual_arm_ownership_unacknowledged')
        status.lease.validate(now,self.limits.ownership_timeout,self.profiles)
        fresh=driver.read_status()
        if not isinstance(fresh,FactoryStatus) or fresh.lease is None or fresh.lease.owner_id!=status.lease.owner_id:
            raise RuntimeError('factory_owner_changed_before_publisher_creation')
        now=self.clock()
        fresh.validate_estop(now,self.limits.ownership_timeout);fresh.lease.validate(now,self.limits.ownership_timeout,self.profiles)
        if self.arm_publisher is not None:raise RuntimeError('transport_already_open')
        if self.message_types is None:
            from rclpy.type_support import check_for_type_support
            from ros2_bridge_msgs.msg import ArmCtrl,MotorCtrl
            SetMotorMulti=None
            if self.profiles:
                from brainco_hand_msgs.msg import SetMotorMulti
            types=(ArmCtrl,MotorCtrl,SetMotorMulti)
            for cls in types:
                if cls is not None:check_for_type_support(cls)
        else:types=self.message_types
        self.formatter=ArmCommandFormatter(types[0],types[1])
        prototype=types[0]()
        if type(prototype.reserved) is not int or prototype.reserved!=0:
            raise ValueError('unverified_nonzero_reserved_default')
        self.hand_type=types[2];self.hand_fields=hand_schema(self.hand_type) if self.profiles else {}
        for profile in self.profiles.values():
            self.encode_hand(profile,profile.opened)
            self.encode_hand(profile,profile.closed)
        try:
            self.arm_publisher=self.node.create_publisher(types[0],'/arm/cmd',1)
            for side in self.profiles:
                self.hand_publishers[side]=self.node.create_publisher(types[2],'/'+side+'_hand/set_motor_multi',1)
        except BaseException:
            self.close();raise

    def encode_hand(self,profile,positions):
        return encode_hand(self.hand_type,self.hand_fields,profile,positions)

    def publish_arm(self,command):
        if self.arm_publisher is None:raise RuntimeError('factory_transport_not_open')
        message=self.formatter.encode(command,kp=self.limits.kp,kd=self.limits.kd,
            ros_stamp=self.node.get_clock().now().to_msg(),label=self.limits.caller_label)
        self.arm_publisher.publish(message)

    def publish_hand(self,side,positions):
        if side not in self.hand_publishers:raise RuntimeError('hand_transport_not_open')
        self.hand_publishers[side].publish(self.encode_hand(self.profiles[side],positions))

    def close(self):
        if self.arm_publisher is not None:self.node.destroy_publisher(self.arm_publisher)
        for publisher in self.hand_publishers.values():self.node.destroy_publisher(publisher)
        self.arm_publisher=None;self.hand_publishers={}


class ManufacturerAdapter:
    """RobotProtocol implementation; admission requires actual factory driver.

    No driver is shipped for the unverified proprietary ownership interface.
    This class implements the complete known output path and its lifecycle,
    but cannot be enabled by a configuration boolean or guessed FSM state.
    """
    simulated=False

    def __init__(self,feedback,transport_factory,limits,profiles=None,*,driver=None,clock=time.monotonic):
        if not isinstance(driver,FactoryControlDriver):
            raise RuntimeError('verified_factory_driver_missing_no_motion_publishers_created')
        evidence=driver.evidence
        if not isinstance(evidence,dict) or any(not isinstance(evidence.get(k),str) or not evidence[k].strip()
            for k in ('controller','version','interface_reference')):
            raise RuntimeError('factory_driver_interface_evidence_missing')
        if not isinstance(feedback,ManufacturerFeedback) or not isinstance(limits,ManufacturerLimits):
            raise TypeError('manufacturer_feedback_and_explicit_limits_required')
        self.profiles=dict(profiles or {})
        if not set(self.profiles)<= {'left','right'} or any(not isinstance(v,HandCalibration) for v in self.profiles.values()):
            raise ValueError('independent_measured_hand_profiles_required')
        if any(side not in limits.hand_raw_rate or side not in limits.hand_raw_lead for side in self.profiles):
            raise ValueError('explicit_hand_rate_and_feedback_lead_required')
        self.feedback,self.transport_factory,self.limits,self.driver,self.clock=feedback,transport_factory,limits,driver,clock
        self.lock=threading.RLock();self.transport=None;self.state='disabled';self.owner=None
        self.reason='explicit_operator_enable_required';self._requested_at=None
        self._last_command_stamp=None;self._last_output=None;self._last_target=None;self._last_arm_output=None
        self._stop_confirmed=True
        self.hand_positions={};self._hand_float={};self._last_hand_output={};self._last_hand_intent={}
        self.hand_diagnostics={}

    @property
    def hardware_writes_enabled(self):return self.state in ('armed','holding') and self.transport is not None

    @property
    def commanded_joint_positions(self):
        """Last accepted arm target, including the manufacturer's latched hold."""
        with self.lock:
            return tuple(self._last_target) if self._last_target is not None else None

    def read_state(self):
        try:
            status=self.driver.read_status()
            if status.physical_estop is True:estop=True
            else:
                status.validate_estop(self.clock(),self.limits.ownership_timeout)
                estop=status.physical_estop
        except Exception:estop=None
        return self.feedback.read_state(estop)

    def request_enable(self,explicit_operator_action=False):
        with self.lock:
            if explicit_operator_action is not True:return SubmitResult(False,'explicit_operator_enable_required')
            if self.state!='disabled':return SubmitResult(False,'disable_before_rearming')
            if not self.feedback.snapshot(require_hands=self.profiles)['ready']:
                return SubmitResult(False,'manufacturer_feedback_not_ready')
            try:
                status=self.driver.read_status();status.validate_estop(self.clock(),self.limits.ownership_timeout)
                self._requested_at=self.clock();self.driver.request_dual_arm_control(tuple(self.profiles))
                self.state='acquiring';self.reason='waiting_for_factory_dual_arm_ack'
                return self.poll_enable()
            except Exception as exc:
                return self._fault('factory_acquisition_failed:'+str(exc))

    def poll_enable(self):
        with self.lock:
            if self.state!='acquiring':return SubmitResult(False,'no_factory_acquisition_in_progress')
            now=self.clock()
            if now-self._requested_at>self.limits.acquisition_timeout:return self._fault('factory_acquisition_timeout')
            try:
                status=self.driver.read_status();now=self.clock();status.validate_estop(now,self.limits.ownership_timeout)
                if status.lease is None:return SubmitResult(False,'waiting_for_factory_dual_arm_ack')
                status.lease.validate(now,self.limits.ownership_timeout,self.profiles)
                if status.lease.acknowledged_at<self._requested_at:return SubmitResult(False,'waiting_for_new_factory_ack')
                snapshot=self.feedback.snapshot(require_hands=self.profiles)
                if not snapshot['ready']:raise ValueError(';'.join(snapshot['issues']))
                if snapshot['mode']['current_state']!=status.lease.mode:raise ValueError('factory_mode_ack_feedback_mismatch')
                for side,profile in self.profiles.items():
                    raw=snapshot['hands'][side]['feedback']['positions']
                    if any(not low<=v<=high for v,low,high in zip(raw,profile.lower,profile.upper)):
                        raise ValueError('actual_hand_outside_measured_profile:'+side)
                    self.hand_positions[side]=tuple(round(v) for v in raw)
                    self._hand_float[side]=tuple(float(v) for v in raw)
                    self._last_hand_output[side]=now
                transport=self.transport_factory()
                transport.open(self.driver,status)
                self.transport=transport;self.owner=status.lease.owner_id
                self._last_target=snapshot['positions'];self._last_output=now
                self._last_arm_output=now
                self.state='armed';self.reason='factory_acknowledged_waiting_for_fresh_operator_clutch'
                # Crucially, no arm or hand message is sent by enable itself.
                return SubmitResult(True,self.reason)
            except Exception as exc:
                return self._fault('factory_admission_failed:'+str(exc))

    def measured_hand_initialization(self):
        """Expose measured starting positions and an explicitly labelled fit.

        A scalar closure is a projection onto the calibrated open/close line,
        not a separately measured physical quantity. When the fit residual is
        large, keep six-channel feedback and require deliberate new hand input.
        """
        with self.lock:
            result={}
            snapshot=self.feedback.snapshot(require_hands=self.profiles)
            if not snapshot['ready']:raise RuntimeError('fresh_measured_hand_initialization_required')
            for side in self.profiles:
                raw=snapshot['hands'][side]['feedback']['positions']
                profile=self.profiles[side]
                result[side]=measured_projection(profile,raw,self.limits.hand_raw_lead[side])
            return result

    def _check(self):
        now=self.clock()
        if not self.hardware_writes_enabled:raise ValueError('hardware_output_disabled')
        if self._last_output is None or not 0<=now-self._last_output<=self.limits.watchdog_timeout:
            raise ValueError('manufacturer_command_watchdog_expired')
        status=self.driver.read_status();now=self.clock();status.validate_estop(now,self.limits.ownership_timeout)
        if status.lease is None:raise ValueError('factory_ownership_lost')
        status.lease.validate(now,self.limits.ownership_timeout,self.profiles)
        if status.lease.owner_id!=self.owner:raise ValueError('factory_owner_changed')
        snapshot=self.feedback.snapshot(require_hands=self.profiles)
        if not snapshot['ready']:raise ValueError(';'.join(snapshot['issues']))
        if snapshot['mode']['current_state']!=status.lease.mode:raise ValueError('factory_mode_changed')
        return now,snapshot

    def submit(self,command):
        with self.lock:
            if not isinstance(command,JointCommand):raise TypeError('validated_JointCommand_required')
            try:
                now,snapshot=self._check()
                if not 0<=now-command.stamp<=self.limits.command_timeout:raise ValueError('stale_or_future_joint_command')
                if self._last_command_stamp is not None and command.stamp<=self._last_command_stamp:
                    raise ValueError('replayed_joint_command')
                dt=min(.05,now-self._last_arm_output)
                if dt<=0:raise ValueError('nonadvancing_command_clock')
                for i,(q,v) in enumerate(zip(command.positions,command.velocities)):
                    if not self.limits.lower[i]<=q<=self.limits.upper[i]:raise ValueError('joint_limit_violation')
                    if abs(v)>self.limits.velocity[i] or abs(q-self._last_target[i])>self.limits.velocity[i]*dt+1e-9:
                        raise ValueError('joint_velocity_violation')
                    if abs(q-snapshot['positions'][i])>self.limits.feedback_lead[i]:raise ValueError('joint_feedback_lead_violation')
                self.transport.publish_arm(command)
                self._last_target=command.positions;self._last_output=now;self._last_arm_output=now;self._last_command_stamp=command.stamp
                self.state='armed';return SubmitResult(True,'published_through_verified_factory_owner')
            except Exception as exc:
                return self._fault('manufacturer_output_failed:'+str(exc))

    def submit_hand(self,side,closure,*,intent_stamp,operation=None):
        """Call only for a NEW tracked operator trigger/open-button intent.

        Enable emits nothing. First command starts at measured six-channel
        feedback, not an assumed zero/open pose. Hand rate is a software raw
        position-per-second bound, distinct from vendor speed register units.
        """
        with self.lock:
            try:
                now,snapshot=self._check()
                if side not in self.profiles:raise ValueError('hand_profile_missing')
                if operation not in (None,'close','open'):raise ValueError('unknown_hand_operation')
                if type(intent_stamp) not in (int,float) or not math.isfinite(intent_stamp) or not 0<=now-intent_stamp<=self.limits.command_timeout:
                    raise ValueError('stale_or_invalid_hand_intent')
                if side in self._last_hand_intent and intent_stamp<=self._last_hand_intent[side]:
                    return SubmitResult(False,'no_new_hand_intent')
                profile=self.profiles[side]
                dt=min(.05,now-self._last_hand_output[side])
                if dt<=0:raise ValueError('nonadvancing_hand_clock')
                measured=snapshot['hands'][side]['feedback']['positions']
                values,floating,diagnostics=limit_hand_target(profile,closure,operation=operation,
                    previous=self.hand_positions[side],floating=self._hand_float[side],measured=measured,
                    raw_rate=self.limits.hand_raw_rate[side],raw_lead=self.limits.hand_raw_lead[side],dt=dt)
                self.transport.publish_hand(side,tuple(values))
                self.hand_positions[side]=tuple(values);self._last_hand_output[side]=now
                self._hand_float[side]=tuple(floating)
                self._last_hand_intent[side]=intent_stamp;self._last_output=now
                self.hand_diagnostics[side]=diagnostics
                return SubmitResult(True,'published_calibrated_hand_from_measured_pose')
            except Exception as exc:
                return self._fault('manufacturer_hand_failed:'+str(exc))

    def hold(self):
        with self.lock:
            try:
                now,snapshot=self._check()
                # Entering the verified OEM hold captures the actual pose once.
                # Repeated calls retain it; following fresh feedback every tick
                # would turn a compliant arm's drift into a moving hold target.
                target=self._last_target if self.state=='holding' else snapshot['positions']
                if target is None:raise ValueError('manufacturer_hold_target_missing')
                if any(not low<=q<=high for q,low,high in zip(target,self.limits.lower,self.limits.upper)):
                    raise ValueError('manufacturer_hold_outside_joint_limits')
                if any(abs(q-p)>lead for q,p,lead in zip(target,snapshot['positions'],self.limits.feedback_lead)):
                    raise ValueError('manufacturer_hold_feedback_lead_violation')
                result=self.driver.hold(tuple(target))
                if not isinstance(result,FactoryResult) or result.acknowledged is not True:
                    raise ValueError('manufacturer_hold_unacknowledged')
                self._last_target=tuple(target);self._last_output=now;self._last_arm_output=now;self.state='holding'
                return SubmitResult(True,'manufacturer_hold_acknowledged')
            except Exception as exc:
                return self._fault('manufacturer_hold_failed:'+str(exc))

    def check_watchdog(self):
        with self.lock:
            if self.hardware_writes_enabled:
                try:self._check()
                except Exception as exc:
                    self._fault('manufacturer_watchdog:'+str(exc));return True
            return self.state=='fault'

    def _fault(self,reason):
        self.state='fault';self.reason=str(reason)
        self._stop_confirmed=True
        transport,self.transport=self.transport,None
        if transport is not None:
            try:transport.close()
            except Exception as exc:
                self._stop_confirmed=False;self.reason+=';publisher_cleanup_failed:'+str(exc)
        for method,args in ((self.driver.stop_and_restore,(str(reason),)),(self.driver.release_control,())):
            try:
                result=method(*args)
                if not isinstance(result,FactoryResult) or result.acknowledged is not True:
                    self._stop_confirmed=False
                    self.reason+=';factory_stop_or_release_not_acknowledged'
            except Exception as exc:
                self._stop_confirmed=False;self.reason+=';factory_stop_or_release_failed:'+str(exc)
        self.owner=None
        return SubmitResult(False,self.reason)

    def disarm(self,reason='operator_disabled'):
        with self.lock:
            if self.state=='disabled':return SubmitResult(True,'already_disabled')
            result=self._fault(reason)
            if self._stop_confirmed:
                self.state='disabled'
                return SubmitResult(True,'manufacturer_stop_and_release_acknowledged')
            return result
