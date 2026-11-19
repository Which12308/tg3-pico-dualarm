"""Live TianGong/BrainCo feedback, with no actuator publisher.

Schemas come from the actual installed ROS classes. Receipt timestamps use
this process's monotonic clock; mode diagnostics do not stand in for E-stop.
"""
from dataclasses import asdict, dataclass
import math
from numbers import Real, Integral
import threading
import time

from .preflight import parse_arm_positions, parse_fsm
from .robot import RobotState


POWER_KEY_TOPIC='/power/board/key_status'
POWER_KEY_TYPE='bodyctrl_msgs/msg/PowerBoardKeyStatus'
POWER_KEY_SOURCE=POWER_KEY_TOPIC+':'+POWER_KEY_TYPE
POWER_KEY_FIELDS=('is_estop','is_remote_estop','is_power_on','is_shutting_down','is_battery_unlock')
_NO_EXTERNAL_ESTOP=object()
# BrainCo's published MotorState enum (C SDK, not the unrelated touch status).
# Position control admits IDLE and RUNNING. STALL/end-stop, TURBO continuous
# force and unknown values remain stop conditions for this implementation.
BRAINCO_MOTOR_STATES={0:'idle',1:'running',2:'stall_or_limit',3:'turbo',255:'unknown'}
BRAINCO_MOTOR_STATE_SOURCE='https://brainco.tech/docs/revolimb-hand/en/revo2/c_sdk.html'


@dataclass(frozen=True)
class PowerBoardKeyFeedback:
    """SDK 26.06.x section 3.5.3: both E-stop flags are active-high."""
    is_estop: bool
    is_remote_estop: bool
    is_power_on: bool
    is_shutting_down: bool
    is_battery_unlock: bool
    received_at: float

    @classmethod
    def parse(cls,message,now):
        values=[]
        for field in POWER_KEY_FIELDS:
            try:value=getattr(message,field).data
            except AttributeError as exc:
                raise ValueError('incompatible_PowerBoardKeyStatus_field:'+field) from exc
            if type(value) is not bool:
                raise ValueError('nonboolean_PowerBoardKeyStatus_field:'+field)
            values.append(value)
        return cls(*values,now)

    @property
    def physical_estop(self):return self.is_estop or self.is_remote_estop


def power_key_schema(message_class,bool_class):
    """Validate the installed ROS definitions, never guessed local stand-ins."""
    fields=message_class.get_fields_and_field_types()
    if any(fields.get(name) not in ('std_msgs/Bool','std_msgs/msg/Bool') for name in POWER_KEY_FIELDS):
        raise ValueError('installed_PowerBoardKeyStatus_schema_incompatible')
    if bool_class.get_fields_and_field_types()!={'data':'boolean'}:
        raise ValueError('installed_std_msgs_Bool_schema_incompatible')


def _six(values, label, integer=False):
    try:
        values=tuple(values)
    except TypeError as exc:
        raise ValueError('invalid_hand_'+label) from exc
    if len(values)!=6 or any(isinstance(v,bool) or not isinstance(v,Real) or not math.isfinite(v) for v in values):
        raise ValueError('invalid_hand_'+label)
    if integer and any(not isinstance(v,Integral) for v in values):
        raise ValueError('noninteger_hand_'+label)
    return tuple(int(v) if integer else float(v) for v in values)


@dataclass(frozen=True)
class HandFeedback:
    positions: tuple
    states: tuple
    currents: tuple
    speeds: tuple
    received_at: float

    @classmethod
    def parse(cls,message,now):
        try:
            return cls(_six(message.positions,'positions'),_six(message.states,'states',True),
                _six(message.currents,'currents'),_six(message.speeds,'speeds'),now)
        except AttributeError as exc:
            raise ValueError('incompatible_BrainCo_MotorStatus_fields') from exc


class ManufacturerFeedback:
    def __init__(self,clock=time.monotonic,arm_timeout=.25,mode_timeout=.5,hand_timeout=.25,estop_timeout=.25):
        if any(not isinstance(v,Real) or isinstance(v,bool) or not math.isfinite(v) or not 0<v<=.5
               for v in (arm_timeout,mode_timeout,hand_timeout,estop_timeout)):
            raise ValueError('invalid_manufacturer_feedback_timeout')
        self.clock=clock
        self.arm_timeout,self.mode_timeout,self.hand_timeout=arm_timeout,mode_timeout,hand_timeout
        self.estop_timeout=estop_timeout
        self.lock=threading.RLock()
        self.positions=self.arm_stamp=self.mode=self.mode_stamp=None
        self.power_keys=None
        self.hands=dict.fromkeys(('left','right'))
        self.errors={}
        self.subscriptions=[]
        self.node=None
        self.streams={}

    def _received(self,name,stamp):
        old=self.streams.get(name)
        self.streams[name]=dict(count=1 if old is None else old['count']+1,
            first_received_at=stamp if old is None else old['first_received_at'],last_received_at=stamp,
            last_interval_s=None if old is None else stamp-old['last_received_at'],
            maximum_interval_s=0. if old is None else max(old['maximum_interval_s'],stamp-old['last_received_at']))

    def _stream_status(self,now):
        return {name:dict(value,age_s=now-value['last_received_at']) for name,value in self.streams.items()}

    def arm(self,message):
        with self.lock:
            try:
                self.positions=parse_arm_positions(message);self.arm_stamp=self.clock()
                self._received('arm',self.arm_stamp)
                self.errors.pop('arm',None)
            except (ValueError,TypeError,OverflowError) as exc:
                self.positions=self.arm_stamp=None;self.errors['arm']=str(exc)

    def fsm(self,message):
        with self.lock:
            try:
                self.mode=parse_fsm(message);self.mode_stamp=self.clock()
                self._received('fsm',self.mode_stamp)
                self.errors.pop('fsm',None)
            except (ValueError,TypeError,OverflowError) as exc:
                self.mode=self.mode_stamp=None;self.errors['fsm']=str(exc)

    def hand(self,side,message):
        if side not in self.hands:raise ValueError('unknown_hand_side')
        with self.lock:
            try:
                self.hands[side]=HandFeedback.parse(message,self.clock())
                self._received(side+'_hand',self.hands[side].received_at)
                self.errors.pop(side+'_hand',None)
            except (ValueError,TypeError,OverflowError) as exc:
                self.hands[side]=None;self.errors[side+'_hand']=str(exc)

    def key_status(self,message):
        with self.lock:
            try:
                self.power_keys=PowerBoardKeyFeedback.parse(message,self.clock())
                self._received('power_key_status',self.power_keys.received_at)
                self.errors.pop('power_key_status',None)
            except (ValueError,TypeError,OverflowError) as exc:
                self.power_keys=None;self.errors['power_key_status']=str(exc)

    @staticmethod
    def _fresh(stamp,now,limit):return stamp is not None and 0<=now-stamp<=limit

    def _estop(self,now):
        return self.power_keys.physical_estop if self.power_keys and self._fresh(
            self.power_keys.received_at,now,self.estop_timeout) else None

    def read_state(self,estop=_NO_EXTERNAL_ESTOP):
        with self.lock:
            now=self.clock()
            if self.positions is None or self.arm_stamp is None:
                raise RuntimeError('missing_complete_fourteen_joint_feedback')
            mode=self.mode['current_state'] if self.mode and self._fresh(self.mode_stamp,now,self.mode_timeout) else 'UNKNOWN'
            quality='measured' if self._fresh(self.arm_stamp,now,self.arm_timeout) else 'stale'
            observed_estop=self._estop(now)
            if estop is not _NO_EXTERNAL_ESTOP:
                if estop is not None and type(estop) is not bool:raise ValueError('invalid_external_estop')
                # An additional source may veto, but cannot manufacture release
                # when the documented board feedback is absent or expired.
                observed_estop=True if observed_estop is True or estop is True else (
                    False if observed_estop is False and estop is False else None)
            return RobotState(self.positions,self.arm_stamp,mode,observed_estop,quality)

    def snapshot(self,require_hands=True):
        with self.lock:
            required=set(('left','right')) if require_hands is True else set() if require_hands is False else set(require_hands)
            if not required<=self.hands.keys():raise ValueError('unknown_required_hand_side')
            now=self.clock();issues=[]
            if self.positions is None or not self._fresh(self.arm_stamp,now,self.arm_timeout):issues.append('arm_feedback_missing_or_stale')
            if not self.mode or not self._fresh(self.mode_stamp,now,self.mode_timeout):issues.append('fsm_feedback_missing_or_stale')
            elif self.mode['level']!=0 or self.mode['status']!='running' or self.mode['current_state']!=self.mode['child_state']:
                issues.append('fsm_not_settled_running')
            estop=self._estop(now)
            if estop is None:issues.append('physical_estop_feedback_missing_or_stale')
            elif estop:issues.append('physical_estop_active')
            hands={}
            for side,value in self.hands.items():
                fresh=bool(value and self._fresh(value.received_at,now,self.hand_timeout))
                zero=bool(value and all(v==0 for v in value.states))
                position_safe=bool(value and all(v in (0,1) for v in value.states))
                hands[side]=dict(feedback=asdict(value) if value else None,fresh=fresh,
                                 age_s=now-value.received_at if value else None,
                                 all_state_codes_zero=zero,state_codes_interpreted=True,
                                 states_safe_for_position_control=position_safe,
                                 state_names=[BRAINCO_MOTOR_STATES.get(v,'unknown') for v in value.states] if value else [],
                                 state_semantics_source=BRAINCO_MOTOR_STATE_SOURCE)
                if side in required and not fresh:issues.append(side+'_hand_feedback_missing_or_stale')
                if side in required and value and not position_safe:issues.append(side+'_hand_motor_state_not_safe_for_position_control')
            return dict(ready=not issues,issues=issues,positions=self.positions,arm_stamp=self.arm_stamp,
                mode=dict(self.mode) if self.mode else None,hands=hands,errors=self.errors.copy(),
                physical_estop=estop,physical_estop_source=POWER_KEY_SOURCE,
                physical_estop_stamp=self.power_keys.received_at if self.power_keys else None,
                physical_estop_fresh=estop is not None,
                power_key_status=asdict(self.power_keys) if self.power_keys else None,
                feedback_streams=self._stream_status(now),
                physical_estop_semantics='is_estop.data OR is_remote_estop.data; true means pressed',
                robot_commands_sent=False)

    def subscribe(self,node):
        """Attach only read subscriptions to an existing ROS node.

        Import/support checks finish before any subscription. No fallback ROS
        message definitions are shipped and no hardware publisher is created.
        """
        if self.node is not None:raise RuntimeError('observer_already_attached')
        from rclpy.type_support import check_for_type_support
        from rclpy.qos import qos_profile_sensor_data
        from ros2_bridge_msgs.msg import RobotState as ROSRobotState
        from diagnostic_msgs.msg import DiagnosticStatus
        from brainco_hand_msgs.msg import MotorStatus
        for cls in (ROSRobotState,DiagnosticStatus,MotorStatus):check_for_type_support(cls)
        fields=MotorStatus.get_fields_and_field_types()
        if not {'positions','states','currents','speeds'}<=fields.keys():
            raise RuntimeError('installed_BrainCo_feedback_schema_incompatible')
        power_class=None
        try:
            from bodyctrl_msgs.msg import PowerBoardKeyStatus
            from std_msgs.msg import Bool
            check_for_type_support(PowerBoardKeyStatus);check_for_type_support(Bool)
            power_key_schema(PowerBoardKeyStatus,Bool)
            power_class=PowerBoardKeyStatus
        except Exception as exc:
            # Observation remains useful on a machine lacking this vendor
            # package. It cannot become ready or report released E-stops.
            with self.lock:
                self.power_keys=None
                self.errors['power_key_status']='PowerBoardKeyStatus_subscription_unavailable:'+type(exc).__name__+':'+str(exc)
        self.node=node
        try:
            for cls,topic,callback in (
                (ROSRobotState,'/robot_state',self.arm),
                (DiagnosticStatus,'/hric/robot/rl_state',self.fsm),
                (MotorStatus,'/left_hand/motor_status',lambda msg:self.hand('left',msg)),
                (MotorStatus,'/right_hand/motor_status',lambda msg:self.hand('right',msg))):
                self.subscriptions.append(node.create_subscription(cls,topic,callback,qos_profile_sensor_data))
            if power_class is not None:
                self.subscriptions.append(node.create_subscription(power_class,POWER_KEY_TOPIC,self.key_status,qos_profile_sensor_data))
        except BaseException:
            self.close();raise
        return self

    def close(self):
        if self.node:
            for subscription in self.subscriptions:self.node.destroy_subscription(subscription)
        self.subscriptions=[];self.node=None
