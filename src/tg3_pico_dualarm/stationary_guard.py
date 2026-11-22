"""Zero walking command and live factory command feedback during arm authority.

This proves command isolation and zero requests, not physical balance or
foot contact. The existing manufacturer body controller remains in charge.
"""
from . import safety as motion_policy
import math
import threading
import time

WALK='/hric/robot/cmd_vel'
STAND='/hric/robot/stand_cmd'
STATUS='/hric/robot/cmd_vel_status'
ISOLATED=('/arm/cmd','/right_hand/set_motor_multi','/left_hand/set_motor_multi',WALK,STAND)


def bridge_isolation_remaps():
    result=[]
    for topic in ISOLATED:
        result.extend(('-r',topic+':=/tg3/xarm_retired'+topic))
    return result


class StationaryCommandGuard:
    def __init__(self,node,*,clock=time.monotonic,message_type=None):
        if message_type is None:
            from geometry_msgs.msg import TwistStamped
            from rclpy.qos import qos_profile_sensor_data
            message_type=TwistStamped
            qos=qos_profile_sensor_data
        else:qos=1
        self.node,self.clock,self.type=node,clock,message_type
        self.lock=threading.RLock();self.active=False;self.fault=None
        self.last_zero=None;self.last_audit=None;self.zero_count=0
        self.applied_at=None;self.quiet_since=None;self.applied_zero=False;self.external_request=False
        self.publishers=[]
        self.publisher=node.create_publisher(message_type,WALK,1)
        self.subscription=node.create_subscription(message_type,WALK,self._observe,qos)
        self.status_subscription=node.create_subscription(message_type,STATUS,self._applied,qos)
        self.timer=node.create_timer(.05,self._tick)

    def _audit(self):
        endpoints=self.node.get_publishers_info_by_topic(WALK)
        self.publishers=[dict(node=e.node_name,namespace=e.node_namespace,type=e.topic_type) for e in endpoints]
        own=[e for e in endpoints if e.node_name==self.node.get_name() and e.node_namespace==self.node.get_namespace()]
        if len(own)!=1 or len(endpoints)>2 or any(e.topic_type!='geometry_msgs/msg/TwistStamped' or
                ((e.node_name,e.node_namespace) not in ((self.node.get_name(),self.node.get_namespace()),('hric_path_tracker','/')))
                for e in endpoints):
            raise RuntimeError('unreviewed_walk_writer:'+str(self.publishers))
        if self.node.get_publishers_info_by_topic(STAND):raise RuntimeError('unexpected_stand_pose_writer')
        applied=self.node.get_publishers_info_by_topic(STATUS)
        if len(applied)!=1 or (applied[0].node_name,applied[0].node_namespace,applied[0].topic_type)!=('xmigcs_control_node','/','geometry_msgs/msg/TwistStamped'):
            raise RuntimeError('unique_factory_applied_walk_feedback_required')
        self.last_audit=self.clock()

    def _observe(self,message):
        with self.lock:
            if not self.active:return
            try:
                values=[getattr(vector,name) for vector in (message.twist.linear,message.twist.angular) for name in ('x','y','z')]
                if any(not math.isfinite(v) or v!=0. for v in values):
                    self.fault='nonzero_or_invalid_walk_request_during_VR'
                    self.external_request=True;self.active=False  # Do not fight factory navigation.
            except (AttributeError,TypeError,ValueError):
                self.fault='invalid_walk_request_during_VR';self.external_request=True;self.active=False

    def _applied(self,message):
        with self.lock:
            now=self.clock()
            try:
                # This topic reports planar walking command, not foot contact.
                values=(message.twist.linear.x,message.twist.linear.y,message.twist.angular.z)
                zero=all(math.isfinite(v) and abs(v)<=1e-9 for v in values)
            except (AttributeError,TypeError,ValueError):zero=False
            if not zero or self.applied_at is None or now-self.applied_at>.15:self.quiet_since=None
            if zero and self.quiet_since is None:self.quiet_since=now
            self.applied_at=now;self.applied_zero=zero
            if self.active and not zero:
                self.fault='factory_applied_walk_command_not_zero'
                self.external_request=True;self.active=False

    def _quiet(self):
        now=self.clock()
        if (not self.applied_zero or self.applied_at is None or not 0<=now-self.applied_at<=.15
                or self.quiet_since is None or now-self.quiet_since<.3):
            raise RuntimeError('fresh_factory_zero_walk_feedback_300ms_required')

    def _zero(self):
        message=self.type();message.header.stamp=self.node.get_clock().now().to_msg()
        message.header.frame_id='tg3_vr_stationary'
        self.publisher.publish(message);self.last_zero=self.clock();self.zero_count+=1

    def begin(self):
        with self.lock:
            if self.fault:raise RuntimeError('stationary_guard_fault_latched:'+self.fault)
            self._audit()
            if self.applied_at is not None and not self.applied_zero:
                raise RuntimeError('factory_applied_walk_nonzero_before_admission')
            self.active=True;self._zero()

    def poll_ready(self):
        with self.lock:
            if self.fault:raise RuntimeError('stationary_guard:'+self.fault)
            if not self.active or self.last_zero is None or not 0<=self.clock()-self.last_zero<=.15:
                raise RuntimeError('fresh_stationary_zero_request_required')
            try:self._quiet()
            except RuntimeError:return False
            return True

    def _tick(self):
        with self.lock:
            if not self.active:return
            try:
                if self.last_audit is None or self.clock()-self.last_audit>=.1:self._audit()
                self._zero()
            except Exception as exc:self.fault=str(exc)

    def check(self):
        if motion_policy.LIMITS_MANAGED_EXTERNALLY:return
        with self.lock:
            if self.fault:raise RuntimeError('stationary_guard:'+self.fault)
            self._quiet()
            if not self.active or self.last_zero is None or not 0<=self.clock()-self.last_zero<=.15:
                raise RuntimeError('fresh_stationary_zero_request_required')

    def end(self):
        with self.lock:
            if self.active:self._zero()
            self.active=False

    def status(self):
        with self.lock:return dict(active=self.active,fault=self.fault,zero_commands=self.zero_count,
            zero_age_s=None if self.last_zero is None else self.clock()-self.last_zero,
            applied_zero=self.applied_zero,applied_age_s=None if self.applied_at is None else self.clock()-self.applied_at,
            quiet_since=self.quiet_since,external_request=self.external_request,walk_publishers=list(self.publishers),
            physical_balance_verified=False)

    def close(self):
        try:self.end()
        finally:
            self.node.destroy_timer(self.timer);self.node.destroy_subscription(self.subscription)
            self.node.destroy_subscription(self.status_subscription)
            self.node.destroy_publisher(self.publisher)
