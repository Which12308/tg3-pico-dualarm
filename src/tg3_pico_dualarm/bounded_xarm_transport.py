"""Measured factory-output limiter with a guarded native hybrid candidate.

Native EAIHardware must publish only on /tg3/xarm/arm_raw. This boundary
is the sole /arm/cmd publisher and starts without an actuator publisher.
The field-tested initial envelope is 0.02 rad/s and 0.03 rad feedback lead.
"""
# Keep the closed-source factory's initial position output off actuators.
# The measured relay is the sole real arm publisher; it never changes the
# manufacturer mode, gains, reserved bits, IDs, or ownership service flow.
import copy,math,threading
from .xarm_adapter import XArmRosTransport
class BoundedXArmTransport(XArmRosTransport):
    bounded_output_enforced=True
    initial_alignment_required=True
    def __init__(self,node,settings,*,holding_profile=None,**kwargs):
        from ros2_bridge_msgs.msg import ArmCtrl
        super().__init__(node,settings,**kwargs)
        self.arm_type=ArmCtrl
        self.holding_profile=holding_profile
        self.feedback=None;self.adapter=None;self.actual_publisher=None
        self.output_fault=None;self.last_actual=None;self.last_actual_time=None
        self.feedback_fault_details=None
        self.alignment=None;self.last_alignment=None
        self.actual_count=0;self.maximum_step=0.;self.output_mutex=threading.RLock()
        self.raw_subscription=node.create_subscription(ArmCtrl,'/tg3/xarm/arm_raw',self.raw,1)
    def check_publishers(self):
        endpoints=self.node.get_publishers_info_by_topic(self.settings.topic)
        if any(not self._own_endpoint(e) for e in endpoints):raise RuntimeError('competing_joint_target_writer')
        subscribers=self.node.get_subscriptions_info_by_topic(self.settings.topic)
        selected=[e for e in subscribers if e.node_name==self.settings.controller]
        if len(selected)!=1 or selected[0].topic_type!='std_msgs/msg/Float64MultiArray':raise RuntimeError('joint_controller_missing_or_wrong_type')
        raw=self.node.get_publishers_info_by_topic('/tg3/xarm/arm_raw')
        if len(raw)!=1 or raw[0].node_name!='EAIHardware' or raw[0].node_namespace!='/' or raw[0].topic_type!='ros2_bridge_msgs/msg/ArmCtrl':
            raise RuntimeError('exclusive_actual_factory_raw_output_required')
        real=self.node.get_publishers_info_by_topic('/arm/cmd')
        if any(not self._own_endpoint(e) or e.topic_type!='ros2_bridge_msgs/msg/ArmCtrl' for e in real) or len(real)>1:raise RuntimeError('competing_real_arm_writer')
        if self.output_fault:raise RuntimeError('bounded_factory_output:'+self.output_fault)
        if self.adapter and self.adapter.state=='armed' and self.last_actual_time is not None and self.clock()-self.last_actual_time>.1:
            raise RuntimeError('bounded_factory_output_stale')
    def open_publisher(self):
        # Open only the controller-input publisher. Real actuator output stays
        # absent until measured/native target alignment has been observed.
        super().open_publisher()

    def begin_initial_alignment(self,target):
        from .initial_alignment import InitialOutputAlignment
        with self.output_mutex:
            if self.actual_publisher is not None:raise RuntimeError('actuator_output_exists_before_alignment')
            self.alignment=InitialOutputAlignment(target,self.clock())
            self.publish(target)

    def alignment_status(self):
        with self.output_mutex:
            return self.alignment.status() if self.alignment is not None else self.last_alignment

    def poll_initial_alignment(self,target):
        with self.output_mutex:
            if self.output_fault:raise RuntimeError('bounded_factory_output:'+self.output_fault)
            if self.alignment is None:raise RuntimeError('initial_alignment_missing')
            self.publish(target)
            return self.alignment.ready(self.clock())

    def commit_initial_alignment(self):
        with self.output_mutex:
            if self.alignment is None or not self.alignment.ready(self.clock()):
                raise RuntimeError('initial_raw_alignment_not_ready')
            self.check_publishers()
            state=self.adapter._feedback_ok()
            if max(abs(a-b) for a,b in zip(state.positions,self.alignment.seed))>self.alignment.tolerance:
                raise RuntimeError('initial_alignment_measured_pose_moved')
            self.last_alignment=self.alignment.status();self.last_alignment['phase']='confirmed_before_output'
            self.last_actual=tuple(state.positions);self.last_actual_time=self.clock()
            self.actual_publisher=self.node.create_publisher(self.arm_type,'/arm/cmd',1)
            self.alignment=None
            return self.last_actual

    def _raw_motors(self,message):
        profile=getattr(self,'holding_profile',None)
        if message.mode!=(1 if profile else 0) or message.reserved!=0:
            raise RuntimeError('verified_arm_mode_zero_reserved_required')
        motors={m.name:m for m in message.ctrl};ids=tuple(range(11,18))+tuple(range(21,28))
        if len(message.ctrl)!=14 or set(motors)!=set(ids):raise RuntimeError('exact_fourteen_raw_motor_ids_required')
        for i,motor_id in enumerate(ids):
            raw=motors[motor_id]
            if not all(math.isfinite(v) for v in (raw.pos,raw.spd,raw.kp,raw.kd)):
                raise RuntimeError('nonfinite_raw_arm_field')
            if not self.settings.lower[i]<=raw.pos<=self.settings.upper[i]:raise RuntimeError('raw_position_outside_limits')
            if profile:
                if abs(raw.kp-profile.kp[i])>1e-6 or abs(raw.kd-profile.kd[i])>1e-6:
                    raise RuntimeError('factory_hybrid_gains_not_confirmed:'+str(motor_id))
                low=(-18.,-20.,-9.,-8.,-4.,-1.5,-1.5)[i%7]
                high=(15.,20.,9.,8.,4.,1.5,1.5)[i%7]
                if not math.isfinite(raw.tor) or not low<=raw.tor<=high:
                    raise RuntimeError('factory_feedforward_torque_outside_reviewed_bounds:'+str(motor_id))
            elif raw.spd<=0:raise RuntimeError('positive_factory_position_speed_required')
        return motors,ids

    def _observe_initial_raw(self,message):
        self.adapter._fresh_status();state=self.adapter._feedback_ok();now=self.clock()
        motors,ids=self._raw_motors(message)
        self.alignment.observe(tuple(motors[i].pos for i in ids),state.positions,now)

    def raw(self,message):
        with self.output_mutex:
            if getattr(self,'alignment',None) is not None and self.adapter is not None and self.adapter.state=='acquiring':
                try:
                    if not self.output_fault:self._observe_initial_raw(message)
                except Exception as exc:self.output_fault=str(exc)
                return  # Observing raw data never forwards it to actuators.
            if self.actual_publisher is None or self.adapter is None or self.adapter.state!='armed':return
            try:
                self.adapter._fresh_status();state=self.adapter._feedback_ok();now=self.clock()
                motors,ids=self._raw_motors(message)
                dt=now-self.last_actual_time
                if not 0<dt<=.1:raise RuntimeError('raw_output_timing_invalid')
                output=copy.deepcopy(message);formatted={m.name:m for m in output.ctrl};next_values=[]
                for i,motor_id in enumerate(ids):
                    raw=motors[motor_id]
                    speed=min(.02,self.settings.velocity_cap[i]);lead=min(.03,self.settings.feedback_lead[i]);cap=min(dt,.02)*speed
                    if not all(math.isfinite(v) for v in (raw.pos,raw.spd,raw.kp,raw.kd)):raise RuntimeError('nonfinite_raw_arm_field')
                    if not self.settings.lower[i]<=raw.pos<=self.settings.upper[i]:raise RuntimeError('raw_position_outside_limits')
                    if abs(self.last_actual[i]-state.positions[i])>lead:
                        self.feedback_fault_details=dict(motor_id=motor_id,measured_rad=state.positions[i],
                            last_actual_rad=self.last_actual[i],raw_target_rad=raw.pos,
                            error_rad=abs(self.last_actual[i]-state.positions[i]),limit_rad=lead,
                            kp=raw.kp,kd=raw.kd,raw_speed=raw.spd,dt_s=dt,
                            actual_frames_before_fault=self.actual_count)
                        raise RuntimeError('real_output_feedback_lead')
                    goal=self.last_actual[i]+max(-cap,min(cap,raw.pos-self.last_actual[i]))
                    if abs(goal-state.positions[i])>lead:raise RuntimeError('next_real_output_feedback_lead')
                    formatted[motor_id].pos=goal
                    # Hybrid spd is signed velocity feedforward, not the
                    # positive position-mode speed limit. Match the bounded
                    # actual target, including exact zero during holding.
                    formatted[motor_id].spd=((goal-self.last_actual[i])/dt
                        if getattr(self,'holding_profile',None) else min(raw.spd,speed))
                    next_values.append(goal)
                self.actual_publisher.publish(output)
                self.maximum_step=max(self.maximum_step,max(abs(a-b) for a,b in zip(next_values,self.last_actual)))
                self.last_actual=tuple(next_values);self.last_actual_time=now;self.actual_count+=1
            except Exception as exc:
                self.output_fault=str(exc)
                if self.actual_publisher is not None:
                    publisher=self.actual_publisher;self.actual_publisher=None;self._retire_publisher(publisher)
    def close_publisher(self):
        super().close_publisher()
        with self.output_mutex:
            if getattr(self,'alignment',None) is not None:
                self.last_alignment=self.alignment.status();self.last_alignment['phase']='aborted_without_actuator_output'
                self.alignment=None
            if self.actual_publisher is not None:
                publisher=self.actual_publisher;self.actual_publisher=None;self._retire_publisher(publisher)
    def close(self):
        super().close()
        if self.raw_subscription is not None:self.node.destroy_subscription(self.raw_subscription);self.raw_subscription=None

