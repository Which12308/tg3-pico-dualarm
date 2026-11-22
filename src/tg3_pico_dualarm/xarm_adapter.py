"""XARM dual-arm ROS boundary, based on the supplied external-user manual.

No vendor SDK or invented ownership lease is used. All ROS service operations
are bounded call_async operations driven by poll_enable/check_watchdog. A ROS
executor must spin independently; this module never spins within a callback.
"""
from . import safety as motion_policy
from dataclasses import dataclass
import math
from numbers import Real
import threading
import time
from types import SimpleNamespace

from .model import JOINT_NAMES
from .robot import JointCommand,SubmitResult


CONTROLLER='jointspace_ruckig_qp_controller'
TOPIC='/jointspace_commands_Dual'
EMPTY=('empty_L_controller','empty_R_controller')
# The installed Tiangong3 controller declares `joints`, not `joint_names`.
# rclcpp returns an empty GetParameters response if any requested name is
# undeclared, so do not mix speculative aliases into this reviewed contract.
PARAMETERS=('vel_limits','acc_limits','jerk_limits','joints')


def _vector(values,label,positive=False):
    values=tuple(values)
    if len(values)!=14 or any(isinstance(v,bool) or not isinstance(v,Real) or not math.isfinite(v)
                             or (positive and v<=0) for v in values):
        raise ValueError('invalid_xarm_'+label)
    return tuple(float(v) for v in values)


@dataclass(frozen=True)
class XArmSettings:
    velocity_cap: tuple
    lower: tuple
    upper: tuple
    feedback_lead: tuple
    limits_evidence: str
    controller: str=CONTROLLER
    topic: str=TOPIC
    service_timeout: float=.15
    command_timeout: float=.1
    watchdog_timeout: float=.15
    feedback_timeout: float=.25
    audit_interval: float=.5
    status_timeout: float=1.
    transition_timeout: float=5.

    def __post_init__(self):
        for name in ('velocity_cap','lower','upper','feedback_lead'):
            object.__setattr__(self,name,_vector(getattr(self,name),name,name in ('velocity_cap','feedback_lead')))
        if any(a>=b for a,b in zip(self.lower,self.upper)):raise ValueError('invalid_xarm_joint_bounds')
        if not isinstance(self.limits_evidence,str) or not self.limits_evidence.strip():
            raise ValueError('xarm_joint_limit_evidence_required')
        # Different controllers/topics need a new reviewed wire contract.
        if self.controller!=CONTROLLER or self.topic!=TOPIC:raise ValueError('unreviewed_xarm_controller_or_topic')
        for name,low,high in (('service_timeout',0,.15),('command_timeout',0,.15),
            ('watchdog_timeout',0,.15),('feedback_timeout',0,.25),('audit_interval',.5,2.),
            ('status_timeout',.5,2.),('transition_timeout',0,10.)):
            value=getattr(self,name)
            if isinstance(value,bool) or not isinstance(value,Real) or not math.isfinite(value) or not low<=value<=high or value<=0:
                raise ValueError('invalid_xarm_'+name)
        if self.status_timeout<self.audit_interval+2*self.service_timeout:
            raise ValueError('xarm_status_timeout_too_short_for_audit')


def _fields(cls):return cls.get_fields_and_field_types()


class XArmRosTransport:
    """Real ROS message classes, discovered service names, no implicit writes."""
    bounded_output_enforced=False

    def __init__(self,node,settings,*,clock=time.monotonic):
        from rclpy.type_support import check_for_type_support
        from std_msgs.msg import Float64MultiArray
        from std_srvs.srv import SetBool
        from controller_manager_msgs.srv import ListControllers,SwitchController
        from rcl_interfaces.srv import GetParameters,SetParameters
        from rcl_interfaces.msg import Parameter,ParameterValue
        from eai_manipulator_msgs.msg import WarnAndErrorInfo
        self.node,self.settings,self.clock=node,settings,clock
        self.types=dict(array=Float64MultiArray,bool=SetBool,list=ListControllers,switch=SwitchController,
                        get=GetParameters,set=SetParameters,parameter=Parameter,value=ParameterValue)
        for cls in (*self.types.values(),WarnAndErrorInfo):check_for_type_support(cls)
        if _fields(Float64MultiArray).get('data') not in ('sequence<double>','sequence<float64>'):
            raise RuntimeError('installed_xarm_array_schema_incompatible')
        if _fields(SetBool.Request)!={'data':'boolean'} or _fields(SetBool.Response).get('success')!='boolean':
            raise RuntimeError('installed_xarm_SetBool_schema_incompatible')
        switch_fields=_fields(SwitchController.Request)
        if {'activate_controllers','deactivate_controllers'}<=switch_fields.keys():
            self.switch_names=('activate_controllers','deactivate_controllers')
        elif {'start_controllers','stop_controllers'}<=switch_fields.keys():
            self.switch_names=('start_controllers','stop_controllers')
        else:raise RuntimeError('installed_xarm_switch_schema_incompatible')
        if not {'strictness','timeout'}<=switch_fields.keys() or _fields(SwitchController.Response).get('ok')!='boolean':
            raise RuntimeError('installed_xarm_switch_schema_incompatible')
        if not {'level','code','source','info'}<=_fields(WarnAndErrorInfo).keys():
            raise RuntimeError('installed_xarm_warning_schema_incompatible')
        self.clients={};self.names={};self.publisher=None;self.pending=None
        self._pending_late_enable=False
        self.uncertain_mutation=False
        self.hardware_enabled=False
        self.warning_error=None;self.last_warning=None
        self.subscription=node.create_subscription(WarnAndErrorInfo,'/manipulator/warn_and_error_info',self._warning,10)

    def _warning(self,message):
        try:
            if not all(isinstance(getattr(message,name),str) for name in ('level','source','info')):
                raise ValueError('invalid_xarm_warning_fields')
            self.last_warning=dict(level=message.level,code=message.code,source=message.source,
                                   info=message.info,received_at=self.clock())
            if message.level.lower()=='error':self.warning_error=dict(self.last_warning)
        except (ValueError,AttributeError) as exc:self.warning_error=dict(error=str(exc))

    def discover(self):
        graph={name:tuple(types) for name,types in self.node.get_service_names_and_types()}
        def unique(suffix,typ):
            matches=[name for name,types in graph.items() if name.endswith(suffix) and types==(typ,)]
            if len(matches)!=1:raise RuntimeError('xarm_service_missing_or_ambiguous:'+suffix)
            return matches[0]
        list_name=unique('/list_controllers','controller_manager_msgs/srv/ListControllers')
        manager=list_name.rsplit('/',1)[0]
        expected=dict(list=(list_name,'controller_manager_msgs/srv/ListControllers'),
            switch=(manager+'/switch_controller','controller_manager_msgs/srv/SwitchController'),
            auto=(manager+'/auto_switch_mode','std_srvs/srv/SetBool'),
            enable=('/EAIHardware/set_arm_enable','std_srvs/srv/SetBool'))
        # Select parameter services from the actual controller node, including
        # an installed namespace. Never use the custom activate-only service.
        get_name=unique('/'+self.settings.controller+'/get_parameters','rcl_interfaces/srv/GetParameters')
        expected.update(get=(get_name,'rcl_interfaces/srv/GetParameters'),
                        set=(get_name.rsplit('/',1)[0]+'/set_parameters','rcl_interfaces/srv/SetParameters'))
        for key,(name,typ) in expected.items():
            if graph.get(name)!=(typ,):raise RuntimeError('xarm_service_type_unverified:'+name)
        if self.names and self.names!={key:value[0] for key,value in expected.items()}:
            raise RuntimeError('xarm_service_graph_changed')
        if not self.clients:
            try:
                for key,(name,_) in expected.items():
                    cls=self.types['bool' if key in ('auto','enable') else key]
                    self.clients[key]=self.node.create_client(cls,name)
                    self.names[key]=name
            except Exception:
                self.close_clients();raise
        return dict(self.names)

    def start(self,kind,**kwargs):
        if self.pending is not None:raise RuntimeError('xarm_service_already_pending')
        client=self.clients[kind]
        if not client.service_is_ready():raise RuntimeError('xarm_service_unavailable:'+self.names[kind])
        cls=self.types['bool' if kind in ('auto','enable') else kind];request=cls.Request()
        if kind in ('auto','enable'):
            if type(kwargs['value']) is not bool:raise ValueError('xarm_boolean_request_required')
            request.data=kwargs['value']
        elif kind=='switch':
            setattr(request,self.switch_names[0],list(kwargs.get('activate',())))
            setattr(request,self.switch_names[1],list(kwargs.get('deactivate',())))
            request.strictness=2
            for name in ('activate_asap','start_asap'):
                if hasattr(request,name):setattr(request,name,False)
            request.timeout.sec=0;request.timeout.nanosec=int(self.settings.service_timeout*1e9)
        elif kind=='get':request.names=list(PARAMETERS)
        elif kind=='set':
            parameter=self.types['parameter']();parameter.name='vel_limits'
            value=self.types['value']();value.type=8;value.double_array_value=list(kwargs['velocity'])
            parameter.value=value;request.parameters=[parameter]
        future=client.call_async(request)
        self._pending_late_enable=(kind=='enable' and kwargs['value'] is True) or (
            kind=='switch' and CONTROLLER in kwargs.get('activate',()))
        self.pending=(kind,future,self.clock()+self.settings.service_timeout)

    def poll(self):
        if self.pending is None:return None
        kind,future,deadline=self.pending
        if self.clock()>deadline:
            self.cancel();raise RuntimeError('xarm_service_timeout:'+kind)
        if not future.done():return None
        self.pending=None
        try:response=future.result()
        except Exception as exc:raise RuntimeError('xarm_service_failed:'+kind) from exc
        if response is None:raise RuntimeError('xarm_service_empty_response:'+kind)
        return kind,response

    def cancel(self):
        if self.pending is not None:
            # Cancelling a ROS future cannot cancel server-side execution.
            # A later release ACK cannot prove an older mutation won't run.
            if self._pending_late_enable:self.uncertain_mutation=True
            self.pending[1].cancel();self.pending=None

    def _own_endpoint(self,endpoint):
        return (endpoint.node_name==self.node.get_name() and endpoint.node_namespace==self.node.get_namespace())

    def check_publishers(self):
        endpoints=self.node.get_publishers_info_by_topic(self.settings.topic)
        if any(not self._own_endpoint(endpoint) for endpoint in endpoints):
            raise RuntimeError('xarm_competing_target_publisher')
        own=sum(self._own_endpoint(endpoint) for endpoint in endpoints)
        if own>(1 if self.publisher is not None else 0):raise RuntimeError('xarm_duplicate_local_target_publisher')
        subscribers=[endpoint for endpoint in self.node.get_subscriptions_info_by_topic(self.settings.topic)
                     if endpoint.node_name==self.settings.controller]
        if len(subscribers)!=1:
            raise RuntimeError('xarm_target_subscriber_missing_or_ambiguous')
        if subscribers[0].topic_type!='std_msgs/msg/Float64MultiArray':
            raise RuntimeError('xarm_target_subscriber_type_unverified')
        lower=self.node.get_publishers_info_by_topic('/arm/cmd')
        if any(endpoint.node_name!='EAIHardware' or endpoint.node_namespace!='/' or
               endpoint.topic_type!='ros2_bridge_msgs/msg/ArmCtrl' for endpoint in lower):
            raise RuntimeError('xarm_competing_or_unverified_lowlevel_arm_publisher')
        if len(lower)>1 or (self.hardware_enabled and len(lower)!=1):
            raise RuntimeError('xarm_lowlevel_EAIHardware_publisher_not_unique_or_missing')

    def open_publisher(self):
        if self.publisher is not None:raise RuntimeError('xarm_publisher_already_open')
        self.check_publishers()
        self.publisher=self.node.create_publisher(self.types['array'],self.settings.topic,10)

    def publish(self,positions):
        if self.publisher is None:raise RuntimeError('xarm_publisher_closed')
        self.check_publishers()
        message=self.types['array']();message.data=list(_vector(positions,'target'))
        self.publisher.publish(message)

    def close_publisher(self):
        if self.publisher is not None:
            publisher=self.publisher;self.publisher=None;self._retire_publisher(publisher)

    def _retire_publisher(self,publisher):
        self.entity_retirements=getattr(self,'entity_retirements',0)+1
        self.last_entity_retirement=time.monotonic()
        self.node.destroy_publisher(publisher)

    def close_clients(self):
        self.cancel()
        for client in self.clients.values():self.node.destroy_client(client)
        self.clients={};self.names={}

    def close(self):
        self.close_publisher();self.close_clients()
        if self.subscription is not None:
            self.node.destroy_subscription(self.subscription);self.subscription=None


def _controllers(response):
    result={}
    for item in response.controller:
        if not isinstance(item.name,str) or not item.name or item.name in result or item.state not in ('active','inactive','unconfigured','finalized'):
            raise ValueError('invalid_xarm_controller_list')
        claimed=tuple(item.claimed_interfaces)
        required=tuple(getattr(item,'required_command_interfaces',()))
        if any(not isinstance(value,str) for value in claimed+required):raise ValueError('invalid_xarm_controller_interfaces')
        result[item.name]=dict(state=item.state,type=item.type,claimed=claimed,required=required)
    return result


def _resources(interfaces):
    resources=set()
    for value in interfaces:
        parts=value.rsplit('/',1)
        if len(parts)!=2 or parts[1]!='position':raise ValueError('xarm_nonposition_or_unknown_claim:'+value)
        resources.add(parts[0])
    return resources


def _arm_controller(name,item):
    interfaces=item['claimed']+item['required']
    return bool(set(value.rsplit('/',1)[0] for value in interfaces)&set(JOINT_NAMES)) or name in (CONTROLLER,*EMPTY) or (
        ('arm' in name.lower() or 'wholebody' in name.lower()) and 'broadcaster' not in name.lower())


class XArmAdapter:
    """Explicit operator admission and observed, asynchronous stop confirmation."""
    simulated=False
    def __init__(self,node,feedback,settings,*,clock=time.monotonic,transport=None,hands_config=None):
        if not isinstance(settings,XArmSettings):raise TypeError('XArmSettings_required')
        self.settings,self.feedback,self.clock=settings,feedback,clock
        self.transport=transport if transport is not None else XArmRosTransport(node,settings,clock=clock)
        self.lock=threading.RLock()
        self.limits=SimpleNamespace(lower=settings.lower,upper=settings.upper,velocity=settings.velocity_cap,
            feedback_lead=settings.feedback_lead,command_timeout=settings.command_timeout,
            watchdog_timeout=settings.watchdog_timeout,evidence=settings.limits_evidence)
        self.state='disabled';self.reason='explicit_operator_enable_required';self.stop_confirmed=False
        self._stage=None;self._started=None;self._enabled=False;self._stop_reason=None
        self._release_error=None
        self._target=None;self._last_stamp=None;self._last_output=None
        self._controllers=None;self._controller_stamp=None;self._parameter_stamp=None;self._auto_stamp=None
        self._next_audit=0.;self._parameter_values=None;self._velocity=None;self._deactivate=()
        self.verified_velocity=None
        from .brainco_adapter import BrainCoHandAdapter
        self.hands=BrainCoHandAdapter(node,feedback,parent=self,config_path=hands_config,clock=clock,
                                    command_timeout=settings.command_timeout)
        self.profiles=self.hands.profiles
        self.hand_positions=self.hands.hand_positions;self.hand_diagnostics=self.hands.hand_diagnostics

    @property
    def hardware_writes_enabled(self):return self.state=='armed' and self.transport.publisher is not None

    @property
    def commanded_joint_positions(self):return self._target

    @property
    def output_interval_s(self):
        with self.lock:
            return None if self._last_output is None else self.clock()-self._last_output

    def read_state(self):return self.feedback.read_state()

    def _feedback_ok(self):
        state=self.read_state();now=self.clock()
        if (state.quality not in (('measured','stale') if motion_policy.LIMITS_MANAGED_EXTERNALLY else ('measured',))
                or (not motion_policy.LIMITS_MANAGED_EXTERNALLY and not 0<=now-state.stamp<=self.settings.feedback_timeout)):
            raise RuntimeError('xarm_joint_feedback_missing_or_stale')
        if (state.estop is True if motion_policy.LIMITS_MANAGED_EXTERNALLY else state.estop is not False):raise RuntimeError('xarm_physical_estop_active_or_unknown')
        preparation=getattr(self,'preparation',None)
        if not motion_policy.LIMITS_MANAGED_EXTERNALLY and preparation is not None and self.state in ('acquiring','armed'):preparation.physical_guard()
        if not motion_policy.LIMITS_MANAGED_EXTERNALLY and self.state in ('acquiring','armed'):
            snapshot=self.feedback.snapshot(require_hands=False)
            if state.mode!='HBWALK' or not snapshot['ready']:
                raise RuntimeError('xarm_settled_running_HBWALK_required')
        if not motion_policy.LIMITS_MANAGED_EXTERNALLY and getattr(self.transport,'warning_error',None):raise RuntimeError('xarm_manipulator_error_latched')
        return state

    def _validate_list(self,response,expected):
        controllers=_controllers(response)
        for name in (CONTROLLER,*EMPTY):
            if name not in controllers:raise RuntimeError('xarm_required_controller_missing:'+name)
        target=controllers[CONTROLLER]
        if expected=='armed':
            if target['state']!='active' or _resources(target['claimed'])!=set(JOINT_NAMES) or len(target['claimed'])!=14:
                raise RuntimeError('xarm_dual_arm_claims_not_exact_fourteen')
            if any(name!=CONTROLLER and item['state']=='active' and _arm_controller(name,item)
                   for name,item in controllers.items()):raise RuntimeError('xarm_competing_arm_controller_active')
        elif expected=='inactive':
            if any(item['state']=='active' and _arm_controller(name,item) for name,item in controllers.items()):
                raise RuntimeError('xarm_arm_controller_not_inactive_before_enable')
        elif expected=='empty':
            for name,names in zip(EMPTY,(JOINT_NAMES[:7],JOINT_NAMES[7:])):
                item=controllers[name]
                if item['state']!='active' or _resources(item['claimed'])!=set(names) or len(item['claimed'])!=7:
                    raise RuntimeError('xarm_empty_hold_claims_unconfirmed:'+name)
            if any(item['state']=='active' and name not in EMPTY and _arm_controller(name,item)
                   for name,item in controllers.items()):raise RuntimeError('xarm_motion_controller_still_active')
        self._controllers=controllers;self._controller_stamp=self.clock()
        return controllers

    def _parameters(self,response):
        if len(response.values)!=len(PARAMETERS):raise RuntimeError('xarm_controller_parameters_missing')
        values={}
        for name,value in zip(PARAMETERS[:3],response.values[:3]):
            if type(value.type) is not int or value.type!=8:raise RuntimeError('xarm_parameter_not_double_array:'+name)
            values[name]=_vector(value.double_array_value,name,True)
        joint_values=[]
        for name,value in zip(PARAMETERS[3:],response.values[3:]):
            if type(value.type) is not int:raise RuntimeError('xarm_joint_name_parameter_type_invalid')
            if value.type==0:continue
            if value.type!=9 or tuple(value.string_array_value)!=JOINT_NAMES:
                raise RuntimeError('xarm_joint_name_parameter_order_unverified:'+name)
            joint_values.append(name)
        if not joint_values:raise RuntimeError('xarm_joint_names_and_vector_order_not_observed')
        values['joint_names']=JOINT_NAMES;values['joint_name_parameters']=tuple(joint_values)
        return values

    def _start(self,stage,kind,**kwargs):
        self.transport.start(kind,**kwargs);self._stage=stage

    def request_enable(self,explicit_operator_action=False):
        with self.lock:
            if explicit_operator_action is not True:return SubmitResult(False,'explicit_operator_enable_required')
            if self.state!='disabled':return SubmitResult(False,'disable_before_rearming')
            if getattr(self.transport,'output_fault',None):
                # Reject before mutating state/services. A known successful
                # prior release must not become an unconfirmed stop merely
                # because another enable was attempted on a faulted session.
                self.reason='factory_output_fault_latched_restart_required:'+str(self.transport.output_fault)
                return SubmitResult(False,self.reason)
            try:
                # Field evidence showed the factory's startup position output
                # bypasses its upper-controller velocity cap. The direct wire
                # transport must stay gated until the measured raw-output
                # limiter and settled-acquisition path are installed.
                if not motion_policy.LIMITS_MANAGED_EXTERNALLY and isinstance(self.transport,XArmRosTransport) and not self.transport.bounded_output_enforced:
                    raise RuntimeError('xarm_verified_initial_output_limiter_required')
                state=self._feedback_ok()
                if not motion_policy.LIMITS_MANAGED_EXTERNALLY and (state.mode!='HBWALK' or not self.feedback.snapshot(require_hands=False)['ready']):
                    raise RuntimeError('xarm_settled_running_HBWALK_required')
                self.transport.discover();self.transport.check_publishers()
                self.stop_confirmed=False;self._started=self.clock();self.state='acquiring'
                self._target=None;self._last_stamp=None;self._last_output=None
                self._start('prepare_auto','auto',value=False)
                self.reason='xarm_enable_pending'
                return SubmitResult(False,self.reason)
            except Exception as exc:return self._fault(str(exc))

    def _ack(self,response,field):
        if getattr(response,field,None) is not True:raise RuntimeError('xarm_service_ack_rejected:'+self._stage)

    def poll_enable(self):
        with self.lock:
            try:
                self._pump()
                return SubmitResult(self.hardware_writes_enabled,self.reason)
            except Exception as exc:return self._fault(str(exc))

    def _pump(self):
        now=self.clock()
        if self.state in ('acquiring','armed','holding'):self._feedback_ok()
        if not motion_policy.LIMITS_MANAGED_EXTERNALLY and self.state=='armed' and self.hands.check_watchdog():raise RuntimeError('xarm_configured_hand_watchdog_failed')
        if self.state in ('acquiring','releasing','holding') and self._stage is not None and now-self._started>self.settings.transition_timeout:
            raise RuntimeError('xarm_transition_timeout:'+str(self._stage))
        if self.state=='armed' and self.transport.pending is None and now>=self._next_audit:
            self.transport.discover();self.transport.check_publishers()
            self._next_audit=now+self.settings.audit_interval
            self._start('audit_auto','auto',value=False)
        if self.state=='acquiring' and self._stage=='initial_alignment':
            self._fresh_status()
            if self.transport.poll_initial_alignment(self._target):
                self._target=self.transport.commit_initial_alignment()
                self._last_output=self.clock()
                self.state='armed';self.reason='xarm_initial_raw_alignment_confirmed'
                self._next_audit=self.clock()+self.settings.audit_interval;self._stage=None
            return
        result=self.transport.poll()
        if result is None:return
        _,response=result;stage=self._stage
        if stage in ('prepare_auto','audit_auto','stop_auto'):
            self._ack(response,'success');self._auto_stamp=now
            self._start({'prepare_auto':'prepare_list','audit_auto':'audit_list','stop_auto':'stop_list'}[stage],'list')
        elif stage=='prepare_list':
            controllers=self._validate_list(response,'initial')
            required=controllers[CONTROLLER]['required']
            if required and (_resources(required)!=set(JOINT_NAMES) or len(required)!=14):
                raise RuntimeError('xarm_required_arm_resources_not_exact_fourteen')
            active=[]
            for name,item in controllers.items():
                if item['state']=='active' and _arm_controller(name,item):
                    # A controller touching body/head/leg resources cannot be
                    # stopped through this arms-only adapter.
                    if not item['claimed'] or not _resources(item['claimed'])<=set(JOINT_NAMES):
                        raise RuntimeError('xarm_active_controller_has_nonarm_or_unknown_resources:'+name)
                    active.append(name)
            if active:self._start('prepare_deactivated','switch',deactivate=active)
            else:
                self._validate_list(response,'inactive');self._start('read_limits','get')
        elif stage=='prepare_deactivated':
            self._ack(response,'ok');self._start('inactive_check','list')
        elif stage=='inactive_check':
            self._validate_list(response,'inactive');self._start('read_limits','get')
        elif stage=='enable_ack':
            self._ack(response,'success');self._enabled=True;self.transport.hardware_enabled=True
            self.transport.check_publishers()
            self._start('activate_ack','switch',activate=(CONTROLLER,))
        elif stage=='read_limits':
            values=self._parameters(response);self._parameter_values=values
            if motion_policy.LIMITS_MANAGED_EXTERNALLY:
                self._velocity=tuple(values['vel_limits'])
                self._parameter_stamp=now;self.verified_velocity=None
                self._start('pre_enable_inactive_check','list')
                return
            self._velocity=tuple(min(actual,cap) for actual,cap in zip(values['vel_limits'],self.settings.velocity_cap))
            self._start('set_limits','set',velocity=self._velocity)
        elif stage=='set_limits':
            if len(response.results)!=1 or response.results[0].successful is not True:
                raise RuntimeError('xarm_velocity_cap_set_rejected')
            self._start('verify_limits','get')
        elif stage=='verify_limits':
            values=self._parameters(response)
            if values['vel_limits']!=self._velocity or any(values[name]!=self._parameter_values[name] for name in ('acc_limits','jerk_limits')):
                raise RuntimeError('xarm_velocity_cap_readback_or_unchanged_dynamics_failed')
            self._parameter_values=values;self._parameter_stamp=now
            self.verified_velocity=self._velocity
            self._start('pre_enable_inactive_check','list')
        elif stage=='pre_enable_inactive_check':
            self._validate_list(response,'inactive');self.transport.check_publishers()
            self._start('enable_ack','enable',value=True)
        elif stage=='activate_ack':
            self._ack(response,'ok');self._start('active_check','list')
        elif stage=='active_check':
            self._validate_list(response,'armed');self.transport.check_publishers()
            state=self._feedback_ok();self.transport.open_publisher()
            self._target=state.positions;self._last_output=now
            if not motion_policy.LIMITS_MANAGED_EXTERNALLY and getattr(self.transport,'initial_alignment_required',False):
                self.transport.begin_initial_alignment(self._target)
                self._stage='initial_alignment';self.reason='xarm_waiting_initial_raw_alignment_no_actuator_output'
                return
            self.state='armed';self.reason='xarm_active_claims_and_enable_ack_observed'
            self._next_audit=now+self.settings.audit_interval;self._stage=None
        elif stage=='audit_list':
            self._validate_list(response,'armed');self._start('audit_limits','get')
        elif stage=='audit_limits':
            values=self._parameters(response)
            if values!=self._parameter_values:raise RuntimeError('xarm_controller_parameters_changed')
            self._parameter_stamp=now;self._stage=None
        elif stage=='stop_list':
            controllers=self._validate_list(response,'initial')
            deactivate=[]
            for name,item in controllers.items():
                if item['state']=='active' and name not in EMPTY and _arm_controller(name,item):
                    if not item['claimed'] or not _resources(item['claimed'])<=set(JOINT_NAMES):
                        raise RuntimeError('xarm_cannot_stop_nonarm_or_unknown_resource_controller')
                    deactivate.append(name)
            activate=[name for name in EMPTY if controllers[name]['state']!='active']
            if activate or deactivate:self._start('empty_ack','switch',activate=activate,deactivate=deactivate)
            else:
                self._validate_list(response,'empty');self._empty_confirmed()
        elif stage=='empty_ack':
            self._ack(response,'ok');self._start('empty_check','list')
        elif stage=='empty_check':
            self._validate_list(response,'empty');self._empty_confirmed()
        elif stage=='disable_ack':
            self._ack(response,'success');self._enabled=False;self.transport.hardware_enabled=False
            if getattr(self.transport,'uncertain_mutation',False):
                raise RuntimeError('xarm_late_mutation_cannot_be_excluded_after_timeout_or_cancel')
            self.stop_confirmed=True
            self.state='disabled';self.reason='xarm_empty_observed_and_arm_disable_ack';self._stage=None
        elif stage=='emergency_disable_ack':
            self._ack(response,'success');self._enabled=False;self.transport.hardware_enabled=False
            self.state='fault';self.reason='xarm_stop_unconfirmed:'+str(self._release_error)
            self.stop_confirmed=False;self._stage=None

    def _empty_confirmed(self):
        self._start('disable_ack','enable',value=False)

    def _fresh_status(self):
        if motion_policy.LIMITS_MANAGED_EXTERNALLY:return
        preparation=getattr(self,'preparation',None)
        guard=getattr(preparation,'command_guard',None)
        if guard:guard.check()
        now=self.clock()
        for stamp in (self._controller_stamp,self._parameter_stamp,self._auto_stamp):
            if stamp is None or not 0<=now-stamp<=self.settings.status_timeout:
                raise RuntimeError('xarm_controller_parameter_or_auto_ack_stale')

    def submit(self,command):
        with self.lock:
            if not isinstance(command,JointCommand):raise TypeError('JointCommand_required')
            try:
                self._pump()
                if not self.hardware_writes_enabled:return SubmitResult(False,'xarm_output_not_armed:'+self.state)
                self._fresh_status();state=self._feedback_ok();now=self.clock()
                if motion_policy.LIMITS_MANAGED_EXTERNALLY:
                    self.transport.publish(command.positions)
                    self._target=command.positions;self._last_stamp=command.stamp;self._last_output=now
                    return SubmitResult(True,'xarm_target_published_limits_external')
                if not 0<=now-command.stamp<=self.settings.command_timeout:raise RuntimeError('xarm_command_stale_or_future')
                if self._last_stamp is not None and command.stamp<=self._last_stamp:raise RuntimeError('xarm_command_replayed')
                dt=now-self._last_output
                if not 0<dt<=self.settings.watchdog_timeout:raise RuntimeError('xarm_output_gap_or_nonpositive_dt')
                for i,(q,v) in enumerate(zip(command.positions,command.velocities)):
                    if not self.limits.lower[i]<=q<=self.limits.upper[i]:raise RuntimeError('xarm_joint_bound_exceeded')
                    if abs(v)>self._velocity[i]+1e-9 or abs(q-self._target[i])>self._velocity[i]*dt+1e-9:
                        raise RuntimeError('xarm_joint_velocity_cap_exceeded')
                    if abs(q-state.positions[i])>self.limits.feedback_lead[i]:raise RuntimeError('xarm_feedback_lead_exceeded')
                self.transport.publish(command.positions)
                self._target=command.positions;self._last_stamp=command.stamp;self._last_output=now
                return SubmitResult(True,'xarm_fourteen_joint_target_published')
            except Exception as exc:return self._fault(str(exc))

    def _stop(self,reason):
        hand_result=self.hands.invalidate('xarm_stopping:'+str(reason))
        self.transport.close_publisher()
        # The only auto request this adapter issues is false. Reuse its
        # pending disable ACK rather than cancelling an in-flight mutation
        # merely because the operator stopped during a periodic audit.
        pending_auto=bool(self.transport.pending and self.transport.pending[0]=='auto')
        if not pending_auto:self.transport.cancel()
        self.stop_confirmed=False;self._stop_reason=str(reason)
        self._release_error=None
        self._started=self.clock();self.state='releasing'
        self.reason='xarm_release_pending:'+str(reason)
        if not hand_result.accepted:raise RuntimeError('xarm_hand_publisher_cleanup_unconfirmed:'+hand_result.reason)
        self.transport.discover()
        if pending_auto:self._stage='stop_auto'
        else:self._start('stop_auto','auto',value=False)
        return SubmitResult(False,self.reason)

    def hold(self):
        with self.lock:
            if self.state!='armed' or self._target is None:return SubmitResult(False,'xarm_hold_requires_armed')
            # A clutch pause keeps the last accepted target. It never chases a
            # measured spring-back pose and never claims a hardware hold ACK.
            result=self.submit(JointCommand(self._target,(0.,)*14,self.clock()))
            return SubmitResult(result.accepted,'xarm_last_target_republished_for_clutch_pause' if result.accepted else result.reason)

    def disarm(self,reason='operator_disarm'):
        with self.lock:
            if self.state=='disabled':
                # Default-off station cleanup/watchdogs must never initiate
                # controller or hardware mutations before operator admission.
                return SubmitResult(True,self.reason if self.stop_confirmed else 'xarm_already_disabled_no_session_to_release')
            if self.state=='releasing':return SubmitResult(False,self.reason)
            try:return self._stop(reason)
            except Exception as exc:return self._fault(str(exc))

    def _fault(self,reason):
        if motion_policy.LIMITS_MANAGED_EXTERNALLY and 'physical_estop' not in str(reason):
            self.reason=str(reason)
            return SubmitResult(False,self.reason)
        hand_result=self.hands.invalidate('xarm_fault:'+str(reason))
        if not hand_result.accepted:reason+=';hand_publisher_cleanup_unconfirmed:'+hand_result.reason
        self.transport.close_publisher();self.transport.cancel()
        if self.state in ('acquiring','armed','holding'):
            try:return self._stop('fault:'+reason)
            except Exception as exc:reason+=';stop_start_failed:'+str(exc)
        if self.state=='releasing' and self._stage!='emergency_disable_ack':
            # If switching/observing empty failed, still request hardware
            # release. Its ACK cannot turn that failed stop into a success.
            self._release_error=reason;self._started=self.clock()
            self.reason='xarm_emergency_arm_disable_pending:'+reason
            try:
                self._start('emergency_disable_ack','enable',value=False)
                return SubmitResult(False,self.reason)
            except Exception as exc:reason+=';arm_disable_start_failed:'+str(exc)
        self.state='fault';self.reason='xarm_stop_unconfirmed:'+reason;self.stop_confirmed=False;self._stage=None
        return SubmitResult(False,self.reason)

    def check_watchdog(self):
        with self.lock:
            try:
                if not motion_policy.LIMITS_MANAGED_EXTERNALLY and self.state=='armed' and (self._last_output is None or self.clock()-self._last_output>self.settings.watchdog_timeout):
                    self._stop('command_watchdog');return True
                self._pump()
                if self.state=='armed':self._fresh_status()
                return self.state in ('releasing','fault')
            except Exception as exc:self._fault(str(exc));return True

    def status(self):
        with self.lock:
            return dict(state=self.state,reason=self.reason,hardware_writes_enabled=self.hardware_writes_enabled,
                stop_confirmed=self.stop_confirmed,stage=self._stage,controllers=self._controllers,
                stop_failure=self._release_error,last_stop_reason=self._stop_reason,
                controller_received_at=self._controller_stamp,parameters=self._parameter_values,
                parameter_received_at=self._parameter_stamp,auto_switch_disable_ack_at=self._auto_stamp,
                auto_switch_observation='disable_service_ack_only_no_documented_readback',
                lowlevel_arm_output_expected=('/EAIHardware -> /tg3/xarm/arm_raw -> measured limiter -> /arm/cmd'
                    if getattr(self.transport,'bounded_output_enforced',False) else '/EAIHardware -> /arm/cmd:ros2_bridge_msgs/msg/ArmCtrl'),
                bounded_output=dict(enforced=getattr(self.transport,'bounded_output_enforced',False),
                    holding_profile=(self.transport.holding_profile.status() if getattr(self.transport,'holding_profile',None) else None),
                    actual_count=getattr(self.transport,'actual_count',None),
                    maximum_step_rad=getattr(self.transport,'maximum_step',None),
                    fault=getattr(self.transport,'output_fault',None),
                    initial_alignment=self.transport.alignment_status() if hasattr(self.transport,'alignment_status') else None,
                    feedback_fault_details=getattr(self.transport,'feedback_fault_details',None)),
                hardware_arm_enable_acknowledged=self._enabled,services=dict(self.transport.names),
                hand_output_enabled=self.hands.hardware_writes_enabled,hands=self.hands.status(),
                commanded_joint_positions=self._target,
                verified_velocity=self.verified_velocity,
                manipulator_warning=getattr(self.transport,'last_warning',None))

    def measured_hand_initialization(self):return self.hands.measured_hand_initialization()

    def submit_hand(self,*args,**kwargs):return self.hands.submit_hand(*args,**kwargs)

    def tick_transition(self):return self.poll_enable()


class XArmRosSession:
    """Private ROS context/executor; only explicit enable mutates hardware."""
    @classmethod
    def create(cls,*,settings,wait_for_feedback_s=1.,monotonic=time.monotonic,hands_config=None,managed=False):
        return cls(settings=settings,wait_for_feedback_s=wait_for_feedback_s,monotonic=monotonic,hands_config=hands_config,managed=managed)

    def __init__(self,*,settings,wait_for_feedback_s=1.,monotonic=time.monotonic,hands_config=None,managed=False):
        if not isinstance(settings,XArmSettings):raise TypeError('XArmSettings_required')
        if isinstance(wait_for_feedback_s,bool) or not isinstance(wait_for_feedback_s,Real) or not 0<=wait_for_feedback_s<=5:
            raise ValueError('invalid_xarm_feedback_wait')
        import rclpy
        from rclpy.context import Context
        from rclpy.node import Node
        from rclpy.executors import SingleThreadedExecutor
        from .manufacturer_feedback import ManufacturerFeedback
        self.context=Context();self.node=None;self.executor=None;self.feedback=None;self.robot=None
        self.closed=False;self.cleanup_errors=[];self.executor_error=None
        self._exit=threading.Event();self._thread=None
        try:
            rclpy.init(args=[],context=self.context)
            self.node=Node('tg3_xarm_'+str(time.monotonic_ns()),context=self.context,
                enable_rosout=False,start_parameter_services=False,use_global_arguments=False)
            self.feedback=ManufacturerFeedback(clock=monotonic).subscribe(self.node)
            transport=None
            if managed:
                from .bounded_xarm_transport import BoundedXArmTransport
                from .holding_profile import load_holding_profile
                profile=load_holding_profile()
                transport=BoundedXArmTransport(self.node,settings,clock=monotonic,holding_profile=profile)
            self.robot=XArmAdapter(self.node,self.feedback,settings,clock=monotonic,hands_config=hands_config,transport=transport)
            if managed:
                from .xarm_preparation import XArmPreparation
                from .stationary_guard import StationaryCommandGuard
                transport.adapter=self.robot;transport.feedback=self.feedback
                guard=StationaryCommandGuard(self.node,clock=monotonic)
                self.robot.preparation=XArmPreparation(self.node,self.feedback,clock=monotonic,
                    arm_mode=profile.arm_mode,command_guard=guard)
            self.executor=SingleThreadedExecutor(context=self.context);self.executor.add_node(self.node)
            self._thread=threading.Thread(target=self._spin,name='xarm-readonly-executor',daemon=True)
            self._thread.start()
            deadline=time.monotonic()+wait_for_feedback_s
            while wait_for_feedback_s and time.monotonic()<deadline:
                try:self.robot._feedback_ok();break
                except RuntimeError:time.sleep(.01)
            # Absence of live feedback leaves the session disabled. This allows
            # diagnostics on a powered-off robot without an enable attempt.
        except BaseException:
            self.close();raise

    def _spin(self):
        retired_seen=0
        try:
            while not self._exit.is_set() and self.context.ok():
                try:self.executor.spin_once(timeout_sec=.02)
                except Exception as exc:
                    from rclpy.handle import InvalidHandle
                    transport=self.robot.transport if self.robot else None
                    generation=getattr(transport,'entity_retirements',0)
                    recent=time.monotonic()-getattr(transport,'last_entity_retirement',-float('inf'))<=.15
                    if isinstance(exc,InvalidHandle) and recent and retired_seen<generation:
                        # A retired publisher can remain in one cached wait set.
                        # Rebuild that set so disable/restore service ACKs still
                        # run. Each own retirement permits at most one retry;
                        # persistent or unrelated handle failures remain fatal.
                        retired_seen+=1
                        continue
                    raise
        except Exception as exc:
            self.executor_error=str(exc)
            if self.robot:
                with self.robot.lock:self.robot._fault('xarm_executor_failed:'+str(exc))

    def status(self):
        result=self.robot.status() if self.robot else {}
        result.update(closed=self.closed,executor_error=self.executor_error,cleanup_errors=list(self.cleanup_errors))
        return result

    def close(self):
        if self.closed:return
        # Only cleanup may wait. The control-loop APIs above never wait/spin.
        if self.robot is not None and self.robot.state!='disabled':
            self.robot.disarm('session_close');deadline=time.monotonic()+2.
            while self.robot.state=='releasing' and time.monotonic()<deadline and self.executor_error is None:
                self.robot.tick_transition();time.sleep(.01)
            if not self.robot.stop_confirmed:self.cleanup_errors.append('xarm_stop_unconfirmed_on_close:'+self.robot.reason)
        preparation=getattr(self.robot,'preparation',None)
        if preparation is not None:
            preparation.cancel();deadline=time.monotonic()+5.
            try:
                while time.monotonic()<deadline:
                    safe=self.robot.stop_confirmed or (self.robot.state=='disabled' and not self.robot._enabled)
                    if preparation.restore_step(safe_to_restore=safe):break
                    time.sleep(.01)
                else:self.cleanup_errors.append('factory_mode_restoration_unconfirmed')
            except Exception as exc:self.cleanup_errors.append('factory_mode_restoration:'+str(exc))
        self._exit.set()
        for label,operation in (
            ('preparation',lambda:preparation.close() if preparation is not None else None),
            ('hands',lambda:self.robot.hands.close() if self.robot else None),
            ('transport',lambda:self.robot.transport.close() if self.robot else None),
            ('feedback',lambda:self.feedback.close() if self.feedback else None),
            ('executor',lambda:self.executor.shutdown(timeout_sec=1.) if self.executor else None)):
            try:
                result=operation()
                if hasattr(result,'accepted') and result.accepted is not True:
                    self.cleanup_errors.append(label+':'+result.reason)
            except Exception as exc:self.cleanup_errors.append(label+':'+str(exc))
        if self._thread is not None:self._thread.join(timeout=1.)
        if self._thread is not None and self._thread.is_alive():self.cleanup_errors.append('xarm_executor_thread_not_stopped')
        if self.node is not None:
            try:self.node.destroy_node()
            except Exception as exc:self.cleanup_errors.append('node:'+str(exc))
        try:
            if self.context.ok():self.context.shutdown()
        except Exception as exc:self.cleanup_errors.append('context:'+str(exc))
        self.closed=True

    def __enter__(self):return self

    def __exit__(self,*args):self.close()
