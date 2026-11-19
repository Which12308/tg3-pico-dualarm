"""XARM state machine and actual ROS wire shape tested with labelled doubles.

These tests do not establish installed controllers, SDK availability or any
successful physical motion, stop, enable, collision protection or resource claim.
"""
from types import ModuleType,SimpleNamespace as NS
from dataclasses import asdict
import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

from tg3_pico_dualarm.manufacturer_feedback import ManufacturerFeedback
from tg3_pico_dualarm.model import JOINT_NAMES,default_q
from tg3_pico_dualarm.robot import JointCommand
from tg3_pico_dualarm.xarm_adapter import (XArmAdapter,XArmRosTransport,XArmSettings,XArmRosSession,
    CONTROLLER,TOPIC,EMPTY,PARAMETERS)
from test_manufacturer import arm_message,mode_message,power_key_message


def settings(**overrides):
    values=dict(velocity_cap=(.1,)*14,lower=(-3.,)*14,upper=(3.,)*14,
                feedback_lead=(.1,)*14,limits_evidence='TEST ONLY manually reviewed fixture')
    values.update(overrides);return XArmSettings(**values)


def controller(name,state,claims=(),required=()):
    return NS(name=name,state=state,type='TEST ONLY controller',claimed_interfaces=list(claims),
              required_command_interfaces=list(required))


def claims(names=JOINT_NAMES):return tuple(name+'/position' for name in names)


class TransportDouble:
    """No ROS communication: service replies are advanced explicitly by tests."""
    def __init__(self,clock):
        self.clock=clock;self.names={'enable':'TEST ONLY'};self.pending=None
        self.publisher=None;self.events=[];self.warning_error=None;self.last_warning=None
        self.uncertain_mutation=False;self.competitor=False;self.completed=None
        self.controllers={CONTROLLER:controller(CONTROLLER,'inactive',required=claims()),
            EMPTY[0]:controller(EMPTY[0],'active',claims(JOINT_NAMES[:7])),
            EMPTY[1]:controller(EMPTY[1],'active',claims(JOINT_NAMES[7:]))}
        self.params={'vel_limits':(.2,)*14,'acc_limits':(1.,)*14,'jerk_limits':(10.,)*14}
        self.reject=None;self.ignore_switch=False;self.published=[]
        self.joint_names=JOINT_NAMES;self.joint_params_missing=False
    def discover(self):self.events.append(('discover',));return self.names
    def check_publishers(self):
        if self.competitor:raise RuntimeError('xarm_competing_target_publisher')
    def start(self,kind,**kwargs):
        if self.pending is not None:raise AssertionError('parallel service mutation')
        self.events.append((kind,kwargs));self.pending=(kind,kwargs,self.clock()+.15)
    def complete(self):
        if self.pending is None:return
        kind,kwargs,_=self.pending
        if kind in ('auto','enable'):response=NS(success=kind!=self.reject,message='TEST ONLY ACK')
        elif kind=='list':response=NS(controller=list(self.controllers.values()))
        elif kind=='switch':
            if not self.ignore_switch and kind!=self.reject:
                for name in kwargs.get('deactivate',()):self.controllers[name].state='inactive';self.controllers[name].claimed_interfaces=[]
                for name in kwargs.get('activate',()):
                    self.controllers[name].state='active'
                    self.controllers[name].claimed_interfaces=list(claims() if name==CONTROLLER else
                        claims(JOINT_NAMES[:7] if name==EMPTY[0] else JOINT_NAMES[7:]))
            response=NS(ok=kind!=self.reject)
        elif kind=='get':response=NS(values=[NS(type=8,double_array_value=list(self.params[name])) for name in PARAMETERS[:3]]+
            [NS(type=0) if self.joint_params_missing else NS(type=9,string_array_value=list(self.joint_names))])
        elif kind=='set':
            if kind!=self.reject:self.params['vel_limits']=tuple(kwargs['velocity'])
            response=NS(results=[NS(successful=kind!=self.reject)])
        self.completed=(kind,response)
    def poll(self):
        if self.pending and self.clock()>self.pending[2]:
            kind=self.pending[0];self.cancel();raise RuntimeError('xarm_service_timeout:'+kind)
        result=self.completed
        if result:self.completed=None;self.pending=None
        return result
    def cancel(self):
        if self.pending and self.pending[0] in ('enable','switch','auto','set'):self.uncertain_mutation=True
        self.pending=None;self.completed=None
    def open_publisher(self):self.check_publishers();self.publisher=object();self.events.append(('publisher_open',))
    def publish(self,q):self.check_publishers();self.published.append(tuple(q))
    def close_publisher(self):
        if self.publisher:self.events.append(('publisher_close',))
        self.publisher=None


class XArmAdapterTests(unittest.TestCase):
    def setUp(self):
        self.now=[10.];self.clock=lambda:self.now[0]
        self.feedback=ManufacturerFeedback(clock=self.clock);self.refresh()
        self.transport=TransportDouble(self.clock)
        self.robot=XArmAdapter(None,self.feedback,settings(),clock=self.clock,transport=self.transport)

    def test_prior_output_fault_cannot_mutate_enable_or_erase_known_release(self):
        self.transport.output_fault='real_output_feedback_lead';self.robot.stop_confirmed=True
        events=list(self.transport.events)
        result=self.robot.request_enable(True)
        self.assertFalse(result.accepted);self.assertIn('fault_latched',result.reason)
        self.assertEqual(self.robot.state,'disabled');self.assertTrue(self.robot.stop_confirmed)
        self.assertEqual(self.transport.events,events);self.assertIsNone(self.transport.pending)
    def refresh(self):
        self.feedback.arm(arm_message(tuple(default_q())))
        mode=mode_message()
        for value in mode.values:
            if value.key in ('current_state','child_state'):value.value='HBWALK'
        self.feedback.fsm(mode)
        self.feedback.key_status(power_key_message())
    def step(self,dt=.005,complete=True):
        self.now[0]+=dt;self.refresh()
        if complete:self.transport.complete()
        return self.robot.tick_transition()
    def arm(self):
        self.assertFalse(self.robot.request_enable(True).accepted)
        for _ in range(30):
            self.step()
            if self.robot.state!='acquiring':break
        self.assertEqual(self.robot.state,'armed',self.robot.reason)
    def release(self):
        for _ in range(20):
            self.step()
            if self.robot.state!='releasing':break
    def command(self,delta=.0001):
        self.now[0]+=.02;self.refresh()
        return JointCommand(tuple(q+delta for q in self.robot.commanded_joint_positions),(.01,)*14,self.clock())

    def test_constructor_and_implicit_enable_create_no_motion_or_service_requests(self):
        self.assertEqual(self.transport.events,[]);self.assertIsNone(self.transport.publisher)
        self.assertFalse(self.robot.request_enable(False).accepted)
        self.assertEqual(self.transport.events,[]);self.assertEqual(self.robot.profiles,{})
        self.assertIsNone(self.robot.verified_velocity)
        self.assertEqual(self.robot.limits.velocity,(.1,)*14)

    def test_enable_inactive_ack_limits_readback_claims_then_publisher_without_initial_motion(self):
        self.arm();events=self.transport.events
        kinds=[value[0] for value in events]
        self.assertEqual(kinds,['discover','auto','list','switch','list','get','set','get','list','enable','switch','list','publisher_open'])
        self.assertEqual(events[3][1]['deactivate'],list(EMPTY))
        self.assertIs(events[9][1]['value'],True)
        self.assertEqual(self.robot.verified_velocity,(.1,)*14)
        self.assertEqual(self.transport.params['acc_limits'],(1.,)*14)
        self.assertEqual(self.transport.params['jerk_limits'],(10.,)*14)
        self.assertEqual(self.transport.published,[])
        self.assertEqual(self.robot.commanded_joint_positions,tuple(default_q()))
        self.assertFalse(self.robot.stop_confirmed)

    def test_actual_lower_velocity_is_preserved_and_not_increased_to_operator_cap(self):
        self.transport.params['vel_limits']=(.04,)*14;self.arm()
        self.assertEqual(self.robot.verified_velocity,(.04,)*14)
        self.assertEqual(self.transport.params['vel_limits'],(.04,)*14)

    def test_list_after_switch_must_really_be_inactive_before_enable(self):
        self.transport.ignore_switch=True;self.robot.request_enable(True)
        for _ in range(4):self.step()
        self.assertNotIn(('enable',{'value':True}),self.transport.events)
        self.assertIsNone(self.transport.publisher)

    def test_active_controller_cannot_claim_waist_or_legs_and_no_all_body_enable_exists(self):
        self.transport.controllers['endpose_wholebody_controller']=controller('endpose_wholebody_controller','active',
            claims()+('waist_pitch_joint/position',))
        self.robot.request_enable(True);self.step();self.step()
        self.assertNotIn(('enable',{'value':True}),self.transport.events)
        self.assertFalse(self.robot.hardware_writes_enabled)

    def test_claims_after_activation_must_match_exact_fourteen_position_joints(self):
        self.robot.request_enable(True)
        for _ in range(20):
            if self.robot._stage=='active_check':break
            self.step()
        self.transport.controllers[CONTROLLER].claimed_interfaces[-1]='waist_pitch_joint/position'
        self.step()
        self.assertFalse(self.robot.hardware_writes_enabled);self.assertEqual(self.transport.published,[])
        self.assertNotIn(('publisher_open',),self.transport.events)

    def test_joint_vector_order_missing_or_wrong_and_bad_declared_resources_prevent_hardware_enable(self):
        for failure in ('order','missing','required'):
            with self.subTest(failure=failure):
                self.setUp()
                if failure=='order':self.transport.joint_names=tuple(reversed(JOINT_NAMES))
                elif failure=='missing':self.transport.joint_params_missing=True
                else:self.transport.controllers[CONTROLLER].required_command_interfaces=list(claims()[:-1])+['waist_pitch_joint/position']
                self.robot.request_enable(True)
                for _ in range(12):self.step()
                self.assertNotIn(('enable',{'value':True}),self.transport.events)
                self.assertIsNone(self.transport.publisher);self.assertFalse(self.robot.hardware_writes_enabled)

    def test_legacy_list_without_required_interfaces_uses_observed_ordered_joint_parameter(self):
        self.transport.controllers[CONTROLLER].required_command_interfaces=[]
        self.arm();self.assertEqual(self.robot.status()['parameters']['joint_names'],JOINT_NAMES)

    def test_real_false_or_missing_estop_is_required_and_mode_does_not_fill_it(self):
        self.feedback.key_status(power_key_message(remote_estop=True))
        self.assertFalse(self.robot.request_enable(True).accepted)
        self.assertEqual(self.transport.events,[])

    def test_full_fourteen_target_order_and_hold_preserve_commanded_target_then_resume(self):
        self.arm();command=self.command();self.assertTrue(self.robot.submit(command).accepted,self.robot.reason)
        target=command.positions
        self.now[0]+=.02;self.refresh()
        result=self.robot.hold();self.assertTrue(result.accepted,result.reason)
        self.assertEqual(self.transport.published[-1],target)
        self.assertEqual(self.robot.commanded_joint_positions,target)
        self.assertEqual(self.robot.state,'armed')
        command=self.command();self.assertTrue(self.robot.submit(command).accepted,self.robot.reason)
        self.assertEqual(len(self.transport.published[-1]),14)
        self.assertEqual(sum(kind=='enable' for kind,*_ in self.transport.events),1)

    def test_velocity_roundoff_at_verified_cap_does_not_release_hardware(self):
        import math
        self.arm();command=self.command()
        velocity=math.nextafter(self.robot.verified_velocity[0],math.inf)
        result=self.robot.submit(JointCommand(command.positions,(velocity,)*14,command.stamp))
        self.assertTrue(result.accepted,result.reason)
        self.assertEqual(self.robot.state,'armed')
        self.assertEqual(len(self.transport.published),1)

    def test_real_velocity_excess_still_rejects_before_publication(self):
        self.arm();command=self.command()
        velocity=self.robot.verified_velocity[0]+.000001
        result=self.robot.submit(JointCommand(command.positions,(velocity,)*14,command.stamp))
        self.assertFalse(result.accepted)
        self.assertIn('velocity_cap_exceeded',result.reason)
        self.assertEqual(self.transport.published,[])

    def test_bad_commands_stop_output_without_publishing(self):
        for failure in ('bounds','velocity','delta','lead','stale','replay'):
            with self.subTest(failure=failure):
                self.setUp();self.arm();command=self.command();q=list(command.positions);v=list(command.velocities)
                if failure=='bounds':q[0]=4.
                elif failure=='velocity':v[0]=.2
                elif failure=='delta':q[0]+=.02
                elif failure=='lead':
                    self.robot._target=tuple(x+.2 for x in command.positions);q=list(self.robot._target)
                elif failure=='stale':command=JointCommand(command.positions,command.velocities,self.clock()-.2)
                else:self.robot._last_stamp=command.stamp
                if failure not in ('stale','replay'):command=JointCommand(q,v,command.stamp)
                result=self.robot.submit(command)
                self.assertFalse(result.accepted);self.assertFalse(self.robot.hardware_writes_enabled)
                self.assertEqual(self.transport.published,[])
                self.assertEqual(self.robot.state,'releasing')

    def test_stop_is_pending_until_empty_claims_and_disable_ack_and_never_publishes_target(self):
        self.arm();result=self.robot.disarm('TEST stop')
        self.assertFalse(result.accepted);self.assertEqual(self.robot.state,'releasing')
        self.assertFalse(self.robot.stop_confirmed);self.assertIsNone(self.transport.publisher)
        self.release()
        self.assertEqual(self.robot.state,'disabled',self.robot.reason);self.assertTrue(self.robot.stop_confirmed)
        self.assertEqual(self.transport.events[-1],('enable',{'value':False}))
        self.assertEqual(self.transport.published,[])
        self.assertTrue(self.robot.disarm().accepted)

    def test_failed_empty_switch_or_disable_never_claims_disabled_or_confirmed(self):
        for rejected in ('switch','enable'):
            with self.subTest(rejected=rejected):
                self.setUp();self.arm();self.transport.reject=rejected
                self.robot.disarm();self.release()
                self.assertEqual(self.robot.state,'fault');self.assertFalse(self.robot.stop_confirmed)
                self.assertFalse(self.robot.hardware_writes_enabled)

    def test_pending_services_never_wait_and_mutation_timeout_cannot_claim_confirmed_stop(self):
        self.robot.request_enable(True)
        while self.robot._stage!='enable_ack':self.step()
        for _ in range(100):self.assertFalse(self.robot.tick_transition().accepted)
        self.now[0]+=.16;self.refresh();self.robot.tick_transition();self.release()
        self.assertEqual(self.robot.state,'fault');self.assertFalse(self.robot.stop_confirmed)
        self.assertTrue(self.transport.uncertain_mutation)

    def test_stale_feedback_estop_error_and_competitor_remove_publisher(self):
        for failure in ('estop','error','competitor','stale'):
            with self.subTest(failure=failure):
                self.setUp();self.arm();command=self.command()
                if failure=='estop':self.feedback.key_status(power_key_message(estop=True))
                elif failure=='error':self.transport.warning_error={'code':600100}
                elif failure=='competitor':self.transport.competitor=True
                else:self.now[0]+=.26
                self.assertFalse(self.robot.submit(command).accepted)
                self.assertIsNone(self.transport.publisher);self.assertEqual(self.transport.published,[])

    def test_watchdog_starts_real_release_and_audits_rate_is_at_most_two_hz(self):
        self.arm()
        for _ in range(30):
            self.now[0]+=.02;self.refresh();result=self.robot.hold()
            self.assertTrue(result.accepted,result.reason)
            self.transport.complete();self.robot.tick_transition()
        auto=[event for event in self.transport.events if event[0]=='auto']
        self.assertLessEqual(len(auto),3)
        self.now[0]+=.151;self.refresh();self.assertTrue(self.robot.check_watchdog())
        self.assertEqual(self.robot.state,'releasing');self.assertIsNone(self.transport.publisher)

    def test_parameter_change_or_active_claim_loss_during_periodic_audit_releases(self):
        for failure in ('parameters','claims'):
            with self.subTest(failure=failure):
                self.setUp();self.arm();self.now[0]+=.02;self.refresh();self.robot.hold()
                self.robot._next_audit=self.clock()
                if failure=='parameters':self.transport.params['vel_limits']=(.2,)*14
                else:self.transport.controllers[CONTROLLER].claimed_interfaces=[]
                for _ in range(4):self.step()
                self.assertEqual(self.robot.state,'releasing');self.assertIsNone(self.transport.publisher)

    def test_settings_reject_unreviewed_controller_bad_vectors_and_unbounded_timeouts(self):
        for overrides in ({'controller':'single_arm'},{'velocity_cap':(.1,)*7},{'service_timeout':1.},
            {'limits_evidence':''},{'feedback_timeout':True},{'audit_interval':.1}):
            with self.subTest(overrides=overrides),self.assertRaises(ValueError):settings(**overrides)

    def test_optional_calibrated_hand_is_measured_after_arm_admission_and_closed_on_release(self):
        from test_brainco_adapter import HandNode
        from test_manufacturer import HandMessage,hand_message
        from tg3_pico_dualarm.hands import HandCalibration
        profile=HandCalibration((10,)*6,(900,)*6,(1,)*6,(1000,)*6,(20,)*6,(30,)*6,'TEST ONLY fixture')
        node=HandNode()
        with tempfile.TemporaryDirectory() as folder:
            path=Path(folder)/'hands.json'
            path.write_text(json.dumps(dict(hands={'right':asdict(profile)},
                hand_raw_rate={'right':[100]*6},hand_raw_lead={'right':[20]*6})),encoding='utf-8')
            self.robot=XArmAdapter(node,self.feedback,settings(),clock=self.clock,transport=self.transport,hands_config=path)
            self.robot.hands.message_type=HandMessage
            self.assertEqual(list(self.robot.profiles),['right']);self.assertEqual(node.publishers,[])
            self.assertIs(self.robot.hand_positions,self.robot.hands.hand_positions)
            self.assertIs(self.robot.hand_diagnostics,self.robot.hands.hand_diagnostics)
            self.arm();self.assertEqual(node.publishers,[])
            self.feedback.hand('right',hand_message())
            self.assertEqual(self.robot.measured_hand_initialization()['right']['positions'],(100.,)*6)
            publisher=node.publishers[0];self.assertEqual(publisher.messages,[])
            self.now[0]+=.02;self.refresh();self.feedback.hand('right',hand_message())
            self.assertTrue(self.robot.submit_hand('right',1.,intent_stamp=self.clock(),operation='close').accepted)
            self.assertEqual(publisher.messages[-1].positions,[102]*6)
            self.assertTrue(self.robot.status()['hands']['enabled'])
            self.robot.disarm('TEST release')
            self.assertTrue(publisher.closed);self.assertFalse(self.robot.hands.initialized)
            self.assertEqual(len(publisher.messages),1)

    def test_missing_hand_config_keeps_specific_disabled_status_and_no_hand_output(self):
        self.arm();value=self.robot.status()['hands']
        self.assertFalse(value['enabled']);self.assertEqual(value['configured_sides'],[])
        self.assertIn('calibration_missing',value['disabled_sides']['right'])
        self.assertEqual(self.robot.measured_hand_initialization(),{})
        self.assertFalse(self.robot.submit_hand('right',1.,intent_stamp=self.clock()).accepted)


def ros_wire_fixture():
    """Explicit modules approximating ROS generated field access, never evidence."""
    modules={}
    def add(name,**values):
        module=ModuleType(name);module.__dict__.update(values);modules[name]=module
    def message(name,fields,**defaults):
        def init(self):
            for key,value in defaults.items():setattr(self,key,value() if callable(value) else value)
        return type(name,(),{'get_fields_and_field_types':classmethod(lambda cls:dict(fields)),'__init__':init})
    def service(name,request_fields,response_fields,**defaults):
        return type(name,(),{'Request':message(name+'Request',request_fields,**defaults),
                            'Response':message(name+'Response',response_fields)})
    Array=message('Float64MultiArray',{'layout':'std_msgs/MultiArrayLayout','data':'sequence<double>'},data=list)
    SetBool=service('SetBool',{'data':'boolean'},{'success':'boolean','message':'string'},data=False)
    List=service('ListControllers',{}, {'controller':'sequence<controller_manager_msgs/ControllerState>'})
    Switch=service('SwitchController',{'activate_controllers':'sequence<string>',
        'deactivate_controllers':'sequence<string>','strictness':'int32','activate_asap':'boolean',
        'timeout':'builtin_interfaces/Duration'},{'ok':'boolean'},timeout=lambda:NS(sec=0,nanosec=0))
    Get=service('GetParameters',{'names':'sequence<string>'},{'values':'sequence<rcl_interfaces/ParameterValue>'})
    Set=service('SetParameters',{'parameters':'sequence<rcl_interfaces/Parameter>'},{'results':'sequence<rcl_interfaces/SetParametersResult>'})
    Parameter=message('Parameter',{'name':'string','value':'rcl_interfaces/ParameterValue'})
    Value=message('ParameterValue',{'type':'uint8','double_array_value':'sequence<double>'})
    Warn=message('WarnAndErrorInfo',dict(level='string',code='uint32',source='string',info='string'))
    add('rclpy');add('rclpy.type_support',check_for_type_support=lambda cls:None)
    add('std_msgs');add('std_msgs.msg',Float64MultiArray=Array)
    add('std_srvs');add('std_srvs.srv',SetBool=SetBool)
    add('controller_manager_msgs');add('controller_manager_msgs.srv',ListControllers=List,SwitchController=Switch)
    add('rcl_interfaces');add('rcl_interfaces.msg',Parameter=Parameter,ParameterValue=Value)
    add('rcl_interfaces.srv',GetParameters=Get,SetParameters=Set)
    add('eai_manipulator_msgs');add('eai_manipulator_msgs.msg',WarnAndErrorInfo=Warn)
    return modules


class WireNode:
    def __init__(self):
        self.graph=[('/robot/controller_manager/list_controllers',['controller_manager_msgs/srv/ListControllers']),
            ('/robot/controller_manager/switch_controller',['controller_manager_msgs/srv/SwitchController']),
            ('/robot/controller_manager/activate_controller',['controller_manager_msgs/srv/SwitchController']),
            ('/robot/controller_manager/auto_switch_mode',['std_srvs/srv/SetBool']),
            ('/EAIHardware/set_arm_enable',['std_srvs/srv/SetBool']),
            ('/robot/'+CONTROLLER+'/get_parameters',['rcl_interfaces/srv/GetParameters']),
            ('/robot/'+CONTROLLER+'/set_parameters',['rcl_interfaces/srv/SetParameters'])]
        self.clients=[];self.publishers=[];self.subscriptions=[];self.foreign=[];self.future=None
        self.lower=[]
    def get_service_names_and_types(self):return self.graph
    def create_client(self,cls,name):
        requests=[]
        def send(request):requests.append(request);self.future=NS(done=lambda:False,cancel=lambda:None);return self.future
        client=NS(cls=cls,name=name,requests=requests,service_is_ready=lambda:True,call_async=send)
        self.clients.append(client);return client
    def destroy_client(self,client):self.clients.remove(client)
    def create_subscription(self,cls,topic,callback,qos):
        result=NS(cls=cls,topic=topic,callback=callback);self.subscriptions.append(result);return result
    def destroy_subscription(self,value):self.subscriptions.remove(value)
    def get_name(self):return 'test_xarm_only'
    def get_namespace(self):return '/test'
    def get_publishers_info_by_topic(self,topic):return self.foreign+[
        NS(node_name=self.get_name(),node_namespace=self.get_namespace()) for _ in self.publishers] if topic==TOPIC else self.lower
    def get_subscriptions_info_by_topic(self,topic):
        return [NS(node_name=CONTROLLER,node_namespace='/robot',topic_type='std_msgs/msg/Float64MultiArray')]
    def create_publisher(self,cls,topic,qos):
        messages=[];result=NS(cls=cls,topic=topic,messages=messages,publish=messages.append)
        self.publishers.append(result);return result
    def destroy_publisher(self,value):self.publishers.remove(value)


class XArmRosWireTests(unittest.TestCase):
    def setUp(self):self.now=[10.];self.node=WireNode();self.modules=ros_wire_fixture()
    def transport(self):return XArmRosTransport(self.node,settings(),clock=lambda:self.now[0])

    def test_installed_classes_graph_service_names_and_strict_async_switch_request(self):
        with patch.dict(sys.modules,self.modules):
            transport=self.transport();self.assertEqual(self.node.clients,[]);self.assertEqual(self.node.publishers,[])
            names=transport.discover()
            self.assertEqual(names['switch'],'/robot/controller_manager/switch_controller')
            self.assertNotIn('/robot/controller_manager/activate_controller',names.values())
            transport.start('switch',activate=(CONTROLLER,),deactivate=EMPTY)
            client=next(c for c in self.node.clients if c.name==names['switch']);request=client.requests[0]
            self.assertEqual(request.activate_controllers,[CONTROLLER]);self.assertEqual(request.deactivate_controllers,list(EMPTY))
            self.assertEqual(request.strictness,2);self.assertEqual(request.timeout.nanosec,150000000)
            self.assertIsNone(transport.poll())
            self.node.future.done=lambda:True;self.node.future.result=lambda:NS(ok=True)
            self.assertTrue(transport.poll()[1].ok);transport.close()
            self.assertEqual(self.node.clients,[]);self.assertEqual(self.node.subscriptions,[])

    def test_timed_out_future_is_bounded_and_marks_possible_late_server_mutation(self):
        with patch.dict(sys.modules,self.modules):
            transport=self.transport();transport.discover();transport.start('enable',value=True)
            self.now[0]+=.151
            with self.assertRaisesRegex(RuntimeError,'service_timeout'):transport.poll()
            self.assertTrue(transport.uncertain_mutation);self.assertIsNone(transport.pending)
            transport.close()

    def test_missing_ambiguous_or_custom_only_standard_switch_fails_before_requests(self):
        for failure in ('missing','ambiguous','wrong_type'):
            with self.subTest(failure=failure),patch.dict(sys.modules,self.modules):
                node=WireNode()
                if failure=='missing':node.graph=[entry for entry in node.graph if not entry[0].endswith('/switch_controller')]
                elif failure=='ambiguous':node.graph.append(('/other/list_controllers',['controller_manager_msgs/srv/ListControllers']))
                else:node.graph=[(name,['std_srvs/srv/SetBool']) if name.endswith('/switch_controller') else (name,types) for name,types in node.graph]
                transport=XArmRosTransport(node,settings())
                with self.assertRaises(RuntimeError):transport.discover()
                self.assertEqual(node.clients,[]);self.assertEqual(node.publishers,[]);transport.close()

    def test_true_ros_array_class_exact_fourteen_values_and_foreign_writer_prevents_output(self):
        with patch.dict(sys.modules,self.modules):
            transport=self.transport();transport.open_publisher();self.assertEqual(self.node.publishers[0].messages,[])
            target=tuple(i*.01 for i in range(14));transport.publish(target)
            message=self.node.publishers[0].messages[0]
            self.assertIsInstance(message,self.modules['std_msgs.msg'].Float64MultiArray)
            self.assertEqual(message.data,list(target))
            self.node.foreign=[NS(node_name='another_writer',node_namespace='/')]
            with self.assertRaisesRegex(RuntimeError,'competing_target_publisher'):transport.publish(target)
            self.assertEqual(len(self.node.publishers[0].messages),1);transport.close()

    def test_parameter_write_is_typed_double_array_and_only_velocity_changes(self):
        with patch.dict(sys.modules,self.modules):
            transport=self.transport();transport.discover();transport.start('set',velocity=(.05,)*14)
            client=next(c for c in self.node.clients if c.name.endswith('/set_parameters'))
            parameter=client.requests[0].parameters[0]
            self.assertEqual(parameter.name,'vel_limits');self.assertEqual(parameter.value.type,8)
            self.assertEqual(parameter.value.double_array_value,[.05]*14);transport.close()

    def test_lowlevel_armctrl_competitor_rejected_and_enable_requires_unique_actual_hardware_publisher(self):
        with patch.dict(sys.modules,self.modules):
            transport=self.transport();transport.check_publishers()
            transport.hardware_enabled=True
            with self.assertRaisesRegex(RuntimeError,'lowlevel_EAIHardware'):transport.check_publishers()
            self.node.lower=[NS(node_name='EAIHardware',node_namespace='/',topic_type='ros2_bridge_msgs/msg/ArmCtrl')]
            transport.check_publishers()
            self.node.lower.append(NS(node_name='joystick_bridge',node_namespace='/',topic_type='ros2_bridge_msgs/msg/ArmCtrl'))
            with self.assertRaisesRegex(RuntimeError,'lowlevel_arm_publisher'):transport.check_publishers()
            self.assertFalse(any(pub.topic=='/arm/cmd' for pub in self.node.publishers));transport.close()

    def test_get_parameters_requests_only_installed_joints_name_for_order_confirmation(self):
        with patch.dict(sys.modules,self.modules):
            transport=self.transport();transport.discover();transport.start('get')
            client=next(c for c in self.node.clients if c.name.endswith('/get_parameters'))
            self.assertEqual(client.requests[0].names,list(PARAMETERS));transport.close()
            self.assertNotIn('joint_names',client.requests[0].names)

    def test_warning_error_latches_and_incompatible_real_message_schema_fails(self):
        with patch.dict(sys.modules,self.modules):
            transport=self.transport();transport._warning(NS(level='error',code=600100,source='TEST ONLY',info='failure'))
            self.assertEqual(transport.warning_error['code'],600100)
            transport._warning(NS(level='info',code=100102,source='TEST ONLY',info='done'))
            self.assertEqual(transport.warning_error['code'],600100);transport.close()
            array=self.modules['std_msgs.msg'].Float64MultiArray
            array.get_fields_and_field_types=classmethod(lambda cls:{'data':'sequence<float>'})
            with self.assertRaisesRegex(RuntimeError,'array_schema'):self.transport()


class XArmSessionTests(unittest.TestCase):
    def test_retired_publisher_wait_set_error_does_not_kill_disable_ack_executor(self):
        import threading,time
        class InvalidHandle(RuntimeError):pass
        module=ModuleType('rclpy.handle');module.InvalidHandle=InvalidHandle
        session=XArmRosSession.__new__(XArmRosSession)
        session._exit=threading.Event();session.context=NS(ok=lambda:True);session.executor_error=None
        failures=[];calls=[]
        session.robot=NS(lock=threading.RLock(),transport=NS(entity_retirements=1,last_entity_retirement=time.monotonic()),_fault=failures.append)
        def spin_once(**kwargs):
            calls.append('spin')
            if len(calls)==1:raise InvalidHandle('own publisher was retired')
            session._exit.set()
        session.executor=NS(spin_once=spin_once)
        with patch.dict(sys.modules,{'rclpy.handle':module}):session._spin()
        self.assertEqual(len(calls),2);self.assertEqual(failures,[]);self.assertIsNone(session.executor_error)

    def test_persistent_handle_failure_is_not_hidden_by_retirement_retry(self):
        import threading,time
        class InvalidHandle(RuntimeError):pass
        module=ModuleType('rclpy.handle');module.InvalidHandle=InvalidHandle
        session=XArmRosSession.__new__(XArmRosSession)
        session._exit=threading.Event();session.context=NS(ok=lambda:True);session.executor_error=None
        failures=[];calls=[]
        session.robot=NS(lock=threading.RLock(),transport=NS(entity_retirements=1,last_entity_retirement=time.monotonic()),_fault=failures.append)
        def spin_once(**kwargs):
            calls.append('spin');raise InvalidHandle('persistent error')
        session.executor=NS(spin_once=spin_once)
        with patch.dict(sys.modules,{'rclpy.handle':module}):session._spin()
        self.assertEqual(len(calls),2);self.assertEqual(len(failures),1)
        self.assertEqual(session.executor_error,'persistent error')

    def test_private_session_initializes_readonly_and_close_sends_no_hardware_requests_without_enable(self):
        from test_manufacturer_session import fake_ros
        modules,calls=fake_ros();wire=ros_wire_fixture()
        # Keep the independent ROS context/executor and genuine-feedback
        # doubles. Extend only the explicitly labelled message fixtures.
        for name,module in wire.items():
            if name in modules:modules[name].__dict__.update({key:value for key,value in module.__dict__.items() if not key.startswith('__')})
            else:modules[name]=module
        Node=modules['rclpy.node'].Node;original=Node.create_subscription
        def subscribe(node,cls,topic,callback,qos):
            if topic=='/manipulator/warn_and_error_info':
                subscription=NS(topic=topic,callback=callback);node._subscriptions.append(subscription);return subscription
            return original(node,cls,topic,callback,qos)
        Node.create_subscription=subscribe
        with patch.dict(sys.modules,modules):
            session=XArmRosSession.create(settings=settings(),wait_for_feedback_s=0,monotonic=lambda:10.)
            self.assertEqual(session.robot.state,'disabled');self.assertEqual(list(session.node.publishers),[])
            self.assertIs(session.robot.read_state().estop,False)
            self.assertEqual(session.robot.transport.clients,{})
            session.close();session.close()
            self.assertTrue(session.closed);self.assertTrue(calls['nodes'][0].destroyed)
            self.assertEqual(session.cleanup_errors,[])
            self.assertFalse(any(event.startswith('pub:') for event in calls['events']))


if __name__=='__main__':unittest.main()
