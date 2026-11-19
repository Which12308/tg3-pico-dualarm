"""Explicit ROS/provider doubles; these are not native or robot acceptance."""
from dataclasses import asdict
import hashlib
import json
from pathlib import Path
import sys
import tempfile
import threading
from types import ModuleType,SimpleNamespace as NS
import unittest
from unittest.mock import patch

from tg3_pico_dualarm.manufacturer import ManufacturerAdapter,ManufacturerLimits
from tg3_pico_dualarm.manufacturer_session import FactoryRosSession,load_factory_bundle
from tg3_pico_dualarm.model import DualArmModel,default_q
from test_manufacturer import (Driver,DocumentedArmMessage,DocumentedMotorMessage,HandMessage,
                              arm_message,mode_message,hand_message,power_key_message)


def limits():
    model=DualArmModel()
    return ManufacturerLimits((100.,)*14,(5.,)*14,tuple(model.limits[:,0]),tuple(model.limits[:,1]),
        (.1,)*14,(.02,)*14,{}, {},'TEST ONLY verified limits fixture')


class SessionDriver(Driver):
    def __init__(self,node,clock,events):super().__init__(clock);self.node=node;self.trace=events
    def stop_and_restore(self,reason):self.trace.append('driver.stop');return super().stop_and_restore(reason)
    def release_control(self):self.trace.append('driver.release');return super().release_control()
    def close(self):self.trace.append('driver.close')


def fake_ros(emit=True,subscribe_fail=False):
    calls=dict(nodes=[],contexts=[],events=[],emit=emit,subscribe_fail=subscribe_fail,executor_fail=False)
    class Context:
        def __init__(self):self.alive=False;calls['contexts'].append(self)
        def ok(self):return self.alive
        def shutdown(self):self.alive=False;calls['events'].append('context.shutdown')
    class Publisher:
        def __init__(self,topic):self.topic_name=topic;self.messages=[]
        def publish(self,message):self.messages.append(message)
    class Node:
        def __init__(self,name,**kwargs):
            self.name=name;self.options=kwargs;self._publishers=[];self._subscriptions=[];self.destroyed=False
            calls['nodes'].append(self)
        @property
        def publishers(self):return iter(self._publishers)
        def create_publisher(self,cls,topic,qos):
            publisher=Publisher(topic);self._publishers.append(publisher);calls['events'].append('pub:'+topic);return publisher
        def destroy_publisher(self,publisher):
            calls['events'].append('destroy:'+publisher.topic_name)
            if publisher in self._publishers:self._publishers.remove(publisher)
        def create_subscription(self,cls,topic,callback,qos):
            if calls['subscribe_fail'] and len(self._subscriptions)==1:raise RuntimeError('fixture subscribe failure')
            subscription=NS(topic=topic,callback=callback);self._subscriptions.append(subscription)
            if calls['emit']:
                callback(arm_message(tuple(default_q())) if topic=='/robot_state' else mode_message()
                    if topic=='/hric/robot/rl_state' else power_key_message()
                    if topic=='/power/board/key_status' else hand_message())
            return subscription
        def destroy_subscription(self,subscription):
            calls['events'].append('unsubscribe:'+subscription.topic)
            if subscription in self._subscriptions:self._subscriptions.remove(subscription)
        def destroy_node(self):
            self._publishers=[];self._subscriptions=[];self.destroyed=True;calls['events'].append('node.destroy')
        def get_clock(self):return NS(now=lambda:NS(to_msg=lambda:NS(sec=40,nanosec=0)))
    class Executor:
        def __init__(self,context=None):self.stop=threading.Event();self.context=context
        def add_node(self,node):self.node=node
        def spin_once(self,timeout_sec=.05):
            if calls['executor_fail']:raise RuntimeError('fixture executor failure')
            self.stop.wait(.005)
        def shutdown(self,timeout_sec=2.):self.stop.set();calls['events'].append('executor.shutdown');return True
    class MotorStatus:
        @classmethod
        def get_fields_and_field_types(cls):return dict.fromkeys(('positions','states','currents','speeds'),'int16[6]')
    class PowerBoardKeyStatus:
        @classmethod
        def get_fields_and_field_types(cls):
            return dict.fromkeys(('is_estop','is_remote_estop','is_power_on','is_shutting_down','is_battery_unlock'),'std_msgs/Bool')
    class Bool:
        @classmethod
        def get_fields_and_field_types(cls):return {'data':'boolean'}
    modules={}
    def add(name,**values):
        module=ModuleType(name);module.__dict__.update(values);modules[name]=module
    def initialize(**kwargs):calls['init_kwargs']=kwargs;kwargs['context'].alive=True
    add('rclpy',init=initialize);add('rclpy.context',Context=Context);add('rclpy.node',Node=Node)
    add('rclpy.executors',SingleThreadedExecutor=Executor);add('rclpy.qos',qos_profile_sensor_data=object())
    add('rclpy.type_support',check_for_type_support=lambda cls:None)
    add('ros2_bridge_msgs');add('ros2_bridge_msgs.msg',RobotState=object,ArmCtrl=DocumentedArmMessage,MotorCtrl=DocumentedMotorMessage)
    add('diagnostic_msgs');add('diagnostic_msgs.msg',DiagnosticStatus=object)
    add('brainco_hand_msgs');add('brainco_hand_msgs.msg',MotorStatus=MotorStatus,SetMotorMulti=HandMessage)
    add('bodyctrl_msgs');add('bodyctrl_msgs.msg',PowerBoardKeyStatus=PowerBoardKeyStatus)
    add('std_msgs');add('std_msgs.msg',Bool=Bool)
    return modules,calls


class ManufacturerSessionTests(unittest.TestCase):
    def setUp(self):self.now=[10.];self.clock=lambda:self.now[0]

    def test_documented_board_estop_cannot_be_overridden_by_provider(self):
        modules,calls=fake_ros()
        with patch.dict(sys.modules,modules):
            with FactoryRosSession(driver_factory=lambda node:SessionDriver(node,self.clock,calls['events']),
                    limits=limits(),wait_for_feedback_s=.1,monotonic=self.clock) as session:
                session.driver.estop=False
                session.feedback.key_status(power_key_message(remote_estop=True))
                self.assertTrue(session.robot.read_state().estop)
                self.assertFalse(session.robot.request_enable(True).accepted)
                session.feedback.key_status(power_key_message())
                self.assertFalse(session.robot.read_state().estop)
                session.driver.estop=True
                self.assertTrue(session.robot.read_state().estop)
                self.now[0]+=.3;session.driver.estop=False
                self.assertIsNone(session.robot.read_state().estop)
                self.assertEqual(list(session.node.publishers),[])

    def test_no_provider_hardware_session_fails_before_import_node_or_context(self):
        modules,calls=fake_ros()
        with patch.dict(sys.modules,modules),self.assertRaisesRegex(RuntimeError,'before_ROS_creation'):
            FactoryRosSession(limits=limits())
        self.assertEqual(calls['nodes'],[]);self.assertEqual(calls['contexts'],[])

    def test_actual_session_composition_starts_disarmed_and_uses_only_private_context(self):
        modules,calls=fake_ros()
        with patch.dict(sys.modules,modules):
            session=FactoryRosSession(driver_factory=lambda node:SessionDriver(node,self.clock,calls['events']),
                limits=limits(),wait_for_feedback_s=.1,monotonic=self.clock)
            try:
                self.assertIsInstance(session.robot,ManufacturerAdapter)
                self.assertFalse(session.robot.hardware_writes_enabled)
                self.assertEqual(session.status()['control_state'],'disabled')
                self.assertFalse(session.status()['hardware_writes_enabled'])
                self.assertEqual(list(session.node.publishers),[])
                self.assertNotIn('request',session.driver.events)
                self.assertEqual(session.robot.read_state().positions,tuple(default_q()))
                self.assertIs(calls['init_kwargs']['context'],session.context)
                self.assertFalse(session.node.options['use_global_arguments'])
            finally:session.close()
        self.assertTrue(calls['nodes'][0].destroyed);self.assertFalse(calls['contexts'][0].ok())
        self.assertTrue(session.status()['closed']);self.assertFalse(session.status()['executor_running'])
        self.assertEqual(calls['nodes'][0]._subscriptions,[])

    def test_incompatible_installed_control_schema_fails_before_context_or_provider(self):
        modules,calls=fake_ros()
        class ChangedArm(DocumentedArmMessage):
            @classmethod
            def get_fields_and_field_types(cls):return dict(super().get_fields_and_field_types(),mode='int32')
        modules['ros2_bridge_msgs.msg'].ArmCtrl=ChangedArm
        with patch.dict(sys.modules,modules),self.assertRaises(ValueError):
            FactoryRosSession(driver_factory=lambda node:self.fail('provider must not run'),limits=limits(),monotonic=self.clock)
        self.assertEqual(calls['contexts'],[]);self.assertEqual(calls['nodes'],[])

    def test_explicit_enable_then_close_destroys_motion_before_factory_stop_and_ROS_teardown(self):
        modules,calls=fake_ros()
        with patch.dict(sys.modules,modules):
            session=FactoryRosSession(driver_factory=lambda node:SessionDriver(node,self.clock,calls['events']),
                limits=limits(),wait_for_feedback_s=.1,monotonic=self.clock)
            self.assertTrue(session.robot.request_enable(True).accepted)
            self.assertEqual([p.topic_name for p in session.node.publishers],['/arm/cmd'])
            self.assertTrue(all(not p.messages for p in session.node.publishers))
            session.close();session.close()
        trace=calls['events']
        self.assertLess(trace.index('destroy:/arm/cmd'),trace.index('driver.stop'))
        self.assertLess(trace.index('driver.release'),trace.index('driver.close'))
        self.assertLess(trace.index('driver.close'),trace.index('executor.shutdown'))
        self.assertLess(trace.index('executor.shutdown'),trace.index('node.destroy'))

    def test_readonly_observation_can_start_without_gains_or_driver_and_never_publishes(self):
        modules,calls=fake_ros()
        with patch.dict(sys.modules,modules):
            with FactoryRosSession.observe(monotonic=self.clock) as session:
                self.assertTrue(session.observe_only)
                self.assertEqual(session.robot.read_state().quality,'measured')
                self.assertEqual(list(session.node.publishers),[])
                self.assertIsNone(session.driver)
        self.assertTrue(calls['nodes'][0].destroyed)

    def test_feedback_timeout_has_concrete_missing_fields_and_cleans_constructor(self):
        modules,calls=fake_ros(emit=False)
        with patch.dict(sys.modules,modules),self.assertRaisesRegex(RuntimeError,'feedback_wait_timeout:arm_feedback_missing_or_stale,fsm_feedback_missing_or_stale'):
            FactoryRosSession(driver_factory=lambda node:SessionDriver(node,self.clock,calls['events']),
                limits=limits(),wait_for_feedback_s=.03,monotonic=self.clock)
        self.assertTrue(calls['nodes'][0].destroyed);self.assertFalse(calls['contexts'][0].ok())
        self.assertNotIn('pub:/arm/cmd',calls['events'])

    def test_partial_subscription_failure_and_bad_factory_leave_no_native_resources(self):
        for subscribe_fail in (True,False):
            modules,calls=fake_ros(subscribe_fail=subscribe_fail)
            with self.subTest(subscribe_fail=subscribe_fail),patch.dict(sys.modules,modules),self.assertRaises(RuntimeError):
                FactoryRosSession(driver_factory=lambda node:object(),limits=limits(),monotonic=self.clock)
            self.assertTrue(calls['nodes'][0].destroyed);self.assertFalse(calls['contexts'][0].ok())
            self.assertEqual(calls['nodes'][0]._subscriptions,[])

    def test_provider_motion_publisher_during_construction_is_rejected_and_removed(self):
        modules,calls=fake_ros()
        def bad(node):
            node.create_publisher(DocumentedArmMessage,'/arm/cmd',1)
            return SessionDriver(node,self.clock,calls['events'])
        with patch.dict(sys.modules,modules),self.assertRaisesRegex(RuntimeError,'during_construction'):
            FactoryRosSession(driver_factory=bad,limits=limits(),monotonic=self.clock)
        self.assertEqual(calls['nodes'][0]._publishers,[]);self.assertTrue(calls['nodes'][0].destroyed)

    def test_executor_failure_stops_output_and_reenable_cannot_bypass_failed_session(self):
        modules,calls=fake_ros()
        with patch.dict(sys.modules,modules):
            session=FactoryRosSession(driver_factory=lambda node:SessionDriver(node,self.clock,calls['events']),
                limits=limits(),monotonic=self.clock)
            try:
                self.assertTrue(session.robot.request_enable(True).accepted)
                calls['executor_fail']=True
                self.assertTrue(session._failed.wait(.5))
                self.assertFalse(session.robot.hardware_writes_enabled)
                self.assertFalse(session.robot.request_enable(True).accepted)
                self.assertEqual(list(session.node.publishers),[])
            finally:session.close()

    def test_unacknowledged_factory_stop_is_reported_after_all_ROS_resources_are_closed(self):
        modules,calls=fake_ros()
        with patch.dict(sys.modules,modules):
            session=FactoryRosSession(driver_factory=lambda node:SessionDriver(node,self.clock,calls['events']),
                limits=limits(),monotonic=self.clock)
            self.assertTrue(session.robot.request_enable(True).accepted)
            session.driver.stop_ack=False
            with self.assertRaisesRegex(RuntimeError,'not_acknowledged'):session.close()
            self.assertTrue(session.cleanup_errors)
            self.assertTrue(session.node.destroyed)
            self.assertFalse(session.context.ok())
            self.assertEqual(list(session.node.publishers),[])
            self.assertFalse(session._thread.is_alive())
            session.close()

    def test_driver_cleanup_exception_does_not_skip_executor_node_or_context_cleanup(self):
        modules,calls=fake_ros()
        with patch.dict(sys.modules,modules):
            session=FactoryRosSession(driver_factory=lambda node:SessionDriver(node,self.clock,calls['events']),
                limits=limits(),monotonic=self.clock)
            def broken_close():raise RuntimeError('fixture driver cleanup failure')
            session.driver.close=broken_close
            with self.assertRaisesRegex(RuntimeError,'driver_cleanup_failed'):session.close()
            self.assertTrue(session.node.destroyed);self.assertFalse(session.context.ok())
            self.assertFalse(session._thread.is_alive())


class FactoryBundleTests(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory();self.addCleanup(self.temp.cleanup)
        self.directory=Path(self.temp.name)
        self.source=self.directory/'reviewed_driver.py'
        self.source.write_text('from test_manufacturer import Driver\ndef create_driver(node):\n return Driver(lambda:10.)\n',encoding='utf-8')
        self.descriptor=self.directory/'factory.json'

    def write(self,**overrides):
        driver=dict(path=self.source.name,sha256=hashlib.sha256(self.source.read_bytes()).hexdigest(),factory='create_driver',
            evidence=dict(controller='TEST DOUBLE',version='fixture',interface_reference='no actual factory API'))
        value=dict(schema='tg3_factory_bundle_v1',driver=driver,limits=asdict(limits()),hands={})
        value.update(overrides);self.descriptor.write_text(json.dumps(value),encoding='utf-8')
        return value

    def test_reviewed_local_module_is_explicit_hashed_and_does_not_change_sys_path(self):
        self.write();before=list(sys.path)
        bundle=load_factory_bundle(self.descriptor)
        self.assertEqual(bundle.source_path,self.source.resolve())
        driver=bundle.driver_factory(None)
        self.assertIsInstance(driver,Driver);self.assertEqual(sys.path,before)
        self.assertEqual(bundle.profiles,{})

    def test_descriptor_validation_does_not_execute_provider_and_changed_file_is_rejected(self):
        marker=self.directory/'executed.txt'
        self.source.write_text('from pathlib import Path\nPath('+repr(str(marker))+').write_text("executed")\n',encoding='utf-8')
        self.write();bundle=load_factory_bundle(self.descriptor)
        self.assertFalse(marker.exists())
        self.source.write_text(self.source.read_text()+'# changed\n',encoding='utf-8')
        with self.assertRaisesRegex(RuntimeError,'changed_after'):bundle.driver_factory(None)
        self.assertFalse(marker.exists())

    def test_wrong_hash_boolean_authority_and_outside_model_limits_are_rejected(self):
        value=self.write();value['driver']['sha256']='0'*64
        self.descriptor.write_text(json.dumps(value),encoding='utf-8')
        with self.assertRaisesRegex(ValueError,'sha256_mismatch'):load_factory_bundle(self.descriptor)
        self.write(hardware_enabled=True)
        with self.assertRaisesRegex(ValueError,'schema_or_fields'):load_factory_bundle(self.descriptor)
        value=self.write();value['limits']['lower']=[-100]+list(value['limits']['lower'][1:])
        self.descriptor.write_text(json.dumps(value),encoding='utf-8')
        with self.assertRaisesRegex(ValueError,'exceed_actual_model'):load_factory_bundle(self.descriptor)

    def test_runtime_provider_evidence_must_match_reviewed_descriptor(self):
        value=self.write();value['driver']['evidence']['controller']='different reviewed controller'
        self.descriptor.write_text(json.dumps(value),encoding='utf-8')
        bundle=load_factory_bundle(self.descriptor)
        with self.assertRaisesRegex(RuntimeError,'evidence_differs'):bundle.driver_factory(None)

    def test_uri_and_dotted_factory_are_not_implicit_plugin_imports(self):
        for path,name in (('https://example.invalid/provider.py','create_driver'),(self.source.name,'other_module.factory')):
            value=self.write();value['driver'].update(path=path,factory=name)
            self.descriptor.write_text(json.dumps(value),encoding='utf-8')
            with self.assertRaises(ValueError):load_factory_bundle(self.descriptor)


if __name__=='__main__':unittest.main()
