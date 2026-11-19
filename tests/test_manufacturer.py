"""Adapter integration with explicit test doubles; no ROS/physical robot claims."""
from dataclasses import replace
import threading
from types import SimpleNamespace as NS
import unittest

from tg3_pico_dualarm.hands import HandCalibration
from tg3_pico_dualarm.manufacturer import ManufacturerAdapter,ManufacturerLimits,ROSFactoryTransport,hand_schema
from tg3_pico_dualarm.manufacturer_contract import FactoryControlDriver,FactoryLease,FactoryStatus,FactoryResult
from tg3_pico_dualarm.manufacturer_feedback import ManufacturerFeedback
from tg3_pico_dualarm.manufacturer_discovery import graph_snapshot,TOPICS
from tg3_pico_dualarm.robot import JOINT_IDS,JointCommand
from test_tiangong_protocol import DocumentedArmMessage,DocumentedMotorMessage


class HandMessage:
    @classmethod
    def get_fields_and_field_types(cls):
        return dict(mode='uint8',positions='uint16[6]',speeds='uint16[6]',currents='int16[6]',pwms='uint16[6]',durations='uint16[6]')


class Publisher:
    def __init__(self,topic):self.topic=topic;self.messages=[];self.closed=False
    def publish(self,message):
        if self.closed:raise RuntimeError('closed fixture publisher')
        self.messages.append(message)


class Node:
    def __init__(self):self.publishers=[]
    def create_publisher(self,cls,topic,qos):
        publisher=Publisher(topic);self.publishers.append(publisher);return publisher
    def destroy_publisher(self,publisher):publisher.closed=True
    def get_clock(self):return NS(now=lambda:NS(to_msg=lambda:NS(sec=40,nanosec=55)))


class Driver(FactoryControlDriver):
    """Test-only manufacturer driver, not an implementation of any vendor API."""
    def __init__(self,clock):
        self.clock=clock;self.owned=False;self.owner='fixture-owner';self.estop=False
        self.ack=True;self.ack_time=None;self.stop_ack=True;self.hold_ack=True;self.events=[]
        self.hand_sides=()
    @property
    def evidence(self):return dict(controller='TEST DOUBLE',version='fixture',interface_reference='no actual factory API')
    def request_dual_arm_control(self,hand_sides=()):
        self.events.append('request');self.owned=self.ack;self.hand_sides=hand_sides
    def read_status(self):
        now=self.clock()
        lease=FactoryLease(self.owner,JOINT_IDS,'TEST_ARM_MODE',now if self.ack_time is None else self.ack_time,now+.1,self.hand_sides) if self.owned else None
        return FactoryStatus(lease,self.estop,now,'TEST DOUBLE E-stop')
    def hold(self,positions):self.events.append(('hold',positions));return FactoryResult(self.hold_ack,'fixture')
    def stop_and_restore(self,reason):self.events.append(('stop',reason));return FactoryResult(self.stop_ack,'fixture')
    def release_control(self):self.events.append('release');self.owned=False;return FactoryResult(self.stop_ack,'fixture')


def arm_message(values=None):
    values=values or tuple(i/100 for i in JOINT_IDS)
    return NS(arm=NS(status=[NS(name=j,pos=p,error=0) for j,p in reversed(tuple(zip(JOINT_IDS,values)))]))


def mode_message():
    return NS(level=0,values=[NS(key=k,value=v) for k,v in dict(current_state='TEST_ARM_MODE',child_state='TEST_ARM_MODE',status='running').items()])


def hand_message(position=100):return NS(positions=[position]*6,states=[0]*6,currents=[1,-1,0,0,2,3],speeds=[0]*6)


def power_key_message(estop=False,remote_estop=False):
    """Explicit non-ROS fixture for the documented active-high board fields."""
    return NS(is_estop=NS(data=estop),is_remote_estop=NS(data=remote_estop),is_power_on=NS(data=True),
        is_shutting_down=NS(data=False),is_battery_unlock=NS(data=False))


class ManufacturerTests(unittest.TestCase):
    def test_external_caller_label_reaches_actual_formatted_message(self):
        self.setUp()
        self.limits=replace(self.limits,caller_label=101);self.adapter=self.make_adapter()
        self.enable();self.tick()
        self.assertEqual(self.node.publishers[0].messages[-1].label,101)
        for label in (True,1,100,256,101.):
            with self.subTest(label=label),self.assertRaisesRegex(ValueError,'caller_label'):
                replace(self.limits,caller_label=label)

    def setUp(self):
        self.now=[10.];self.clock=lambda:self.now[0]
        self.feedback=ManufacturerFeedback(clock=self.clock)
        self.refresh()
        self.driver=Driver(self.clock);self.node=Node()
        self.profile=HandCalibration((10,)*6,(900,)*6,(1,)*6,(1000,)*6,(20,)*6,(30,)*6,'TEST ONLY calibrated fixture')
        self.profiles=dict(left=self.profile,right=self.profile)
        self.limits=ManufacturerLimits((100,)*14,(5,)*14,(-3,)*14,(3,)*14,(.1,)*14,(.02,)*14,
            dict(left=(100,)*6,right=(100,)*6),dict(left=(20,)*6,right=(20,)*6),'TEST ONLY factory limits')
        self.adapter=self.make_adapter()

    def refresh(self,values=None):
        self.feedback.arm(arm_message(values));self.feedback.fsm(mode_message())
        self.feedback.hand('left',hand_message());self.feedback.hand('right',hand_message())
        self.feedback.key_status(power_key_message())

    def make_adapter(self,**kwargs):
        profiles=kwargs.pop('profiles',self.profiles)
        return ManufacturerAdapter(self.feedback,
            lambda:ROSFactoryTransport(self.node,profiles,self.limits,
                (DocumentedArmMessage,DocumentedMotorMessage,HandMessage),clock=self.clock),
            self.limits,profiles,driver=kwargs.pop('driver',self.driver),clock=self.clock,**kwargs)

    def enable(self):self.assertTrue(self.adapter.request_enable(True).accepted)

    def tick(self,delta=.001):
        self.now[0]+=.02;self.refresh()
        q=tuple(v+delta for v in self.feedback.positions)
        return self.adapter.submit(JointCommand(q,(.05,)*14,self.clock()))

    def test_missing_runtime_factory_driver_rejects_before_transport_factory_or_publishers(self):
        created=[]
        with self.assertRaisesRegex(RuntimeError,'verified_factory_driver_missing'):
            ManufacturerAdapter(self.feedback,lambda:created.append(True),self.limits,driver=None)
        self.assertEqual(created,[]);self.assertEqual(self.node.publishers,[])

    def test_enable_requires_explicit_operator_and_observed_factory_ack_and_sends_no_initial_pose(self):
        self.assertFalse(self.adapter.request_enable(False).accepted)
        self.assertEqual(self.driver.events,[])
        self.driver.ack=False
        self.assertFalse(self.adapter.request_enable(True).accepted)
        self.assertEqual(self.node.publishers,[])
        self.driver.owned=True
        self.assertTrue(self.adapter.poll_enable().accepted)
        self.assertEqual([p.topic for p in self.node.publishers],['/arm/cmd','/left_hand/set_motor_multi','/right_hand/set_motor_multi'])
        self.assertTrue(all(not p.messages for p in self.node.publishers))
        self.assertEqual(self.adapter.hand_positions['left'],(100,)*6)
        initial=self.adapter.measured_hand_initialization()['left']
        self.assertEqual(initial['positions'],(100,)*6)
        self.assertAlmostEqual(initial['closure_projection'],90/890)
        self.assertTrue(initial['closure_is_projection'])
        self.assertTrue(self.adapter.hardware_writes_enabled)

    def test_one_verified_output_call_contains_both_arms_actual_ids_and_no_reserved_magic(self):
        self.enable();result=self.tick();self.assertTrue(result.accepted,result.reason)
        messages=self.node.publishers[0].messages;self.assertEqual(len(messages),1)
        value=messages[0]
        self.assertEqual([m.name for m in value.ctrl],list(JOINT_IDS));self.assertEqual(value.reserved,0)
        self.assertEqual(value.mode,1);self.assertEqual([m.kp for m in value.ctrl],[100]*14)
        self.assertEqual(value.header.stamp.sec,40)

    def test_hand_first_output_is_rate_limited_from_feedback_not_zero_or_open_pose(self):
        self.enable();self.now[0]+=.02;self.refresh()
        result=self.adapter.submit_hand('right',1.,intent_stamp=self.clock())
        self.assertTrue(result.accepted,result.reason)
        value=self.node.publishers[2].messages[-1]
        self.assertEqual(value.positions,[102]*6);self.assertEqual(value.durations,[0]*6)
        self.assertEqual(value.speeds,[20]*6);self.assertEqual(value.currents,[30]*6)
        again=self.adapter.submit_hand('right',1.,intent_stamp=self.clock())
        self.assertFalse(again.accepted);self.assertEqual(len(self.node.publishers[2].messages),1)
        self.assertTrue(self.adapter.hardware_writes_enabled)

    def test_bad_arm_feedback_immediately_revokes_previous_good_state_and_stops_writer(self):
        self.enable();bad=arm_message();bad.arm.status[0].pos=float('nan');self.feedback.arm(bad)
        with self.assertRaises(RuntimeError):self.feedback.read_state()
        self.now[0]+=.02
        result=self.adapter.submit(JointCommand((0.,)*14,(0.,)*14,self.clock()))
        self.assertFalse(result.accepted);self.assertFalse(self.adapter.hardware_writes_enabled)
        self.assertTrue(all(p.closed and not p.messages for p in self.node.publishers))
        self.assertTrue(any(isinstance(e,tuple) and e[0]=='stop' for e in self.driver.events))
        self.assertEqual(self.driver.events[-1],'release')

    def test_owner_change_estop_unknown_and_stale_ack_cannot_publish_or_auto_resume(self):
        for failure in ('owner','estop','ack'):
            with self.subTest(failure=failure):
                self.setUp();self.enable();self.now[0]+=.02;self.refresh()
                if failure=='owner':self.driver.owner='different-owner'
                elif failure=='estop':self.driver.estop=None
                else:self.driver.ack_time=self.clock()-1.
                result=self.adapter.submit(JointCommand(self.feedback.positions,(0.,)*14,self.clock()))
                self.assertFalse(result.accepted);self.assertEqual(self.adapter.state,'fault')
                self.assertTrue(all(not p.messages for p in self.node.publishers))
                self.assertFalse(self.adapter.request_enable(True).accepted)

    def test_joint_bounds_speed_feedback_lead_and_replay_are_checked_before_publish(self):
        for failure in ('bounds','speed','lead','replay'):
            with self.subTest(failure=failure):
                self.setUp();self.enable();self.now[0]+=.02;self.refresh()
                q=list(self.feedback.positions);v=[0.]*14;stamp=self.clock()
                if failure=='bounds':q[0]=4.
                elif failure=='speed':v[0]=1.
                elif failure=='lead':
                    moved=list(self.feedback.positions);moved[0]-=.1;self.feedback.arm(arm_message(moved))
                else:self.adapter._last_command_stamp=stamp
                result=self.adapter.submit(JointCommand(tuple(q),tuple(v),stamp))
                self.assertFalse(result.accepted);self.assertTrue(all(not p.messages for p in self.node.publishers))

    def test_hold_stop_release_require_real_provider_acknowledgements(self):
        self.enable();result=self.adapter.hold()
        self.assertTrue(result.accepted);self.assertEqual(self.adapter.state,'holding')
        self.assertEqual(self.driver.events[-1],('hold',self.feedback.positions))
        self.assertTrue(all(not p.messages for p in self.node.publishers))
        self.assertTrue(self.adapter.disarm().accepted);self.assertEqual(self.adapter.state,'disabled')
        self.assertFalse(self.adapter.hardware_writes_enabled)
        self.setUp();self.enable();self.driver.stop_ack=False
        self.assertFalse(self.adapter.disarm().accepted);self.assertEqual(self.adapter.state,'fault')

    def test_command_watchdog_and_hand_fault_leave_no_active_motion_publisher(self):
        self.enable();self.now[0]+=.16
        self.assertTrue(self.adapter.check_watchdog());self.assertFalse(self.adapter.hardware_writes_enabled)
        self.assertTrue(all(p.closed for p in self.node.publishers))
        self.setUp();self.enable();bad=hand_message();bad.states[2]=7;self.feedback.hand('left',bad)
        self.now[0]+=.02
        result=self.adapter.submit_hand('right',1.,intent_stamp=self.clock())
        self.assertFalse(result.accepted);self.assertIn('motor_state_not_safe_for_position_control',result.reason)

    def test_unexpected_native_publish_exception_closes_output_and_requests_factory_stop(self):
        class NativeSerializationFailure(Exception):pass
        self.enable()
        def fail(_):raise NativeSerializationFailure('fixture native publisher failed')
        self.node.publishers[0].publish=fail
        result=self.tick()
        self.assertFalse(result.accepted);self.assertFalse(self.adapter.hardware_writes_enabled)
        self.assertTrue(all(p.closed for p in self.node.publishers))
        self.assertEqual(self.driver.events[-1],'release')

    def test_arm_ownership_does_not_implicitly_grant_hand_publish_authority(self):
        self.enable();self.driver.hand_sides=()
        self.now[0]+=.02;self.refresh()
        result=self.adapter.submit_hand('right',1.,intent_stamp=self.clock())
        self.assertFalse(result.accepted);self.assertIn('hand_ownership_not_acknowledged',result.reason)
        self.assertTrue(all(p.closed and not p.messages for p in self.node.publishers))

    def test_optional_hands_do_not_require_hand_feedback_or_create_hand_publishers(self):
        self.adapter=self.make_adapter(profiles={});self.feedback.hands=dict.fromkeys(('left','right'))
        self.enable();self.assertEqual([p.topic for p in self.node.publishers],['/arm/cmd'])

    def test_hand_schema_reflection_handles_optional_durations_and_rejects_wrong_lengths(self):
        class OldHand(HandMessage):
            @classmethod
            def get_fields_and_field_types(cls):
                fields=super().get_fields_and_field_types();fields.pop('durations');return fields
        transport=ROSFactoryTransport(self.node,self.profiles,self.limits,
            (DocumentedArmMessage,DocumentedMotorMessage,OldHand),clock=self.clock)
        self.driver.owned=True;self.driver.hand_sides=('left','right');transport.open(self.driver,self.driver.read_status())
        value=transport.encode_hand(self.profile,(100,)*6)
        self.assertFalse(hasattr(value,'durations'));transport.close()
        class Wrong(OldHand):
            @classmethod
            def get_fields_and_field_types(cls):return dict(super().get_fields_and_field_types(),positions='uint16[7]')
        with self.assertRaisesRegex(ValueError,'six_channel'):hand_schema(Wrong)

    def test_feedback_arrays_fail_closed_and_concurrent_arm_samples_do_not_mix(self):
        bad=hand_message();bad.positions.pop();self.feedback.hand('right',bad)
        self.assertIsNone(self.feedback.snapshot()['hands']['right']['feedback'])
        self.assertFalse(self.feedback.snapshot()['ready'])
        failures=[]
        def writer(value):
            for _ in range(100):self.feedback.arm(arm_message((value,)*14))
        workers=[threading.Thread(target=writer,args=(value,)) for value in (1.,2.)]
        for thread in workers:thread.start()
        for _ in range(100):
            state=self.feedback.read_state()
            if len(set(state.positions))!=1:failures.append(state.positions)
        for thread in workers:thread.join()
        self.assertEqual(failures,[])

    def test_readonly_discovery_reports_actual_consumers_and_services_without_calling_them(self):
        class Graph:
            def get_subscriptions_info_by_topic(self,topic):
                return [NS(node_name='controller_fixture',node_namespace='/',topic_type='ros2_bridge_msgs/msg/ArmCtrl',endpoint_gid=bytes([1]*16))] if topic=='/arm/cmd' else []
            def get_publishers_info_by_topic(self,topic):return []
            def get_service_names_and_types_by_node(self,name,namespace):
                return [('/controller_fixture/custom_service',['fixture_msgs/srv/Unknown'])]
            def create_publisher(self,*args):raise AssertionError('no publishers in discovery')
            def create_client(self,*args):raise AssertionError('no motion calls in graph discovery')
        value=graph_snapshot(Graph())
        self.assertIn('/controller_fixture',value['arm_consumers'])
        self.assertEqual(value['motion_services_called'],[])
        self.assertFalse(value['dual_arm_control_contract_verified'])
        self.assertEqual(set(value['topics']['subscribers']),set(TOPICS))


if __name__=='__main__':unittest.main()
