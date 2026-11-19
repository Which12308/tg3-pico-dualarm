"""Fault-path checks for stationary authority and native hybrid output.

Explicit ROS doubles and synthetic files only; no physical acceptance claim.
"""
import hashlib
import math
from pathlib import Path
import tempfile
from types import SimpleNamespace as NS
import unittest
from unittest.mock import patch
import yaml
from tg3_pico_dualarm.holding_profile import HoldingProfile,load_holding_profile
from tg3_pico_dualarm.stationary_guard import StationaryCommandGuard,bridge_isolation_remaps,WALK,STAND,STATUS,ISOLATED
from tg3_pico_dualarm.supervised_station import FactoryObserver
import test_managed_xarm as managed_fixtures
import test_xarm_adapter as fixtures
import test_initial_alignment as alignment_fixtures


def twist():
    return NS(header=NS(stamp=None,frame_id=''),twist=NS(
        linear=NS(x=0.,y=0.,z=0.),angular=NS(x=0.,y=0.,z=0.)))


class StationaryTests(unittest.TestCase):
    def setUp(self):
        self.now=[1.];self.sent=[]
        self.endpoints={WALK:[NS(node_name='test',node_namespace='/',topic_type='geometry_msgs/msg/TwistStamped')],STAND:[],
            STATUS:[NS(node_name='xmigcs_control_node',node_namespace='/',topic_type='geometry_msgs/msg/TwistStamped')]}
        node=NS(create_publisher=lambda *a:NS(publish=self.sent.append),create_subscription=lambda *a:None,
            create_timer=lambda *a:None,destroy_timer=lambda *a:None,destroy_subscription=lambda *a:None,
            destroy_publisher=lambda *a:None,get_publishers_info_by_topic=lambda topic:self.endpoints[topic],
            get_name=lambda:'test',get_namespace=lambda:'/',get_clock=lambda:NS(now=lambda:NS(to_msg=lambda:None)))
        self.guard=StationaryCommandGuard(node,clock=lambda:self.now[0],message_type=twist)

    def quiet(self):
        for _ in range(9):
            self.now[0]+=.04;self.guard._applied(twist());self.guard._tick()

    def test_disabled_start_has_no_zero_commands_then_heartbeat_is_required(self):
        self.guard._tick();self.assertFalse(self.sent)
        self.guard.begin();self.assertFalse(self.guard.poll_ready());self.assertEqual(len(self.sent),1)
        self.quiet();self.guard.check();count=len(self.sent)
        for _ in range(4):self.now[0]+=.04;self.guard._applied(twist())
        with self.assertRaisesRegex(RuntimeError,'fresh_stationary'):self.guard.check()
        self.guard._tick();self.guard.check();self.assertEqual(len(self.sent),count+1)
        self.guard.end();count=len(self.sent);self.guard._tick();self.assertEqual(len(self.sent),count)
        self.assertFalse(self.guard.status()['physical_balance_verified'])

    def test_competing_walk_or_waist_writer_prevents_admission_before_zero(self):
        for topic in (WALK,STAND):
            self.endpoints[topic].append(NS(node_name='other',node_namespace='/',topic_type='geometry_msgs/msg/TwistStamped'))
            with self.assertRaises(RuntimeError):self.guard.begin()
            self.assertFalse(self.sent);self.assertFalse(self.guard.active)
            self.endpoints[topic].pop()

    def test_nonzero_or_nan_command_latches_and_cannot_auto_resume(self):
        for value in (.01,float('nan')):
            self.setUp();self.guard.begin();self.quiet();message=twist();message.twist.linear.x=value
            self.guard._observe(message)
            with self.assertRaisesRegex(RuntimeError,'nonzero_or_invalid'):self.guard.check()
            self.guard.end()
            with self.assertRaisesRegex(RuntimeError,'latched'):self.guard.begin()

    def test_registered_factory_navigation_may_be_idle_but_nonzero_feedback_yields_without_fighting_it(self):
        self.endpoints[WALK].append(NS(node_name='hric_path_tracker',node_namespace='/',topic_type='geometry_msgs/msg/TwistStamped'))
        self.guard.begin();self.quiet();self.assertTrue(self.guard.poll_ready())
        value=twist();value.twist.angular.z=.1;self.guard._applied(value)
        self.assertTrue(self.guard.external_request);self.assertFalse(self.guard.active)
        count=len(self.sent);self.guard._tick();self.guard.end();self.assertEqual(len(self.sent),count)
        with self.assertRaisesRegex(RuntimeError,'not_zero'):self.guard.check()

    def test_body_transition_waits_for_live_factory_zero_feedback_before_service_or_fsm_mutation(self):
        f=fixtures.XArmAdapterTests();f.setUp();w=managed_fixtures.ModeFixture(f)
        w.prep.command_guard=self.guard;w.prep.begin()
        self.assertEqual(w.prep.phase,'stationary');self.assertFalse(w.calls);self.assertFalse(w.messages)
        self.assertFalse(w.step());self.assertFalse(w.calls)
        self.quiet();self.assertFalse(w.step());self.assertEqual(w.prep.phase,'inactive')
        self.assertEqual(len(w.calls),1);self.assertFalse(w.messages)

    def test_factory_isolation_keeps_fsm_and_estop_but_removes_walk_and_stand_targets(self):
        remaps=bridge_isolation_remaps()
        self.assertEqual(len(remaps),2*len(ISOLATED))
        for topic in ISOLATED:self.assertIn(topic+':=/tg3/xarm_retired'+topic,remaps)
        self.assertFalse(any('fsm_state_cmd' in r or 'estop' in r or '/head/cmd' in r for r in remaps))


class HoldingProfileTests(unittest.TestCase):
    def files(self,root,*,different=False):
        pairs={}
        for index,name in enumerate(('stop.yaml','standzero.yaml')):
            value=dict(motor_num=31,kp_pos=[100.]*31,kd_pos=[7.4]*31)
            if different and index:value['kp_pos'][15]=99.
            raw=yaml.safe_dump(value).encode();(root/name).write_bytes(raw)
            pairs[name]=hashlib.sha256(raw).hexdigest()
        return pairs

    def test_tampered_configuration_or_disagreeing_hold_gains_prevents_loading(self):
        with tempfile.TemporaryDirectory() as folder:
            root=Path(folder);sources=self.files(root)
            with patch('tg3_pico_dualarm.holding_profile.SOURCES',sources):
                profile=load_holding_profile(root);self.assertEqual(profile.kp,(100.,)*14)
                (root/'stop.yaml').write_text('changed',encoding='utf-8')
                with self.assertRaisesRegex(RuntimeError,'changed_review_required'):load_holding_profile(root)
            sources=self.files(root,different=True)
            with patch('tg3_pico_dualarm.holding_profile.SOURCES',sources):
                with self.assertRaisesRegex(RuntimeError,'disagree'):load_holding_profile(root)

    def test_hybrid_uses_service_mode_zero_after_body_settles(self):
        f=fixtures.XArmAdapterTests();f.setUp();w=managed_fixtures.ModeFixture(f)
        w.prep.requested_arm_mode=0;w.ready()
        self.assertEqual(w.mode_writes,[(0,'HBWALK')]);self.assertEqual(w.prep.status()['requested_arm_mode'],0)

    def test_external_body_request_is_not_overwritten_with_gotoSTART_on_release(self):
        f=fixtures.XArmAdapterTests();f.setUp();w=managed_fixtures.ModeFixture(f);w.ready()
        w.prep.command_guard=NS(external_request=True,end=lambda:None,status=lambda:{})
        w.prep.cancel()
        for _ in range(10):
            f.now[0]+=.02;w.refresh('HBWALK')
            if w.prep.restore_step(safe_to_restore=True):break
        self.assertTrue(w.prep.restored)
        self.assertEqual(w.mode,0)
        self.assertNotIn('gotoSTART',w.messages)
        self.assertIn('external_body_preserved',w.prep.reason)


class HybridRawTests(unittest.TestCase):
    def fixture(self):
        relay,messages,message=managed_fixtures.BoundedRawTests().fixture()
        relay.holding_profile=HoldingProfile((100.,)*14,(7.4,)*14,())
        message.mode=1
        for motor in message.ctrl:motor.kp=100.;motor.kd=7.4;motor.tor=.1;motor.spd=0.
        return relay,messages,message

    def test_signed_feedforward_matches_bounded_target_and_holding_is_zero(self):
        for target in (.5,-.5,0.):
            relay,messages,message=self.fixture()
            for motor in message.ctrl:motor.pos=target
            relay.raw(message);self.assertEqual(len(messages),1,relay.output_fault)
            for motor in messages[0].ctrl:
                self.assertLessEqual(abs(motor.spd),.02+1e-12)
                self.assertAlmostEqual(motor.spd,motor.pos/.01)
                self.assertEqual(motor.tor,.1);self.assertEqual(motor.kp,100.)
                if target==0.:self.assertEqual(motor.spd,0.)
            self.assertEqual(message.ctrl[0].spd,0.)

    def test_wrong_gains_mode_reserved_or_torque_never_reach_actuators(self):
        for key,value in (('kp',5.),('kd',float('nan')),('tor',100.),('tor',float('nan')),('mode',0),('reserved',165)):
            relay,messages,message=self.fixture()
            setattr(message if key in ('mode','reserved') else message.ctrl[0],key,value)
            relay.raw(message)
            self.assertFalse(messages,(key,value));self.assertIsNone(relay.actual_publisher)
            self.assertIsNotNone(relay.output_fault)

    def test_default_low_native_gains_are_rejected_before_actuator_publisher_exists(self):
        relay=alignment_fixtures.InitialAlignmentTests().transport()
        relay.holding_profile=HoldingProfile((100.,)*14,(7.4,)*14,())
        relay.begin_initial_alignment((.2,)*14)
        _,_,message=self.fixture()
        for motor in message.ctrl:motor.pos=.2;motor.kp=5.
        relay.raw(message)
        self.assertIn('gains_not_confirmed',relay.output_fault)
        self.assertEqual(relay.real,[]);self.assertIsNone(relay.actual_publisher)


class InactiveParameterTests(unittest.TestCase):
    def fixture(self,*,active=False,writer=False,ignore_set=False):
        observer=object.__new__(FactoryObserver);calls=[]
        values=dict(kp=(5.,)*14,kd=(1.,)*14)
        observer.node=NS(get_publishers_info_by_topic=lambda topic:[object()] if writer else [])
        def call(cls,name,configure=lambda r:None):
            request=NS();configure(request);calls.append((name,request))
            if name.endswith('list_controllers'):
                return NS(controller=[fixtures.controller('jointspace_ruckig_qp_controller','active' if active else 'inactive')])
            if name.endswith('get_parameters'):
                return NS(values=[NS(type=8,double_array_value=values[k]) for k in request.names])
            if name.endswith('set_parameters'):
                if not ignore_set:
                    for p in request.parameters:values[p.name]=tuple(p.value.double_array_value)
                return NS(results=[NS(successful=True),NS(successful=True)])
            raise AssertionError('Unexpected service '+name)
        observer.call=call
        types=dict(get=object,set=object,list=object,parameter=NS,value=NS)
        profile=HoldingProfile((100.,)*14,(7.4,)*14,())
        return observer,types,profile,calls

    def test_parameters_are_set_after_declaration_and_read_back_while_no_actuator_writer_exists(self):
        observer,types,profile,calls=self.fixture()
        result=observer.configure_holding_parameters(profile,types=types)
        self.assertFalse(result['hardware_motion_enabled']);self.assertEqual(result['actuator_publishers'],0)
        self.assertEqual(result['before']['kp'],[5.]*14);self.assertEqual(result['after']['kp'],[100.]*14)
        self.assertEqual([n.rsplit('/',1)[-1] for n,_ in calls],
            ['list_controllers','get_parameters','set_parameters','get_parameters','list_controllers'])

    def test_active_resource_or_existing_writer_prevents_parameter_mutation(self):
        for kwargs in (dict(active=True),dict(writer=True)):
            observer,types,profile,calls=self.fixture(**kwargs)
            with self.assertRaises(RuntimeError):observer.configure_holding_parameters(profile,types=types)
            self.assertFalse(any(n.endswith('set_parameters') for n,_ in calls))

    def test_ack_without_actual_parameter_update_is_not_ready(self):
        observer,types,profile,calls=self.fixture(ignore_set=True)
        with self.assertRaisesRegex(RuntimeError,'readback_mismatch'):
            observer.configure_holding_parameters(profile,types=types)
