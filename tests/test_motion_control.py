"""Explicit manufacturer fixtures only; not physical-controller verification."""
from dataclasses import replace
import unittest
import numpy as np
from tg3_pico_dualarm.controller import Teleoperation
from tg3_pico_dualarm.motion_control import MotionAdmission
from tg3_pico_dualarm.model import DualArmModel,default_q
from tg3_pico_dualarm.robot import ReadOnlyRobot,RobotState
from tg3_pico_dualarm.wireless import InputGateway
import test_manufacturer as factory_fixtures
from test_controller import StubIK
from test_mapping import frame


class MotionControlTests(unittest.TestCase):
    def setUp(self):
        self.fixture=factory_fixtures.ManufacturerTests();self.fixture.setUp()
        self.f=self.fixture
        self.f.refresh(tuple(default_q()))
        self.gate=MotionAdmission(self.f.adapter,tools_calibrated=True,clock=self.f.clock)
        self.seq=0

    def input(self,grip=0.,focused=True,session='motion'):
        f=frame(self.seq,grip)
        self.seq+=1
        return replace(f,session=session,source_time_s=self.f.clock(),received_at=self.f.clock(),focused=focused,scene_visible=True)

    def advance(self):
        self.f.now[0]+=.02
        self.f.refresh(tuple(default_q()))

    def test_attempt_diagnostics_exclude_prior_alignment_and_count_only_new_output(self):
        from types import SimpleNamespace as NS
        alignment=dict(started_at=self.f.clock()-1.,phase='confirmed_before_output')
        transport=NS(actual_count=702,alignment_status=lambda:alignment)
        self.f.adapter.transport=transport
        old=self.input();old=replace(old,received_at=self.f.clock()-.2)
        self.assertFalse(self.gate.request('enable',old,expected_session=old.session)['accepted'])
        status=self.gate.status()
        self.assertEqual(status['attempt_id'],1);self.assertEqual(status['actual_output_count'],0)
        self.assertIsNone(status['initial_output_alignment'])
        self.assertIsNone(status['last_failure']['initial_output_alignment'])
        self.assertEqual(status['last_failure']['attempt_id'],1)
        alignment['started_at']=self.f.clock();transport.actual_count+=3
        self.assertEqual(self.gate.status()['actual_output_count'],3)
        self.assertIs(self.gate.status()['initial_output_alignment'],alignment)
        self.advance();held=self.input(1.)
        self.assertFalse(self.gate.request('enable',held,expected_session=held.session)['accepted'])
        self.assertEqual(self.gate.status()['actual_output_count'],0)
        self.assertIsNone(self.gate.status()['initial_output_alignment'])
        self.f.adapter.state='acquiring'
        self.gate.request('enable',held,expected_session=held.session)
        self.assertEqual(self.gate.attempt_id,2) # Duplicate enable is not another attempt.

    def test_enable_does_not_publish_and_tracking_loss_requires_new_explicit_request(self):
        f=self.input()
        self.assertTrue(self.gate.request('enable',f,expected_session=f.session)['accepted'])
        self.assertTrue(all(not p.messages for p in self.f.node.publishers))
        self.advance();self.gate.tick(self.input(1.,focused=False))
        self.assertFalse(self.f.adapter.hardware_writes_enabled)
        self.advance();self.gate.tick(self.input())
        self.assertFalse(self.f.adapter.hardware_writes_enabled)
        held=self.input(1.)
        self.assertFalse(self.gate.request('enable',held,expected_session=held.session)['accepted'])

    def test_validation_boundary_fetches_newest_frame_without_renewing_old_timestamp(self):
        old=self.input();old=replace(old,received_at=self.f.clock()-.084768778,source_age_s=.075289274)
        fresh=self.input();received=fresh.received_at
        self.gate.frame_provider=lambda:fresh
        result=self.gate.request('enable',old,expected_session=fresh.session)
        self.assertTrue(result['accepted'],result)
        self.assertIs(self.gate.validated_frame,fresh);self.assertEqual(fresh.received_at,received)
        self.assertEqual(old.received_at,self.f.clock()-.084768778)

    def test_exact_observed_160ms_age_still_refused_and_latest_invalid_never_falls_back(self):
        valid=self.input();stale=replace(valid,received_at=self.f.clock()-.084768778,source_age_s=.075289274)
        self.gate.frame_provider=lambda:stale
        result=self.gate.request('enable',valid,expected_session=valid.session)
        self.assertFalse(result['accepted']);self.assertIn('stale',result['reason'])
        self.assertAlmostEqual(self.gate.last_failure['input_timing']['total_age_s'],.160058052,places=8)
        self.assertEqual(self.gate.last_failure['stage'],'enable_input');self.assertFalse(self.f.driver.events)
        for item in (replace(valid,session='changed'),replace(valid,focused=False),replace(valid,right=replace(valid.right,grip=1.))):
            self.gate.frame_provider=lambda:item
            self.assertFalse(self.gate.request('enable',valid,expected_session=valid.session)['accepted'])
        self.assertFalse(self.f.driver.events)

    def test_total_150ms_boundary_and_120ms_source_budget_remain_separate(self):
        from tg3_pico_dualarm.input_validation import validate_frame
        item=replace(self.input(),received_at=.125,source_age_s=.025)
        validate_frame(item,.25)
        with self.assertRaises(ValueError):validate_frame(item,.250001)
        with self.assertRaises(ValueError):validate_frame(replace(item,received_at=.25,source_age_s=.120001),.25)

    def test_tool_passthrough_session_and_neutral_are_required_before_requesting_control(self):
        f=self.input()
        for item in (replace(f,scene_visible=False),replace(f,session='other'),
                     replace(f,right=replace(f.right,trigger=.8))):
            self.assertFalse(self.gate.request('enable',item,expected_session=f.session)['accepted'])
        self.assertEqual(self.f.driver.events,[])
        self.gate.tools_calibrated=False
        self.assertFalse(self.gate.request('enable',f,expected_session=f.session)['accepted'])

    def test_delayed_factory_ack_survives_duplicate_enable_and_is_consumed_once(self):
        self.f.driver.ack=False
        f=self.input()
        result=self.gate.request('enable',f,expected_session=f.session)
        self.assertTrue(result['pending'])
        self.assertFalse(self.gate.consume_enable_ack())
        self.advance();self.f.driver.owned=True
        f=self.input();self.gate.tick(f)
        self.assertTrue(self.gate.just_enabled)
        duplicate=self.gate.request('enable',f,expected_session=f.session)
        self.assertFalse(duplicate['accepted'])
        self.assertEqual(self.gate.session,f.session)
        # Another loop tick cannot erase the event before the consumer reseeds.
        self.gate.tick(f)
        self.assertTrue(self.gate.consume_enable_ack())
        self.assertFalse(self.gate.consume_enable_ack())
        self.advance();self.gate.tick(self.input())
        self.assertTrue(self.f.adapter.hardware_writes_enabled)

    def test_disable_cancels_unconsumed_factory_ack(self):
        f=self.input();self.gate.request('enable',f,expected_session=f.session)
        self.assertTrue(self.gate.just_enabled)
        self.gate.request('disable',f)
        self.assertFalse(self.gate.consume_enable_ack())
        self.assertIsNone(self.gate.session)

    def test_hardware_controller_reseeds_real_measurements_and_publishes_one_fourteen_joint_message(self):
        f=self.input()
        self.gate.request('enable',f,expected_session=f.session)
        c=Teleoperation(self.f.adapter,DualArmModel(),StubIK(),allow_hardware=True,monotonic=self.f.clock)
        c.reseed_after_hardware_enable()
        np.testing.assert_allclose(c.target,default_q())
        self.advance();status=c.tick(self.input())
        self.assertEqual(status['state'],'hardware',status)
        self.assertEqual(len(self.f.node.publishers[0].messages),0) # factory hold, no initial target
        self.advance();status=c.tick(self.input(1.))
        self.assertEqual(status['state'],'hardware',status)
        messages=self.f.node.publishers[0].messages
        self.assertEqual(len(messages),1)
        self.assertEqual([m.name for m in messages[0].ctrl],list(range(11,18))+list(range(21,28)))
        self.assertTrue(status['robot_commands_sent'])
        self.advance();c.tick(self.input(1.,focused=False))
        self.assertFalse(self.f.adapter.hardware_writes_enabled)
        self.assertEqual(len(messages),1)

    def test_generic_readonly_adapter_cannot_be_promoted_by_allow_hardware_flag(self):
        robot=ReadOnlyRobot(lambda:RobotState(tuple(default_q()),self.f.clock(),'mode',False,'measured'))
        with self.assertRaisesRegex(ValueError,'verified_manufacturer_adapter_required'):
            Teleoperation(robot,DualArmModel(),StubIK(),allow_hardware=True)

    def test_controller_executes_continuous_raw_hand_intent_and_one_press_open(self):
        f=self.input();self.gate.request('enable',f,expected_session=f.session)
        c=Teleoperation(self.f.adapter,DualArmModel(),StubIK(),allow_hardware=True,monotonic=self.f.clock)
        c.reseed_after_hardware_enable()
        def step(trigger=0.,buttons=()):
            self.advance()
            for side in self.f.adapter.profiles:
                self.f.feedback.hand(side,factory_fixtures.hand_message(self.f.adapter.hand_positions[side][0]))
            value=self.input()
            value=replace(value,right=replace(value.right,trigger=trigger,buttons=buttons))
            status=c.tick(value)
            self.assertEqual(status['state'],'hardware',status)
            return status
        step()
        for _ in range(420):status=step(1.)
        self.assertEqual(status['hands']['commanded_motor_positions']['right'],(900,)*6)
        held=self.f.adapter.hand_positions['right']
        for _ in range(4):step()
        self.assertEqual(self.f.adapter.hand_positions['right'],held)
        step(buttons=('B',))
        for _ in range(470):status=step()
        self.assertEqual(status['hands']['commanded_motor_positions']['right'],(10,)*6)
        self.assertEqual(len(self.f.node.publishers[0].messages),0)

    def test_control_endpoint_does_not_exist_for_headset_role_and_nonce_cannot_replay(self):
        g=InputGateway(role='relay',key='r'*40)
        g.control_callback=lambda request:dict(accepted=True,queued=True)
        nonce=g.challenge('r'*40)['nonce']
        body=dict(protocol='tg3-native-1',operation='disable',session=None)
        self.assertTrue(g.request_control('r'*40,nonce,body)['queued'])
        with self.assertRaisesRegex(ValueError,'challenge'):g.request_control('r'*40,nonce,body)
        h=InputGateway();token=h.pair(h.open_pairing(),'PICO')
        with self.assertRaisesRegex(ValueError,'not_configured'):
            h.request_control(token,h.challenge(token)['nonce'],body)


if __name__=='__main__':unittest.main()
