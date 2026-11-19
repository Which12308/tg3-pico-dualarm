"""Control-loop integration using explicit ROS doubles; no physical claims."""
from dataclasses import replace
import unittest
from tg3_pico_dualarm.controller import Teleoperation
from tg3_pico_dualarm.model import DualArmModel
from tg3_pico_dualarm.motion_control import MotionAdmission
import test_xarm_adapter as xarm_fixtures
from test_controller import StubIK
from test_mapping import frame


class XArmAdmissionIntegrationTests(unittest.TestCase):
    def setUp(self):
        self.fixture=xarm_fixtures.XArmAdapterTests();self.fixture.setUp();self.f=self.fixture
        self.gate=MotionAdmission(self.f.robot,tools_calibrated=True,clock=self.f.clock)
        self.seq=0

    def input(self,grip=0.):
        value=frame(self.seq,grip);self.seq+=1
        return replace(value,source_time_s=self.f.clock(),received_at=self.f.clock(),scene_visible=True)

    def test_default_disabled_cleanup_never_calls_hardware_services(self):
        self.assertTrue(self.f.robot.disarm('startup_watchdog').accepted)
        self.gate.tick(None)
        self.assertEqual(self.f.transport.events,[])
        self.assertFalse(self.f.robot.stop_confirmed)

    def test_initial_hold_failure_records_joint_differences_without_reseeding_target(self):
        self.f.robot.limits.feedback_lead=(.03,)*14
        self.f.arm();target=self.f.robot.commanded_joint_positions
        self.gate._begin_hold()
        measured=list(target);measured[2]+=.04
        self.f.feedback.arm(xarm_fixtures.arm_message(tuple(measured)))
        with self.assertRaisesRegex(RuntimeError,'initial_hold_feedback_corridor'):
            self.gate._settle_hold()
        self.gate._remember_failure('initial_hold_feedback_corridor_exceeded')
        details=self.gate.last_failure['hold_diagnostics']
        self.assertAlmostEqual(details['differences_rad'][2],.04)
        self.assertEqual(details['motor_ids'][2],13)
        self.assertEqual(details['accepted_target'],list(target))
        self.assertEqual(self.f.robot.commanded_joint_positions,target)
        self.assertFalse(self.f.transport.published)

    def test_hybrid_handoff_keeps_frozen_target_and_admits_clutch_after_half_second_with_small_static_bias(self):
        self.f.robot.limits.feedback_lead=(.03,)*14;self.f.arm()
        self.f.transport.holding_profile=object()
        target=self.f.robot.commanded_joint_positions;self.gate._begin_hold()
        for step in range(27):
            self.f.now[0]+=.02;self.f.refresh();self.f.transport.complete()
            self.f.feedback.arm(xarm_fixtures.arm_message(tuple(q+.01 for q in target)))
            self.gate._settle_hold()
            if step<23:self.assertFalse(self.gate.just_enabled)
            if self.gate.just_enabled:break
        self.assertTrue(self.gate.just_enabled)
        self.assertEqual(self.f.robot.commanded_joint_positions,target)
        self.assertTrue(all(q==target for q in self.f.transport.published))
        self.assertFalse(self.gate.hold_diagnostics['within_0_004rad_target_accuracy'])

    def test_hybrid_handoff_does_not_bypass_existing_feedback_lead_limit(self):
        self.f.robot.limits.feedback_lead=(.03,)*14;self.f.arm()
        self.f.transport.holding_profile=object();target=self.f.robot.commanded_joint_positions
        self.gate._begin_hold()
        self.f.feedback.arm(xarm_fixtures.arm_message(tuple(q+.031 for q in target)))
        with self.assertRaisesRegex(RuntimeError,'feedback_corridor'):self.gate._settle_hold()
        self.assertFalse(self.gate.just_enabled);self.assertFalse(self.f.transport.published)
        self.assertEqual(self.f.robot.commanded_joint_positions,target)

    def test_async_ack_reseeds_once_and_clutch_pause_resumes_without_new_enable(self):
        self.f.transport.params['vel_limits']=(.04,)*14
        value=self.input();self.assertTrue(self.gate.request('enable',value,expected_session=value.session)['pending'])
        for _ in range(35):
            self.f.now[0]+=.005;self.f.refresh();self.f.transport.complete()
            self.gate.tick(self.input())
            if self.f.robot.state=='armed':break
        self.assertEqual(self.f.robot.state,'armed',self.f.robot.reason)
        self.assertTrue(self.gate.consume_enable_ack());self.assertFalse(self.gate.consume_enable_ack())
        control=Teleoperation(self.f.robot,DualArmModel(),StubIK(),allow_hardware=True,monotonic=self.f.clock)
        control.reseed_after_hardware_enable()
        self.assertEqual(control.filter.config.joint_speed,.04)
        self.assertEqual(self.f.transport.published,[])
        for grip in (0.,1.,1.,0.,0.,1.,1.):
            self.f.now[0]+=.02;self.f.refresh()
            status=control.tick(self.input(grip))
            self.assertEqual(status['state'],'hardware',status)
        self.assertEqual(len(self.f.transport.published),7)
        self.gate.request('disable',self.input())
        for _ in range(25):
            self.f.now[0]+=.005;self.f.refresh();self.f.transport.complete();self.gate.tick(None)
            if self.f.robot.state!='releasing':break
        self.assertEqual(self.f.robot.state,'disabled',self.f.robot.reason)
        self.assertTrue(self.f.robot.stop_confirmed)

    def test_stop_during_periodic_auto_disable_waits_for_existing_ack(self):
        self.f.arm()
        self.f.now[0]+=.02;self.f.refresh()
        self.assertTrue(self.f.robot.hold().accepted)
        # Start the periodic audit explicitly without changing the control
        # clock or creating a second request on the same service transport.
        self.f.robot._next_audit=self.f.clock()
        self.f.robot.tick_transition()
        self.assertEqual(self.f.transport.pending[0],'auto')
        self.assertFalse(self.f.robot.disarm('operator_stop_during_audit').accepted)
        self.assertFalse(self.f.transport.uncertain_mutation)
        self.f.release()
        self.assertTrue(self.f.robot.stop_confirmed,self.f.robot.reason)


if __name__=='__main__':unittest.main()
