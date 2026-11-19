"""Raw hand execution with explicit manufacturer test doubles; no real robot."""
from dataclasses import replace
import unittest

from tg3_pico_dualarm.manufacturer_grasp import ManufacturerGrasp
from tg3_pico_dualarm.hands import HandCalibration
import test_manufacturer as fixtures
from test_mapping import frame


class ManufacturerGraspTests(unittest.TestCase):
    def setUp(self):
        self.f=fixtures.ManufacturerTests();self.f.setUp();self.f.enable()
        self.grasp=ManufacturerGrasp(self.f.adapter,clock=self.f.clock)
        self.grasp.initialize();self.seq=0

    def step(self,trigger=0.,buttons=(),*,tracked=True,focused=True,follow=True,left_trigger=0.,session='grasp'):
        self.f.now[0]+=.02;self.f.refresh()
        if follow:
            for side in self.f.adapter.profiles:
                self.f.feedback.hand(side,fixtures.hand_message(self.f.adapter.hand_positions[side][0]))
        self.assertTrue(self.f.adapter.hold().accepted)
        self.seq+=1;value=frame(self.seq)
        value=replace(value,session=session,source_time_s=self.f.clock(),received_at=self.f.clock(),focused=focused,
            right=replace(value.right,trigger=trigger,buttons=buttons,tracked=tracked,
                pose=value.right.pose if tracked else None),left=replace(value.left,trigger=left_trigger))
        return self.grasp.step(value),value

    def test_raw_limiter_completes_all_six_channels_instead_of_stopping_at_scalar_projection(self):
        self.step()
        for _ in range(440):status,_=self.step(1.)
        self.assertEqual(self.f.adapter.hand_positions['right'],(900,)*6)
        self.assertEqual(status['commanded_motor_positions']['right'],(900,)*6)
        self.assertTrue(status['closure_is_projection'])
        self.assertIsNone(status['fault'])

    def test_one_open_press_continues_until_measured_open_target_then_stops(self):
        self.step()
        for _ in range(120):self.step(1.)
        self.step(buttons=('B',))
        for _ in range(180):status,_=self.step()
        self.assertEqual(self.f.adapter.hand_positions['right'],(10,)*6)
        self.assertEqual(status['measured_motor_positions']['right'],(10.,)*6)
        messages=len(self.f.node.publishers[2].messages)
        for _ in range(5):self.step()
        self.assertEqual(len(self.f.node.publishers[2].messages),messages)

    def test_lower_or_released_trigger_cannot_open_previously_closed_motor_channels(self):
        self.step()
        for _ in range(50):self.step(1.)
        before=self.f.adapter.hand_positions['right']
        for _ in range(20):self.step(.2)
        self.assertEqual(self.f.adapter.hand_positions['right'],before)
        count=len(self.f.node.publishers[2].messages)
        for _ in range(20):self.step(0.)
        self.assertEqual(self.f.adapter.hand_positions['right'],before)
        self.assertEqual(len(self.f.node.publishers[2].messages),count)

    def test_lost_tracking_cancels_pending_open_and_requires_fresh_neutral(self):
        self.step();self.step(buttons=('B',));self.step(tracked=False)
        before=self.f.adapter.hand_positions['right'];count=len(self.f.node.publishers[2].messages)
        for _ in range(10):self.step(1.)
        self.assertEqual(self.f.adapter.hand_positions['right'],before)
        self.assertEqual(len(self.f.node.publishers[2].messages),count)
        self.step();self.step(1.)
        self.assertGreater(self.f.adapter.hand_positions['right'][0],before[0])

    def test_candidate_clips_at_feedback_corridor_without_fault_or_contact_claim(self):
        self.step()
        for _ in range(90):status,_=self.step(1.,follow=False)
        self.assertTrue(self.f.adapter.hardware_writes_enabled)
        self.assertEqual(self.f.adapter.hand_positions['right'],(120,)*6)
        diag=status['motor_diagnostics']['right']
        self.assertTrue(diag['feedback_limited'])
        self.assertEqual(diag['limit_interpretation'],'feedback_corridor_only')
        self.assertIn('feedback_corridor',status['reason']['right'])
        self.assertNotIn('contact',status['reason']['right'])
        self.assertIsNone(status['fault'])

    def test_abnormal_feedback_moving_previous_target_outside_corridor_still_faults(self):
        self.step()
        self.f.now[0]+=.02;self.f.refresh()
        self.f.feedback.hand('right',fixtures.hand_message(70))
        result=self.f.adapter.submit_hand('right',1.,intent_stamp=self.f.clock(),operation='close')
        self.assertFalse(result.accepted);self.assertIn('hand_feedback_lead_violation',result.reason)
        self.assertFalse(self.f.adapter.hardware_writes_enabled)

    def test_duplicate_packet_does_not_advance_and_one_occluded_side_does_not_block_other_hand(self):
        self.step();status,value=self.step(1.,tracked=False,left_trigger=1.)
        count=len(self.f.node.publishers[1].messages)
        before=self.f.adapter.hand_positions['left']
        self.grasp.step(value)
        self.assertEqual(len(self.f.node.publishers[1].messages),count)
        self.assertEqual(self.f.adapter.hand_positions['left'],before)
        self.assertTrue(status['hardware_hand_commands_sent'])
        self.assertEqual(status['hardware_hand_sides'],['left'])

    def test_session_change_and_direct_uninitialized_step_do_not_resume_motion(self):
        self.step();self.step(1.)
        count=len(self.f.node.publishers[2].messages)
        self.step(1.,session='new-reference')
        self.assertEqual(len(self.f.node.publishers[2].messages),count)
        self.grasp.invalidate();self.step(1.,session='new-reference')
        self.assertEqual(len(self.f.node.publishers[2].messages),count)

    def test_close_operation_obeys_each_calibrated_direction_and_holds_constant_channels(self):
        self.f.adapter.disarm()
        profile=HandCalibration((10,900,500,10,900,400),(900,10,500,900,10,400),
            (1,)*6,(1000,)*6,(20,)*6,(30,)*6,'TEST ONLY mixed channel directions')
        self.f.adapter=self.f.make_adapter(profiles={'right':profile})
        self.f.enable()
        self.f.now[0]+=.02;self.f.refresh()
        result=self.f.adapter.submit_hand('right',.1111111111,intent_stamp=self.f.clock(),operation='close')
        self.assertTrue(result.accepted,result.reason)
        self.assertEqual(self.f.adapter.hand_positions['right'],(102,100,100,102,100,100))

    def test_slow_fractional_rate_accumulates_instead_of_rounding_every_tick_to_zero(self):
        self.f.adapter.limits.hand_raw_rate['right']=(1.,)*6
        self.step()
        for _ in range(80):status,_=self.step(1.)
        self.assertEqual(self.f.adapter.hand_positions['right'],(102,)*6)
        self.assertIsNone(status['fault'])


if __name__=='__main__':unittest.main()
