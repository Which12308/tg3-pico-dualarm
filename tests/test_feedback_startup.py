import unittest
from tg3_pico_dualarm.manufacturer_feedback import ManufacturerFeedback
from tg3_pico_dualarm.feedback_startup import FeedbackStartupGate
from test_manufacturer import arm_message,mode_message,power_key_message,hand_message


class StartupFeedbackTests(unittest.TestCase):
    def setUp(self):
        self.now=[10.];self.clock=lambda:self.now[0]
        self.feedback=ManufacturerFeedback(clock=self.clock)

    def refresh(self,hand=True):
        self.feedback.arm(arm_message());self.feedback.fsm(mode_message());self.feedback.key_status(power_key_message())
        if hand:self.feedback.hand('right',hand_message())

    def test_body_ready_never_means_missing_required_hand_is_ready(self):
        gate=FeedbackStartupGate(self.feedback,require_hands=('right',),clock=self.clock)
        for _ in range(15):
            self.now[0]+=.05;self.refresh(False)
            self.assertTrue(self.feedback.snapshot(require_hands=False)['ready'])
            self.assertFalse(gate.poll())
        self.assertIn('right_hand_feedback_missing_or_stale',gate.status()['issues'])
        for _ in range(6):self.now[0]+=.05;self.refresh();gate.poll()
        self.assertTrue(gate.ready);self.assertNotIn('left_hand_feedback_missing_or_stale',gate.status()['issues'])

    def test_late_hand_startup_does_not_extend_live_freshness(self):
        gate=FeedbackStartupGate(self.feedback,require_hands=('right',),clock=self.clock,stable_s=.1)
        for _ in range(5):self.now[0]+=.05;self.refresh();gate.poll()
        self.assertTrue(gate.ready)
        self.now[0]+=.251;self.refresh(False)
        self.assertFalse(gate.poll());self.assertFalse(self.feedback.snapshot(require_hands=('right',))['ready'])
        self.assertGreater(self.feedback.snapshot()['hands']['right']['age_s'],.25)

    def test_timeout_reports_stream_counts_ages_and_missing_hand(self):
        gate=FeedbackStartupGate(self.feedback,require_hands=('right',),clock=self.clock,timeout=1.)
        self.refresh(False);gate.poll();self.now[0]+=1.01;self.refresh(False)
        with self.assertRaisesRegex(RuntimeError,'right_hand_feedback_missing_or_stale'):gate.poll()
        self.assertFalse(gate.ready)
        streams=gate.status()['feedback_streams']
        self.assertEqual(streams['arm']['count'],2);self.assertAlmostEqual(streams['arm']['maximum_interval_s'],1.01)

    def test_estop_or_guard_veto_resets_stable_window(self):
        gate=FeedbackStartupGate(self.feedback,require_hands=('right',),clock=self.clock,stable_s=.1)
        self.refresh();gate.poll();self.now[0]+=.05;self.feedback.key_status(power_key_message(estop=True))
        self.assertFalse(gate.poll())
        self.now[0]+=.06;self.refresh();self.assertFalse(gate.poll())
        guarded=FeedbackStartupGate(self.feedback,clock=self.clock,guard=lambda:(_ for _ in ()).throw(RuntimeError('physical_remote_stale')))
        self.assertFalse(guarded.poll());self.assertEqual(guarded.status()['guard_error'],'physical_remote_stale')

    def test_vendor_running_state_is_allowed_but_stall_turbo_unknown_and_stale_are_blocked(self):
        self.refresh()
        for code in (0,1,2,3,7,255):
            message=hand_message();message.states[0]=code;self.feedback.hand('right',message)
            snap=self.feedback.snapshot(require_hands=('right',))
            self.assertEqual(snap['ready'],code in (0,1),snap)
            self.assertEqual(snap['hands']['right']['all_state_codes_zero'],code==0)
            self.assertTrue(snap['hands']['right']['state_codes_interpreted'])
        message=hand_message();message.states[0]=1;self.feedback.hand('right',message)
        self.now[0]+=.251;self.refresh(False)
        self.assertFalse(self.feedback.snapshot(require_hands=('right',))['ready'])


if __name__=='__main__':unittest.main()
