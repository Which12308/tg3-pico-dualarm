from dataclasses import replace
import unittest
from tg3_pico_dualarm.hands import ContinuousHands,HandCalibration,BrainCoAdapter
from test_mapping import frame


class HandTests(unittest.TestCase):
    def setUp(self):
        self.hands=ContinuousHands();self.seq=0;self.now=1.
        self.send()

    def send(self,trigger=0.,buttons=(),left_trigger=0.,left_buttons=(),tracked=True,**changes):
        f=frame(self.seq);f=replace(f,**changes)
        self.seq+=1;self.now=f.received_at
        f=replace(f,right=replace(f.right,trigger=trigger,buttons=buttons,tracked=tracked),
                    left=replace(f.left,trigger=left_trigger,buttons=left_buttons))
        return self.hands.update(f,self.now,.02)

    def test_partial_trigger_gradual_close_then_release_retains_grasp(self):
        first=self.send(.55)['closure']['right'];self.assertAlmostEqual(first,.01)
        for _ in range(60):status=self.send(.55)
        self.assertAlmostEqual(status['closure']['right'],.5)
        for _ in range(20):status=self.send()
        self.assertAlmostEqual(status['closure']['right'],.5)
        self.assertEqual(status['closure']['left'],0)
        self.assertFalse(status['hardware_hand_commands_sent'])

    def test_releasing_trigger_cancels_unfinished_closure_immediately(self):
        first=self.send(1.)['closure']['right']
        for _ in range(20):status=self.send()
        self.assertEqual(status['closure']['right'],first)

    def test_y_opens_left_b_opens_right_and_trigger_must_release_after_open(self):
        for _ in range(30):self.send(1,left_trigger=1)
        status=self.send(1,buttons=('B',),left_trigger=1)
        self.assertLess(status['closure']['right'],status['closure']['left'])
        for _ in range(30):status=self.send(1,buttons=('B',),left_trigger=1)
        self.assertAlmostEqual(status['closure']['right'],0)
        self.assertGreater(status['closure']['left'],.5)
        for _ in range(10):status=self.send(1,left_trigger=1,left_buttons=('Y',))
        self.assertAlmostEqual(status['closure']['right'],0)
        self.send();self.assertGreater(self.send(1)['closure']['right'],0)

    def test_loss_freezes_grasp_and_cancels_old_ramp_without_opening_object(self):
        for _ in range(5):self.send(1)
        before=self.hands.closure['right']
        self.send(1,tracked=False)
        for _ in range(10):status=self.send(1)
        self.assertAlmostEqual(status['closure']['right'],before)
        self.send();self.assertGreater(self.send(1)['closure']['right'],before)

    def test_restart_requires_neutral_and_retained_closure_is_not_an_initial_motor_pose(self):
        self.send(1);before=self.hands.closure['right']
        self.send(1,session='new');self.assertEqual(self.hands.closure['right'],before)
        new=ContinuousHands();new.update(replace(frame(0,1.),right=replace(frame(0).right,trigger=1.)),1.,.02)
        self.assertEqual(new.closure['right'],0)

    def test_manufacturer_adapter_uses_per_channel_endpoints_not_bottle_sequences(self):
        profile=HandCalibration((10,20,30,40,50,60),(110,120,130,140,150,160),
            (0,)*6,(200,)*6,(10,)*6,(20,)*6,'test fixture, not robot calibration')
        class Message:
            def get_fields_and_field_types(self):return dict.fromkeys(('mode','positions','speeds','currents','pwms'),'fixture')
        value=BrainCoAdapter(profile,Message).format(.5)
        self.assertEqual(value.positions,[60,70,80,90,100,110])
        self.assertEqual(value.speeds,[10]*6);self.assertEqual(value.currents,[20]*6)
        with self.assertRaises(ValueError):BrainCoAdapter(None,Message)
        with self.assertRaises(ValueError):HandCalibration((0,)*6,(1,)*6,(0,)*6,(10,)*6,(0,)*6,(0,)*6,'')


if __name__=='__main__':unittest.main()
