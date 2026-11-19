"""Clutch continuity with measured lag; manufacturer/IK fixtures, no hardware."""
from dataclasses import replace
import unittest
import numpy as np

from tg3_pico_dualarm.controller import Teleoperation
from tg3_pico_dualarm.model import DualArmModel,default_q
import test_manufacturer as fixtures
from test_controller import StubIK
from test_mapping import frame


class ManufacturerArmClutchTests(unittest.TestCase):
    def setUp(self):
        self.f=fixtures.ManufacturerTests();self.f.setUp();self.f.refresh(tuple(default_q()));self.f.enable()
        self.ik=StubIK();self.controller=Teleoperation(self.f.adapter,DualArmModel(),self.ik,
            allow_hardware=True,monotonic=self.f.clock)
        self.controller.reseed_after_hardware_enable();self.sequence=0

    def step(self,left=0.,right=0.,*,measured=None,left_tracked=True):
        self.f.now[0]+=.02;self.f.refresh(tuple(default_q()) if measured is None else tuple(measured))
        self.sequence+=1;item=frame(self.sequence,right,left_grip=left)
        item=replace(item,source_time_s=self.f.clock(),received_at=self.f.clock(),scene_visible=True,
            left=replace(item.left,tracked=left_tracked,pose=item.left.pose if left_tracked else None))
        return self.controller.tick(item)

    def create_six_milliradian_feedback_lag(self):
        self.step()
        for _ in range(8):status=self.step(1.,1.)
        self.assertEqual(status['state'],'hardware',status)
        np.testing.assert_allclose(self.controller.target[[0,7]]-default_q()[[0,7]],[.006,.006],atol=1e-10)

    def test_one_arm_release_retains_accepted_target_and_other_arm_keeps_moving(self):
        self.create_six_milliradian_feedback_lag()
        before=self.controller.target.copy();status=self.step(0.,1.)
        self.assertEqual(status['state'],'hardware',status)
        self.assertFalse(status['active']['left']);self.assertTrue(status['active']['right'])
        np.testing.assert_array_equal(self.controller.target[:7],before[:7])
        self.assertGreater(self.controller.target[7],before[7])
        self.assertTrue(self.f.adapter.hardware_writes_enabled)

    def test_regrip_latches_measured_posture_without_jumping_accepted_joint_target(self):
        self.create_six_milliradian_feedback_lag();self.step(0.,1.)
        before=self.controller.target.copy();status=self.step(1.,1.)
        self.assertEqual(status['state'],'hardware',status)
        np.testing.assert_array_equal(self.ik.anchors['left'],default_q()[:7])
        delta=self.controller.target-before
        self.assertLessEqual(np.max(abs(delta)),.1*.02+1e-9)
        self.assertGreater(self.controller.target[0],default_q()[0]+.005)

    def test_one_occluded_arm_retains_target_without_stopping_tracked_arm(self):
        self.create_six_milliradian_feedback_lag();before=self.controller.target.copy()
        status=self.step(1.,1.,left_tracked=False)
        self.assertEqual(status['state'],'hardware',status)
        np.testing.assert_array_equal(self.controller.target[:7],before[:7])
        self.assertGreater(self.controller.target[7],before[7])

    def test_both_release_latches_actual_hold_once_and_repeated_feedback_drift_does_not_move_it(self):
        self.create_six_milliradian_feedback_lag()
        status=self.step()
        self.assertEqual(status['state'],'hardware',status)
        held=tuple(default_q());self.assertEqual(self.f.adapter.commanded_joint_positions,held)
        drift=default_q()+.003
        for _ in range(4):status=self.step(measured=drift)
        self.assertEqual(status['state'],'hardware',status)
        self.assertEqual(self.f.adapter.commanded_joint_positions,held)
        np.testing.assert_array_equal(self.controller.target,held)
        holds=[event[1] for event in self.f.driver.events if isinstance(event,tuple) and event[0]=='hold']
        self.assertTrue(all(tuple(value)==held for value in holds[-5:]))
        before=self.controller.target.copy();status=self.step(0.,1.,measured=drift)
        self.assertEqual(status['state'],'hardware',status)
        self.assertLessEqual(np.max(abs(self.controller.target-before)),.002+1e-9)

    def test_repeated_hold_rejects_feedback_that_exceeds_configured_corridor(self):
        self.step()
        status=self.step(measured=default_q()+.03)
        self.assertEqual(status['state'],'disarmed',status)
        self.assertIn('hold_feedback_lead_violation',status['reason'])
        self.assertFalse(self.f.adapter.hardware_writes_enabled)

    def test_reseed_retains_acknowledged_target_when_measurements_advance_before_first_tick(self):
        acknowledged=self.f.adapter.commanded_joint_positions
        self.f.now[0]+=.02;self.f.refresh(tuple(default_q()+.006))
        self.controller.reseed_after_hardware_enable()
        np.testing.assert_array_equal(self.controller.target,acknowledged)


if __name__=='__main__':unittest.main()
