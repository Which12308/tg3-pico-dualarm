import unittest
import numpy as np
from tg3_pico_dualarm.model import DualArmModel,default_q
from tg3_pico_dualarm.safety import JointSafetyFilter,SafetyError,SafetyLimits


class SafetyTests(unittest.TestCase):
    def setUp(self):
        self.model=DualArmModel();self.filter=JointSafetyFilter(self.model);self.q=default_q()

    def test_large_request_obeys_acceleration_speed_and_feedback_bounds(self):
        target,velocity=self.filter.step(self.q+.2,self.q,self.q,np.zeros(14),.02)
        self.assertLessEqual(max(abs(velocity)),.010000001)
        self.assertLessEqual(max(abs(target-self.q)),.035)
        for side,offset in (('left',0),('right',7)):
            before=self.model.arms[side].fk(self.q[offset:offset+7])
            after=self.model.arms[side].fk(target[offset:offset+7])
            self.assertLessEqual(np.linalg.norm(after[:3,3]-before[:3,3]),.04*.02+1e-9)

    def test_stale_seed_invalid_feedback_and_overrun_rejected(self):
        for measured,previous,dt in ((self.q,self.q+.04,.02),(self.q,self.q,.2),
                                     (self.q*np.nan,self.q,.02)):
            with self.assertRaises(SafetyError):
                self.filter.step(self.q,measured,previous,np.zeros(14),dt)

    def test_ordinary_goal_arrival_brakes_with_the_same_acceleration_bound(self):
        previous=self.q.copy();old_velocity=np.zeros(14);old_velocity[6]=.12
        # An active solver at its requested pose is not an emergency hold.
        # The filter must keep braking through further increments, not erase
        # an existing velocity in one tick merely because the request is zero.
        for expected in (.11,.10,.09,.08,.07,.06,.05,.04,.03,.02,.01,0.):
            target,velocity=self.filter.step(previous,previous,previous,old_velocity,.02)
            self.assertAlmostEqual(velocity[6],expected,places=9)
            self.assertLessEqual(np.max(abs(velocity-old_velocity)),.5*.02+1e-9)
            previous=target;old_velocity=velocity

    def test_reversal_brakes_to_zero_before_reversing(self):
        previous=self.q.copy();old_velocity=np.zeros(14);old_velocity[6]=.04
        signs=[]
        for _ in range(6):
            proposed=previous.copy();proposed[6]-=.01
            target,velocity=self.filter.step(proposed,previous,previous,old_velocity,.02)
            self.assertLessEqual(np.max(abs(velocity-old_velocity)),.5*.02+1e-9)
            signs.append(velocity[6])
            previous=target;old_velocity=velocity
        np.testing.assert_allclose(signs,[.03,.02,.01,0.,-.01,-.02],atol=1e-9)

    def test_feedback_boundary_cannot_silently_override_required_braking(self):
        measured=self.q.copy();previous=self.q.copy();previous[6]+=.035
        old_velocity=np.zeros(14);old_velocity[6]=.08
        with self.assertRaisesRegex(SafetyError,'acceleration_and_position_bounds_infeasible'):
            self.filter.step(previous,measured,previous,old_velocity,.02)

    def test_explicit_low_speed_protective_hold_freezes_then_resumes_with_feedback(self):
        guarded=JointSafetyFilter(self.model,SafetyLimits(joint_speed=.02,feedback_lead=.03),boundary_hold=True)
        measured=self.q.copy();previous=self.q.copy();previous[13]+=.03
        old_velocity=np.zeros(14);old_velocity[13]=.02
        proposed=previous.copy();proposed[13]+=.01;proposed[6]+=.01
        target,velocity=guarded.step(proposed,measured,previous,old_velocity,.02)
        np.testing.assert_array_equal(target[7:],previous[7:])
        np.testing.assert_array_equal(velocity[7:],np.zeros(7))
        self.assertGreater(target[6],previous[6])
        self.assertEqual(guarded.last_diagnostics['boundary_holds'][0]['side'],'right')
        self.assertTrue(guarded.last_diagnostics['boundary_holds'][0]['acceleration_override'])
        resumed,v=guarded.step(proposed,target,target,velocity,.02)
        self.assertGreater(resumed[13],target[13]);self.assertLessEqual(v[13],.010000001)
        self.assertEqual(guarded.last_diagnostics['boundary_holds'],[])

    def test_protective_hold_does_not_admit_target_already_outside_feedback_window(self):
        guarded=JointSafetyFilter(self.model,SafetyLimits(joint_speed=.02,feedback_lead=.03),boundary_hold=True)
        previous=self.q.copy();previous[13]+=.03001
        with self.assertRaisesRegex(SafetyError,'previous_target_outside_feedback_window'):
            guarded.step(previous,self.q,previous,np.zeros(14),.02)

    def test_protective_hold_cannot_be_enabled_for_higher_speed(self):
        with self.assertRaisesRegex(ValueError,'low_speed'):
            JointSafetyFilter(self.model,boundary_hold=True)

    def test_joint_limit_boundary_cannot_silently_override_required_braking(self):
        previous=self.q.copy();previous[6]=self.model.limits[6,1]
        old_velocity=np.zeros(14);old_velocity[6]=.08
        with self.assertRaisesRegex(SafetyError,'acceleration_and_position_bounds_infeasible'):
            self.filter.step(previous,previous,previous,old_velocity,.02)

    def test_cartesian_reduction_preserves_acceleration_during_tracking(self):
        previous=self.q.copy();velocity=np.zeros(14)
        for _ in range(35):
            proposed=previous.copy();proposed[:7]+=.2
            target,next_velocity=self.filter.step(proposed,previous,previous,velocity,.02)
            self.assertLessEqual(np.max(abs(next_velocity-velocity)),.5*.02+1e-9)
            self.assertLessEqual(np.max(abs(next_velocity)),.24+1e-9)
            before=self.model.arms['left'].fk(previous[:7]);after=self.model.arms['left'].fk(target[:7])
            distance=np.linalg.norm(after[:3,3]-before[:3,3])
            angle=np.arccos(np.clip((np.trace(after[:3,:3]@before[:3,:3].T)-1)/2,-1,1))
            self.assertLessEqual(distance,.04*.02+1e-9)
            self.assertLessEqual(angle,.15*.02+1e-9)
            previous=target;velocity=next_velocity

    def test_impossible_cartesian_braking_rejects_without_violating_acceleration(self):
        # A prior velocity can be incompatible with a newly tightened active
        # Cartesian envelope. No zero/over-decelerated command may be emitted.
        strict=JointSafetyFilter(self.model,SafetyLimits(rotation_speed=.01))
        old_velocity=np.zeros(14);old_velocity[6]=.12
        with self.assertRaisesRegex(SafetyError,'acceleration_and_cartesian_bounds_infeasible'):
            strict.step(self.q,self.q,self.q,old_velocity,.02)
