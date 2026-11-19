from dataclasses import replace
import math
import unittest
import numpy as np
from tg3_pico_dualarm.mapping import Calibration,RelativeMapper,rotation
from tg3_pico_dualarm.tracking import Pose,ControllerState,TrackingFrame


def frame(seq,grip=0.,position=(0.,1.,0.),*,left_grip=0.,head=None):
    controller=ControllerState(Pose(position,(0.,0.,0.,1.)),True,True,grip,0.,(),False)
    left=replace(controller,grip=left_grip)
    return TrackingFrame('test',seq,1.+seq*.02,1.+seq*.02,
        head or Pose((0.,1.6,0.),(0.,0.,0.,1.)),left,controller)


class MappingTests(unittest.TestCase):
    def setUp(self):
        self.calibration=Calibration.from_head(frame(0).head)
        self.mapper=RelativeMapper(self.calibration)
        self.robot=dict(left=np.eye(4),right=np.eye(4))

    def send(self,f):return self.mapper.update(f,self.robot,f.received_at)

    def test_independent_hands_and_calibrated_forward(self):
        self.send(frame(0));self.send(frame(1,1.))
        result=self.send(frame(2,1.,(0.,1.,-.01)))
        self.assertTrue(result.active['right']);self.assertFalse(result.active['left'])
        np.testing.assert_allclose(result.targets['right'][:3,3],[.01,0,0],atol=1e-12)

    def test_analog_grip_ramp_preserves_fresh_release_without_early_motion(self):
        self.send(frame(0))
        for seq,value in enumerate((.2,.5,.85),1):
            result=self.send(frame(seq,value))
            self.assertFalse(result.active['right'])
        result=self.send(frame(4,1.))
        self.assertTrue(result.active['right'],result.reasons)
        self.assertTrue(result.relatch['right'])
        np.testing.assert_allclose(result.targets['right'],self.robot['right'])
        moved=self.send(frame(5,1.,(0.,1.,-.01)))
        np.testing.assert_allclose(moved.targets['right'][:3,3],[.01,0,0],atol=1e-12)

    def test_partial_release_of_active_grip_cannot_reengage_without_neutral(self):
        self.send(frame(0));self.send(frame(1,1.))
        self.assertFalse(self.send(frame(2,.5)).active['right'])
        self.assertFalse(self.send(frame(3,1.)).active['right'])
        self.send(frame(4,0.));self.send(frame(5,.5))
        self.assertTrue(self.send(frame(6,1.)).active['right'])

    def test_rising_grip_without_prior_neutral_cannot_engage(self):
        for seq,value in enumerate((.2,.5,.85,1.)):
            self.assertFalse(self.send(frame(seq,value)).active['right'])

    def test_head_heading_calibration_rotates_forward_into_robot_forward(self):
        head=Pose((0.,1.6,0.),(0.,math.sin(math.pi/4),0.,math.cos(math.pi/4)))
        c=Calibration.from_head(head)
        np.testing.assert_allclose(c.xr_to_robot@[-1.,0.,0.],[1.,0.,0.],atol=1e-12)

    def test_regrasp_reanchors_without_target_jump(self):
        self.send(frame(0));self.send(frame(1,1.))
        self.send(frame(2,1.,(0.,1.,-.01)))
        self.send(frame(3,0.,(0.,1.,-.2)))
        self.robot['right'][:3,3]=[.12,.01,.15]
        result=self.send(frame(4,1.,(0.,1.,-.2)))
        np.testing.assert_allclose(result.targets['right'],self.robot['right'])

    def test_reconnect_held_cannot_start_and_duplicate_cannot_release(self):
        self.assertFalse(self.send(frame(0,1.)).active['right'])
        self.send(frame(1));self.send(frame(2,1.))
        expired=frame(3,1.)
        self.mapper.update(expired,self.robot,expired.received_at+1)
        self.assertFalse(self.send(frame(3)).active['right'])
        self.assertFalse(self.send(frame(4,1.)).active['right'])

    def test_jump_requires_release_and_does_not_affect_other_hand(self):
        self.send(frame(0));self.send(frame(1,1.,left_grip=1.))
        f=frame(2,1.,(0.,1.,-.3),left_grip=1.)
        f=replace(f,left=replace(f.left,pose=Pose((0.,1.,0.),(0.,0.,0.,1.))))
        result=self.send(f)
        self.assertFalse(result.active['right']);self.assertTrue(result.active['left'])
        self.assertEqual(result.reasons['right'],'controller_jump')

    def test_position_only_ignores_wrist_orientation(self):
        self.mapper=RelativeMapper(self.calibration,'position')
        self.send(frame(0));self.send(frame(1,1.))
        f=frame(2,1.)
        f=replace(f,right=replace(f.right,pose=Pose((0.,1.,0.),(0.,0.,math.sin(.1),math.cos(.1)))))
        np.testing.assert_allclose(self.send(f).targets['right'][:3,:3],np.eye(3))

    def test_pose_maps_all_rotation_axes(self):
        for axis in range(3):
            self.mapper=RelativeMapper(self.calibration)
            self.send(frame(0));self.send(frame(1,1.))
            q=np.zeros(4);q[axis]=math.sin(.05);q[3]=math.cos(.05)
            f=frame(2,1.);f=replace(f,right=replace(f.right,pose=Pose((0.,1.,0.),tuple(q))))
            expected=self.calibration.xr_to_robot@rotation(q)@self.calibration.xr_to_robot.T
            np.testing.assert_allclose(self.send(f).targets['right'][:3,:3],expected,atol=1e-12)

    def test_direct_nonfinite_grip_cannot_bypass_wire_validation(self):
        self.send(frame(0));f=frame(1)
        f=replace(f,right=replace(f.right,grip=float('nan')))
        result=self.send(f)
        self.assertFalse(result.active['right'])
        self.assertEqual(result.reasons['right'],'invalid_tracking_frame')
