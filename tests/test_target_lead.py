"""Mapper keeps targets near the measured TCP and inside the workspace sphere.

Offline only. A slow arm must not let the hand target run away, and a hand
moving beyond the clutch workspace holds the target instead of ending the clutch.
"""
import math
import unittest
import numpy as np
from tg3_pico_dualarm.mapping import Calibration,RelativeMapper,rotation
from tg3_pico_dualarm.tracking import Pose
from dataclasses import replace
from test_mapping import frame


class TargetLeadTests(unittest.TestCase):
    def setUp(self):
        self.calibration=Calibration.from_head(frame(0).head)
        self.mapper=RelativeMapper(self.calibration)
        self.robot=dict(left=np.eye(4),right=np.eye(4))
        self.send(frame(0));self.assertTrue(self.send(frame(1,1.)).active['right'])
        self.seq=2

    def send(self,f):return self.mapper.update(f,self.robot,f.received_at)

    def move(self,forward):
        # Small steps: stays under the 8 cm / 1 m/s controller jump guard.
        result=self.send(frame(self.seq,1.,(0.,1.,-forward)));self.seq+=1
        return result

    def test_target_leads_still_robot_by_at_most_limit(self):
        for step in range(1,11):result=self.move(.01*step)
        self.assertTrue(result.active['right']);self.assertEqual(result.reasons['right'],'active')
        np.testing.assert_allclose(result.targets['right'][:3,3],[.03,0,0],atol=1e-12)
        self.assertIn('target_lead_limited',result.limits['right'])

    def test_target_is_unchanged_once_robot_catches_up(self):
        for step in range(1,6):result=self.move(.01*step)
        self.robot['right']=np.eye(4);self.robot['right'][:3,3]=[.04,0,0]
        result=self.move(.05)
        np.testing.assert_allclose(result.targets['right'][:3,3],[.05,0,0],atol=1e-12)
        self.assertEqual(result.limits['right'],())

    def test_hand_beyond_workspace_holds_target_on_sphere_and_resumes_without_regrip(self):
        for step in range(1,31):
            result=self.move(.01*step)
            self.robot['right']=result.targets['right'].copy()  # an arm that keeps up
        self.assertTrue(result.active['right'])
        self.assertAlmostEqual(float(np.linalg.norm(result.targets['right'][:3,3])),.2,places=9)
        self.assertIn('workspace_clamped',result.limits['right'])
        for step in range(29,9,-1):result=self.move(.01*step);self.robot['right']=result.targets['right'].copy()
        self.assertTrue(result.active['right'])
        np.testing.assert_allclose(result.targets['right'][:3,3],[.10,0,0],atol=1e-12)
        self.assertEqual(result.limits['right'],())

    def test_rotation_lead_is_limited_toward_hand_orientation(self):
        q=(0.,0.,math.sin(.25),math.cos(.25))  # 0.5 rad about the controller axis
        f=frame(self.seq,1.);f=replace(f,right=replace(f.right,pose=Pose((0.,1.,0.),q)))
        target=self.send(f).targets['right'][:3,:3]
        angle=math.acos(float(np.clip((np.trace(target)-1)/2,-1,1)))
        self.assertAlmostEqual(angle,.15,places=9)
        full=self.calibration.xr_to_robot@rotation(q)@self.calibration.xr_to_robot.T
        axis=lambda r:np.array([r[2,1]-r[1,2],r[0,2]-r[2,0],r[1,0]-r[0,1]])
        np.testing.assert_allclose(axis(target)/np.linalg.norm(axis(target)),axis(full)/np.linalg.norm(axis(full)),atol=1e-9)

    def test_invalid_lead_limits_rejected(self):
        for kwargs in (dict(max_target_lead=0.),dict(max_target_lead=.3),dict(max_target_lead_rotation=float('nan')),
                       dict(max_target_lead=True)):
            with self.subTest(**kwargs),self.assertRaises(ValueError):RelativeMapper(self.calibration,**kwargs)


if __name__=='__main__':unittest.main()
