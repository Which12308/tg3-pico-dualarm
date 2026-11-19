from dataclasses import replace
import math
import unittest
import numpy as np
from tg3_pico_dualarm.mapping import Calibration,RelativeMapper,identity_tools,rotation
from tg3_pico_dualarm.tracking import Pose
from test_mapping import frame


class ToolCalibrationTests(unittest.TestCase):
    def test_measured_tool_transforms_preserve_clutch_without_a_jump(self):
        controller=identity_tools();robot=identity_tools()
        controller['right'][:3,3]=[0.,.02,0.]
        robot['right'][:3,3]=[.1,0.,0.]
        calibration=Calibration(np.eye(3),translation_scale=1.,controller_to_hand=controller,
            tcp_to_hand=robot,tool_offsets_calibrated=True)
        mapper=RelativeMapper(calibration)
        poses=dict(left=np.eye(4),right=np.eye(4))
        mapper.update(frame(0),poses,1.)
        result=mapper.update(frame(1,1.),poses,1.02)
        np.testing.assert_allclose(result.targets['right'],np.eye(4),atol=1e-12)
        self.assertTrue(calibration.as_dict()['tool_offset_calibrated'])
        robot['right'][0,3]=5
        self.assertAlmostEqual(calibration.tcp_to_hand['right'][0,3],.1)

    def test_rotation_moves_the_configured_grasp_pivot_and_returns_tcp_target(self):
        controller=identity_tools();robot=identity_tools()
        controller['right'][:3,3]=[0.,.02,0.]
        robot['right'][:3,3]=[.1,0.,0.]
        c=Calibration(np.eye(3),translation_scale=1.,controller_to_hand=controller,tcp_to_hand=robot)
        mapper=RelativeMapper(c);poses=dict(left=np.eye(4),right=np.eye(4))
        mapper.update(frame(0),poses,1.);mapper.update(frame(1,1.),poses,1.02)
        q=(0.,0.,math.sin(.05),math.cos(.05))
        f=frame(2,1.);f=replace(f,right=replace(f.right,pose=Pose((0.,1.,0.),q)))
        result=mapper.update(f,poses,1.04)
        palm=result.targets['right']@c.tcp_to_hand['right']
        expected_delta=rotation(q)@np.array([0.,.02,0.])-np.array([0.,.02,0.])
        np.testing.assert_allclose(palm[:3,3],np.array([.1,0,0])+expected_delta,atol=1e-12)
        np.testing.assert_allclose(palm[:3,:3],rotation(q),atol=1e-12)

    def test_invalid_or_one_sided_tool_calibration_is_rejected(self):
        with self.assertRaises(ValueError):Calibration(np.eye(3),tcp_to_hand={'right':np.eye(4)})
        invalid=identity_tools();invalid['left'][0,0]=-1
        with self.assertRaises(ValueError):Calibration(np.eye(3),controller_to_hand=invalid)
        invalid=identity_tools();invalid['left'][0,3]=float('nan')
        with self.assertRaises(ValueError):Calibration(np.eye(3),tcp_to_hand=invalid)
