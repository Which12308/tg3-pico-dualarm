import hashlib
from pathlib import Path
import tempfile
import unittest
import xml.etree.ElementTree as ET

import numpy as np

from tg3_pico_dualarm.model import (ArmModel, BASE_FRAME, DualArmModel, JOINT_NAMES,
                                    MODEL_DIR, MODEL_SHA256, MOTOR_IDS, default_q)


class ModelTests(unittest.TestCase):
    def test_supplied_geometry_and_actual_motor_order(self):
        self.assertEqual(hashlib.sha256((MODEL_DIR/'tiangong3.urdf').read_bytes()).hexdigest(), MODEL_SHA256)
        model = DualArmModel()
        self.assertEqual(model.joint_names, JOINT_NAMES)
        self.assertEqual(model.motor_ids, tuple(range(11, 18))+tuple(range(21, 28)))
        self.assertEqual(model.motor_ids, MOTOR_IDS)
        self.assertEqual(model.arms['left'].joint_names[1], 'shoulder_roll_l_joint')
        self.assertGreater(model.arms['left'].limits[1, 1], 3.)
        self.assertLess(model.arms['right'].limits[1, 0], -3.)
        for side in ('left', 'right'):
            self.assertAlmostEqual(model.arms[side].limits[3, 0], -2.41)

    def test_both_geometric_jacobians_match_finite_difference(self):
        for index, side in enumerate(('left', 'right')):
            arm = ArmModel(side)
            q = default_q()[index*7:(index+1)*7]
            transform, jacobian = arm.pose_and_jacobian(q)
            for joint in range(7):
                epsilon = 1e-7
                shifted = q.copy(); shifted[joint] += epsilon
                after = arm.fk(shifted)
                linear = (after[:3, 3]-transform[:3, 3])/epsilon
                skew = ((after[:3, :3]-transform[:3, :3])/epsilon)@transform[:3, :3].T
                angular = np.array([skew[2, 1], skew[0, 2], skew[1, 0]])
                np.testing.assert_allclose(linear, jacobian[:3, joint], atol=5e-8)
                np.testing.assert_allclose(angular, jacobian[3:, joint], atol=5e-8)

    def test_extraction_is_only_fixed_waist_and_two_actual_arm_chains(self):
        model = DualArmModel()
        root = ET.parse(model.kinematic_urdf()).getroot()
        joints = root.findall('joint')
        self.assertEqual(len(joints), 16)
        self.assertEqual(tuple(j.get('name') for j in joints if j.get('type') == 'revolute'), JOINT_NAMES)
        self.assertEqual(len(root.findall('link')), 17)
        self.assertFalse(root.findall('.//mesh'))
        self.assertFalse(root.findall('.//collision'))
        parents = {j.find('child').get('link'): j.find('parent').get('link') for j in joints}
        for side in ('left', 'right'):
            link = side+'_tcp_link'; count = 0
            while link != BASE_FRAME:
                link = parents[link]; count += 1
            self.assertEqual(count, 8)

    def test_link_positions_end_at_the_same_tcp(self):
        model = DualArmModel()
        q = default_q()
        for index, side in enumerate(('left', 'right')):
            arm = model.arms[side]
            value = q[index*7:(index+1)*7]
            np.testing.assert_allclose(arm.links(value)[side+'_tcp_link'], arm.fk(value)[:3, 3])

    def test_geometric_upper_bound_comes_from_chain_and_contains_valid_fk(self):
        for index, side in enumerate(('left', 'right')):
            arm = ArmModel(side)
            expected = sum(np.linalg.norm(j.origin[:3, 3]) for j in arm.joints[1:])+np.linalg.norm(arm.tip[:3, 3])
            self.assertAlmostEqual(arm.maximum_reach, expected)
            for elbow in (-.4, -.8, -1.8):
                q = default_q()[index*7:(index+1)*7]; q[3] = elbow
                self.assertTrue(arm.within_geometric_reach(arm.fk(q)[:3, 3]))
            self.assertFalse(arm.within_geometric_reach(arm.shoulder_origin+[arm.maximum_reach+.01, 0, 0]))

    def test_rejects_changed_geometry_and_nonfinite_fk(self):
        with tempfile.TemporaryDirectory() as folder:
            filename = Path(folder)/'tiangong3.urdf'
            filename.write_bytes((MODEL_DIR/'tiangong3.urdf').read_bytes()+b'\n')
            with self.assertRaisesRegex(ValueError, 'differs'):
                ArmModel('left', filename)
        for side in ('left', 'right'):
            with self.assertRaises(ValueError):
                ArmModel(side).fk([float('nan')]*7)


if __name__ == '__main__':
    unittest.main()
