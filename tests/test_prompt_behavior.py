"""Acceptance behavior from the user's complete two-controller prompt."""
import ast
from dataclasses import replace
from pathlib import Path
import unittest
import numpy as np
from tg3_pico_dualarm.mapping import Calibration,RelativeMapper
from tg3_pico_dualarm.tracking import Pose
from test_mapping import frame


class PromptBehaviorTests(unittest.TestCase):
    def test_both_controllers_have_independent_targets_and_one_side_can_release(self):
        mapper=RelativeMapper(Calibration(np.eye(3),translation_scale=1.))
        poses={'left':np.eye(4),'right':np.eye(4)}
        mapper.update(frame(0),poses,1.)
        mapper.update(frame(1,1.,left_grip=1.),poses,1.02)
        moving=frame(2,1.,(.01,1.,0.),left_grip=1.)
        moving=replace(moving,left=replace(moving.left,pose=Pose((0.,1.01,0.),(0.,0.,0.,1.))))
        result=mapper.update(moving,poses,1.04)
        np.testing.assert_allclose(result.targets['right'][:3,3],[.01,0,0])
        np.testing.assert_allclose(result.targets['left'][:3,3],[0,.01,0])
        release=replace(moving,sequence=3,source_time_s=1.06,received_at=1.06,left=replace(moving.left,grip=0.))
        result=mapper.update(release,poses,1.06)
        self.assertFalse(result.active['left']);self.assertTrue(result.active['right'])

    def test_ab_buttons_do_not_load_old_special_arm_or_bottle_behavior(self):
        mapper=RelativeMapper(Calibration(np.eye(3)))
        poses={'left':np.eye(4),'right':np.eye(4)}
        mapper.update(frame(0),poses,1.)
        anchor=mapper.update(frame(1,1.),poses,1.02)
        pressed=frame(2,1.)
        pressed=replace(pressed,right=replace(pressed.right,buttons=('A','B')))
        result=mapper.update(pressed,poses,1.04)
        np.testing.assert_allclose(result.targets['right'],anchor.targets['right'])

    def test_runtime_imports_are_independent_of_previous_remote_program(self):
        root=Path(__file__).resolve().parents[1]/'src/tg3_pico_dualarm'
        forbidden={'right_arm_hand_remote','trial_switch','run_pnp','bottle_sequence','grasp_profile','joystick_bridge'}
        for path in root.rglob('*.py'):
            tree=ast.parse(path.read_text(encoding='utf-8'))
            for node in ast.walk(tree):
                imports=[]
                if isinstance(node,ast.Import):imports=[a.name for a in node.names]
                elif isinstance(node,ast.ImportFrom):imports=[node.module or '']
                for name in imports:
                    self.assertFalse(set(name.split('.'))&forbidden,(path,name))
