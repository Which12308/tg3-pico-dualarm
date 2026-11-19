"""Native tests execute real Placo when available; no fake solve substitutes."""
import importlib.util
from unittest.mock import Mock
import unittest

import numpy as np

from tg3_pico_dualarm.kinematics import (Kinematics, KinematicsError, checked_pose,
    REGULARIZATION_MIN, REGULARIZATION_MAX, singularity_regularization_weight)
from tg3_pico_dualarm.model import DualArmModel, default_q, rotation


class ValidationTests(unittest.TestCase):
    def test_adaptive_regularization_is_finite_bounded_and_increases_toward_singular(self):
        values = [singularity_regularization_weight(v) for v in (.1, .01, .005, .001, 0.)]
        self.assertEqual(values[:2], [REGULARIZATION_MIN, REGULARIZATION_MIN])
        self.assertEqual(values[-1], REGULARIZATION_MAX)
        self.assertEqual(values, sorted(values))
        for bad in (-1., np.nan, np.inf):
            with self.assertRaises(KinematicsError): singularity_regularization_weight(bad)

    def test_actual_near_singular_arm_gets_damping_without_slowing_other_arm(self):
        engine = Kinematics.__new__(Kinematics)
        engine.model = DualArmModel()
        engine.modes = {'left': 'pose', 'right': 'pose'}
        engine.position_weight = 1.; engine.orientation_weight = 1.
        engine.regularization_task = Mock()
        q = default_q()
        q[:7] = 0.; q[3] = -.177
        diagnostics = engine._regularization_policy(q, {'left': True, 'right': True})
        self.assertLess(diagnostics['left']['smallest_weighted_singular_value'], 1e-3)
        self.assertGreater(diagnostics['left']['delta_q_l2_weight'], 5e-4)
        self.assertEqual(diagnostics['right']['delta_q_l2_weight'], REGULARIZATION_MIN)
        weights = {call.args[0]: call.args[1] for call in engine.regularization_task.set_joint_weight.call_args_list}
        for name in engine.model.arms['left'].joint_names:
            self.assertEqual(weights[name], diagnostics['left']['delta_q_l2_weight'])
        for name in engine.model.arms['right'].joint_names:
            self.assertEqual(weights[name], REGULARIZATION_MIN)

    def test_unlocked_torso_is_explicitly_unsupported(self):
        with self.assertRaisesRegex(KinematicsError, 'unlocked_torso_waist_not_supported'):
            Kinematics(torso_waist_locked=False)

    def test_rejects_reflections_and_nonrigid_targets(self):
        for bad in (np.diag([-1., 1., 1., 1.]), np.ones((4, 4)), np.full((4, 4), np.nan)):
            with self.assertRaises(KinematicsError): checked_pose(bad)

    def test_released_does_not_call_solver_and_preserves_applied_target(self):
        # This tests gate behavior only. Native IK tests below never use mocks.
        engine = Kinematics.__new__(Kinematics)
        engine.model = DualArmModel()
        engine._anchors = {'left': np.ones(7), 'right': np.ones(7)}
        engine.solver = Mock()
        q = default_q()
        out = engine.solve(q, {}, {'left': False, 'right': False}, .02)
        np.testing.assert_array_equal(out, q)
        self.assertIsNot(out, q)
        engine.solver.solve.assert_not_called()
        self.assertEqual(engine._anchors, {'left': None, 'right': None})

    def test_invalid_seed_target_and_tick_are_rejected_before_native_solve(self):
        engine = Kinematics.__new__(Kinematics)
        engine.model = DualArmModel()
        engine.solver = Mock()
        q = default_q(); active = {'left': True, 'right': False}
        for values, targets, dt in ((np.full(14, np.nan), {'left': np.eye(4)}, .02),
                                   (q, {}, .02), (q, {'left': np.eye(4)}, 1.)):
            with self.assertRaises(KinematicsError): engine.solve(values, targets, active, dt)
        engine.solver.solve.assert_not_called()

    def test_posture_latch_copies_measured_joints_and_refuses_limits(self):
        engine = Kinematics.__new__(Kinematics)
        engine.model = DualArmModel()
        engine._anchors = {'left': None, 'right': None}
        q = default_q()[:7]
        engine.latch_posture('left', q)
        reference = engine._anchors['left'].copy()
        q[0] += .1
        np.testing.assert_array_equal(engine._anchors['left'], reference)
        with self.assertRaises(KinematicsError): engine.latch_posture('left', np.full(7, 100.))

    def test_certainly_unreachable_goal_rejected_before_solver(self):
        engine = Kinematics.__new__(Kinematics)
        engine.model = DualArmModel()
        engine.solver = Mock()
        pose = np.eye(4); pose[:3, 3] = [100., 100., 100.]
        with self.assertRaisesRegex(KinematicsError, 'target_outside_geometric_reach:left'):
            engine.solve(default_q(), {'left': pose}, {'left': True, 'right': False}, .02)
        engine.solver.solve.assert_not_called()
        self.assertEqual(engine.last_diagnostics['reachability'], 'unreachable')

    def test_residual_progress_is_not_confused_with_velocity_lag(self):
        engine = Kinematics.__new__(Kinematics)
        engine.model = DualArmModel()
        engine.modes = {'left': 'pose', 'right': 'pose'}
        engine.position_weight = 1.; engine.orientation_weight = .5
        engine._progress = {'left': None, 'right': None}
        goal = engine.model.fk(default_q())['left']
        jacobian = np.eye(6, 7)
        for tick in range(150):
            pose = goal.copy(); pose[0, 3] -= .1-.0002*tick
            report = engine._residual_status('left', goal, pose, jacobian, .02)
            self.assertEqual(report['status'], 'tracking')
            self.assertEqual(report['reachability'], 'not_proven')
        engine._progress = {'left': None, 'right': None}
        for _ in range(110):
            pose = goal.copy(); pose[0, 3] -= .05
            report = engine._residual_status('left', goal, pose, jacobian, .02)
        self.assertEqual(report['status'], 'stalled')
        moved_goal = goal.copy(); moved_goal[1, 3] += .01
        self.assertEqual(engine._residual_status('left', moved_goal, pose, jacobian, .02)['status'], 'tracking')

    def test_position_mode_residual_excludes_rotation_and_weights_translation(self):
        engine = Kinematics.__new__(Kinematics)
        engine.modes = {'left': 'position', 'right': 'position'}
        engine.position_weight = 4.; engine.orientation_weight = 100.
        engine._progress = {'left': None, 'right': None}
        goal = np.eye(4)
        pose = goal.copy(); pose[0, 3] = .01; pose[:3, :3] = rotation([0, 0, 1], 1.)
        report = engine._residual_status('left', goal, pose, np.eye(6, 7), .02)
        self.assertAlmostEqual(report['weighted_residual'], .02)
        pose[0, 3] = 0.
        self.assertEqual(engine._residual_status('left', goal, pose, np.eye(6, 7), .02)['status'], 'converged')


NATIVE_AVAILABLE = importlib.util.find_spec('placo') is not None


@unittest.skipUnless(NATIVE_AVAILABLE, 'Actual native Placo is unavailable; not replaced by fake IK')
class NativePlacoTests(unittest.TestCase):
    def test_task_weights_and_locked_torso_are_actual_solver_policy(self):
        engine = Kinematics(position_weight=2., orientation_weight=.3, torso_waist_locked=True,
                            solve_budget_s=1.)
        q = default_q()
        q = engine.solve(q, engine.model.fk(q), {'left': True, 'right': True}, .02)
        self.assertTrue(engine.torso_waist_locked)
        self.assertEqual(engine.position_weight, 2.)
        self.assertEqual(engine.orientation_weight, .3)
        self.assertEqual(engine.last_diagnostics['position_weight'], 2.)
        self.assertEqual(engine.last_diagnostics['orientation_weight'], .3)
        self.assertEqual(len(q), 14)

    def test_native_fk_matches_both_supplied_chains(self):
        engine = Kinematics(solve_budget_s=1.)
        q = default_q(); engine._seed(q)
        expected = engine.model.fk(q)
        for side in ('left', 'right'):
            np.testing.assert_allclose(engine.robot.get_T_world_frame(side+'_tcp_link'), expected[side], atol=1e-8)

    def test_stationary_clutches_do_not_drive_redundant_joints(self):
        engine = Kinematics(solve_budget_s=1.)
        for elbow in (-.4, -.8, -1.2):
            q = default_q(); q[[3, 10]] = elbow
            original = q.copy(); goals = engine.model.fk(q)
            engine.reset()
            for i, side in enumerate(('left', 'right')):
                engine.latch_posture(side, q[i*7:(i+1)*7])
            for _ in range(80):
                q = engine.solve(q, goals, {'left': True, 'right': True}, .02)
            np.testing.assert_allclose(q, original, atol=1e-6)

    def test_both_arms_track_small_independent_pose_targets(self):
        engine = Kinematics(solve_budget_s=1.)
        q = default_q(); goals = engine.model.fk(q)
        goals['left'][0, 3] += .005
        goals['right'][1, 3] -= .005
        for _ in range(140):
            prior = q.copy()
            q = engine.solve(q, goals, {'left': True, 'right': True}, .02)
            self.assertLessEqual(np.max(np.abs(q-prior)), .24*.02+1e-7)
        final = engine.model.fk(q)
        for side in ('left', 'right'):
            self.assertLess(np.linalg.norm(final[side][:3, 3]-goals[side][:3, 3]), .002)

    def test_position_mode_ignores_rotation_target_without_moving_other_arm(self):
        engine = Kinematics(mode='position', solve_budget_s=1.)
        q = default_q(); original = q.copy(); goals = engine.model.fk(q)
        goals['left'][:3, :3] = rotation([0, 0, 1], .4)@goals['left'][:3, :3]
        for _ in range(40):
            q = engine.solve(q, goals, {'left': True, 'right': False}, .02)
        np.testing.assert_allclose(q, original, atol=1e-6)
        np.testing.assert_array_equal(q[7:], original[7:])

    def test_native_solver_failure_is_never_accepted_as_previous_solution(self):
        engine = Kinematics(solve_budget_s=1.)
        native = engine.solver
        class Failure:
            def __getattr__(self, key): return getattr(native, key)
            def solve(self, _apply): raise RuntimeError('injected native failure')
        engine.solver = Failure()
        q = default_q()
        with self.assertRaisesRegex(KinematicsError, 'ik_failed'):
            engine.solve(q, engine.model.fk(q), {'left': True, 'right': True}, .02)

    def test_actual_near_singular_pose_remains_finite_bounded_and_damped(self):
        engine = Kinematics(solve_budget_s=1.)
        q = np.zeros(14)
        # The verified model has elbow offsets: q=0 is not singular. This
        # within-limits posture has weighted sigma_min approximately 2.2e-5.
        q[[3, 10]] = -.177
        goals = engine.model.fk(q)
        # A small inward displacement is inside the reach bound even when the
        # straight arm cannot follow all Cartesian directions in one tick.
        for side in ('left', 'right'):
            radial = goals[side][:3, 3]-engine.model.arms[side].shoulder_origin
            goals[side][:3, 3] -= radial/np.linalg.norm(radial)*.002
        for tick in range(20):
            old = q.copy()
            q = engine.solve(q, goals, {'left': True, 'right': True}, .02)
            self.assertTrue(np.all(np.isfinite(q)))
            self.assertLessEqual(np.max(np.abs(q-old)), .24*.02+1e-7)
            self.assertTrue(np.all(q >= engine.model.limits[:, 0]-1e-8))
            self.assertTrue(np.all(q <= engine.model.limits[:, 1]+1e-8))
            for side in ('left', 'right'):
                self.assertIn('smallest_weighted_singular_value', engine.last_diagnostics['target_errors'][side])
                if tick == 0:
                    policy = engine.last_diagnostics['singularity_policy']['arms'][side]
                    self.assertLess(policy['smallest_weighted_singular_value'], 1e-3)
                    self.assertGreater(policy['delta_q_l2_weight'], 5e-4)

    def test_unreachable_far_goal_never_calls_native_solver(self):
        engine = Kinematics(solve_budget_s=1.)
        q = default_q(); goals = engine.model.fk(q)
        goals['right'][0, 3] += 10.
        with self.assertRaisesRegex(KinematicsError, 'target_outside_geometric_reach:right'):
            engine.solve(q, goals, {'left': True, 'right': True}, .02)
        self.assertFalse(engine.last_diagnostics['solve_called'])


if __name__ == '__main__':
    unittest.main()
