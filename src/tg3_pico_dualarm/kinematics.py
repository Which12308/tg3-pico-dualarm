"""Native Placo dual-arm IK. There is deliberately no actuator or ROS output."""
from __future__ import annotations

from . import safety as motion_policy
import time
import numpy as np

from .model import DualArmModel, JOINT_NAMES, SIDES, default_q, finite_vector

REGULARIZATION_MIN = 1e-6
REGULARIZATION_MAX = 1e-3
SINGULARITY_THRESHOLD = .01
CONDITIONING_ORIENTATION_LENGTH_M = .2


class KinematicsError(RuntimeError):
    def __init__(self, reason):
        self.reason = reason
        super().__init__(reason)


def checked_pose(value):
    pose = np.asarray(value, dtype=float)
    if pose.shape != (4, 4) or not np.all(np.isfinite(pose)):
        raise KinematicsError('invalid_target_pose')
    if (not np.allclose(pose[3], [0, 0, 0, 1], atol=1e-8)
            or not np.allclose(pose[:3, :3].T@pose[:3, :3], np.eye(3), atol=1e-6)
            or not np.isclose(np.linalg.det(pose[:3, :3]), 1., atol=1e-6)):
        raise KinematicsError('invalid_target_rotation')
    return pose.copy()


def rotation_error(actual, desired):
    return float(np.arccos(np.clip((np.trace(actual@desired.T)-1)/2, -1, 1)))


def singularity_regularization_weight(smallest_singular_value):
    """Smooth bounded delta-q cost; this does not attract a new robot pose."""
    value = float(smallest_singular_value)
    if not np.isfinite(value) or value < 0:
        raise KinematicsError('invalid_conditioning_metric')
    fraction = max(0., 1.-value/SINGULARITY_THRESHOLD)
    return REGULARIZATION_MIN+(REGULARIZATION_MAX-REGULARIZATION_MIN)*fraction**2


class Kinematics:
    """Solve from the last applied target, with an explicitly latched posture.

    The runtime owns fresh measured feedback, acceleration/lead/workspace guards,
    and command arbitration. Calling this class cannot send robot commands.
    Native Placo is mandatory: there is no simulated/fake solver fallback.
    """
    def __init__(self, modelroot=None, mode='pose', velocity_limit=.24,
                 posture_weight=1e-4, solve_budget_s=.05,
                 position_weight=1., orientation_weight=1., torso_waist_locked=True,joint_limits=None):
        if torso_waist_locked is not True:
            raise KinematicsError('unlocked_torso_waist_not_supported')
        self.model = DualArmModel(modelroot)
        if joint_limits is not None:self.model.restrict_joint_limits(*joint_limits)
        initial_q=np.clip(default_q(),self.model.limits[:,0],self.model.limits[:,1])
        self.modes = {side: mode for side in SIDES} if isinstance(mode, str) else dict(mode)
        if set(self.modes) != set(SIDES) or any(m not in ('pose', 'position') for m in self.modes.values()):
            raise ValueError('mode must be pose or position for each arm')
        if not np.isfinite(velocity_limit) or velocity_limit<=0 or (not motion_policy.LIMITS_MANAGED_EXTERNALLY and velocity_limit>.30):
            raise ValueError('invalid velocity limit')
        if not np.isfinite(posture_weight) or not 0 < posture_weight <= .01:
            raise ValueError('invalid posture weight')
        if not np.isfinite(solve_budget_s) or solve_budget_s<=0 or (not motion_policy.LIMITS_MANAGED_EXTERNALLY and solve_budget_s>1.):
            raise ValueError('invalid solve budget')
        for name, weight in (('position', position_weight), ('orientation', orientation_weight)):
            if not np.isfinite(weight) or not 0 < weight <= 1e4:
                raise ValueError('invalid '+name+' weight')
        self.velocity_limit = float(velocity_limit)
        self.solve_budget_s = float(solve_budget_s)
        self.position_weight = float(position_weight)
        self.orientation_weight = float(orientation_weight)
        self.torso_waist_locked = True
        self._anchors = {side: None for side in SIDES}
        self._progress = {side: None for side in SIDES}
        self.last_diagnostics = dict(backend='placo', initialized=False)
        try:
            import placo
        except (ImportError, OSError) as exc:
            raise KinematicsError('native_placo_unavailable: '+str(exc)) from exc
        try:
            self.robot = placo.RobotWrapper(str(self.model.kinematic_urdf(velocity_limit)))
            self.solver = placo.KinematicsSolver(self.robot)
            self.solver.mask_fbase(True)
            self.solver.enable_joint_limits(not motion_policy.LIMITS_MANAGED_EXTERNALLY)
            self.solver.enable_velocity_limits(not motion_policy.LIMITS_MANAGED_EXTERNALLY)
            self._seed(initial_q)
            self.tasks = {}
            for side in SIDES:
                frame = side+'_tcp_link'
                pose = np.array(self.robot.get_T_world_frame(frame), copy=True)
                if self.modes[side] == 'pose':
                    task = self.solver.add_frame_task(frame, pose)
                    task.configure(side+'_hand', 'soft', self.position_weight, self.orientation_weight)
                else:
                    task = self.solver.add_position_task(frame, pose[:3, 3])
                    task.configure(side+'_hand', 'soft', self.position_weight)
                self.tasks[side] = task
            self.posture_task = self.solver.add_joints_task()
            self.posture_task.configure('clutch_posture', 'soft', float(posture_weight))
            self.posture_task.set_joints(dict(zip(JOINT_NAMES, map(float, initial_q))))
            # Placo's public RegularizationTask adds ||W delta_q||² with zero
            # target. Per-joint weights let one singular arm slow down without
            # unnecessarily damping the other arm. No autonomous nullspace
            # attraction or manipulability task is introduced.
            self.regularization_task = self.solver.add_regularization_task(REGULARIZATION_MIN)
        except Exception as exc:
            raise KinematicsError('native_placo_setup_failed: '+str(exc)) from exc
        self.last_diagnostics = dict(backend='placo', initialized=True,
                                     base_frame=self.model.base_frame, modes=self.modes.copy(),
                                     torso_waist_locked=True,
                                     position_weight=self.position_weight,
                                     orientation_weight=self.orientation_weight)

    def _seed(self, q):
        for name, value in zip(JOINT_NAMES, q):
            self.robot.set_joint(name, float(value))
        self.robot.update_kinematics()

    def latch_posture(self, side, q7):
        if side not in SIDES:
            raise ValueError('side must be left or right')
        value = finite_vector(q7, 7)
        limits = self.model.arms[side].limits
        if not motion_policy.LIMITS_MANAGED_EXTERNALLY and (np.any(value < limits[:, 0]) or np.any(value > limits[:, 1])):
            raise KinematicsError('posture_reference_outside_soft_limits')
        self._anchors[side] = value.copy()

    def reset(self, side=None):
        sides = SIDES if side is None else (side,)
        for item in sides:
            if item not in SIDES:
                raise ValueError('side must be left or right')
            self._anchors[item] = None
            if hasattr(self, '_progress'):
                self._progress[item] = None

    def _weighted_singular_values(self, side, jacobian):
        matrix = jacobian.copy()
        matrix[:3] *= np.sqrt(self.position_weight)
        matrix[3:] *= CONDITIONING_ORIENTATION_LENGTH_M*np.sqrt(self.orientation_weight)
        if self.modes[side] == 'position':
            matrix = matrix[:3]
        return np.linalg.svd(matrix, compute_uv=False)

    def _regularization_policy(self, seed, active):
        diagnostics = {}
        for i, side in enumerate(SIDES):
            value = None
            if active[side]:
                _, jacobian = self.model.arms[side].pose_and_jacobian(seed[i*7:(i+1)*7])
                value = float(self._weighted_singular_values(side, jacobian)[-1])
                weight = singularity_regularization_weight(value)
            else:
                weight = REGULARIZATION_MIN
            for name in self.model.arms[side].joint_names:
                self.regularization_task.set_joint_weight(name, weight)
            diagnostics[side] = dict(smallest_weighted_singular_value=value,
                                      delta_q_l2_weight=float(weight),
                                      active=bool(active[side]))
        return diagnostics

    def _residual_status(self, side, goal, pose, jacobian, dt):
        position_error = float(np.linalg.norm(pose[:3, 3]-goal[:3, 3]))
        orientation_error = rotation_error(pose[:3, :3], goal[:3, :3])
        pose_mode = self.modes[side] == 'pose'
        residual = float(np.hypot(np.sqrt(self.position_weight)*position_error,
                                  CONDITIONING_ORIENTATION_LENGTH_M*np.sqrt(self.orientation_weight)*orientation_error
                                  if pose_mode else 0.))
        status = 'converged' if position_error <= .002 and (not pose_mode or orientation_error <= .01) else 'tracking'
        previous = self._progress[side]
        # A moving target or a changed control mode must not inherit a stale
        # lack-of-progress verdict. Velocity-limited progress is normal.
        changed = (previous is None
                   or np.linalg.norm(goal[:3, 3]-previous['goal'][:3, 3]) > 1e-5
                   or (pose_mode and rotation_error(goal[:3, :3], previous['goal'][:3, :3]) > 1e-4))
        if changed or status == 'converged':
            previous = dict(goal=goal.copy(), elapsed=0., residual=residual)
        else:
            previous['elapsed'] += dt
            if previous['elapsed'] >= 2.:
                if residual >= previous['residual']-1e-5:
                    status = 'stalled'
                else:
                    previous = dict(goal=goal.copy(), elapsed=0., residual=residual)
        self._progress[side] = previous
        singular_values = self._weighted_singular_values(side, jacobian)
        return dict(translation_m=position_error, rotation_rad=orientation_error,
                    weighted_residual=residual, status=status,
                    geometric_reach='within_upper_bound', reachability='not_proven',
                    smallest_weighted_singular_value=float(singular_values[-1]),
                    near_singular=bool(singular_values[-1] < SINGULARITY_THRESHOLD))

    def solve(self, measured14, targets, active, dt):
        """Return a proposed 14-vector from seed14 (the last applied target).

        `measured14` retains the integration contract's argument name, but must
        be the runtime's last applied target. Fresh measured joints are provided
        separately through latch_posture when the clutch is newly accepted.
        """
        try:
            q = finite_vector(measured14, 14)
        except (TypeError, ValueError) as exc:
            raise KinematicsError('invalid_seed') from exc
        if not np.isfinite(dt) or dt<=0 or (not motion_policy.LIMITS_MANAGED_EXTERNALLY and dt>.05):
            raise KinematicsError('invalid_tick_interval')
        if set(active) != set(SIDES) or any(not isinstance(active[s], (bool, np.bool_)) for s in SIDES):
            raise KinematicsError('invalid_active_arms')
        if set(targets)-set(SIDES):
            raise KinematicsError('unknown_target_arm')
        if not motion_policy.LIMITS_MANAGED_EXTERNALLY and (np.any(q < self.model.limits[:, 0]) or np.any(q > self.model.limits[:, 1])):
            raise KinematicsError('seed_outside_soft_limits')
        goals = {side: checked_pose(targets[side]) for side in SIDES if active[side] and side in targets}
        if any(active[side] and side not in goals for side in SIDES):
            raise KinematicsError('active_arm_missing_target')
        if not any(active.values()):
            self.reset()
            self.last_diagnostics = dict(backend='placo', solve_called=False, reason='both_released')
            return q.copy()
        for side, goal in goals.items():
            if not motion_policy.LIMITS_MANAGED_EXTERNALLY and not self.model.arms[side].within_geometric_reach(goal[:3, 3]):
                self.last_diagnostics = dict(backend='placo', solve_called=False,
                    reason='target_outside_geometric_reach', arm=side, reachability='unreachable',
                    shoulder_distance_m=float(np.linalg.norm(goal[:3, 3]-self.model.arms[side].shoulder_origin)),
                    maximum_reach_m=self.model.arms[side].maximum_reach)
                raise KinematicsError('target_outside_geometric_reach:'+side)
        self._seed(q)
        reference = q.copy()
        for i, side in enumerate(SIDES):
            arm_slice = slice(i*7, (i+1)*7)
            if not active[side]:
                self._anchors[side] = None
                self._progress[side] = None
                goal = np.array(self.robot.get_T_world_frame(side+'_tcp_link'), copy=True)
            else:
                if self._anchors[side] is None:
                    self._anchors[side] = q[arm_slice].copy()
                reference[arm_slice] = self._anchors[side]
                goal = goals[side]
            if self.modes[side] == 'pose':
                self.tasks[side].T_world_frame = goal
            else:
                self.tasks[side].target_world = goal[:3, 3]
        self.posture_task.set_joints(dict(zip(JOINT_NAMES, map(float, reference))))
        regularization = self._regularization_policy(q, active)
        self.solver.dt = float(dt)
        start = time.perf_counter()
        try:
            self.solver.solve(True)
        except Exception as exc:
            self.last_diagnostics = dict(backend='placo', solve_called=True, reason='ik_failed')
            raise KinematicsError('ik_failed: '+str(exc)) from exc
        elapsed = time.perf_counter()-start
        self.last_diagnostics = dict(backend='placo', solve_called=True, solve_s=float(elapsed),
                                     modes=self.modes.copy(), active=dict(active),
                                     torso_waist_locked=True, position_weight=self.position_weight,
                                     orientation_weight=self.orientation_weight,
                                     singularity_policy=dict(strategy='adaptive_delta_q_l2',
                                         conditioning_orientation_length_m=CONDITIONING_ORIENTATION_LENGTH_M,
                                         threshold=SINGULARITY_THRESHOLD, arms=regularization))
        if not motion_policy.LIMITS_MANAGED_EXTERNALLY and elapsed > self.solve_budget_s:
            raise KinematicsError('ik_solve_over_budget')
        proposed = np.array([self.robot.get_joint(name) for name in JOINT_NAMES], dtype=float)
        if not np.all(np.isfinite(proposed)):
            raise KinematicsError('ik_returned_nonfinite')
        for i, side in enumerate(SIDES):
            if not active[side]:
                proposed[i*7:(i+1)*7] = q[i*7:(i+1)*7]
        if not motion_policy.LIMITS_MANAGED_EXTERNALLY and (np.any(proposed < self.model.limits[:, 0]-1e-8)
                or np.any(proposed > self.model.limits[:, 1]+1e-8)):
            raise KinematicsError('ik_joint_limit_violation')
        if not motion_policy.LIMITS_MANAGED_EXTERNALLY and np.max(np.abs(proposed-q)) > self.velocity_limit*dt+1e-7:
            raise KinematicsError('ik_velocity_limit_violation')
        residuals = {}
        for i, side in enumerate(SIDES):
            if active[side]:
                pose, jacobian = self.model.arms[side].pose_and_jacobian(proposed[i*7:(i+1)*7])
                residuals[side] = self._residual_status(side, goals[side], pose, jacobian, dt)
        self.last_diagnostics['target_errors'] = residuals
        stalled = [side for side, report in residuals.items() if report['status'] == 'stalled']
        if stalled and not motion_policy.LIMITS_MANAGED_EXTERNALLY:
            raise KinematicsError('ik_target_stalled:'+','.join(stalled))
        return proposed
