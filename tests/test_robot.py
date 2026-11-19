import math
import unittest

from tg3_pico_dualarm.robot import (
    DryRunRobot, JointCommand, ReadOnlyRobot, RobotProtocol, RobotState,
)


class Clock:
    now = 10.
    def __call__(self):
        return self.now


class RobotTests(unittest.TestCase):
    def test_dry_run_accepts_all_fourteen_targets_and_marks_simulated_state(self):
        clock = Clock()
        robot = DryRunRobot((.1,) * 14, monotonic=clock)
        command = JointCommand(tuple(i / 100 for i in range(14)), (.01,) * 14, clock())
        self.assertTrue(isinstance(robot, RobotProtocol))
        self.assertTrue(robot.submit(command).accepted)
        self.assertEqual(robot.read_state().joint_positions, command.positions)
        self.assertEqual(robot.commands, [command])
        self.assertEqual(robot.read_state().quality, "simulated")
        self.assertEqual(robot.read_state().mode, "DRY_RUN")
        self.assertIsNone(robot.read_state().estop)
        self.assertFalse(robot.hardware_writes_enabled)

    def test_command_replay_future_age_and_disarm_are_rejected_without_state_change(self):
        clock = Clock()
        robot = DryRunRobot(monotonic=clock)
        command = JointCommand((.1,) * 14, (0.,) * 14, clock())
        self.assertTrue(robot.submit(command).accepted)
        self.assertEqual(robot.submit(command).reason, "nonincreasing_command_stamp")
        self.assertFalse(robot.submit(JointCommand((.5,) * 14, (0.,) * 14, clock() + 1)).accepted)
        clock.now += 1
        self.assertFalse(robot.submit(command).accepted)
        robot.disarm()
        self.assertFalse(robot.submit(JointCommand((.5,) * 14, (0.,) * 14, clock())).accepted)
        self.assertEqual(robot.read_state().joint_positions, command.positions)
        self.assertIsNone(robot.read_state().estop)
        robot.arm()
        self.assertTrue(robot.submit(JointCommand((.2,) * 14, (0.,) * 14, clock())).accepted)

    def test_history_is_bounded_and_caller_cannot_mutate_it(self):
        clock = Clock()
        robot = DryRunRobot(monotonic=clock, max_history=2)
        for i in range(4):
            clock.now += .02
            self.assertTrue(robot.submit(JointCommand((i / 10,) * 14, (0.,) * 14, clock())).accepted)
        self.assertEqual(len(robot.commands), 2)
        robot.commands.clear()
        self.assertEqual(len(robot.commands), 2)

    def test_read_only_adapter_has_no_hardware_enable_path(self):
        state = RobotState((0.,) * 14, 1., "SWR", None, "measured")
        robot = ReadOnlyRobot(lambda: state)
        self.assertIs(robot.read_state(), state)
        result = robot.submit(JointCommand((.1,) * 14, (0.,) * 14, 1.))
        self.assertFalse(result.accepted)
        self.assertIn("verified_dual_arm_adapter_missing", result.reason)
        self.assertFalse(hasattr(robot, "arm"))

    def test_invalid_shapes_nonfinite_values_and_estop_are_not_silently_coerced(self):
        for values in [(0.,) * 7, (0.,) * 13 + (math.nan,), (0.,) * 13 + (True,)]:
            with self.subTest(values=values), self.assertRaises(ValueError):
                JointCommand(values, (0.,) * 14, 1.)
        with self.assertRaises(ValueError):
            JointCommand((0.,) * 14, (0.,) * 14, float("inf"))
        with self.assertRaises(ValueError):
            RobotState((0.,) * 14, 1., "SWR", 0, "measured")


if __name__ == "__main__":
    unittest.main()
