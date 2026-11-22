"""Original-chain behavior with external limits; all hardware uses test doubles."""
from dataclasses import replace
from functools import partial
import os
from pathlib import Path
import subprocess
import sys
import threading
import time
import unittest
from unittest.mock import patch

import numpy as np

from tg3_pico_dualarm import safety
from tg3_pico_dualarm.brainco_adapter import limit_hand_target
from tg3_pico_dualarm.controller import Teleoperation
from tg3_pico_dualarm.kinematics_worker import ProcessKinematics
from tg3_pico_dualarm.mapping import Calibration, RelativeMapper
from tg3_pico_dualarm.model import DualArmModel, default_q
from tg3_pico_dualarm.robot import DryRunRobot, JointCommand
from tg3_pico_dualarm.tracking import LatestTracking
from tg3_pico_dualarm.watchdog import CommandWatchdog
import test_controller as controller_fixtures
import test_kinematics_worker as worker_fixtures
import test_managed_xarm as raw_fixtures
import test_manufacturer as factory_fixtures
import test_mapping as mapping_fixtures
import test_xarm_adapter as xarm_fixtures


class ExternalLimitsTests(unittest.TestCase):
    def setUp(self):
        self.policy = patch.object(safety, "LIMITS_MANAGED_EXTERNALLY", True)
        self.policy.start()
        self.addCleanup(self.policy.stop)

    def test_fresh_process_defaults_to_external_limits(self):
        environment = dict(os.environ)
        source = str(Path(__file__).resolve().parents[1] / "src")
        environment["PYTHONPATH"] = source + os.pathsep + environment.get("PYTHONPATH", "")
        result = subprocess.run(
            [sys.executable, "-c",
             "from tg3_pico_dualarm.safety import LIMITS_MANAGED_EXTERNALLY; print(LIMITS_MANAGED_EXTERNALLY)"],
            capture_output=True, text=True, check=True, env=environment, timeout=15.)
        self.assertEqual(result.stdout.strip(), "True")

    def frame(self, sequence=1, grip=1.):
        return mapping_fixtures.frame(sequence, grip)

    def test_latest_receiver_retains_old_and_nonadvancing_valid_input(self):
        receiver = LatestTracking(clock=lambda: 100.)
        value = self.frame()
        packet = replace(value, source_age_s=20.).to_input_payload()
        first = receiver.accept(packet, now=100., additional_age_s=10.)
        self.assertIsNotNone(first)
        self.assertIsNotNone(receiver.accept(packet, now=101.))
        self.assertIs(receiver.sample(10000.), receiver.latest)
        self.assertFalse(receiver.latest.right.release_required)

    def test_invalid_numeric_packet_does_not_replace_last_valid_packet(self):
        receiver = LatestTracking(clock=lambda: 100.)
        packet = self.frame().to_input_payload()
        valid = receiver.accept(packet)
        packet["right"]["pose"]["position"] = [float("nan"), 0., 0.]
        self.assertIsNone(receiver.accept(packet))
        self.assertIs(receiver.sample(), valid)

    def test_original_mapper_accepts_large_jump_and_old_input(self):
        model = DualArmModel()
        mapper = RelativeMapper(Calibration(np.eye(3)))
        first = replace(self.frame(), received_at=1., source_age_s=20.)
        self.assertTrue(mapper.update(first, model.fk(default_q()), 100.).active["right"])
        moved = replace(first, right=replace(first.right, pose=replace(
            first.right.pose, position=(2., 2., 2.))))
        result = mapper.update(moved, model.fk(default_q()), 200.)
        self.assertTrue(result.active["right"])
        self.assertTrue(mapper.within_workspace("right", result.targets["right"]))

    def controller(self):
        model = DualArmModel()
        robot = DryRunRobot(default_q(), monotonic=lambda: 100.)
        engine = controller_fixtures.StubIK()
        controller = Teleoperation(robot, model, engine, monotonic=lambda: 100.)
        return controller, robot, engine

    def test_original_controller_passes_ik_output_without_motion_clamping(self):
        controller, robot, engine = self.controller()
        desired = np.asarray(default_q()) + 5.
        engine.solve = lambda *args: desired.copy()
        result = controller.tick(replace(self.frame(), received_at=1.), .5)
        self.assertEqual(result["state"], "dry_run", result)
        np.testing.assert_allclose(robot.read_state().joint_positions, desired)
        np.testing.assert_allclose(robot.commands[-1].velocities, np.full(14, 10.))

    def test_solver_error_reports_without_latching_stop_and_next_input_recovers(self):
        controller, robot, engine = self.controller()
        engine.fail = True
        result = controller.tick(self.frame())
        self.assertEqual(result["state"], "control_error")
        self.assertEqual(robot.commands, [])
        engine.fail = False
        self.assertEqual(controller.tick(self.frame(2))["state"], "dry_run")

    def test_manual_stop_remains_latched_against_later_original_chain_ticks(self):
        controller, robot, engine = self.controller()
        controller.tick(self.frame())
        controller.disarm("operator_disarm")
        count = len(robot.commands)
        before = robot.read_state().joint_positions
        for sequence in range(2, 5):
            self.assertEqual(controller.tick(self.frame(sequence))["state"], "disarmed")
        self.assertEqual(len(robot.commands), count)
        self.assertEqual(robot.read_state().joint_positions, before)

    def test_manual_stop_discards_result_from_an_inflight_original_ik_call(self):
        controller, robot, engine = self.controller()
        entered, release = threading.Event(), threading.Event()
        def solve(*args):
            entered.set()
            if not release.wait(2.):
                raise RuntimeError("test did not release the IK double")
            return np.asarray(default_q()) + 5.
        engine.solve = solve
        worker = threading.Thread(target=lambda: controller.tick(self.frame()))
        worker.start()
        try:
            self.assertTrue(entered.wait(1.))
            controller.disarm("operator_disarm")
            release.set()
            worker.join(2.)
            self.assertFalse(worker.is_alive())
            self.assertEqual(robot.commands, [])
            self.assertEqual(controller.last_status["state"], "disarmed")
        finally:
            release.set()
            worker.join(2.)

    def test_command_watchdog_does_not_automatically_call_stop(self):
        now = [0.]
        stopped = []
        watchdog = CommandWatchdog(lambda: stopped.append(True), monotonic=lambda: now[0])
        now[0] = 100.
        self.assertFalse(watchdog.check())
        watchdog.start()
        self.assertIsNone(watchdog.thread)
        self.assertEqual(stopped, [])

    def test_native_worker_waits_past_old_deadline_without_invalidating(self):
        engine = ProcessKinematics(
            _solver_factory=partial(worker_fixtures.BoundarySolverDouble, behavior="delay"),
            startup_timeout_s=10.)
        self.addCleanup(engine.close)
        q = default_q()
        started = time.perf_counter()
        result = engine.solve(q, engine.model.fk(q), dict(left=True, right=True), .2)
        self.assertGreater(time.perf_counter() - started, .20)
        np.testing.assert_allclose(result, q)
        self.assertFalse(engine.faulted)

    def test_xarm_original_adapter_passes_large_old_target_and_does_not_write_velocity_cap(self):
        fixture = xarm_fixtures.XArmAdapterTests()
        fixture.setUp()
        fixture.arm()
        self.assertFalse(any(event[0] == "set" for event in fixture.transport.events))
        desired = tuple(q + 5. for q in fixture.robot.commanded_joint_positions)
        result = fixture.robot.submit(JointCommand(desired, (20.,) * 14, 1.))
        self.assertTrue(result.accepted, result)
        self.assertEqual(fixture.transport.published[-1], desired)

    def test_xarm_feedback_age_and_gap_do_not_automatically_stop(self):
        fixture = xarm_fixtures.XArmAdapterTests()
        fixture.setUp()
        fixture.arm()
        fixture.now[0] += 10.
        desired = tuple(q + 1. for q in fixture.robot.commanded_joint_positions)
        result = fixture.robot.submit(JointCommand(desired, (10.,) * 14, 1.))
        self.assertTrue(result.accepted, result)
        self.assertEqual(fixture.robot.state, "armed")

    def test_xarm_manual_disable_still_closes_publisher_and_runs_release(self):
        fixture = xarm_fixtures.XArmAdapterTests()
        fixture.setUp()
        fixture.arm()
        fixture.robot.disarm("explicit_operator_disabled")
        self.assertIsNone(fixture.transport.publisher)
        fixture.release()
        self.assertEqual(fixture.robot.state, "disabled")
        self.assertTrue(fixture.robot.stop_confirmed)

    def test_physical_estop_input_still_closes_xarm_output(self):
        fixture = xarm_fixtures.XArmAdapterTests()
        fixture.setUp()
        fixture.arm()
        fixture.feedback.key_status(factory_fixtures.power_key_message(estop=True))
        result = fixture.robot.submit(JointCommand(
            fixture.robot.commanded_joint_positions, (0.,) * 14, fixture.clock()))
        self.assertFalse(result.accepted)
        self.assertIsNone(fixture.transport.publisher)
        self.assertEqual(fixture.robot.state, "releasing")

    def test_raw_factory_output_is_forwarded_unchanged_past_old_bounds(self):
        relay, messages, message = raw_fixtures.BoundedRawTests().fixture()
        for motor in message.ctrl:
            motor.pos = 5.
        relay.last_actual_time = 0.
        relay.raw(message)
        self.assertEqual(len(messages), 1)
        self.assertIs(messages[0], message)
        self.assertEqual(messages[0].ctrl[0].spd, 3.14)
        self.assertEqual(messages[0].ctrl[0].pos, 5.)
        self.assertIsNone(relay.output_fault)

    def test_raw_output_after_manual_disable_is_not_forwarded(self):
        relay, messages, message = raw_fixtures.BoundedRawTests().fixture()
        relay.adapter.state = "disabled"
        relay.raw(message)
        self.assertEqual(messages, [])

    def test_factory_adapter_passes_original_command_without_joint_limits(self):
        fixture = factory_fixtures.ManufacturerTests()
        fixture.setUp()
        fixture.enable()
        desired = (5.,) * 14
        result = fixture.adapter.submit(JointCommand(desired, (20.,) * 14, 1.))
        self.assertTrue(result.accepted, result)
        self.assertEqual(fixture.adapter.commanded_joint_positions, desired)
        self.assertEqual(len(fixture.node.publishers[0].messages), 1)

    def test_factory_manual_stop_still_calls_stop_restore_and_release(self):
        fixture = factory_fixtures.ManufacturerTests()
        fixture.setUp()
        fixture.enable()
        result = fixture.adapter.disarm("operator_disarm")
        self.assertTrue(result.accepted, result)
        self.assertIn("release", fixture.driver.events)
        self.assertEqual(fixture.adapter.state, "disabled")

    def test_hand_target_goes_directly_to_calibrated_endpoint_without_rate_corridor(self):
        fixture = factory_fixtures.ManufacturerTests()
        fixture.setUp()
        values, floating, diagnostics = limit_hand_target(
            fixture.profile, 1., operation="close", previous=(100,) * 6,
            floating=(100.,) * 6, measured=(100.,) * 6,
            raw_rate=(1.,) * 6, raw_lead=(1.,) * 6, dt=.001)
        self.assertEqual(values, fixture.profile.closed)
        self.assertFalse(diagnostics["feedback_limited"])

    def test_lower_trigger_retains_original_close_operation_semantics(self):
        fixture = factory_fixtures.ManufacturerTests()
        fixture.setUp()
        values, _, _ = limit_hand_target(
            fixture.profile, .1, operation="close", previous=(800,) * 6,
            floating=(800.,) * 6, measured=(100.,) * 6,
            raw_rate=(1.,) * 6, raw_lead=(1.,) * 6, dt=.001)
        self.assertEqual(values, (800,) * 6)


if __name__ == "__main__":
    unittest.main()
