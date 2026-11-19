import contextlib
import io
import json
from pathlib import Path
import sys
import threading
from types import ModuleType, SimpleNamespace as NS
import unittest
from unittest.mock import patch

from tg3_pico_dualarm.preflight import CONTROL_TOPICS
from tg3_pico_dualarm.robot import JointCommand
from tg3_pico_dualarm.ros_bridge import (
    DIAGNOSTIC_TOPIC, DRY_RUN_JOINT_TOPIC, RosDryRunSession, build_bridge_class, create_bridge, main,
)


class RosBridgeTests(unittest.TestCase):
    def fake_ros(self):
        created = dict(nodes=0, publishers=[], subscriptions=[], messages={}, destroyed=False,
                       shutdown=False, executor_shutdown=False, contexts=0)
        class Node:
            def __init__(self, name, **kwargs):
                created["nodes"] += 1
            def create_publisher(self, typ, topic, qos):
                created["publishers"].append(topic)
                created["messages"][topic] = []
                return NS(publish=lambda message: created["messages"][topic].append(message))
            def create_subscription(self, typ, topic, callback, qos):
                created["subscriptions"].append((topic, callback))
                return NS()
            def create_timer(self, period, callback):
                created["timer"] = (period, callback)
                return NS()
            def get_clock(self):
                return NS(now=lambda: NS(to_msg=lambda: NS(sec=10, nanosec=0)))
            def destroy_node(self):
                created["destroyed"] = True
        class JointState:
            def __init__(self):
                self.header = NS()
                self.name = self.position = self.velocity = self.effort = []
        class String:
            def __init__(self):
                self.data = ""
        class Context:
            def __init__(self):
                created["contexts"] += 1
            def ok(self):
                return not created["shutdown"]
            def shutdown(self):
                created["shutdown"] = True
        class Executor:
            def __init__(self, **kwargs):
                pass
            def add_node(self, node):
                self.node = node
            def spin(self):
                self.node.publish_observation()
            def shutdown(self):
                created["executor_shutdown"] = True
        modules = {}
        def module(name, **attrs):
            value = ModuleType(name)
            value.__dict__.update(attrs)
            modules[name] = value
        module("rclpy", init=lambda **kwargs: created.update(init_arguments=kwargs))
        module("rclpy.node", Node=Node)
        module("rclpy.qos", qos_profile_sensor_data=NS())
        module("rclpy.context", Context=Context)
        module("rclpy.executors", SingleThreadedExecutor=Executor)
        module("rclpy.type_support", check_for_type_support=lambda cls: None)
        module("sensor_msgs")
        module("sensor_msgs.msg", JointState=JointState)
        module("std_msgs")
        module("std_msgs.msg", String=String)
        module("ros2_bridge_msgs")
        module("ros2_bridge_msgs.msg", RobotState=object)
        module("diagnostic_msgs")
        module("diagnostic_msgs.msg", DiagnosticStatus=object)
        return modules, created

    def test_hardware_mode_fails_before_ros_import_or_node_creation(self):
        with patch("tg3_pico_dualarm.ros_bridge.build_bridge_class") as factory:
            with self.assertRaisesRegex(RuntimeError, "authorized TianGong dual-arm"):
                create_bridge(mode="hardware")
            factory.assert_not_called()
        modules, calls = self.fake_ros()
        with patch.dict(sys.modules, modules), contextlib.redirect_stderr(io.StringIO()) as stderr:
            self.assertEqual(main(["--mode", "hardware"]), 2)
        self.assertFalse(json.loads(stderr.getvalue())["hardware_writes_enabled"])
        self.assertEqual(calls["nodes"], 0)
        self.assertEqual(calls["contexts"], 0)
        self.assertEqual(calls["publishers"], [])

    def test_dry_run_publishes_standard_telemetry_only(self):
        modules, calls = self.fake_ros()
        with patch.dict(sys.modules, modules):
            node = create_bridge(monotonic=lambda: 10.)
            command = JointCommand((.1,) * 14, (.02,) * 14, 10.)
            self.assertTrue(node.submit_simulated(command).accepted)
            node.publish_observation()
            message = calls["messages"][DRY_RUN_JOINT_TOPIC][-1]
            self.assertEqual(len(message.name), 14)
            self.assertEqual(message.position, [.1] * 14)
            self.assertEqual(message.velocity, [])
            self.assertIn("simulated", message.header.frame_id)
            diagnostic = json.loads(calls["messages"][DIAGNOSTIC_TOPIC][-1].data)
            self.assertTrue(diagnostic["simulated"])
            self.assertFalse(diagnostic["hardware_writes_enabled"])
            self.assertIsNone(diagnostic["physical_estop"])
            self.assertEqual(diagnostic["last_simulated_command"]["velocities"], [.02] * 14)
            self.assertEqual(set(calls["publishers"]), {DRY_RUN_JOINT_TOPIC, DIAGNOSTIC_TOPIC})
            self.assertFalse(set(calls["publishers"]) & set(CONTROL_TOPICS))
            self.assertEqual(calls["subscriptions"], [])
            node.disarm()
            self.assertFalse(node.submit_simulated(command).accepted)
            node.publish_observation()
            self.assertTrue(json.loads(calls["messages"][DIAGNOSTIC_TOPIC][-1].data)["software_disarmed"])

    def test_actual_feedback_is_observed_without_overwriting_simulation(self):
        modules, calls = self.fake_ros()
        with patch.dict(sys.modules, modules):
            node = create_bridge(observe_robot=True, monotonic=lambda: 10.)
            callbacks = dict(calls["subscriptions"])
            ids = tuple(range(11, 18)) + tuple(range(21, 28))
            callbacks["/robot_state"](NS(arm=NS(status=[NS(name=i, pos=.2, error=0) for i in ids])))
            callbacks["/hric/robot/rl_state"](NS(level=0, values=[NS(key=k, value=v) for k, v in
                       dict(current_state="SWR", child_state="SWR", status="running").items()]))
            self.assertEqual(node.observed_robot.read_state().joint_positions, (.2,) * 14)
            self.assertNotEqual(node.read_state().joint_positions, (.2,) * 14)
            self.assertFalse(node.observed_robot.submit(JointCommand((.2,) * 14, (0.,) * 14, 10.)).accepted)
            node.publish_observation()
            diagnostic = json.loads(calls["messages"][DIAGNOSTIC_TOPIC][-1].data)
            self.assertEqual(diagnostic["actual_robot_observation"]["mode"], "SWR")
            self.assertFalse(diagnostic["robot_commands_sent"])

    def test_constructor_hardware_request_and_invalid_rate_never_create_node(self):
        modules, calls = self.fake_ros()
        with patch.dict(sys.modules, modules):
            cls = build_bridge_class()
            with self.assertRaises(RuntimeError):
                cls(mode="hardware")
            with self.assertRaises(ValueError):
                cls(publish_rate_hz=float("nan"))
        self.assertEqual(calls["nodes"], 0)

    def test_main_forwards_ros_arguments_and_cleans_up(self):
        modules, calls = self.fake_ros()
        with patch.dict(sys.modules, modules), contextlib.redirect_stdout(io.StringIO()) as stdout:
            result = main(["--mode", "dry_run", "--ros-args", "-r", "__node:=inspection_only"])
        self.assertEqual(result, 0)
        self.assertFalse(json.loads(stdout.getvalue())["robot_commands_sent"])
        self.assertEqual(calls["init_arguments"]["args"], ["--ros-args", "-r", "__node:=inspection_only"])
        self.assertTrue(calls["destroyed"] and calls["shutdown"] and calls["executor_shutdown"])

    def test_ament_resources_and_launch_default_do_not_enable_hardware(self):
        root = Path(__file__).resolve().parents[1]
        self.assertTrue((root / "resource/tg3_pico_dualarm").is_file())
        self.assertIn("ament_python", (root / "package.xml").read_text())
        self.assertIn("lib/tg3_pico_dualarm", (root / "setup.cfg").read_text())
        launch = (root / "launch/teleop.launch.py").read_text()
        self.assertIn('default_value="dry_run"', launch)
        self.assertNotIn("/arm/cmd", launch)

    def test_embedded_session_shares_controller_targets_with_ros_joint_state_telemetry(self):
        modules, calls = self.fake_ros()
        stopped = threading.Event()
        publish = threading.Event()
        published = threading.Event()
        started = threading.Event()
        class Executor:
            def __init__(self, **kwargs):
                pass
            def add_node(self, node):
                self.node = node
            def spin(self):
                started.set()
                while not stopped.is_set():
                    if publish.wait(.05):
                        publish.clear()
                        if not stopped.is_set():
                            self.node.publish_observation()
                            published.set()
            def shutdown(self, **kwargs):
                calls["executor_shutdown"] = True
                stopped.set()
                publish.set()
        modules["rclpy.executors"].SingleThreadedExecutor = Executor
        with patch.dict(sys.modules, modules):
            session = RosDryRunSession(monotonic=lambda: 10.)
            try:
                self.assertTrue(started.wait(1.))
                command = JointCommand((.11,) * 14, (.01,) * 14, 10.)
                self.assertTrue(session.robot.submit(command).accepted)
                self.assertEqual(session.robot.read_state().joint_positions, command.positions)
                publish.set()
                self.assertTrue(published.wait(1.))
                self.assertEqual(calls["messages"][DRY_RUN_JOINT_TOPIC][-1].position, [.11] * 14)
                status = json.loads(calls["messages"][DIAGNOSTIC_TOPIC][-1].data)
                self.assertEqual(status["last_simulated_command"]["positions"], [.11] * 14)
                self.assertFalse(status["hardware_writes_enabled"])
                self.assertFalse(set(calls["publishers"]) & set(CONTROL_TOPICS))
                self.assertEqual(calls["contexts"], 1)
            finally:
                session.close()
                session.close()
            self.assertFalse(session._thread.is_alive())
            self.assertTrue(calls["destroyed"] and calls["shutdown"] and calls["executor_shutdown"])
            with self.assertRaisesRegex(RuntimeError, "closed"):
                session.robot.submit(command)

    def test_session_executor_failure_disarms_simulator_and_refuses_new_targets(self):
        modules, calls = self.fake_ros()
        failure = threading.Event()
        class Executor:
            def __init__(self, **kwargs):
                pass
            def add_node(self, node):
                pass
            def spin(self):
                failure.wait(1.)
                raise RuntimeError("injected telemetry failure")
            def shutdown(self, **kwargs):
                failure.set()
                calls["executor_shutdown"] = True
        modules["rclpy.executors"].SingleThreadedExecutor = Executor
        with patch.dict(sys.modules, modules):
            session = RosDryRunSession(monotonic=lambda: 10.)
            try:
                failure.set()
                self.assertTrue(session._failed.wait(1.))
                with self.assertRaisesRegex(RuntimeError, "executor failed"):
                    session.robot.submit(JointCommand((.1,) * 14, (0.,) * 14, 10.))
                self.assertEqual(session.node.read_state().mode, "DRY_RUN_DISARMED")
            finally:
                session.close()
            self.assertTrue(calls["destroyed"] and calls["shutdown"])

    def test_session_setup_failure_closes_private_context_before_any_publisher(self):
        modules, calls = self.fake_ros()
        with patch.dict(sys.modules, modules), \
                patch("tg3_pico_dualarm.ros_bridge.create_bridge", side_effect=RuntimeError("setup failed")):
            with self.assertRaisesRegex(RuntimeError, "setup failed"):
                RosDryRunSession()
        self.assertTrue(calls["shutdown"])
        self.assertEqual(calls["publishers"], [])


if __name__ == "__main__":
    unittest.main()
