from types import ModuleType, SimpleNamespace as NS
import sys
import unittest
from unittest.mock import patch

from tg3_pico_dualarm.preflight import (
    CONTROL_TOPICS, SnapshotAccumulator, parse_arm_positions, parse_fsm,
    publisher_checks, run, _message_schemas,
)
from tg3_pico_dualarm.robot import JOINT_IDS


def arm_message():
    # Deliberately reversed input verifies that ordering follows IDs, not array order.
    return NS(arm=NS(status=[NS(name=i, pos=i / 100, error=b"\x00") for i in reversed(JOINT_IDS)]))


def fsm_message(current="SWR", child=None, status="running"):
    return NS(level=b"\x00", values=[NS(key=k, value=v) for k, v in
              dict(current_state=current, child_state=child or current, status=status).items()])


def factory_graph():
    return {topic: [dict(node="joystick_bridge", namespace="/", gid="01" * 16)] for topic in CONTROL_TOPICS}


class PreflightTests(unittest.TestCase):
    def test_real_nested_schema_requires_all_fourteen_valid_unique_motors(self):
        message = arm_message()
        self.assertEqual(parse_arm_positions(message), tuple(i / 100 for i in JOINT_IDS))
        message.arm.status.append(message.arm.status[0])
        with self.assertRaisesRegex(ValueError, "duplicate"):
            parse_arm_positions(message)
        message = arm_message()
        message.arm.status.pop()
        with self.assertRaisesRegex(ValueError, "missing"):
            parse_arm_positions(message)
        message = arm_message()
        message.arm.status[0].error = b"\x01"
        with self.assertRaisesRegex(ValueError, "motor error"):
            parse_arm_positions(message)

    def test_duplicate_fsm_keys_do_not_override_a_bad_mode(self):
        message = fsm_message()
        message.values.append(NS(key="status", value="running"))
        with self.assertRaisesRegex(ValueError, "duplicate"):
            parse_fsm(message)

    def test_fresh_observation_never_grants_hardware_or_invents_physical_estop(self):
        clock = lambda: 10.
        snapshot = SnapshotAccumulator(monotonic=clock)
        snapshot.arm(arm_message())
        snapshot.mode(fsm_message())
        report = snapshot.report(factory_graph(), sbus_publishers=1)
        self.assertTrue(report["observation_ready"])
        self.assertFalse(report["hardware_ready"])
        self.assertFalse(report["hardware_writes_enabled"])
        self.assertIsNone(report["physical_estop"])
        self.assertIsNone(snapshot.read_state().estop)
        self.assertEqual(report["motion_publishers_created"], [])
        stale = snapshot.report(factory_graph(), sbus_publishers=1, now=11.)
        self.assertFalse(stale["observation_ready"])
        self.assertEqual(stale["robot_state"]["quality"], "stale")

    def test_invalid_new_frame_invalidates_the_previous_good_feedback(self):
        snapshot = SnapshotAccumulator(monotonic=lambda: 10.)
        snapshot.arm(arm_message())
        broken = arm_message()
        broken.arm.status[0].pos = float("nan")
        snapshot.arm(broken)
        with self.assertRaisesRegex(RuntimeError, "no complete"):
            snapshot.read_state()
        self.assertFalse(snapshot.report(factory_graph(), sbus_publishers=1)["observation_ready"])

    def test_wrong_or_mismatched_mode_is_reported_even_with_fresh_feedback(self):
        snapshot = SnapshotAccumulator(monotonic=lambda: 10.)
        snapshot.arm(arm_message())
        for message in (fsm_message("START"), fsm_message("SWR", child="HBWALK")):
            snapshot.mode(message)
            self.assertIn("observed_mode_is_not_running_SWR_or_HBWALK",
                          snapshot.report(factory_graph(), sbus_publishers=1)["issues"])

    def test_missing_extra_and_unresolved_publishers_have_distinct_diagnostics(self):
        graph = factory_graph()
        graph[CONTROL_TOPICS[0]] = []
        graph[CONTROL_TOPICS[1]].append(dict(node="foreign", namespace="/"))
        graph[CONTROL_TOPICS[2]] = [dict(node="_NODE_NAME_UNKNOWN_", namespace="_NODE_NAMESPACE_UNKNOWN_")]
        checks = publisher_checks(graph)
        self.assertEqual(checks[CONTROL_TOPICS[0]]["missing_expected_count"], 1)
        self.assertEqual(checks[CONTROL_TOPICS[0]]["additional_or_different_count"], 0)
        self.assertEqual(checks[CONTROL_TOPICS[1]]["additional_or_different_count"], 1)
        self.assertEqual(checks[CONTROL_TOPICS[2]]["unresolved_metadata_count"], 1)
        self.assertFalse(any(checks[topic]["matches_observed_factory"] for topic in CONTROL_TOPICS[:3]))

    def test_probe_rejects_short_discovery_window_before_importing_ros(self):
        for seconds in (1., float("nan"), 121.):
            with self.subTest(seconds=seconds), self.assertRaises(ValueError):
                run(seconds)

    def test_installed_message_introspection_catches_interface_drift(self):
        expected = {
            "ros2_bridge_msgs/msg/ArmCtrl": {x: "observed" for x in ("header", "mode", "label", "reserved", "ctrl")},
            "ros2_bridge_msgs/msg/MotorCtrl": {x: "observed" for x in ("name", "kp", "kd", "pos", "spd", "tor", "cur")},
            "ros2_bridge_msgs/msg/RobotState": {"arm": "observed"},
        }
        classes = {name: NS(get_fields_and_field_types=lambda fields=fields: fields)
                   for name, fields in expected.items()}
        self.assertEqual(_message_schemas(classes), expected)
        classes["ros2_bridge_msgs/msg/RobotState"] = NS(get_fields_and_field_types=lambda: {"invented_joint_state": "float"})
        with self.assertRaisesRegex(RuntimeError, "schema differs"):
            _message_schemas(classes)

    def test_persistent_ros_probe_subscribes_only_and_cleans_up_its_context(self):
        clock = NS(now=10.)
        calls = dict(contexts=0, destroyed=False, shutdown=False, executor_shutdown=False)
        class Context:
            def __init__(self):
                calls["contexts"] += 1
            def ok(self):
                return not calls["shutdown"]
            def shutdown(self):
                calls["shutdown"] = True
        class Node:
            # No create_publisher method: an attempted command output fails this test.
            def __init__(self, name, **kwargs):
                self.subscriptions = []
                self.name = name
            def create_subscription(self, typ, topic, callback, qos):
                self.subscriptions.append((topic, callback))
                return NS()
            def get_publishers_info_by_topic(self, topic):
                return [NS(node_name="joystick_bridge", node_namespace="/", endpoint_gid=bytes([1]) * 16,
                           topic_type="observed/custom_interface")]
            def count_publishers(self, topic):
                return 1
            def get_topic_names_and_types(self):
                return [("/robot_state", ["ros2_bridge_msgs/msg/RobotState"])]
            def get_publisher_names_and_types_by_node(self, *args):
                return []
            def get_name(self):
                return self.name
            def get_namespace(self):
                return "/"
            def destroy_node(self):
                calls["destroyed"] = True
        class Executor:
            def __init__(self, **kwargs):
                pass
            def add_node(self, node):
                self.node = node
            def spin_once(self, timeout_sec):
                clock.now += timeout_sec
                for topic, callback in self.node.subscriptions:
                    if topic == "/robot_state":
                        callback(arm_message())
                    elif topic == "/hric/robot/rl_state":
                        callback(fsm_message())
            def shutdown(self):
                calls["executor_shutdown"] = True
        modules = {}
        def module(name, **attrs):
            value = ModuleType(name)
            value.__dict__.update(attrs)
            modules[name] = value
        def message_type(fields):
            return type("ObservedMessage", (), {"get_fields_and_field_types": classmethod(lambda cls: dict.fromkeys(fields, "observed"))})
        module("rclpy", init=lambda **kwargs: None)
        module("rclpy.context", Context=Context)
        module("rclpy.executors", SingleThreadedExecutor=Executor)
        module("rclpy.node", Node=Node)
        module("rclpy.qos", qos_profile_sensor_data=NS())
        module("rclpy.type_support", check_for_type_support=lambda cls: None)
        module("ros2_bridge_msgs")
        module("ros2_bridge_msgs.msg", ArmCtrl=message_type(("header", "mode", "label", "reserved", "ctrl")),
               MotorCtrl=message_type(("name", "kp", "kd", "pos", "spd", "tor", "cur")), RobotState=message_type(("arm",)))
        module("brainco_hand_msgs")
        module("brainco_hand_msgs.msg", MotorStatus=message_type(("positions", "states")), SetMotorMulti=message_type(("positions",)))
        module("diagnostic_msgs")
        module("diagnostic_msgs.msg", DiagnosticStatus=object)
        module("geometry_msgs")
        module("geometry_msgs.msg", TwistStamped=object)
        module("std_msgs")
        module("std_msgs.msg", String=object)
        accumulator = SnapshotAccumulator
        with patch.dict(sys.modules, modules), patch("tg3_pico_dualarm.preflight.time.monotonic", lambda: clock.now), \
                patch("tg3_pico_dualarm.preflight.SnapshotAccumulator", lambda: accumulator(monotonic=lambda: clock.now)):
            report = run(5.)
        self.assertTrue(report["observation_ready"])
        self.assertFalse(report["hardware_ready"])
        self.assertEqual(report["motion_publishers_created"], [])
        self.assertEqual(calls["contexts"], 1)
        self.assertTrue(calls["destroyed"] and calls["shutdown"] and calls["executor_shutdown"])


if __name__ == "__main__":
    unittest.main()
