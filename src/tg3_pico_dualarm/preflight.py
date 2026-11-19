"""Read-only inspection of this robot's observed ROS interfaces.

No command publisher, controller replacement, mode request, or motor output is
created here. Graph identities and receipt-time freshness are observations,
not a grant of dual-arm control authority.
"""
from __future__ import annotations

import argparse
from collections import Counter
from dataclasses import asdict
import json
import math
import time
from typing import Any, Callable, Mapping
import uuid

from .robot import JOINT_IDS, RobotState


ARM_STATE_TOPIC = "/robot_state"
FSM_STATE_TOPIC = "/hric/robot/rl_state"
JOY_TOPIC = "/sbus_data"
CONTROL_TOPICS = (
    "/arm/cmd", "/right_hand/set_motor_multi", "/hric/robot/cmd_vel",
    "/hric/robot/fsm_state_cmd", "/hric/robot/stand_cmd", "/hric/robot/fsm_resume_cmd",
)
EXPECTED_FACTORY_OWNER = ("joystick_bridge", "/")
ARM_MAX_AGE_S = .25
FSM_MAX_AGE_S = .5


def _code(value: Any) -> int:
    if isinstance(value, (bytes, bytearray)):
        if len(value) != 1:
            raise ValueError("invalid byte-valued status")
        return value[0]
    if isinstance(value, bool):
        raise ValueError("boolean is not a motor status code")
    result = int(value)
    if result != value:
        raise ValueError("noninteger status code")
    return result


def parse_arm_positions(message: Any) -> tuple[float, ...]:
    """Parse the verified RobotState.arm.status[].name/pos/error structure."""
    positions: dict[int, float] = {}
    try:
        for item in message.arm.status:
            raw_id = item.name
            if isinstance(raw_id, bool):
                raise ValueError("invalid motor ID")
            motor_id = int(raw_id)
            if isinstance(raw_id, float) and motor_id != raw_id:
                raise ValueError("noninteger motor ID")
            if motor_id not in JOINT_IDS:
                continue
            if motor_id in positions:
                raise ValueError(f"duplicate arm motor ID {motor_id}")
            if _code(item.error) != 0:
                raise ValueError(f"motor error at ID {motor_id}")
            if isinstance(item.pos, (bool, str, bytes)):
                raise ValueError(f"invalid position at ID {motor_id}")
            position = float(item.pos)
            if not math.isfinite(position):
                raise ValueError(f"nonfinite position at ID {motor_id}")
            positions[motor_id] = position
    except (AttributeError, TypeError, OverflowError) as exc:
        raise ValueError("invalid RobotState.arm.status structure") from exc
    missing = sorted(set(JOINT_IDS) - positions.keys())
    if missing:
        raise ValueError("missing arm motor IDs: " + ",".join(map(str, missing)))
    return tuple(positions[motor_id] for motor_id in JOINT_IDS)


def parse_fsm(message: Any) -> dict[str, Any]:
    """The observed diagnostic supplies mode/status, not physical E-stop."""
    required = {"current_state", "child_state", "status"}
    values: dict[str, str] = {}
    try:
        for item in message.values:
            key = str(item.key).strip()
            if key not in required:
                continue
            if key in values:
                raise ValueError("duplicate FSM diagnostic key: " + key)
            values[key] = str(item.value).strip()
        if not required.issubset(values):
            raise ValueError("missing FSM diagnostic keys")
        level = _code(message.level)
    except (AttributeError, TypeError, OverflowError) as exc:
        raise ValueError("invalid FSM diagnostic structure") from exc
    return dict(values, level=level)


def _gid(value: Any) -> str | None:
    if isinstance(value, (str, bool, int)):
        return None
    try:
        raw = bytes(value)
    except (TypeError, ValueError, OverflowError):
        return None
    return raw.hex() if raw and any(raw) else None


def publisher_checks(publishers: Mapping[str, list[dict[str, Any]]]) -> dict[str, Any]:
    """Distinguish missing endpoints, extra owners and unresolved metadata.

    This is deliberately the strict observed factory identity. Known shared
    navigation endpoints are reported as additional owners; no fresh baseline
    or an apparently silent message sample can enable hardware in this probe.
    """
    result: dict[str, Any] = {}
    for topic in CONTROL_TOPICS:
        owners = publishers.get(topic, [])
        seen = Counter((item.get("node"), item.get("namespace")) for item in owners)
        expected = Counter((EXPECTED_FACTORY_OWNER,))
        unresolved = [item for item in owners if item.get("node") in (None, "_NODE_NAME_UNKNOWN_")
                      or item.get("namespace") in (None, "_NODE_NAMESPACE_UNKNOWN_")]
        result[topic] = {
            "expected": [{"node": EXPECTED_FACTORY_OWNER[0], "namespace": EXPECTED_FACTORY_OWNER[1]}],
            "observed": list(owners),
            "missing_expected_count": sum((expected - seen).values()),
            "additional_or_different_count": sum((seen - expected).values()),
            "unresolved_metadata_count": len(unresolved),
            "matches_observed_factory": not unresolved and seen == expected,
        }
    return result


class SnapshotAccumulator:
    """Invalidate preceding good feedback immediately when a bad frame arrives."""

    def __init__(self, *, monotonic: Callable[[], float] = time.monotonic) -> None:
        self._clock = monotonic
        self.positions: tuple[float, ...] | None = None
        self.arm_stamp: float | None = None
        self.fsm: dict[str, Any] | None = None
        self.fsm_stamp: float | None = None
        self.errors: Counter[str] = Counter()
        self.received: Counter[str] = Counter()
        self.command_activity: Counter[str] = Counter()

    def arm(self, message: Any) -> None:
        self.received[ARM_STATE_TOPIC] += 1
        try:
            self.positions = parse_arm_positions(message)
            self.arm_stamp = self._clock()
        except (ValueError, TypeError, OverflowError) as exc:
            self.positions = self.arm_stamp = None
            self.errors["arm:" + str(exc)[:160]] += 1

    def mode(self, message: Any) -> None:
        self.received[FSM_STATE_TOPIC] += 1
        try:
            self.fsm = parse_fsm(message)
            self.fsm_stamp = self._clock()
        except (ValueError, TypeError, OverflowError) as exc:
            self.fsm = self.fsm_stamp = None
            self.errors["fsm:" + str(exc)[:160]] += 1

    @staticmethod
    def _fresh(stamp: float | None, now: float, maximum: float) -> bool:
        return stamp is not None and 0 <= now - stamp <= maximum

    def read_state(self, now: float | None = None) -> RobotState:
        now = self._clock() if now is None else now
        if self.positions is None or self.arm_stamp is None:
            raise RuntimeError("no complete valid fourteen-joint feedback")
        fresh_mode = self._fresh(self.fsm_stamp, now, FSM_MAX_AGE_S)
        mode = self.fsm["current_state"] if fresh_mode and self.fsm else "UNKNOWN"
        quality = "measured" if self._fresh(self.arm_stamp, now, ARM_MAX_AGE_S) else "stale"
        return RobotState(self.positions, self.arm_stamp, mode, None, quality)

    def report(self, publishers: Mapping[str, list[dict[str, Any]]], *,
               sbus_publishers: int | None = None, now: float | None = None) -> dict[str, Any]:
        now = self._clock() if now is None else now
        arm_fresh = self.positions is not None and self._fresh(self.arm_stamp, now, ARM_MAX_AGE_S)
        mode_fresh = self.fsm is not None and self._fresh(self.fsm_stamp, now, FSM_MAX_AGE_S)
        mode_running = bool(mode_fresh and self.fsm and self.fsm["level"] == 0
                            and self.fsm["current_state"] == self.fsm["child_state"]
                            and self.fsm["current_state"] in ("SWR", "HBWALK")
                            and self.fsm["status"] == "running")
        checks = publisher_checks(publishers)
        issues = []
        if not arm_fresh:
            issues.append("missing_or_stale_fourteen_joint_feedback")
        if not mode_fresh:
            issues.append("missing_or_stale_fsm_feedback")
        elif not mode_running:
            issues.append("observed_mode_is_not_running_SWR_or_HBWALK")
        if sbus_publishers != 1:
            issues.append("unique_existing_sbus_publisher_not_observed")
        if not all(value["matches_observed_factory"] for value in checks.values()):
            issues.append("control_publisher_identity_requires_review")
        state = asdict(self.read_state(now)) if self.positions is not None else None
        return {
            "passive_only": True, "hardware_writes_enabled": False,
            "hardware_ready": False, "observation_ready": not issues,
            "joint_order_motor_ids": list(JOINT_IDS), "robot_state": state,
            "physical_estop": None, "physical_estop_source": "not_verified",
            "software_disarm_is_physical_estop": False,
            "ages_s": {"arm": None if self.arm_stamp is None else now - self.arm_stamp,
                       "fsm": None if self.fsm_stamp is None else now - self.fsm_stamp},
            "fsm_diagnostic": self.fsm, "publisher_checks": checks,
            "sbus_publisher_count": sbus_publishers,
            "received_messages": dict(self.received), "errors": dict(self.errors),
            "observed_command_messages": dict(self.command_activity),
            "issues": issues,
            "hardware_write_blockers": ["verified_dual_arm_mode_and_arbitration_adapter_missing",
                                        "physical_estop_state_interface_not_verified",
                                        "bundled_adapters_are_read_only_or_simulated"],
            "motion_publishers_created": [],
        }


def _message_schemas(classes: Mapping[str, Any]) -> dict[str, Any]:
    schemas = {name: dict(cls.get_fields_and_field_types()) for name, cls in classes.items()}
    required = {
        "ros2_bridge_msgs/msg/ArmCtrl": {"header", "mode", "label", "reserved", "ctrl"},
        "ros2_bridge_msgs/msg/MotorCtrl": {"name", "kp", "kd", "pos", "spd", "tor", "cur"},
        "ros2_bridge_msgs/msg/RobotState": {"arm"},
    }
    for name, fields in required.items():
        if not fields.issubset(schemas[name]):
            raise RuntimeError("installed message schema differs from inspected interface: " + name)
    return schemas


def run(seconds: float = 10.) -> dict[str, Any]:
    """Observe one persistent ROS context; no one-second discovery resets."""
    if isinstance(seconds, bool) or not math.isfinite(seconds) or not 5 <= seconds <= 120:
        raise ValueError("seconds must be in 5..120 for graph discovery")
    try:
        import rclpy
        from rclpy.context import Context
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.node import Node
        from rclpy.qos import qos_profile_sensor_data
        from rclpy.type_support import check_for_type_support
        from ros2_bridge_msgs.msg import ArmCtrl, MotorCtrl, RobotState as ROSRobotState
        from brainco_hand_msgs.msg import MotorStatus, SetMotorMulti
        from diagnostic_msgs.msg import DiagnosticStatus
        from geometry_msgs.msg import TwistStamped
        from std_msgs.msg import String
    except ImportError as exc:
        raise RuntimeError("ROS 2 Jazzy/custom interfaces unavailable; run on the robot after sourcing ~/xos/setup.bash") from exc
    classes = {
        "ros2_bridge_msgs/msg/ArmCtrl": ArmCtrl,
        "ros2_bridge_msgs/msg/MotorCtrl": MotorCtrl,
        "ros2_bridge_msgs/msg/RobotState": ROSRobotState,
        "brainco_hand_msgs/msg/MotorStatus": MotorStatus,
        "brainco_hand_msgs/msg/SetMotorMulti": SetMotorMulti,
    }
    try:
        for cls in classes.values():
            check_for_type_support(cls)
    except (ImportError, RuntimeError) as exc:
        raise RuntimeError("ROS message native type support unavailable in this Python environment") from exc
    schemas = _message_schemas(classes)
    context = Context()
    node = executor = None
    rclpy.init(args=[], context=context)
    try:
        node = Node("tg3_dualarm_readonly_" + uuid.uuid4().hex[:8], context=context,
                    enable_rosout=False, start_parameter_services=False, use_global_arguments=False)
        snapshot = SnapshotAccumulator()
        subscriptions = [
            node.create_subscription(ROSRobotState, ARM_STATE_TOPIC, snapshot.arm, qos_profile_sensor_data),
            node.create_subscription(DiagnosticStatus, FSM_STATE_TOPIC, snapshot.mode, qos_profile_sensor_data),
        ]
        def command_callback(topic: str) -> Callable[[Any], None]:
            def received(message: Any) -> None:
                snapshot.command_activity[topic] += 1
            return received
        for typ, topic in ((TwistStamped, "/hric/robot/cmd_vel"), (String, "/hric/robot/fsm_state_cmd")):
            subscriptions.append(node.create_subscription(typ, topic, command_callback(topic), qos_profile_sensor_data))
        executor = SingleThreadedExecutor(context=context)
        executor.add_node(node)
        started = time.monotonic()
        deadline = started + seconds
        while time.monotonic() < deadline and context.ok():
            executor.spin_once(timeout_sec=min(.1, max(0., deadline - time.monotonic())))
        now = time.monotonic()
        publishers = {}
        for topic in CONTROL_TOPICS:
            publishers[topic] = [dict(node=e.node_name, namespace=e.node_namespace,
                                      gid=_gid(e.endpoint_gid), topic_type=e.topic_type)
                                 for e in node.get_publishers_info_by_topic(topic)]
        report = snapshot.report(publishers, sbus_publishers=node.count_publishers(JOY_TOPIC), now=now)
        relevant = set(CONTROL_TOPICS) | {ARM_STATE_TOPIC, FSM_STATE_TOPIC, JOY_TOPIC}
        report["topic_types"] = {name: types for name, types in node.get_topic_names_and_types() if name in relevant}
        own = node.get_publisher_names_and_types_by_node(node.get_name(), node.get_namespace())
        report["motion_publishers_created"] = sorted(name for name, _ in own if name in CONTROL_TOPICS)
        if report["motion_publishers_created"]:
            raise RuntimeError("read-only probe unexpectedly created a motion publisher")
        report.update(duration_s=now - started, message_schemas=schemas,
                      feedback_time_basis="local_monotonic_receipt_time; not an invented message timestamp",
                      command_activity_note="message counts only; no source attribution or promise of future silence")
        return report
    finally:
        if executor is not None:
            executor.shutdown()
        if node is not None:
            node.destroy_node()
        if context.ok():
            context.shutdown()


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--seconds", type=float, default=10.)
    args = parser.parse_args(argv)
    try:
        report = run(args.seconds)
    except (RuntimeError, ValueError) as exc:
        print(json.dumps(dict(passive_only=True, hardware_writes_enabled=False,
                              motion_publishers_created=[], error=str(exc)), ensure_ascii=False, indent=2))
        return 2
    print(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False))
    return 0 if report["observation_ready"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
