"""Official ArmCtrl message formatting only; no hardware output or authority.

Checked against the manufacturer's public single-joint source, sampled schema,
and SDK 26.06.x PDF section 4.1.3.1 (pages 24-26). The PDF documents the dual-arm
interface; encoding fourteen items does NOT confirm actual control authority.

Official hybrid example sets mode=1, label=0, header.frame_id='arm', and
name/kp/kd/pos/spd/tor. It does not set a reserved takeover magic value.
The PDF also permits external caller labels 101-255 and documents an optional
zeroing-only joint_ids field. Zeroing and reserved takeover values are refused.
"""
from __future__ import annotations

import math
from numbers import Real

from .model import JOINT_NAMES, MOTOR_IDS
from .robot import JointCommand


OFFICIAL_SOURCE = "https://github.com/Open-X-Humanoid/xhumanoid_sdk/blob/main/single_joint_control/python/single_joint_control/cmd_publisher.py"
OFFICIAL_SCHEMA = "https://github.com/Open-X-Humanoid/xhumanoid_sdk/blob/main/single_joint_control/README.md#消息格式"
OFFICIAL_SDK_DOCUMENT = "具身天工3.0 SDK文档 （26.06.x）.pdf"
SCHEMA_CHECKED_DATE = "2026-11-15"


class ProtocolError(ValueError):
    pass


def _fields(cls) -> dict[str, str]:
    try:
        fields = cls.get_fields_and_field_types()
        if not isinstance(fields, dict):
            raise TypeError
        return {str(key): str(value).replace(" ", "").replace("/msg/", "/") for key, value in fields.items()}
    except (AttributeError, TypeError, ValueError) as exc:
        raise ProtocolError("installed message class lacks ROS field introspection") from exc


def inspect_schemas(arm_type, motor_type) -> dict:
    arm, motor = _fields(arm_type), _fields(motor_type)
    if set(arm) != {"header", "mode", "label", "reserved", "ctrl"}:
        raise ProtocolError("ArmCtrl fields differ from the inspected official schema")
    if arm["header"] != "std_msgs/Header" or any(arm[key] != "uint8" for key in ("mode", "label", "reserved")):
        raise ProtocolError("ArmCtrl header/mode/label/reserved types differ from the official schema")
    if arm["ctrl"] not in ("sequence<ros2_bridge_msgs/MotorCtrl>", "ros2_bridge_msgs/MotorCtrl[]"):
        raise ProtocolError("ArmCtrl.ctrl must be the documented unbounded MotorCtrl array")
    base_motor_fields = {"name", "kp", "kd", "pos", "spd", "tor", "cur"}
    if set(motor) not in (base_motor_fields, base_motor_fields | {"joint_ids"}) or motor.get("name") != "uint16":
        raise ProtocolError("MotorCtrl fields/ID type differ from the two known schemas")
    if "joint_ids" in motor and motor["joint_ids"] != "string":
        raise ProtocolError("MotorCtrl.joint_ids must have the documented string type")
    if any(motor[key] not in ("double", "float64") for key in ("kp", "kd", "pos", "spd", "tor", "cur")):
        raise ProtocolError("MotorCtrl numeric fields must have the documented float64 type")
    return dict(checked_date=SCHEMA_CHECKED_DATE, source=OFFICIAL_SOURCE, schema_source=OFFICIAL_SCHEMA,
                sdk_document=OFFICIAL_SDK_DOCUMENT, sdk_section="4.1.3.1", sdk_pages=[24, 25, 26],
                arm=arm, motor=motor, motor_schema_variant=("sdk_with_joint_ids" if "joint_ids" in motor else "sampled_without_joint_ids"),
                motor_ids=list(MOTOR_IDS), joint_names=list(JOINT_NAMES),
                formatter_only=True, hardware_writes_enabled=False,
                dual_arm_interface_documented=True, dual_arm_mode_contract_verified=False,
                actual_control_authority_verified=False, reserved_magic_used=False,
                zeroing_enabled=False)


def _gains(values, name: str) -> tuple[float, ...]:
    try:
        result = tuple(values)
    except TypeError as exc:
        raise ProtocolError(name + " must contain fourteen explicit gains") from exc
    if len(result) != 14:
        raise ProtocolError(name + " must contain fourteen explicit gains")
    if any(isinstance(value, bool) or not isinstance(value, Real) for value in result):
        raise ProtocolError(name + " gains must be finite nonnegative numbers")
    try:
        result = tuple(float(value) for value in result)
    except (OverflowError, ValueError) as exc:
        raise ProtocolError(name + " gains must be finite nonnegative numbers") from exc
    if any(not math.isfinite(value) or value < 0 for value in result):
        raise ProtocolError(name + " gains must be finite nonnegative numbers")
    return result


class ArmCommandFormatter:
    """Produce an in-memory official message; this object cannot publish it.

    Gains must be supplied explicitly by the caller. Neither the official
    single-joint example's gains nor archived factory gains are generalized to
    a fourteen-joint operating policy. ROS time is supplied separately from the
    command's monotonic freshness stamp; no cross-clock conversion is invented.
    """

    hardware_writes_enabled = False
    formatter_only = True

    def __init__(self, arm_type=None, motor_type=None):
        if (arm_type is None) != (motor_type is None):
            raise ProtocolError("provide both installed message classes or neither")
        if arm_type is None:
            try:
                from ros2_bridge_msgs.msg import ArmCtrl, MotorCtrl
            except ImportError as exc:
                raise RuntimeError("installed ros2_bridge_msgs is required for official message formatting") from exc
            arm_type, motor_type = ArmCtrl, MotorCtrl
        self.schema_report = inspect_schemas(arm_type, motor_type)
        self._has_joint_ids = "joint_ids" in self.schema_report["motor"]
        self._arm_type, self._motor_type = arm_type, motor_type

    def encode(self, command: JointCommand, *, kp, kd, ros_stamp, label=0):
        """Format hybrid mode only; label 0 preserves the legacy omitted value.

        Explicit external caller labels may be 101-255. Labels 1-100 belong to
        internal callers and are never accepted here.
        """
        if not isinstance(command, JointCommand):
            raise ProtocolError("a validated fourteen-joint JointCommand is required")
        if type(label) is not int or not (label == 0 or 101 <= label <= 255):
            raise ProtocolError("label must be 0 (legacy default) or an external caller integer 101-255")
        kp, kd = _gains(kp, "kp"), _gains(kd, "kd")
        try:
            sec, nanosec = ros_stamp.sec, ros_stamp.nanosec
        except AttributeError as exc:
            raise ProtocolError("provide a ROS timestamp with sec and nanosec; monotonic time is not ROS time") from exc
        if type(sec) is not int or not 0 <= sec <= 2**31-1 or type(nanosec) is not int or not 0 <= nanosec < 1_000_000_000:
            raise ProtocolError("invalid ROS timestamp")
        message = self._arm_type()
        # No setter or caller argument exists for undocumented reserved magic.
        if type(message.reserved) is not int or message.reserved != 0:
            raise ProtocolError("nonzero reserved default is not established by the official example")
        message.header.stamp.sec = sec
        message.header.stamp.nanosec = nanosec
        message.header.frame_id = "arm"
        message.mode = 1
        message.label = label
        for index, motor_id in enumerate(MOTOR_IDS):
            motor = self._motor_type()
            if self._has_joint_ids:
                # This field only belongs to SDK zeroing mode, which this
                # hybrid formatter never enables. Reject a polluted default.
                joint_ids = getattr(motor, "joint_ids", None)
                if type(joint_ids) is not str or joint_ids != "":
                    raise ProtocolError("MotorCtrl.joint_ids default must be an empty string; zeroing is unsupported")
            motor.name = motor_id
            motor.kp, motor.kd = kp[index], kd[index]
            motor.pos, motor.spd = command.positions[index], command.velocities[index]
            motor.tor = 0.
            # cur is unused by the documented hybrid control law. Preserve the
            # installed message default, matching the public hybrid example.
            message.ctrl.append(motor)
        return message
