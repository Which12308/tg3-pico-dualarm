import math
from types import SimpleNamespace as NS
import unittest

from tg3_pico_dualarm.model import JOINT_NAMES, MOTOR_IDS
from tg3_pico_dualarm.robot import JointCommand
from tg3_pico_dualarm.tiangong_protocol import ArmCommandFormatter, ProtocolError, inspect_schemas


class DocumentedArmMessage:
    """Explicit test double for the public schema, not an installed ROS binding."""
    @classmethod
    def get_fields_and_field_types(cls):
        return dict(header="std_msgs/Header", mode="uint8", label="uint8", reserved="uint8",
                    ctrl="sequence<ros2_bridge_msgs/MotorCtrl>")
    def __init__(self):
        self.header = NS(stamp=NS(sec=0, nanosec=0), frame_id="")
        self.mode, self.label, self.reserved = 0, 0, 0
        self.ctrl = []


class DocumentedMotorMessage:
    @classmethod
    def get_fields_and_field_types(cls):
        return dict(name="uint16", **dict.fromkeys(("kp", "kd", "pos", "spd", "tor", "cur"), "double"))
    def __init__(self):
        self.cur = 0.


class SdkMotorMessage(DocumentedMotorMessage):
    @classmethod
    def get_fields_and_field_types(cls):
        return dict(super().get_fields_and_field_types(), joint_ids="string")

    def __init__(self):
        super().__init__()
        self.joint_ids = ""


class ProtocolTests(unittest.TestCase):
    def setUp(self):
        self.formatter = ArmCommandFormatter(DocumentedArmMessage, DocumentedMotorMessage)
        self.command = JointCommand(tuple(i*.01 for i in range(14)), tuple(-i*.001 for i in range(14)), 100.)
        self.options = dict(kp=tuple(range(14)), kd=(.1,)*14, ros_stamp=NS(sec=50, nanosec=123))

    def test_fourteen_motor_formatter_uses_official_fields_and_archived_id_order_only(self):
        message = self.formatter.encode(self.command, **self.options)
        self.assertEqual((message.mode, message.label, message.reserved), (1, 0, 0))
        self.assertEqual(message.header.frame_id, "arm")
        self.assertEqual((message.header.stamp.sec, message.header.stamp.nanosec), (50, 123))
        self.assertEqual([motor.name for motor in message.ctrl], list(MOTOR_IDS))
        self.assertEqual([motor.pos for motor in message.ctrl], list(self.command.positions))
        self.assertEqual([motor.spd for motor in message.ctrl], list(self.command.velocities))
        self.assertEqual([motor.kp for motor in message.ctrl], list(range(14)))
        self.assertTrue(all(motor.tor == motor.cur == 0 for motor in message.ctrl))
        self.assertEqual(self.formatter.schema_report["joint_names"], list(JOINT_NAMES))
        self.assertFalse(self.formatter.schema_report["dual_arm_mode_contract_verified"])
        self.assertFalse(hasattr(self.formatter, "publish"))

    def test_external_caller_labels_are_explicit_and_system_labels_are_rejected(self):
        for label in (0, 101, 200, 255):
            with self.subTest(label=label):
                message = self.formatter.encode(self.command, **self.options, label=label)
                self.assertEqual(message.label, label)
                self.assertEqual((message.mode, message.reserved), (1, 0))
        for label in (*range(1, 101), -1, 256, True, False, 101., "101", None):
            with self.subTest(label=label), self.assertRaisesRegex(ProtocolError, "external caller"):
                self.formatter.encode(self.command, **self.options, label=label)

    def test_both_known_motor_schemas_encode_hybrid_without_zeroing(self):
        for motor_type, variant in ((DocumentedMotorMessage, "sampled_without_joint_ids"),
                                    (SdkMotorMessage, "sdk_with_joint_ids")):
            with self.subTest(variant=variant):
                formatter = ArmCommandFormatter(DocumentedArmMessage, motor_type)
                message = formatter.encode(self.command, **self.options, label=101)
                self.assertEqual(len(message.ctrl), 14)
                self.assertEqual((message.mode, message.label, message.reserved), (1, 101, 0))
                self.assertEqual([motor.name for motor in message.ctrl], list(MOTOR_IDS))
                self.assertTrue(all(getattr(motor, "joint_ids", "") == "" for motor in message.ctrl))
                self.assertEqual(formatter.schema_report["motor_schema_variant"], variant)
                self.assertTrue(formatter.schema_report["dual_arm_interface_documented"])
                self.assertFalse(formatter.schema_report["actual_control_authority_verified"])
                self.assertFalse(formatter.schema_report["zeroing_enabled"])
                self.assertFalse(formatter.hardware_writes_enabled)
                self.assertFalse(hasattr(formatter, "publish"))

    def test_zeroing_field_must_exist_and_have_an_empty_string_default(self):
        for default in ("11", "11,21", None, 0, [], False):
            class NonemptyZeroingDefault(SdkMotorMessage):
                def __init__(self):
                    super().__init__()
                    self.joint_ids = default
            with self.subTest(default=default), self.assertRaisesRegex(ProtocolError, "empty string"):
                ArmCommandFormatter(DocumentedArmMessage, NonemptyZeroingDefault).encode(self.command, **self.options)

        class MissingZeroingDefault(SdkMotorMessage):
            def __init__(self):
                DocumentedMotorMessage.__init__(self)
        with self.assertRaisesRegex(ProtocolError, "empty string"):
            ArmCommandFormatter(DocumentedArmMessage, MissingZeroingDefault).encode(self.command, **self.options)

    def test_motor_schema_field_or_type_drift_is_rejected(self):
        for base in (DocumentedMotorMessage, SdkMotorMessage):
            mutations = (dict(extra="float64"), dict(name="uint8"), dict(cur="float32"),
                         dict(joint_ids="wstring"), dict(joint_ids="sequence<string>"))
            for mutation in mutations:
                class ChangedMotor(base):
                    @classmethod
                    def get_fields_and_field_types(cls):
                        return dict(super().get_fields_and_field_types(), **mutation)
                with self.subTest(base=base.__name__, mutation=mutation), self.assertRaises(ProtocolError):
                    inspect_schemas(DocumentedArmMessage, ChangedMotor)

            for missing in base.get_fields_and_field_types():
                if missing == "joint_ids":
                    continue  # Removing this field produces the other known schema.
                class IncompleteMotor(base):
                    @classmethod
                    def get_fields_and_field_types(cls):
                        return {key: value for key, value in super().get_fields_and_field_types().items() if key != missing}
                with self.subTest(base=base.__name__, missing=missing), self.assertRaises(ProtocolError):
                    inspect_schemas(DocumentedArmMessage, IncompleteMotor)

    def test_reserved_takeover_magic_is_never_inserted_and_nonzero_default_is_rejected(self):
        class UnknownDefault(DocumentedArmMessage):
            def __init__(self):
                super().__init__()
                self.reserved = 165
        formatter = ArmCommandFormatter(UnknownDefault, DocumentedMotorMessage)
        with self.assertRaisesRegex(ProtocolError, "reserved default"):
            formatter.encode(self.command, **self.options)

    def test_installed_schema_drift_or_bounded_array_is_rejected(self):
        for bad in (dict(ctrl="sequence<ros2_bridge_msgs/MotorCtrl,7>"), dict(mode="int32"),
                    dict(header="unknown_msgs/Header")):
            class Changed(DocumentedArmMessage):
                @classmethod
                def get_fields_and_field_types(cls):
                    return dict(super().get_fields_and_field_types(), **bad)
            with self.subTest(bad=bad), self.assertRaises(ProtocolError):
                inspect_schemas(Changed, DocumentedMotorMessage)

    def test_gains_are_explicit_validated_and_never_reused_from_single_joint_example(self):
        for gains in ((1.,)*7, (math.nan,)+(1.,)*13, (-1.,)+(1.,)*13, (True,)+(1.,)*13):
            with self.subTest(gains=gains), self.assertRaises(ProtocolError):
                self.formatter.encode(self.command, **dict(self.options, kp=gains))
        with self.assertRaises(TypeError):
            self.formatter.encode(self.command, ros_stamp=self.options["ros_stamp"])

    def test_ros_and_host_monotonic_time_are_distinct_and_no_authority_is_granted(self):
        for stamp in (100., NS(sec=True, nanosec=0), NS(sec=1, nanosec=1_000_000_000)):
            with self.subTest(stamp=stamp), self.assertRaises(ProtocolError):
                self.formatter.encode(self.command, **dict(self.options, ros_stamp=stamp))
        self.assertTrue(self.formatter.formatter_only)
        self.assertFalse(self.formatter.hardware_writes_enabled)


if __name__ == "__main__":
    unittest.main()
