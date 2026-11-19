from dataclasses import replace
import math
from types import SimpleNamespace as NS
import unittest

from tg3_pico_dualarm.input_validation import validate_frame, validate_controller
from tg3_pico_dualarm.tracking import ControllerState, Pose, TrackingFrame


def valid_frame():
    pose = Pose((.1, 1., -.3), (0., 0., 0., 1.))
    hand = ControllerState(pose, True, True, 0., 0., (), False)
    return TrackingFrame("direct-public-api", 1, 987654., 10., pose, hand, hand,
                         source_age_s=.01, scene_visible=True)


class InputValidationTests(unittest.TestCase):
    def test_valid_dataclass_preserves_independent_source_clock_and_is_not_modified(self):
        frame = valid_frame()
        self.assertIsNone(validate_frame(frame, 10.02))
        self.assertEqual(frame.source_time_s, 987654.)
        self.assertEqual(frame.received_at, 10.)
        self.assertTrue(frame.right.connected)

    def test_nonfinite_or_out_of_range_analog_cannot_become_an_active_grip(self):
        frame = valid_frame()
        for field in ("grip", "trigger"):
            for value in (math.nan, math.inf, -math.inf, -.01, 1.01, True, "1", 10**400):
                broken = replace(frame, right=replace(frame.right, **{field: value}))
                with self.subTest(field=field, value=value), self.assertRaises(ValueError):
                    validate_frame(broken, 10.02)

    def test_strict_boolean_validity_flags_and_connection_consistency(self):
        frame = valid_frame()
        for field in ("connected", "tracked", "release_required"):
            with self.subTest(field=field), self.assertRaises(ValueError):
                validate_frame(replace(frame, left=replace(frame.left, **{field: 1})), 10.)
        for field in ("focused", "scene_visible"):
            with self.subTest(field=field), self.assertRaises(ValueError):
                validate_frame(replace(frame, **{field: 1}), 10.)
        with self.assertRaises(ValueError):
            validate_controller(replace(frame.left, connected=False))
        with self.assertRaises(ValueError):
            validate_controller(replace(frame.left, pose=None))

    def test_missing_invalid_or_cross_shaped_axes_and_button_names_are_rejected(self):
        frame = valid_frame()
        for axes in ((0.,), (0., 0., 0.), ((0., 0.), (0., 0.)), (0., math.nan), (1.01, 0.), "00"):
            with self.subTest(axes=axes), self.assertRaises(ValueError):
                validate_frame(replace(frame, right=replace(frame.right, axes=axes)), 10.)
        for buttons in (["A"], ("A", "A"), ("",), (1,), ("X" * 33,)):
            with self.subTest(buttons=buttons), self.assertRaises(ValueError):
                validate_frame(replace(frame, right=replace(frame.right, buttons=buttons)), 10.)
        without_axes = NS(**{key: value for key, value in vars(frame.left).items() if key != "axes"})
        with self.assertRaises(ValueError):
            validate_controller(without_axes)

    def test_sequence_timestamps_and_total_age_are_validated_without_cross_host_clock_math(self):
        frame = valid_frame()
        invalid = [replace(frame, sequence=value) for value in (-1, True, 1.5, 2**53)]
        invalid += [replace(frame, **{field: value}) for field, value in
                    (("source_time_s", math.nan), ("source_time_s", -1.), ("received_at", math.inf),
                     ("received_at", 10.1), ("source_age_s", -.01), ("source_age_s", .121),
                     ("session", ""), ("source", "unverified"))]
        for item in invalid:
            with self.subTest(item=item), self.assertRaises(ValueError):
                validate_frame(item, 10.02)
        with self.assertRaises(ValueError):
            validate_frame(replace(frame, source_age_s=.12), 10.04)
        self.assertIsNone(validate_frame(replace(frame, source_time_s=.001), 10.02))

    def test_pose_shapes_finite_coordinates_and_unit_quaternions_are_enforced(self):
        frame = valid_frame()
        poses = [Pose((0., 0.), (0., 0., 0., 1.)), Pose((0., math.nan, 0.), (0., 0., 0., 1.)),
                 Pose((21., 0., 0.), (0., 0., 0., 1.)), Pose((0., 0., 0.), (0., 0., 0., 0.)),
                 Pose((0., 0., 0.), (0., 0., math.inf, 1.)), Pose((True, 0., 0.), (0., 0., 0., 1.))]
        for pose in poses:
            for item in (replace(frame, head=pose), replace(frame, right=replace(frame.right, pose=pose))):
                with self.subTest(pose=pose), self.assertRaises(ValueError):
                    validate_frame(item, 10.)
        self.assertIsNone(validate_frame(replace(frame, head=None), 10.))

    def test_benign_tracking_loss_remains_a_valid_inactive_sample(self):
        frame = valid_frame()
        lost = replace(frame, focused=False, head=None,
                       left=replace(frame.left, pose=None, connected=False, tracked=False, release_required=True),
                       right=replace(frame.right, pose=None, tracked=False, release_required=True))
        self.assertIsNone(validate_frame(lost, 10.))


if __name__ == "__main__":
    unittest.main()
