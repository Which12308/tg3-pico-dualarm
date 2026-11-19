import copy
import json
from types import SimpleNamespace
import unittest

from tg3_pico_dualarm.tracking import (LatestTracking, Pose, TrackingFrame,
    XRoboToolkitInput, inspect_pico, strict_json)


def payload(sequence=1, session='test-session', timestamp=None):
    pose = {'position': [0., 1., -.4], 'orientation_xyzw': [0., 0., 0., 1.]}
    controller = {'pose': pose, 'connected': True, 'tracked': True, 'grip': 0.,
                  'trigger': 0., 'buttons': [], 'axes': [0., 0.]}
    return dict(session=session, sequence=sequence, source_time_s=sequence*.02 if timestamp is None else timestamp,
                head=copy.deepcopy(pose), left=copy.deepcopy(controller), right=copy.deepcopy(controller),
                focused=True, source_age_s=0., scene_visible=True)


class TrackingTests(unittest.TestCase):
    def setUp(self):
        self.now = 100.
        self.cache = LatestTracking(lambda: self.now)

    def accept(self, frame):
        self.now += .02
        return self.cache.accept(frame)

    def test_head_and_both_controllers_are_independent_measured_poses(self):
        frame = payload()
        frame['left']['pose']['position'] = [-.2, 1.1, -.5]
        frame['right']['pose']['position'] = [.3, .9, -.6]
        parsed = self.accept(frame)
        self.assertEqual(parsed.head.position, (0., 1., -.4))
        self.assertEqual(parsed.left.pose.position, (-.2, 1.1, -.5))
        self.assertEqual(parsed.right.pose.position, (.3, .9, -.6))
        self.assertFalse(parsed.left.release_required)
        self.assertFalse(parsed.right.release_required)
        self.assertEqual(parsed.received_at, self.now)

    def test_first_held_grip_must_release_and_repress(self):
        frame = payload(); frame['left']['grip'] = 1.
        self.assertTrue(self.accept(frame).left.release_required)
        frame = payload(2)
        self.assertFalse(self.accept(frame).left.release_required)
        frame = payload(3); frame['left']['grip'] = 1.
        self.assertFalse(self.accept(frame).left.release_required)

    def test_one_controller_disconnect_does_not_invalidate_other(self):
        self.accept(payload())
        frame = payload(2)
        frame['left'].update(connected=False, tracked=False, pose=None)
        frame['right']['grip'] = 1.
        parsed = self.accept(frame)
        self.assertFalse(parsed.left.tracked)
        self.assertTrue(parsed.left.release_required)
        self.assertTrue(parsed.right.tracked)
        self.assertFalse(parsed.right.release_required)
        frame = payload(3); frame['left']['grip'] = 1.
        self.assertTrue(self.accept(frame).left.release_required)
        self.assertFalse(self.accept(payload(4)).left.release_required)

    def test_all_controls_must_be_neutral_for_rearm(self):
        frame = payload(); frame['right']['trigger'] = .8
        self.assertTrue(self.accept(frame).right.release_required)
        frame = payload(2); frame['right']['buttons'] = ['A']
        self.assertTrue(self.accept(frame).right.release_required)
        frame = payload(3); frame['right']['axes'] = [.5, 0.]
        self.assertTrue(self.accept(frame).right.release_required)
        self.assertFalse(self.accept(payload(4)).right.release_required)

    def test_sample_expiry_clears_tracking_and_requires_new_neutral(self):
        self.accept(payload())
        self.now += .151
        expired = self.cache.sample()
        self.assertFalse(expired.focused)
        self.assertIsNone(expired.head)
        self.assertFalse(expired.right.tracked)
        frame = payload(2); frame['right']['grip'] = 1.
        self.assertTrue(self.accept(frame).right.release_required)
        self.assertFalse(self.accept(payload(3)).right.release_required)

    def test_gap_without_any_sample_call_still_requires_release(self):
        self.accept(payload())
        self.now += .2
        frame = payload(2); frame['left']['grip'] = .8
        self.assertTrue(self.accept(frame).left.release_required)

    def test_source_age_is_added_to_receiving_clock_age(self):
        frame = payload(timestamp=4000000.)
        frame['source_age_s'] = .1
        self.accept(frame)
        self.now += .051
        self.assertFalse(self.cache.sample().right.tracked)

    def test_server_transport_age_reduces_remaining_budget_without_mutating_payload(self):
        frame = payload(); original = copy.deepcopy(frame)
        parsed = self.cache.accept(frame, additional_age_s=.119)
        self.assertAlmostEqual(parsed.source_age_s, .119)
        self.assertEqual(frame, original)
        self.assertEqual(parsed.source_time_s, original['source_time_s'])
        self.now += .032
        self.assertFalse(self.cache.sample().right.tracked)
        self.assertTrue(self.cache.sample().right.release_required)

    def test_source_and_server_age_cannot_exceed_acceptance_budget(self):
        frame = payload(); frame['source_age_s'] = .02
        self.assertIsNone(self.cache.accept(frame, additional_age_s=.11))
        self.assertIn('expired_source_and_transport_age', self.cache.last_error)

    def test_replay_or_nonadvancing_source_stamp_revokes_old_frame(self):
        self.accept(payload())
        self.assertIsNone(self.accept(payload()))
        self.assertFalse(self.cache.sample().right.tracked)
        self.assertIn('replayed', self.cache.last_error)

    def test_retired_session_cannot_replace_new_stream(self):
        self.accept(payload())
        self.accept(payload(1, session='new'))
        self.assertIsNone(self.accept(payload(2)))
        self.assertIn('retired', self.cache.last_error)

    def test_new_session_held_hand_needs_its_own_release(self):
        self.accept(payload())
        frame = payload(1, session='new'); frame['right']['grip'] = 1.
        parsed = self.accept(frame)
        self.assertTrue(parsed.right.release_required)
        self.assertFalse(parsed.left.release_required)

    def test_replaced_source_requires_release_even_if_loss_frame_was_not_delivered(self):
        frame = payload(); frame['left']['source_id'] = 'left-original'
        self.accept(frame)
        frame = payload(2); frame['left'].update(source_id='left-replacement', grip=1.)
        frame['right']['grip'] = 1.
        parsed = self.accept(frame)
        self.assertTrue(parsed.left.release_required)
        self.assertFalse(parsed.right.release_required)

    def test_focus_loss_revokes_both_and_rearm_is_not_synthetic(self):
        self.accept(payload())
        frame = payload(2); frame['focused'] = False
        self.assertFalse(self.accept(frame).right.tracked)
        frame = payload(3); frame['right']['grip'] = 1.
        self.assertTrue(self.accept(frame).right.release_required)

    def test_invalid_values_never_leave_a_previously_valid_target(self):
        for mutate in (lambda d: d['right']['pose']['position'].__setitem__(0, float('nan')),
                       lambda d: d['left'].__setitem__('tracked', 1),
                       lambda d: d.__setitem__('source_age_s', .121),
                       lambda d: d['right']['pose'].__setitem__('orientation_xyzw', [0, 0, 0, 0]),
                       lambda d: d.__setitem__('sequence', True),
                       lambda d: d.__setitem__('received_at', 0.),
                       lambda d: d['left'].__setitem__('buttons', ['A', 'A'])):
            self.cache = LatestTracking(lambda: self.now)
            self.accept(payload())
            bad = payload(2); mutate(bad)
            self.assertIsNone(self.accept(bad))
            self.assertFalse(self.cache.sample().left.tracked)
            self.assertFalse(self.cache.sample().right.tracked)

    def test_wire_roundtrip_does_not_accept_a_client_receive_clock(self):
        frame = self.accept(payload())
        wire = json.loads(json.dumps(frame.to_input_payload()))
        parsed = TrackingFrame.from_payload(wire, 123.)
        self.assertEqual(parsed.received_at, 123.)
        self.assertEqual(parsed.head, frame.head)
        with self.assertRaises(ValueError):
            TrackingFrame.from_payload(frame.to_payload(), 123.)

    def test_json_duplicates_and_nonfinite_tokens_are_rejected(self):
        for raw in ('{"x":1,"x":2}', '{"x":NaN}', '{"x":Infinity}'):
            with self.assertRaises(ValueError): strict_json(raw)

    def test_no_mutation_of_accepted_caller_data(self):
        frame = payload()
        parsed = self.accept(frame)
        frame['right']['pose']['position'][0] = 4.
        self.assertEqual(parsed.right.pose.position[0], 0.)
        self.assertEqual(self.cache.sample(), parsed)


class NativeAndAdbTests(unittest.TestCase):
    def sdk(self):
        sdk = SimpleNamespace(timestamp=100, init=lambda: None, close=lambda: None)
        sdk.get_time_stamp_ns = lambda: sdk.timestamp
        sdk.get_headset_pose = lambda: [0., 1., 0., 0., 0., 0., 1.]
        for side in ('left', 'right'):
            setattr(sdk, f'get_{side}_controller_pose', lambda: [0., 1., -.4, 0., 0., 0., 1.])
            setattr(sdk, f'get_{side}_grip', lambda: 0.)
            setattr(sdk, f'get_{side}_trigger', lambda: .3)
            setattr(sdk, f'get_{side}_axis', lambda: [0., 0.])
        for button in 'ABXY': setattr(sdk, f'get_{button}_button', lambda: False)
        return sdk

    def test_numeric_native_pose_is_not_tracking_confidence(self):
        sdk = self.sdk(); adapter = XRoboToolkitInput(sdk, clock=lambda: 1.)
        adapter.poll(); sdk.timestamp += 1
        frame = adapter.poll()
        self.assertFalse(frame.left.tracked)
        self.assertFalse(frame.right.connected)
        self.assertIsNone(frame.head)
        self.assertIn('no_verified', adapter.last_error)

    def test_native_requires_timestamp_advancement_and_individual_validity(self):
        now = [1.]; sdk = self.sdk()
        adapter = XRoboToolkitInput(sdk, clock=lambda: now[0],
            validity_provider=lambda: {'head': True, 'left': True, 'right': False})
        self.assertFalse(adapter.poll().left.tracked)
        sdk.timestamp += 1
        frame = adapter.poll()
        self.assertTrue(frame.left.tracked)
        self.assertFalse(frame.right.tracked)
        self.assertIsNotNone(frame.head)
        now[0] += .121
        self.assertFalse(adapter.poll().left.tracked)

    def test_adb_inspection_only_uses_devices_and_getprop(self):
        calls = []
        def run(args, **kwargs):
            calls.append(args)
            if args[-1] == 'devices': value = 'List of devices attached\nserial device\n'
            elif args[-1] == 'ro.product.model': value = 'A9210'
            else: value = 'test'
            return SimpleNamespace(stdout=value)
        result = inspect_pico('adb', run=run)
        self.assertTrue(result['pico_4_ultra_confirmed'])
        self.assertFalse(result['device_modified'])
        self.assertEqual(len(calls), 5)
        self.assertTrue(all('getprop' in command or command[-1] == 'devices' for command in calls))


if __name__ == '__main__':
    unittest.main()
