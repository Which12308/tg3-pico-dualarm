"""Timestamped head/controller input. This module cannot send robot commands.

Coordinates are the input device's tracking space, in metres and XYZW
quaternions. Robot coordinate conversion belongs in the controller adapter.
"""
from __future__ import annotations

from collections import Counter
from dataclasses import asdict, dataclass, replace
import importlib
import json
import math
from pathlib import Path
import subprocess
import threading
import time
from typing import Callable, Mapping

from .input_policy import NORMAL

MAX_AGE_S = .150
MAX_SOURCE_AGE_S = .120


def _number(value, name: str) -> float:
    if type(value) not in (int, float) or not math.isfinite(value):
        raise ValueError(f'{name} must be a finite number')
    return float(value)


def _vector(value, size: int, name: str, bound: float | None = None) -> tuple[float, ...]:
    if not isinstance(value, (list, tuple)) or len(value) != size:
        raise ValueError(f'{name} must have {size} values')
    result = tuple(_number(v, name) for v in value)
    if bound is not None and any(abs(v) > bound for v in result):
        raise ValueError(f'{name} outside input range')
    return result


@dataclass(frozen=True)
class Pose:
    position: tuple[float, float, float]
    orientation_xyzw: tuple[float, float, float, float]

    @classmethod
    def from_payload(cls, value) -> 'Pose':
        if not isinstance(value, dict) or set(value) != {'position', 'orientation_xyzw'}:
            raise ValueError('pose requires position and orientation_xyzw')
        position = _vector(value['position'], 3, 'position', 20.)
        orientation = _vector(value['orientation_xyzw'], 4, 'orientation')
        norm = math.sqrt(sum(v*v for v in orientation))
        if abs(norm - 1.) > .02:
            raise ValueError('orientation must be a unit XYZW quaternion')
        return cls(position, tuple(v/norm for v in orientation))


@dataclass(frozen=True)
class ControllerState:
    pose: Pose | None = None
    connected: bool = False
    tracked: bool = False
    grip: float = 0.
    trigger: float = 0.
    buttons: tuple[str, ...] = ()
    release_required: bool = True
    axes: tuple[float, float] = (0., 0.)
    source_id: str = ''

    @property
    def neutral(self) -> bool:
        return (self.grip <= .1 and self.trigger <= .1 and not self.buttons
                and all(abs(v) <= .2 for v in self.axes))

    @classmethod
    def from_payload(cls, value) -> 'ControllerState':
        required = {'pose', 'connected', 'tracked', 'grip', 'trigger', 'buttons'}
        if not isinstance(value, dict) or not required <= value.keys() or set(value) - required - {'axes', 'source_id'}:
            raise ValueError('invalid controller fields')
        connected, tracked = value['connected'], value['tracked']
        if type(connected) is not bool or type(tracked) is not bool:
            raise ValueError('controller validity must be boolean')
        if tracked and not connected:
            raise ValueError('a tracked controller must be connected')
        grip = _number(value['grip'], 'grip')
        trigger = _number(value['trigger'], 'trigger')
        if not (0. <= grip <= 1. and 0. <= trigger <= 1.):
            raise ValueError('grip and trigger must be in [0, 1]')
        buttons = value['buttons']
        if (not isinstance(buttons, list) or len(buttons) > 16
                or any(not isinstance(b, str) or not 1 <= len(b) <= 32 for b in buttons)
                or len(set(buttons)) != len(buttons)):
            raise ValueError('invalid pressed buttons')
        axes = _vector(value.get('axes', [0., 0.]), 2, 'axes', 1.)
        source_id = value.get('source_id', '')
        if not isinstance(source_id, str) or len(source_id) > 128:
            raise ValueError('invalid controller source_id')
        pose = Pose.from_payload(value['pose']) if value['pose'] is not None else None
        if tracked and pose is None:
            raise ValueError('tracked controller has no pose')
        return cls(pose if tracked else None, connected, tracked, grip, trigger, tuple(buttons), True, axes, source_id)


@dataclass(frozen=True)
class TrackingFrame:
    session: str
    sequence: int
    source_time_s: float
    received_at: float
    head: Pose | None
    left: ControllerState
    right: ControllerState
    focused: bool = True
    source: str = 'webxr'
    source_age_s: float = 0.
    scene_visible: bool = False

    @classmethod
    def from_payload(cls, value, received_at: float, policy=NORMAL) -> 'TrackingFrame':
        required = {'session', 'sequence', 'source_time_s', 'head', 'left', 'right', 'focused', 'source_age_s'}
        optional = {'source', 'scene_visible'}
        if not isinstance(value, dict) or not required <= value.keys() or set(value)-required-optional:
            raise ValueError('unknown or missing tracking fields')
        session, sequence = value['session'], value['sequence']
        if not isinstance(session, str) or not 1 <= len(session) <= 128:
            raise ValueError('invalid session')
        if type(sequence) is not int or not 0 <= sequence <= 2**53-1:
            raise ValueError('invalid sequence')
        source_time = _number(value['source_time_s'], 'source_time_s')
        age = _number(value['source_age_s'], 'source_age_s')
        if source_time < 0 or not 0 <= age <= policy.source_age_s:
            raise ValueError('invalid or expired source timestamp')
        focused, scene_visible = value['focused'], value.get('scene_visible', False)
        if type(focused) is not bool or type(scene_visible) is not bool:
            raise ValueError('focus and scene visibility must be boolean')
        source = value.get('source', 'webxr')
        if source not in ('webxr', 'xrobotoolkit', 'openxr'):
            raise ValueError('unknown tracking source')
        head = Pose.from_payload(value['head']) if value['head'] is not None else None
        left, right = (ControllerState.from_payload(value[side]) for side in ('left', 'right'))
        if not focused:
            head = None
            left, right = (replace(c, pose=None, tracked=False) for c in (left, right))
        return cls(session, sequence, source_time, _number(received_at, 'received_at'),
                   head, left, right, focused, source, age, scene_visible)

    def to_payload(self) -> dict:
        return asdict(self)

    def to_input_payload(self) -> dict:
        """Wire fields accepted by LatestTracking, without trusted receive data."""
        value = asdict(self)
        value.pop('received_at')
        for side in ('left', 'right'):
            value[side].pop('release_required')
            value[side]['buttons'] = list(value[side]['buttons'])
        return value


class LatestTracking:
    """One latest frame, no action queue; clocks from different hosts never mix.

    source_time_s is checked only against previous timestamps in that session.
    Expiry uses the receiving monotonic clock plus the declared frame age.
    Browser nonce deadlines are enforced separately by the loopback receiver.
    Each hand requires an observed fresh neutral input after its own loss.
    """
    def __init__(self, clock: Callable[[], float] = time.monotonic, max_age_s: float | None = None, *, policy=NORMAL):
        self.policy = policy
        self.clock, self.max_age_s = clock, policy.total_age_s if max_age_s is None else min(max_age_s, policy.total_age_s)
        self.lock = threading.RLock()
        self.latest: TrackingFrame | None = None
        self.retired: set[str] = set()
        self.release_required = {'left': True, 'right': True}
        self.counters = Counter()
        self.last_error = 'waiting_for_tracking'

    def invalidate(self, reason: str = 'input_invalid') -> None:
        with self.lock:
            if reason=='tracking_expired' and self.last_error!=reason:self.counters['expired']+=1
            self.release_required = {'left': True, 'right': True}
            if self.latest is not None:
                self.latest = replace(self.latest, head=None, focused=False,
                    left=replace(self.latest.left, pose=None, tracked=False, release_required=True),
                    right=replace(self.latest.right, pose=None, tracked=False, release_required=True))
            self.last_error = reason

    def accept(self, payload, now: float | None = None, *, additional_age_s: float = 0.) -> TrackingFrame | None:
        with self.lock:
            now = self.clock() if now is None else now
            try:
                frame = TrackingFrame.from_payload(payload, now, self.policy)
                transport_age = _number(additional_age_s, 'additional_age_s')
                if transport_age < 0 or frame.source_age_s+transport_age > min(self.policy.source_age_s, self.max_age_s):
                    raise ValueError('expired_source_and_transport_age')
                # A server-issued nonce bounds the transport age using this
                # receiving clock only. Preserve source_time_s as reported;
                # never subtract a headset clock from a workstation clock.
                frame = replace(frame, source_age_s=frame.source_age_s+transport_age)
                old = self.latest
                if frame.session in self.retired:
                    raise ValueError('retired_session')
                if old is not None:
                    if now < old.received_at:
                        raise ValueError('receiving_clock_moved_backwards')
                    if frame.session == old.session:
                        if frame.sequence <= old.sequence or frame.source_time_s <= old.source_time_s:
                            raise ValueError('replayed_or_nonadvancing_frame')
                        if now-old.received_at+old.source_age_s > self.max_age_s:
                            self.release_required = {'left': True, 'right': True}
                    else:
                        if len(self.retired) >= 64:
                            raise ValueError('session_budget_exhausted_restart_receiver')
                        self.retired.add(old.session)
                        self.release_required = {'left': True, 'right': True}
                states = {}
                for side in ('left', 'right'):
                    controller = getattr(frame, side)
                    if old is not None and controller.source_id != getattr(old, side).source_id:
                        self.release_required[side] = True
                    if not frame.focused or not controller.connected or not controller.tracked:
                        self.release_required[side] = True
                    elif controller.neutral:
                        self.release_required[side] = False
                    states[side] = replace(controller, release_required=self.release_required[side])
                self.latest = replace(frame, **states)
                self.counters['accepted'] += 1
                self.last_error = ''
                return self.latest
            except (TypeError, ValueError) as error:
                self.counters['rejected'] += 1
                self.invalidate(str(error))
                return None

    def sample(self, now: float | None = None) -> TrackingFrame | None:
        with self.lock:
            now = self.clock() if now is None else now
            if self.latest is None:
                return None
            age = now-self.latest.received_at+self.latest.source_age_s
            if age < 0 or age > self.max_age_s:
                self.invalidate('tracking_expired')
            return self.latest

    def status(self, now: float | None = None) -> dict:
        with self.lock:
            now = self.clock() if now is None else now
            frame = self.sample(now)
            return {'input_profile': self.policy.name, 'total_age_limit_s': self.max_age_s, 'source_age_limit_s': self.policy.source_age_s, 'source': 'tracking', 'robot_commands_sent': False,
                    'input_source': None if frame is None else frame.source,
                    'received': frame is not None,
                    'sample_age_s': None if frame is None else max(0., now-frame.received_at+frame.source_age_s),
                    'session': None if frame is None else frame.session,
                    'sequence': None if frame is None else frame.sequence,
                    'focused': bool(frame and frame.focused),
                    'head_tracked': bool(frame and frame.head),
                    'left_tracked': bool(frame and frame.left.tracked),
                    'right_tracked': bool(frame and frame.right.tracked),
                    'left_grip': None if frame is None else frame.left.grip,
                    'right_grip': None if frame is None else frame.right.grip,
                    'left_release_required': self.release_required['left'],
                    'right_release_required': self.release_required['right'],
                    'counters': dict(self.counters), 'last_error': self.last_error}


class XRoboToolkitInput:
    """Native SDK acquisition with explicit, per-device validity policy.

    The verified SDK getters expose poses/buttons and one source timestamp,
    but no controller tracking-confidence/connected flags. A numeric pose or
    advancing global timestamp does not establish that either hand is tracked.
    Without a validity_provider, poses are read for diagnostics but are marked
    untracked. Use the WebXR receiver for explicit per-controller validity.
    This class does not start services, install applications, or control robots.
    """
    def __init__(self, sdk=None, *, validity_provider: Callable[[], Mapping[str, bool]] | None = None,
                 clock: Callable[[], float] = time.monotonic):
        self.sdk = sdk if sdk is not None else importlib.import_module('xrobotoolkit_sdk')
        self.validity_provider, self.clock = validity_provider, clock
        self.session = 'xrt-' + __import__('secrets').token_hex(12)
        self.sequence = 0
        self.last_timestamp = None
        self.timestamp_advanced_at = None
        self.last_error = ''

    def start(self):
        self.sdk.init()

    def close(self):
        self.sdk.close()

    def poll(self) -> TrackingFrame:
        now = self.clock()
        timestamp = _number(self.sdk.get_time_stamp_ns(), 'SDK timestamp')
        if self.last_timestamp is None:
            self.timestamp_advanced_at = None
        elif timestamp > self.last_timestamp:
            self.timestamp_advanced_at = now
        elif timestamp < self.last_timestamp:
            self.session = 'xrt-' + __import__('secrets').token_hex(12)
            self.timestamp_advanced_at = None
        self.last_timestamp = timestamp
        fresh = (timestamp > 0 and self.timestamp_advanced_at is not None
                 and 0 <= now-self.timestamp_advanced_at <= MAX_SOURCE_AGE_S)
        validity = dict(self.validity_provider() or {}) if self.validity_provider else {}
        self.last_error = '' if self.validity_provider else 'native_sdk_has_no_verified_per_controller_validity_use_webxr'
        def read_pose(getter):
            try:
                raw = list(getter())
                return Pose.from_payload({'position': raw[:3], 'orientation_xyzw': raw[3:]})
            except (TypeError, ValueError, RuntimeError):
                return None
        head = read_pose(self.sdk.get_headset_pose)
        controllers = {}
        for side, primary, secondary in (('left', 'X', 'Y'), ('right', 'A', 'B')):
            pose = read_pose(getattr(self.sdk, f'get_{side}_controller_pose'))
            valid = fresh and validity.get(side) is True and pose is not None
            grip = _number(getattr(self.sdk, f'get_{side}_grip')(), side+' grip')
            trigger = _number(getattr(self.sdk, f'get_{side}_trigger')(), side+' trigger')
            if not 0 <= grip <= 1 or not 0 <= trigger <= 1:
                valid = False
                grip = trigger = 0.
            buttons = [label for label in (primary, secondary)
                       if bool(getattr(self.sdk, f'get_{label}_button')())]
            for getter, label in ((f'get_{side}_axis_click', 'thumbstick'), (f'get_{side}_menu_button', 'menu')):
                if bool(getattr(self.sdk, getter, lambda: False)()):
                    buttons.append(label)
            axes = _vector(list(getattr(self.sdk, f'get_{side}_axis')()), 2, side+' axes', 1.)
            controllers[side] = ControllerState(pose if valid else None, valid, valid, grip, trigger, tuple(buttons), True, axes)
        self.sequence += 1
        return TrackingFrame(self.session, self.sequence, timestamp/1e9, now,
            head if fresh and validity.get('head') is True else None,
            controllers['left'], controllers['right'], fresh, 'xrobotoolkit',
            0. if self.timestamp_advanced_at is None else max(0., now-self.timestamp_advanced_at), False)


def inspect_pico(adb: str | Path = 'adb', serial: str | None = None, *, run=subprocess.run) -> dict:
    """Read-only device/model/version query; does not reverse ports or launch apps."""
    def command(*args):
        result = run([str(adb), *args], capture_output=True, text=True, encoding='utf-8',
                     errors='replace', timeout=10, check=True)
        return result.stdout.strip()
    devices = [line.split()[0] for line in command('devices').splitlines()[1:]
               if len(line.split()) >= 2 and line.split()[1] == 'device']
    if serial is not None:
        devices = [value for value in devices if value == serial]
    if len(devices) != 1:
        raise ValueError('select one authorized PICO device with --serial')
    selected = devices[0]
    props = {key: command('-s', selected, 'shell', 'getprop', key)
             for key in ('ro.product.model', 'ro.product.manufacturer', 'ro.build.display.id', 'ro.build.version.release')}
    model = props['ro.product.model']
    confirmed = model == 'A9210' or model.replace(' ', '').lower() == 'pico4ultra'
    return {'model': model, 'pico_4_ultra_confirmed': confirmed, 'properties': props,
            'robot_commands_sent': False, 'device_modified': False}


def strict_json(raw: bytes | str):
    def pairs(items):
        result = {}
        for key, value in items:
            if key in result:
                raise ValueError('duplicate JSON key')
            result[key] = value
        return result
    return json.loads(raw, object_pairs_hook=pairs,
                      parse_constant=lambda value: (_ for _ in ()).throw(ValueError(value)))
