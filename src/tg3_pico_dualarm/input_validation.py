"""Validate public tracking objects before relative mapping or IK.

Wire parsing and this boundary are independent: direct dataclass callers must
not bypass numeric/validity checks. Source timestamps belong to the source
clock; only receipt age and declared source age are combined with host time.
"""
from __future__ import annotations

from . import safety as motion_policy
import math
from .input_policy import NORMAL
from numbers import Real


MAX_SOURCE_AGE_S = .12
MAX_TOTAL_AGE_S = .15
MAX_SEQUENCE = 2**53 - 1


def _number(value, name: str) -> float:
    if isinstance(value, bool) or not isinstance(value, Real):
        raise ValueError(name + " must be a finite number")
    try:
        value = float(value)
    except (ValueError, OverflowError) as exc:
        raise ValueError(name + " must be a finite number") from exc
    if not math.isfinite(value):
        raise ValueError(name + " must be a finite number")
    return value


def _attribute(value, name: str):
    try:
        return getattr(value, name)
    except (AttributeError, TypeError) as exc:
        raise ValueError("missing tracking field: " + name) from exc


def _vector(value, length: int, name: str, *, bound: float | None = None) -> tuple[float, ...]:
    if isinstance(value, (str, bytes, bytearray, dict)):
        raise ValueError(name + " must contain exactly " + str(length) + " numbers")
    try:
        if len(value) != length:
            raise ValueError(name + " must contain exactly " + str(length) + " numbers")
        result = tuple(_number(item, name) for item in value)
    except TypeError as exc:
        raise ValueError(name + " must contain exactly " + str(length) + " numbers") from exc
    if len(result) != length or (bound is not None and any(abs(item) > bound for item in result)):
        raise ValueError(name + " is outside its valid range")
    return result


def validate_pose(pose, *, name: str = "pose") -> None:
    _vector(_attribute(pose, "position"), 3, name + ".position", bound=None if motion_policy.LIMITS_MANAGED_EXTERNALLY else 20.)
    quaternion = _vector(_attribute(pose, "orientation_xyzw"), 4, name + ".orientation_xyzw")
    norm = math.sqrt(sum(item*item for item in quaternion))
    if not .98 <= norm <= 1.02:
        raise ValueError(name + " quaternion must be unit XYZW")


def validate_controller(controller, *, side: str = "controller") -> None:
    for field in ("connected", "tracked", "release_required"):
        if type(_attribute(controller, field)) is not bool:
            raise ValueError(side + "." + field + " must be boolean")
    connected, tracked = controller.connected, controller.tracked
    if tracked and not connected:
        raise ValueError(side + " cannot be tracked while disconnected")
    for field in ("grip", "trigger"):
        value = _number(_attribute(controller, field), side + "." + field)
        if not 0. <= value <= 1.:
            raise ValueError(side + "." + field + " must be in [0,1]")
    buttons = _attribute(controller, "buttons")
    if (not isinstance(buttons, tuple) or len(buttons) > 16
            or any(not isinstance(item, str) or not 1 <= len(item) <= 32 for item in buttons)
            or len(set(buttons)) != len(buttons)):
        raise ValueError(side + ".buttons must be a tuple of unique valid names")
    _vector(_attribute(controller, "axes"), 2, side + ".axes", bound=1.)
    source_id = _attribute(controller, "source_id")
    if not isinstance(source_id, str) or len(source_id) > 128:
        raise ValueError(side + ".source_id is invalid")
    pose = _attribute(controller, "pose")
    if tracked and pose is None:
        raise ValueError(side + " is tracked without a pose")
    if pose is not None:
        validate_pose(pose, name=side + ".pose")


def validate_frame(frame, now: float, policy=NORMAL) -> None:
    """Return None for valid input; raise ValueError without modifying it.

    This function checks one frame's structural validity and freshness. Replay
    or nonadvancing source time across two frames remains the receiver/mapper's
    session-aware responsibility; no cross-host timestamp subtraction is used.
    """
    now = _number(now, "now")
    if now < 0:
        raise ValueError("host monotonic clock must be nonnegative")
    session = _attribute(frame, "session")
    if not isinstance(session, str) or not 1 <= len(session) <= 128:
        raise ValueError("invalid tracking session")
    sequence = _attribute(frame, "sequence")
    if type(sequence) is not int or not 0 <= sequence <= MAX_SEQUENCE:
        raise ValueError("invalid tracking sequence")
    source_time = _number(_attribute(frame, "source_time_s"), "source_time_s")
    received = _number(_attribute(frame, "received_at"), "received_at")
    source_age = _number(_attribute(frame, "source_age_s"), "source_age_s")
    if source_time < 0 or received < 0:
        raise ValueError("tracking timestamps must be nonnegative")
    if not motion_policy.LIMITS_MANAGED_EXTERNALLY and not 0 <= source_age <= policy.source_age_s:
        raise ValueError("source_age_s is outside the validity budget")
    if not motion_policy.LIMITS_MANAGED_EXTERNALLY and (not 0 <= now-received or now-received+source_age > policy.total_age_s):
        raise ValueError("tracking frame is stale or from the host clock's future")
    for field in ("focused", "scene_visible"):
        if type(_attribute(frame, field)) is not bool:
            raise ValueError(field + " must be boolean")
    source = _attribute(frame, "source")
    if source not in ("webxr", "xrobotoolkit", "openxr"):
        raise ValueError("unsupported tracking source")
    head = _attribute(frame, "head")
    if head is not None:
        validate_pose(head, name="head")
    validate_controller(_attribute(frame, "left"), side="left")
    validate_controller(_attribute(frame, "right"), side="right")
