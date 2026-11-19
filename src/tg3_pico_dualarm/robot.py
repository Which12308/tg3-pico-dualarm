"""Robot boundary. Bundled adapters never publish a hardware command.

All vectors use left 11..17, then right 21..27, in radians / radians per second.
The deterministic dry-run follower is useful for pipeline/replay tests; it is
not a dynamics simulator or evidence of a successful physical grasp.
"""
from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import math
import time
from typing import Callable, Protocol, Sequence, runtime_checkable


JOINT_IDS = tuple(range(11, 18)) + tuple(range(21, 28))
DOF = len(JOINT_IDS)


def _vector(values: Sequence[float], label: str) -> tuple[float, ...]:
    try:
        raw = tuple(values)
        if len(raw) != DOF or any(isinstance(v, (bool, str, bytes)) for v in raw):
            raise ValueError
        result = tuple(float(v) for v in raw)
    except (TypeError, ValueError, OverflowError) as exc:
        raise ValueError(f"{label} must contain fourteen finite numbers") from exc
    if not all(math.isfinite(v) for v in result):
        raise ValueError(f"{label} must contain fourteen finite numbers")
    return result


def _timestamp(value: float, label: str) -> float:
    if isinstance(value, (bool, str, bytes)):
        raise ValueError(f"{label} must be a finite nonnegative monotonic time")
    try:
        value = float(value)
    except (TypeError, ValueError, OverflowError) as exc:
        raise ValueError(f"invalid {label}") from exc
    if not math.isfinite(value) or value < 0:
        raise ValueError(f"{label} must be a finite nonnegative monotonic time")
    return value


@dataclass(frozen=True)
class RobotState:
    joint_positions: tuple[float, ...]
    timestamp: float
    mode: str
    estop: bool | None
    quality: str

    def __post_init__(self) -> None:
        object.__setattr__(self, "joint_positions", _vector(self.joint_positions, "joint_positions"))
        object.__setattr__(self, "timestamp", _timestamp(self.timestamp, "timestamp"))
        if not isinstance(self.mode, str) or not self.mode:
            raise ValueError("mode must identify the observed or simulated mode")
        if self.estop is not None and type(self.estop) is not bool:
            raise ValueError("estop must be True, False, or None (unknown)")
        if not isinstance(self.quality, str) or not self.quality:
            raise ValueError("quality must be explicit")

    @property
    def positions(self) -> tuple[float, ...]:
        return self.joint_positions

    @property
    def stamp(self) -> float:
        return self.timestamp


@dataclass(frozen=True)
class JointCommand:
    positions: tuple[float, ...]
    velocities: tuple[float, ...]
    stamp: float

    def __post_init__(self) -> None:
        object.__setattr__(self, "positions", _vector(self.positions, "positions"))
        object.__setattr__(self, "velocities", _vector(self.velocities, "velocities"))
        object.__setattr__(self, "stamp", _timestamp(self.stamp, "stamp"))


@dataclass(frozen=True)
class SubmitResult:
    accepted: bool
    reason: str


@runtime_checkable
class RobotProtocol(Protocol):
    def read_state(self) -> RobotState: ...
    def submit(self, command: JointCommand) -> SubmitResult: ...


class DryRunRobot:
    """Accept targets into a bounded history and update only simulated joints.

    The caller supplies the model's default pose when it needs a useful arm
    posture. A zero default here is merely a fourteen-value simulated state.
    Command timestamps must come from this adapter's monotonic clock domain.
    Software disarm does not represent the robot's physical emergency stop.
    """

    hardware_writes_enabled = False
    simulated = True

    def __init__(self, initial_positions: Sequence[float] | None = None, *,
                 monotonic: Callable[[], float] = time.monotonic,
                 max_history: int = 1000, max_command_age: float = .25) -> None:
        if type(max_history) is not int or max_history <= 0:
            raise ValueError("max_history must be a positive integer")
        if not math.isfinite(max_command_age) or max_command_age <= 0:
            raise ValueError("max_command_age must be positive and finite")
        self._clock = monotonic
        self._positions = _vector(initial_positions if initial_positions is not None else (0.,) * DOF,
                                  "initial_positions")
        self._commands: deque[JointCommand] = deque(maxlen=max_history)
        self._last_stamp: float | None = None
        self._max_age = float(max_command_age)
        self._disarmed = False

    @property
    def commands(self) -> list[JointCommand]:
        return list(self._commands)

    def read_state(self) -> RobotState:
        return RobotState(self._positions, self._clock(),
                          "DRY_RUN_DISARMED" if self._disarmed else "DRY_RUN",
                          None, "simulated")

    def submit(self, command: JointCommand) -> SubmitResult:
        if not isinstance(command, JointCommand):
            raise TypeError("submit requires a validated JointCommand")
        if self._disarmed:
            return SubmitResult(False, "software_disarmed")
        now = _timestamp(self._clock(), "clock")
        if not 0 <= now - command.stamp <= self._max_age:
            return SubmitResult(False, "stale_or_future_command")
        if self._last_stamp is not None and command.stamp <= self._last_stamp:
            return SubmitResult(False, "nonincreasing_command_stamp")
        self._commands.append(command)
        self._positions = command.positions
        self._last_stamp = command.stamp
        return SubmitResult(True, "accepted_in_dry_run_only")

    def disarm(self) -> None:
        self._disarmed = True

    def arm(self) -> None:
        """Re-enable the simulator only; this grants no hardware authority."""
        self._disarmed = False


class ReadOnlyRobot:
    """Expose observed state while rejecting every hardware write request.

    A future verified hardware writer must be a separately supplied adapter;
    there is deliberately no enable flag, magic token, or ROS publisher here.
    """

    hardware_writes_enabled = False
    simulated = False

    def __init__(self, state_reader: Callable[[], RobotState]) -> None:
        self._reader = state_reader

    def read_state(self) -> RobotState:
        result = self._reader()
        if not isinstance(result, RobotState):
            raise TypeError("state_reader must return RobotState")
        return result

    def submit(self, command: JointCommand) -> SubmitResult:
        if not isinstance(command, JointCommand):
            raise TypeError("submit requires a validated JointCommand")
        return SubmitResult(False, "hardware_read_only_verified_dual_arm_adapter_missing")
