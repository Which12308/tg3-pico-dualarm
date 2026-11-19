"""ROS 2 observability for the dry-run adapter; hardware writes stay closed.

The only publishers are standard telemetry on /tg3_teleop/*. No publisher is
created for /arm/cmd, hands, whole-body modes, base velocity, or emergency stop.
Actual feedback may be subscribed for comparison but never drives simulated
state or grants authority. A hardware mode request fails before node creation.
"""
from __future__ import annotations

import argparse
from dataclasses import asdict
import json
import math
import sys
import threading
import time
from typing import Any, Callable

from .model import JOINT_NAMES, default_q
from .preflight import ARM_STATE_TOPIC, FSM_STATE_TOPIC, SnapshotAccumulator
from .robot import DryRunRobot, JointCommand, ReadOnlyRobot, SubmitResult


DRY_RUN_JOINT_TOPIC = "/tg3_teleop/dry_run/joint_states"
DIAGNOSTIC_TOPIC = "/tg3_teleop/diagnostics"
HARDWARE_BLOCK_REASON = (
    "hardware mode unavailable: the authorized TianGong dual-arm ownership, "
    "mode hand-off, physical E-stop feedback and recovery adapter are not verified"
)


def _mode(mode: str) -> None:
    if mode == "hardware":
        raise RuntimeError(HARDWARE_BLOCK_REASON + "; no ROS node or motion publisher was created")
    if mode != "dry_run":
        raise ValueError("mode must be dry_run or hardware")


def build_bridge_class():
    """Load ROS only when requested, allowing normal imports on desktop hosts."""
    try:
        from rclpy.node import Node
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import JointState
        from std_msgs.msg import String
    except ImportError as exc:
        raise RuntimeError("ROS 2 is unavailable; source the Jazzy environment before running bridge_node") from exc

    class DryRunBridge(Node):
        """Expose one simulated RobotProtocol and optional read-only feedback.

        An embedded Teleoperation controller may call submit_simulated() or
        use robot_adapter. There is deliberately no ROS command subscription
        that could be confused with an actuator input or control authorization.
        """

        def __init__(self, *, mode: str = "dry_run", observe_robot: bool = False,
                     publish_rate_hz: float = 20., initial_positions=None,
                     monotonic: Callable[[], float] = time.monotonic, **node_kwargs) -> None:
            _mode(mode)  # Before super(): even parameter-event publishers must not precede the gate.
            if type(observe_robot) is not bool:
                raise ValueError("observe_robot must be a bool")
            if (isinstance(publish_rate_hz, bool) or not math.isfinite(publish_rate_hz)
                    or not 1 <= publish_rate_hz <= 100):
                raise ValueError("publish_rate_hz must be in 1..100")
            # Import and native-support validation precede publishers, so a
            # mismatched custom-message installation leaves no partial bridge.
            observation_types = None
            if observe_robot:
                try:
                    from ros2_bridge_msgs.msg import RobotState as ROSRobotState
                    from diagnostic_msgs.msg import DiagnosticStatus
                    from rclpy.type_support import check_for_type_support
                    for cls in (ROSRobotState, DiagnosticStatus):
                        check_for_type_support(cls)
                    observation_types = (ROSRobotState, DiagnosticStatus)
                except (ImportError, RuntimeError) as exc:
                    raise RuntimeError("robot observation requires installed ROS custom-message native type support") from exc
            self.robot_adapter = DryRunRobot(default_q() if initial_positions is None else initial_positions,
                                             monotonic=monotonic)
            self._monotonic = monotonic
            self._observed = SnapshotAccumulator(monotonic=monotonic)
            self.observed_robot = ReadOnlyRobot(self._observed.read_state)
            self._last_result: SubmitResult | None = None
            self._last_command: JointCommand | None = None
            self._state_lock = threading.RLock()
            self._observing_robot = observe_robot
            super().__init__("tg3_pico_dualarm_bridge", enable_rosout=False, **node_kwargs)
            self._joint_publisher = self.create_publisher(JointState, DRY_RUN_JOINT_TOPIC, 10)
            self._diagnostic_publisher = self.create_publisher(String, DIAGNOSTIC_TOPIC, 10)
            self._subscriptions = []
            if observation_types is not None:
                self._subscriptions = [
                    self.create_subscription(observation_types[0], ARM_STATE_TOPIC,
                                             self._observed.arm, qos_profile_sensor_data),
                    self.create_subscription(observation_types[1], FSM_STATE_TOPIC,
                                             self._observed.mode, qos_profile_sensor_data),
                ]
            self._timer = self.create_timer(1. / float(publish_rate_hz), self.publish_observation)

        def submit_simulated(self, command: JointCommand) -> SubmitResult:
            """Programmatic dry-run target entry; never emits a robot command."""
            with self._state_lock:
                result = self.robot_adapter.submit(command)
                self._last_result = result
                if result.accepted:
                    self._last_command = command
                return result

        def read_state(self):
            with self._state_lock:
                return self.robot_adapter.read_state()

        def disarm(self, reason: str = "software_disarm") -> None:
            with self._state_lock:
                self.robot_adapter.disarm()
                self._last_result = SubmitResult(False, reason)

        def publish_observation(self) -> None:
            with self._state_lock:
                state = self.robot_adapter.read_state()
                last_command, last_result = self._last_command, self._last_result
            message = JointState()
            message.header.stamp = self.get_clock().now().to_msg()
            message.header.frame_id = "tg3_dry_run_simulated_waist_pitch_link"
            message.name = list(JOINT_NAMES)
            message.position = list(state.joint_positions)
            # This ideal follower has no continuous measured velocity or
            # effort. Desired velocities are observable as command data below,
            # rather than repeated forever as simulated sensor measurements.
            message.velocity = []
            self._joint_publisher.publish(message)
            now = self._monotonic()
            observed = None
            observation_error = None
            if self._observing_robot:
                try:
                    observed = asdict(self.observed_robot.read_state())
                except RuntimeError as exc:
                    observation_error = str(exc)
            payload = dict(
                schema="tg3_pico_dualarm_ros_bridge_v1", mode="dry_run",
                simulated=True, hardware_writes_enabled=False, robot_commands_sent=False,
                physical_estop=None, physical_estop_source="not_verified",
                software_disarmed=state.mode == "DRY_RUN_DISARMED",
                simulated_state=asdict(state),
                last_submit_result=None if last_result is None else asdict(last_result),
                last_simulated_command=None if last_command is None else asdict(last_command),
                actual_robot_observation=observed, observation_error=observation_error,
                observation_enabled=self._observing_robot,
                observed_feedback_ages_s={
                    "arm": None if self._observed.arm_stamp is None else now-self._observed.arm_stamp,
                    "fsm": None if self._observed.fsm_stamp is None else now-self._observed.fsm_stamp,
                },
                hardware_blockers=[HARDWARE_BLOCK_REASON], motion_publishers_created=[],
            )
            diagnostic = String()
            diagnostic.data = json.dumps(payload, ensure_ascii=False, allow_nan=False, separators=(",", ":"))
            self._diagnostic_publisher.publish(diagnostic)

    return DryRunBridge


def create_bridge(*, mode: str = "dry_run", **kwargs):
    _mode(mode)
    return build_bridge_class()(mode=mode, **kwargs)


class _SessionRobot:
    """RobotProtocol facade; shares exactly the node's simulated state."""
    hardware_writes_enabled = False
    simulated = True

    def __init__(self, session):
        self._session = session

    def read_state(self):
        self._session._check_running()
        return self._session.node.read_state()

    def submit(self, command: JointCommand) -> SubmitResult:
        self._session._check_running()
        return self._session.node.submit_simulated(command)

    def disarm(self) -> None:
        self._session._check_running()
        self._session.node.disarm("controller_software_disarm")


class RosDryRunSession:
    """Embed ROS telemetry in the same dry-run loop used by live PICO input.

    The private context avoids changing another ROS user's default context.
    A failed executor disarms the simulated adapter and rejects further facade
    calls; it never falls back to a hardware publisher or a second simulator.
    """

    def __init__(self, *, observe_robot: bool = False, publish_rate_hz: float = 20.,
                 initial_positions=None, monotonic: Callable[[], float] = time.monotonic) -> None:
        self.context = self.node = self.executor = self._thread = None
        self._closing = threading.Event()
        self._failed = threading.Event()
        self._error: BaseException | None = None
        self._closed = False
        self._close_lock = threading.RLock()
        try:
            try:
                import rclpy
                from rclpy.context import Context
                from rclpy.executors import SingleThreadedExecutor
            except ImportError as exc:
                raise RuntimeError("ROS 2 is unavailable; source the Jazzy environment before using --ros") from exc
            self.context = Context()
            rclpy.init(args=[], context=self.context)
            self.node = create_bridge(mode="dry_run", observe_robot=observe_robot,
                                      publish_rate_hz=publish_rate_hz,
                                      initial_positions=initial_positions, monotonic=monotonic,
                                      context=self.context)
            self.executor = SingleThreadedExecutor(context=self.context)
            self.executor.add_node(self.node)
            self.robot = _SessionRobot(self)
            self._thread = threading.Thread(target=self._spin, name="tg3-dry-run-ros", daemon=True)
            self._thread.start()
            self._check_running()
        except BaseException:
            self.close()
            raise

    def _spin(self) -> None:
        try:
            self.executor.spin()
            if not self._closing.is_set():
                raise RuntimeError("ROS executor stopped unexpectedly")
        except BaseException as exc:
            if not self._closing.is_set():
                self._error = exc
                self._failed.set()
                try:
                    self.node.disarm("ros_executor_failed")
                except Exception:
                    pass

    def _check_running(self) -> None:
        if self._failed.is_set():
            raise RuntimeError("ROS dry-run telemetry executor failed: " + str(self._error)) from self._error
        if self._closing.is_set() or self._closed:
            raise RuntimeError("ROS dry-run session is closed")

    def close(self) -> None:
        with self._close_lock:
            if self._closed:
                return
            self._closing.set()
            try:
                if self.node is not None:
                    self.node.disarm("ros_session_closed")
                if self.executor is not None:
                    self.executor.shutdown(timeout_sec=2.)
            finally:
                try:
                    if self.context is not None and self.context.ok():
                        self.context.shutdown()
                    if self._thread is not None and self._thread is not threading.current_thread():
                        self._thread.join(timeout=2.)
                finally:
                    if self.node is not None:
                        self.node.destroy_node()
                    self._closed = True
            if self._thread is not None and self._thread.is_alive():
                raise RuntimeError("ROS dry-run executor did not stop during cleanup")

    def __enter__(self):
        self._check_running()
        return self

    def __exit__(self, *_exc):
        self.close()


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mode", choices=("dry_run", "hardware"), default="dry_run")
    parser.add_argument("--observe-robot", action="store_true",
                        help="subscribe to existing feedback only; never enable hardware writes")
    parser.add_argument("--publish-rate", type=float, default=20.)
    args, ros_args = parser.parse_known_args(sys.argv[1:] if argv is None else argv)
    context = node = executor = None
    try:
        _mode(args.mode)
        try:
            import rclpy
            from rclpy.context import Context
            from rclpy.executors import SingleThreadedExecutor
        except ImportError as exc:
            raise RuntimeError("ROS 2 is unavailable; source the Jazzy environment before running bridge_node") from exc
        context = Context()
        rclpy.init(args=ros_args, context=context)
        node = create_bridge(mode=args.mode, observe_robot=args.observe_robot,
                             publish_rate_hz=args.publish_rate, context=context)
        executor = SingleThreadedExecutor(context=context)
        executor.add_node(node)
        print(json.dumps(dict(mode="dry_run", hardware_writes_enabled=False,
                              joint_state_topic=DRY_RUN_JOINT_TOPIC, diagnostics_topic=DIAGNOSTIC_TOPIC,
                              robot_commands_sent=False)), flush=True)
        executor.spin()
        return 0
    except KeyboardInterrupt:
        if node is not None:
            node.disarm("operator_interrupt")
        return 0
    except (RuntimeError, ValueError, OSError) as exc:
        print(json.dumps(dict(error=str(exc), hardware_writes_enabled=False,
                              robot_commands_sent=False), ensure_ascii=False), file=sys.stderr)
        return 2
    finally:
        if executor is not None:
            executor.shutdown()
        if node is not None:
            node.destroy_node()
        if context is not None and context.ok():
            context.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
