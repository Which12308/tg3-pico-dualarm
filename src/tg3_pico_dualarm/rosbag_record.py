"""Supervise existing rosbag2 against topics actually discovered on this robot.

This module starts no camera driver and creates no actuator publisher. Image
and CameraInfo topics are selected by their installed ROS type, never by an
invented D455 topic name. Readiness requires subscriber logs plus a storage file.
"""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import json
import math
import os
from pathlib import Path
import re
import signal
import subprocess
import sys
import time
from typing import Callable, Mapping
import uuid


CAMERA_TYPES = frozenset(("sensor_msgs/msg/Image", "sensor_msgs/msg/CompressedImage", "sensor_msgs/msg/CameraInfo"))
KNOWN_TOPICS = {
    "/tg3/xarm/arm_raw": "ros2_bridge_msgs/msg/ArmCtrl",
    "/jointspace_commands_Dual": "std_msgs/msg/Float64MultiArray",
    "/power/board/key_status": "bodyctrl_msgs/msg/PowerBoardKeyStatus",
    "/manipulator/warn_and_error_info": "eai_manipulator_msgs/msg/WarnAndErrorInfo",
    "/joint_states": "sensor_msgs/msg/JointState",
    "/robot_state": "ros2_bridge_msgs/msg/RobotState",
    "/arm/cmd": "ros2_bridge_msgs/msg/ArmCtrl",
    "/sbus_data": "sensor_msgs/msg/Joy",
    "/hric/robot/rl_state": "diagnostic_msgs/msg/DiagnosticStatus",
    "/right_hand/motor_status": "brainco_hand_msgs/msg/MotorStatus",
    "/right_hand/set_motor_multi": "brainco_hand_msgs/msg/SetMotorMulti",
    "/left_hand/motor_status": "brainco_hand_msgs/msg/MotorStatus",
    "/left_hand/set_motor_multi": "brainco_hand_msgs/msg/SetMotorMulti",
}
COMMAND_TOPICS=frozenset(('/jointspace_commands_Dual','/arm/cmd',
    '/right_hand/set_motor_multi','/left_hand/set_motor_multi'))


def select_topics(discovered: Mapping[str, list[str]], require_camera: bool = False) -> dict[str, str]:
    result = {}
    for name, types in discovered.items():
        if not isinstance(name, str) or not name.startswith("/") or name.startswith("-"):
            continue
        if not isinstance(types, (list, tuple)) or len(types) != 1:
            continue
        typ = types[0]
        if typ in CAMERA_TYPES or KNOWN_TOPICS.get(name) == typ:
            result[name] = typ
    # Do not record simultaneous raw and already-published compressed copies
    # of one stream. CameraInfo remains; no camera/encoder is started here.
    for name,typ in tuple(result.items()):
        if typ=='sensor_msgs/msg/Image' and any(result.get(name+suffix)=='sensor_msgs/msg/CompressedImage'
                for suffix in ('/compressed','/compressedDepth')):result.pop(name)
    if require_camera and not any(typ in CAMERA_TYPES - {"sensor_msgs/msg/CameraInfo"} for typ in result.values()):
        raise RuntimeError("no actual Image or CompressedImage topic discovered; camera recording was required")
    if not result:
        raise RuntimeError("no supported robot feedback/command or camera topics discovered")
    return dict(sorted(result.items()))


def observed_topics(node):
    result={}
    for name,types in node.get_topic_names_and_types():
        live=node.count_publishers(name)>0
        if not live and name in COMMAND_TOPICS and list(types)==[KNOWN_TOPICS[name]]:
            # Real actuator/controller subscriptions already establish these
            # topics. Subscribe before enable so the first action is retained.
            live=any(endpoint.topic_type==KNOWN_TOPICS[name] for endpoint in node.get_subscriptions_info_by_topic(name))
        if live:result[name]=list(types)
    return result


def discover_topics(seconds: float = 5.) -> dict[str, list[str]]:
    if not math.isfinite(seconds) or not 1 <= seconds <= 30:
        raise ValueError("discovery seconds must be in 1..30")
    try:
        import rclpy
        from rclpy.context import Context
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.node import Node
    except ImportError as exc:
        raise RuntimeError("ROS 2 is unavailable; source the robot's ROS environment before recording") from exc
    context = Context()
    node = executor = None
    rclpy.init(args=[], context=context)
    try:
        node = Node("tg3_rosbag_discovery_" + uuid.uuid4().hex[:8], context=context,
                    enable_rosout=False, start_parameter_services=False, use_global_arguments=False)
        executor = SingleThreadedExecutor(context=context)
        executor.add_node(node)
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline and context.ok():
            executor.spin_once(timeout_sec=min(.1, max(0., deadline-time.monotonic())))
        return observed_topics(node)
    finally:
        if executor is not None:
            executor.shutdown()
        if node is not None:
            node.destroy_node()
        if context.ok():
            context.shutdown()


def readiness(log_path: Path, bag_path: Path, topics: Mapping[str, str]) -> tuple[bool, list[str]]:
    try:
        with log_path.open("rb") as stream:
            stream.seek(0, 2)
            size = stream.tell()
            stream.seek(max(0, size-2_000_000))
            log = stream.read().decode("utf-8", errors="replace")
    except FileNotFoundError:
        log = ""
    missing = [name for name in topics if not re.search(
        r"Subscribed to topic\s+['\"]" + re.escape(name) + r"['\"]", log)]
    storage = [path for pattern in ("*.db3", "*.mcap") for path in bag_path.glob(pattern)
               if path.is_file() and path.stat().st_size > 0]
    reasons = ["waiting_for_subscription:" + name for name in missing]
    if not storage:
        reasons.append("waiting_for_initialized_storage")
    return not reasons, reasons


def stop_recorder(process, *, killpg=os.killpg if hasattr(os, "killpg") else None) -> list[str]:
    """Stop this newly created child session only; parent Ctrl+C is not relayed twice."""
    actions = []
    if process.poll() is not None:
        return actions
    if killpg is None:
        raise RuntimeError("process-group signalling is unavailable")
    # Popen.poll()==None leaves this child unreaped, so its PID cannot be reused
    # by an unrelated process between this check and signalling its own session.
    for sig, label, timeout in ((signal.SIGINT, "SIGINT", 10.), (signal.SIGTERM, "SIGTERM", 5.),
                                (getattr(signal, "SIGKILL", 9), "SIGKILL", 2.)):
        if process.poll() is not None:
            break
        try:
            killpg(process.pid, sig)
            actions.append(label)
        except ProcessLookupError:
            pass
        try:
            process.wait(timeout=timeout)
            break
        except subprocess.TimeoutExpired:
            continue
    if process.poll() is None:
        raise RuntimeError("rosbag recorder did not exit after bounded stop")
    return actions


def record(output: str | Path = "recordings_ros", *, seconds: float | None = None,
           require_camera: bool = False, readiness_timeout: float = 5.,
           discovery: Callable[[], Mapping[str, list[str]]] = discover_topics,
           popen=subprocess.Popen, monotonic=time.monotonic, sleep=time.sleep,
           platform_name: str = os.name, killpg=None, stop_event=None, status_callback=None) -> dict:
    if platform_name != "posix":
        raise RuntimeError("ROS recording requires a Linux ROS 2 environment; no recorder was started")
    if seconds is not None and (isinstance(seconds, bool) or not math.isfinite(seconds) or not 0 < seconds <= 86400):
        raise ValueError("seconds must be finite and in (0,86400]")
    if not math.isfinite(readiness_timeout) or not 0 < readiness_timeout <= 60:
        raise ValueError("readiness_timeout must be in (0,60]")
    selected = select_topics(discovery(), require_camera=require_camera)
    directory = Path(output) / (datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ") + "_" + uuid.uuid4().hex[:8])
    directory.mkdir(parents=True, exist_ok=False)
    metadata_path = directory / "session.json"
    bag_path, log_path = directory / "bag", directory / "recorder.log"
    metadata = dict(schema="tg3_rosbag_session_v1", directory=str(directory), topics=selected,
                    ready=False, closed_cleanly=False, robot_commands_sent=False,
                    camera_driver_started=False, camera_topics=[name for name, typ in selected.items() if typ in CAMERA_TYPES],
                    commands_subscribed_before_enable=True,
                    requested_duration_s=seconds, error=None, stop_actions=[], recorder_exit_code=None)
    def save():
        metadata_path.write_text(json.dumps(metadata, ensure_ascii=False, indent=2, allow_nan=False), encoding="utf-8")
        if status_callback:status_callback(dict(metadata))
    save()
    process = None
    started = monotonic()
    try:
        with log_path.open("wb") as log:
            process = popen(["ros2", "bag", "record", "--include-unpublished-topics", "-o", str(bag_path), *selected],
                            stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            deadline = monotonic() + readiness_timeout
            while True:
                if stop_event is not None and stop_event.is_set():
                    raise RuntimeError('recording_stopped_before_ready')
                if process.poll() is not None:
                    raise RuntimeError("rosbag recorder exited before readiness; inspect recorder.log")
                ready, reasons = readiness(log_path, bag_path, selected)
                if ready:
                    metadata.update(ready=True, readiness_duration_s=monotonic()-started)
                    save()
                    break
                if monotonic() >= deadline:
                    raise RuntimeError("rosbag readiness timed out: " + ";".join(reasons))
                sleep(.05)
            duration_deadline = None if seconds is None else monotonic()+seconds
            while duration_deadline is None or monotonic() < duration_deadline:
                if stop_event is not None and stop_event.is_set():
                    metadata['end_reason']='station_stopped'
                    break
                if process.poll() is not None:
                    raise RuntimeError("rosbag recorder exited unexpectedly during recording")
                sleep(.05)
    except KeyboardInterrupt:
        metadata["end_reason"] = "operator_interrupt"
    except (OSError, RuntimeError, ValueError) as exc:
        metadata["error"] = str(exc)
    finally:
        if process is not None:
            try:
                kwargs = {} if killpg is None else {"killpg": killpg}
                metadata["stop_actions"] = stop_recorder(process, **kwargs)
            except (OSError, RuntimeError) as exc:
                metadata["error"] = metadata["error"] or str(exc)
            metadata["recorder_exit_code"] = process.poll()
        metadata["duration_s"] = monotonic()-started
        metadata["closed_cleanly"] = bool(metadata["ready"] and metadata["error"] is None
                                           and metadata["recorder_exit_code"] in (0, -signal.SIGINT, 128+signal.SIGINT)
                                           and metadata["stop_actions"] in ([], ["SIGINT"]))
        if metadata["error"] is None and not metadata["closed_cleanly"]:
            metadata["error"] = "recorder did not close cleanly; inspect recorder.log and bag files"
        save()
    return metadata


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=Path("recordings_ros"))
    parser.add_argument("--seconds", type=float)
    parser.add_argument("--require-camera", action="store_true")
    parser.add_argument("--readiness-timeout", type=float, default=5.)
    args = parser.parse_args(argv)
    try:
        result = record(args.output, seconds=args.seconds, require_camera=args.require_camera,
                        readiness_timeout=args.readiness_timeout)
    except (RuntimeError, OSError, ValueError) as exc:
        result = dict(ready=False, closed_cleanly=False, error=str(exc), robot_commands_sent=False)
    print(json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False))
    return 0 if result.get("closed_cleanly") else 2


if __name__ == "__main__":
    raise SystemExit(main())
