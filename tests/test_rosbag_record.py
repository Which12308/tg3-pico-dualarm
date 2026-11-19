import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import unittest
from types import SimpleNamespace as NS

from tg3_pico_dualarm.rosbag_record import readiness, record, select_topics, stop_recorder,observed_topics


TOPICS = {"/robot_state": ["ros2_bridge_msgs/msg/RobotState"],
          "/actual_camera/color": ["sensor_msgs/msg/Image"],
          "/actual_camera/params": ["sensor_msgs/msg/CameraInfo"],
          "/invented_d455/image": ["std_msgs/msg/String"]}


class RecorderTests(unittest.TestCase):
    def test_actual_command_subscribers_are_recorded_before_enable_without_inventing_images(self):
        graph={'/arm/cmd':['ros2_bridge_msgs/msg/ArmCtrl'],'/jointspace_commands_Dual':['std_msgs/msg/Float64MultiArray'],
            '/stale_camera':['sensor_msgs/msg/Image'],'/right_hand/set_motor_multi':['wrong/msg/Type'],
            '/robot_state':['ros2_bridge_msgs/msg/RobotState']}
        node=NS(get_topic_names_and_types=lambda:list(graph.items()),count_publishers=lambda name:int(name=='/robot_state'),
            get_subscriptions_info_by_topic=lambda name:[NS(topic_type=graph[name][0])])
        selected=observed_topics(node)
        self.assertEqual(set(selected),{'/arm/cmd','/jointspace_commands_Dual','/robot_state'})

    def test_compressed_color_and_depth_avoid_duplicate_raw_recording_and_keep_camera_info(self):
        graph={'/camera/color/image':['sensor_msgs/msg/Image'],'/camera/color/image/compressed':['sensor_msgs/msg/CompressedImage'],
            '/camera/depth/image':['sensor_msgs/msg/Image'],'/camera/depth/image/compressedDepth':['sensor_msgs/msg/CompressedImage'],
            '/camera/color/camera_info':['sensor_msgs/msg/CameraInfo']}
        selected=select_topics(graph)
        self.assertNotIn('/camera/color/image',selected);self.assertNotIn('/camera/depth/image',selected)
        self.assertEqual(len(selected),3);self.assertIn('/camera/color/camera_info',selected)
    def test_discovery_uses_actual_types_and_requires_a_real_image_topic(self):
        selected = select_topics(TOPICS, require_camera=True)
        self.assertIn("/actual_camera/color", selected)
        self.assertNotIn("/invented_d455/image", selected)
        self.assertNotIn("/arm/cmd", selected)
        with self.assertRaisesRegex(RuntimeError, "no actual Image"):
            select_topics({"/params": ["sensor_msgs/msg/CameraInfo"]}, require_camera=True)
        with self.assertRaises(RuntimeError):
            select_topics({"/robot_state": ["wrong/msg/Type"]})

    def test_ready_requires_all_subscription_logs_and_nonempty_storage(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            log, bag = root / "log", root / "bag"
            bag.mkdir()
            selected = select_topics(TOPICS)
            log.write_text("Listening for topics...")
            (bag / "data.db3").write_bytes(b"sqlite initialized")
            self.assertFalse(readiness(log, bag, selected)[0])
            log.write_text("\n".join(f"[rosbag2_recorder]: Subscribed to topic '{name}'" for name in selected))
            self.assertTrue(readiness(log, bag, selected)[0])
            (bag / "data.db3").write_bytes(b"")
            self.assertFalse(readiness(log, bag, selected)[0])

    def harness(self, ready=True, exit_early=False):
        clock = [10.]
        calls = dict(signals=[], process=None, command=None, options=None)
        class Process:
            pid = 12345
            returncode = 7 if exit_early else None
            def poll(self):
                return self.returncode
            def wait(self, timeout):
                if self.returncode is None:
                    self.returncode = 0
                return self.returncode
        def popen(command, **options):
            calls.update(command=command, options=options, process=Process())
            if ready:
                bag = Path(command[command.index("-o")+1])
                bag.mkdir()
                (bag / "bag_0.mcap").write_bytes(b"initialized mcap storage")
                options["stdout"].write("\n".join(f"Subscribed to topic '{name}'" for name in command[command.index('-o')+2:]).encode())
                options["stdout"].flush()
            return calls["process"]
        def killpg(pid, sig):
            calls["signals"].append((pid, sig))
        def sleep(duration):
            clock[0] += duration
        return calls, clock, popen, killpg, sleep

    def test_supervised_bounded_recording_saves_truthful_metadata_and_stops_once(self):
        calls, clock, popen, killpg, sleep = self.harness()
        with tempfile.TemporaryDirectory() as folder:
            result = record(folder, seconds=.1, require_camera=True, discovery=lambda: TOPICS,
                            platform_name="posix", popen=popen, killpg=killpg,
                            monotonic=lambda: clock[0], sleep=sleep)
            self.assertTrue(result["ready"])
            self.assertTrue(result["closed_cleanly"])
            self.assertIsNone(result["error"])
            self.assertEqual(calls["signals"], [(12345, signal.SIGINT)])
            self.assertTrue(calls["options"]["start_new_session"])
            self.assertEqual(calls["command"][:3], ["ros2", "bag", "record"])
            self.assertFalse(result["robot_commands_sent"])
            metadata = json.loads((Path(result["directory"]) / "session.json").read_text())
            self.assertTrue(metadata["ready"] and metadata["closed_cleanly"])

    def test_readiness_timeout_and_child_crash_do_not_report_success(self):
        for ready, early in ((False, False), (True, True)):
            calls, clock, popen, killpg, sleep = self.harness(ready, early)
            with self.subTest(ready=ready, early=early), tempfile.TemporaryDirectory() as folder:
                result = record(folder, seconds=.1, readiness_timeout=.1, discovery=lambda: TOPICS,
                                platform_name="posix", popen=popen, killpg=killpg,
                                monotonic=lambda: clock[0], sleep=sleep)
                self.assertFalse(result["ready"])
                self.assertFalse(result["closed_cleanly"])
                self.assertIsNotNone(result["error"])

    def test_failed_exec_is_written_as_error(self):
        with tempfile.TemporaryDirectory() as folder:
            def missing(*args, **kwargs):
                raise FileNotFoundError("ros2 executable missing")
            result = record(folder, seconds=.1, discovery=lambda: TOPICS,
                            platform_name="posix", popen=missing)
            self.assertFalse(result["ready"])
            self.assertIn("missing", result["error"])

    def test_stop_escalates_only_the_owned_child_group_and_never_repeats_sigint(self):
        calls = []
        class Process:
            pid = 33
            returncode = None
            def poll(self):
                return self.returncode
            def wait(self, timeout):
                if len(calls) == 1:
                    raise subprocess.TimeoutExpired("ros2", timeout)
                self.returncode = 0
        process = Process()
        actions = stop_recorder(process, killpg=lambda pid, sig: calls.append((pid, sig)))
        self.assertEqual(actions, ["SIGINT", "SIGTERM"])
        self.assertEqual(calls, [(33, signal.SIGINT), (33, signal.SIGTERM)])
        self.assertEqual(stop_recorder(process, killpg=lambda *args: self.fail("already exited")), [])

    def test_windows_and_missing_camera_fail_before_creating_any_recorder(self):
        with self.assertRaisesRegex(RuntimeError, "Linux ROS 2"):
            record(platform_name="nt", popen=lambda *args: self.fail("must not start"))
        with self.assertRaisesRegex(RuntimeError, "no actual Image"):
            record(platform_name="posix", discovery=lambda: {"/robot_state": TOPICS["/robot_state"]},
                   require_camera=True, popen=lambda *args: self.fail("must not start"))

    @unittest.skipUnless(os.name == "posix", "requires real POSIX child sessions/signals")
    def test_real_bounded_child_process_is_stopped_and_its_storage_is_observed(self):
        script = (
            "import pathlib,signal,sys,time\n"
            "signal.signal(signal.SIGINT,lambda *_:sys.exit(0))\n"
            "bag=pathlib.Path(sys.argv[1]);bag.mkdir()\n"
            "(bag/'sample.db3').write_bytes(b'initialized test storage')\n"
            "for name in sys.argv[2:]:print(\"Subscribed to topic '\"+name+\"'\",flush=True)\n"
            "while True:time.sleep(.02)\n"
        )
        def child(command, **options):
            # Exercise real supervision without importing ROS or starting a
            # camera/robot publisher. Only the rosbag CLI executable is replaced.
            return subprocess.Popen([sys.executable, "-u", "-c", script, *command[command.index('-o')+1:]], **options)
        with tempfile.TemporaryDirectory() as folder:
            result = record(folder, seconds=.1, discovery=lambda: TOPICS, popen=child,
                            readiness_timeout=5., require_camera=True)
            self.assertTrue(result["ready"] and result["closed_cleanly"])
            self.assertEqual(result["stop_actions"], ["SIGINT"])
            self.assertEqual(result["recorder_exit_code"], 0)


if __name__ == "__main__":
    unittest.main()
