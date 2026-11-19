import json
from pathlib import Path
import tempfile
import unittest
import test_controller
from tg3_pico_dualarm.recording import SessionRecorder


class FailureRecordingTests(unittest.TestCase):
    def test_initiating_solver_failure_and_timing_survive_watchdog_and_shutdown(self):
        fixture=test_controller.ControllerTests();fixture.setUp()
        with tempfile.TemporaryDirectory() as folder:
            recorder=SessionRecorder(folder,{'physical_robot_commands_sent':False})
            fixture.c.recorder=recorder;fixture.tick();fixture.tick(1.)
            command_count=len(fixture.robot.commands)
            fixture.ik.fail=True
            fixture.ik.last_diagnostics={'backend':'explicit_test_stub','solve_s':.23}
            fixture.ik.reset=lambda side=None:setattr(fixture.ik,'last_diagnostics',{})
            status=fixture.tick(1.)
            self.assertEqual(len(fixture.robot.commands),command_count)
            self.assertEqual(status['failure_details']['last_ik']['solve_s'],.23)
            fixture.c.disarm('command_watchdog_expired');fixture.c.disarm('station_stopped');recorder.close()
            summary=json.loads((recorder.directory/'summary.json').read_text())
            self.assertEqual(summary['terminal_reason'],'station_stopped')
            failure=summary['first_control_failure']
            self.assertIn('injected_native_failure',failure['reason'])
            self.assertEqual(failure['details']['last_ik']['solve_s'],.23)
            self.assertGreaterEqual(failure['details']['tick_wall_s'],0.)

    def test_invalid_failure_details_cannot_corrupt_record_or_failure_summary(self):
        with tempfile.TemporaryDirectory() as folder:
            recorder=SessionRecorder(folder,{'physical_robot_commands_sent':False})
            with self.assertRaises(ValueError):
                recorder.event('control_failed:invalid',1.,details={'elapsed':float('nan')})
            self.assertIsNone(recorder.first_control_failure);self.assertIsNone(recorder.terminal_reason)
            self.assertEqual(recorder.events,0);recorder.close()
            self.assertEqual((recorder.directory/'steps.jsonl').read_text(),'')

    def test_solver_failure_is_saved_as_event_with_terminal_reason(self):
        fixture=test_controller.ControllerTests();fixture.setUp()
        with tempfile.TemporaryDirectory() as folder:
            recorder=SessionRecorder(folder,{'physical_robot_commands_sent':False})
            fixture.c.recorder=recorder
            fixture.tick();fixture.tick(1.)
            fixture.ik.fail=True;fixture.tick(1.)
            recorder.close()
            rows=[json.loads(row) for row in (recorder.directory/'steps.jsonl').read_text().splitlines()]
            self.assertEqual(rows[-1]['event'],'disarm')
            self.assertIn('injected_native_failure',rows[-1]['reason'])
            summary=json.loads((recorder.directory/'summary.json').read_text())
            self.assertIn('injected_native_failure',summary['terminal_reason'])

    def test_recording_failure_does_not_disarm_and_is_not_labeled_clean(self):
        fixture=test_controller.ControllerTests();fixture.setUp()
        with tempfile.TemporaryDirectory() as folder:
            recorder=SessionRecorder(folder,{'physical_robot_commands_sent':False})
            fixture.c.recorder=recorder
            def fail(_record):raise OSError('disk full')
            recorder.append=fail
            status=fixture.tick()
            self.assertNotEqual(status['state'],'disarmed')
            self.assertIn('disk full',status['recording_error'])
            recorder.close()
            summary=json.loads((recorder.directory/'summary.json').read_text())
            self.assertFalse(summary['closed_cleanly']);self.assertTrue(summary['recording_failed'])
