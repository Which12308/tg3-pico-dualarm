import json
from pathlib import Path
import tempfile
import unittest
from tg3_pico_dualarm.cli import demo
from tg3_pico_dualarm.recording import SessionRecorder


class RecordingTests(unittest.TestCase):
    def test_preview_records_raw_input_targets_and_truthful_no_ik_metadata(self):
        with tempfile.TemporaryDirectory() as folder:
            result=demo(100,record=folder,preview=True)
            self.assertTrue(result['passed']);self.assertFalse(result['native_placo_used'])
            self.assertEqual(result['simulated_commands'],0)
            directory=Path(result['recording'])
            metadata=json.loads((directory/'metadata.json').read_text())
            records=[json.loads(line) for line in (directory/'steps.jsonl').read_text().splitlines()]
            self.assertFalse(metadata['physical_robot_commands_sent'])
            self.assertFalse(metadata['camera_frames_included'])
            self.assertEqual(len(records),101)
            self.assertIn('head',records[10]['tracking'])
            self.assertIn('left',records[30]['mapping']['targets'])
            self.assertTrue(all(r['command'] is None for r in records))
            self.assertTrue(json.loads((directory/'summary.json').read_text())['closed_cleanly'])
            self.assertFalse(metadata['native_placo_used'])
            self.assertTrue(metadata['cartesian_preview_only'])

    def test_recording_rejects_nonfinite_values_without_partial_json(self):
        with tempfile.TemporaryDirectory() as folder:
            recorder=SessionRecorder(folder,{'physical_robot_commands_sent':False})
            with self.assertRaises(ValueError):recorder.append({'broken':float('nan')})
            recorder.append({'timestamp':1.,'event':'valid'})
            recorder.close();recorder.close()
            rows=(recorder.directory/'steps.jsonl').read_text().splitlines()
            self.assertEqual(len(rows),1);self.assertEqual(json.loads(rows[0])['event'],'valid')
