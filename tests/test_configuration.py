import json
from pathlib import Path
import tempfile
import unittest
from tg3_pico_dualarm.configuration import load,read_json_yaml
from tg3_pico_dualarm.watchdog import CommandWatchdog


class ConfigurationTests(unittest.TestCase):
    def test_default_is_safe_and_hardware_cannot_be_enabled_by_configuration(self):
        root=Path(__file__).resolve().parents[1]/'config'
        config=load(root)
        self.assertFalse(config['robot']['hardware_motion_enabled'])
        with tempfile.TemporaryDirectory() as folder:
            dest=Path(folder)
            for path in root.iterdir():(dest/path.name).write_bytes(path.read_bytes())
            bad=config['robot'].copy();bad['mode']='hardware'
            (dest/'robot.yaml').write_text(json.dumps(bad))
            with self.assertRaisesRegex(ValueError,'hardware_mode_requires_verified'):
                load(dest)

    def test_watchdog_latches_expiry_and_does_not_auto_rearm(self):
        clock=[1.];expired=[]
        watchdog=CommandWatchdog(lambda:expired.append(True),monotonic=lambda:clock[0])
        clock[0]+=.1;self.assertFalse(watchdog.check());watchdog.beat()
        clock[0]+=.16;self.assertTrue(watchdog.check())
        watchdog.beat();self.assertTrue(watchdog.check())
        self.assertEqual(expired,[True])
