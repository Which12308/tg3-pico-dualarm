"""Real deployment profile must narrow software limits and never enable output."""
import json
from pathlib import Path
import tempfile
import unittest
from tg3_pico_dualarm.xarm_config import load_xarm_settings
from tg3_pico_dualarm.model import DualArmModel


ROOT=Path(__file__).resolve().parents[1]


class XArmConfigurationTests(unittest.TestCase):
    def test_checked_default_uses_actual_model_order_and_low_speed(self):
        settings=load_xarm_settings(ROOT/'config/xarm.yaml')
        self.assertEqual(settings.controller,'jointspace_ruckig_qp_controller')
        self.assertEqual(settings.topic,'/jointspace_commands_Dual')
        self.assertEqual(settings.velocity_cap,(.02,)*14)
        self.assertEqual(settings.lower,tuple(DualArmModel().limits[:,0]))
        self.assertEqual(settings.upper,tuple(DualArmModel().limits[:,1]))

    def test_profile_rejects_other_controller_and_relaxed_or_nonfinite_caps(self):
        value=json.loads((ROOT/'config/xarm.yaml').read_text(encoding='utf-8'))
        with tempfile.TemporaryDirectory() as folder:
            path=Path(folder)/'xarm.yaml'
            for field,bad in (('controller','wholebody_controller'),('topic','/arm/cmd'),
                ('velocity_cap_rad_s',.021),('velocity_cap_rad_s',.11),('velocity_cap_rad_s',True),
                ('feedback_lead_rad',.031),('feedback_lead_rad',float('nan')),
                ('limits_evidence',''),('hardware_motion_enabled',True)):
                candidate=dict(value);candidate[field]=bad
                path.write_text(json.dumps(candidate),encoding='utf-8')
                with self.subTest(field=field,bad=bad),self.assertRaises(ValueError):load_xarm_settings(path)


if __name__=='__main__':unittest.main()
