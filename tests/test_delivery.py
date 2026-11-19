import hashlib
import io
import json
from pathlib import Path
import tempfile
import unittest
import zipfile
import numpy as np
from tg3_pico_dualarm.deployment import make_robot_bundle,validate_host
from tg3_pico_dualarm.readiness import readiness
from tg3_pico_dualarm.tool_setup import measured_tools,transform_from_mm_degrees

ROOT=Path(__file__).resolve().parents[1]


class DeliveryTests(unittest.TestCase):
    def test_deployment_bundle_is_deterministic_and_excludes_secrets_logs_and_venvs(self):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory)
            for name in ('src/main.py','.desktop/known_hosts','.venv/key','recordings/run.json',
                         'artifacts/result.json','headset/generated_client/file','desktop/ui.py','config/vr.yaml'):
                path=root/name;path.parent.mkdir(parents=True,exist_ok=True);path.write_text('fixture')
            a=make_robot_bundle(root,{'config/vr.yaml':b'{"mode":"position"}'})
            b=make_robot_bundle(root,{'config/vr.yaml':b'{"mode":"position"}'})
            self.assertEqual(hashlib.sha256(a).digest(),hashlib.sha256(b).digest())
            with zipfile.ZipFile(io.BytesIO(a)) as z:
                self.assertEqual(set(z.namelist()),{'src/main.py','config/vr.yaml'})
                self.assertEqual(json.loads(z.read('config/vr.yaml'))['mode'],'position')

    def test_readiness_never_turns_full_input_or_offline_success_into_hardware_authority(self):
        value=readiness(tracking=dict(head_tracked=True,left_tracked=True,right_tracked=True),native_ik=True,
            tools_calibrated=True,ros_feedback=True,headset_build=True,hardware_acceptance=True)
        self.assertTrue(value['wireless_input_ready']);self.assertFalse(value['hardware_ready'])
        self.assertFalse(value['hardware_motion_enabled']);self.assertGreaterEqual(len(value['blockers']),4)

    def test_tool_form_has_real_units_axis_order_and_requires_measurement_confirmation(self):
        matrix=np.array(transform_from_mm_degrees([100,0,0,0,0,90]))
        np.testing.assert_allclose(matrix[:3,3],[.1,0,0])
        np.testing.assert_allclose(matrix[:3,:3]@[1,0,0],[0,1,0],atol=1e-12)
        entries={name:{side:[0]*6 for side in ('left','right')} for name in ('controller_to_hand','tcp_to_hand')}
        with self.assertRaises(ValueError):measured_tools({},entries,'',True)
        with self.assertRaises(ValueError):measured_tools({},entries,'fixture',False)
        self.assertTrue(measured_tools({},entries,'fixture',True)['tool_offsets_calibrated'])

    def test_ssh_target_cannot_be_interpreted_as_shell_options_or_commands(self):
        validate_host('192.0.2.10','ubuntu')
        for host,user in (('-oProxyCommand=x','ubuntu'),('host;bad','ubuntu'),('good','root;bad')):
            with self.assertRaises(ValueError):validate_host(host,user)

    def test_defaults_match_wireless_contract_one_to_one_and_hardware_off(self):
        from tg3_pico_dualarm.configuration import load
        value=load(ROOT/'config')
        self.assertEqual(value['vr']['translation_scale'],1.)
        self.assertEqual(value['vr']['orientation_scale'],1.)
        self.assertFalse(value['robot']['hardware_motion_enabled'])


if __name__=='__main__':unittest.main()
