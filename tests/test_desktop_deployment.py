"""Deployment/installer checks without SSH, ADB devices or physical motion."""
from dataclasses import asdict
import hashlib
import io
import json
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace as NS
import unittest
from unittest.mock import patch
import zipfile

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'desktop'))
from install_headset import artifact_info,install,prepare_usb
from tg3_pico_dualarm.deployment import make_robot_bundle,reviewed_factory_overrides,RobotDeployment
from test_manufacturer_session import limits


class BundleUploadTests(unittest.TestCase):
    def test_new_reviewed_config_files_enter_once_deterministically_and_research_is_pruned(self):
        with tempfile.TemporaryDirectory() as folder:
            root=Path(folder)
            for name in ('src/main.py','manufacturer_research/secret.py','.headset_tools/sdk.zip',
                         'config/factory.json','config/factory_driver.py','headset_native/cpp/main.cpp'):
                file=root/name;file.parent.mkdir(parents=True,exist_ok=True);file.write_text('not selected')
            overrides={'config/factory.json':b'{"reviewed":true}','config/factory_driver.py':b'# selected source'}
            first=make_robot_bundle(root,overrides);second=make_robot_bundle(root,overrides)
            self.assertEqual(first,second)
            with zipfile.ZipFile(io.BytesIO(first)) as z:
                self.assertEqual(set(z.namelist()),{'src/main.py','config/factory.json','config/factory_driver.py'})
                self.assertEqual(z.read('config/factory_driver.py'),b'# selected source')
            with zipfile.ZipFile(io.BytesIO(make_robot_bundle(root))) as z:
                self.assertEqual(z.namelist(),['src/main.py'])

    def test_override_paths_cannot_escape_or_replace_application_source(self):
        with tempfile.TemporaryDirectory() as folder:
            for name in ('/config/f.py','../config/f.py','config/../f.py','config\\f.py','C:/config/f.py',
                         'config//f.py','config/./f.py','src/control.py'):
                with self.subTest(name=name),self.assertRaisesRegex(ValueError,'override_path'):
                    make_robot_bundle(folder,{name:b'test'})

    def test_only_reviewed_json_and_exact_driver_source_are_uploaded_and_descriptor_path_is_rebased(self):
        with tempfile.TemporaryDirectory() as folder:
            root=Path(folder);driver=root/'my_sdk_driver.py';marker=root/'executed'
            driver.write_text('raise RuntimeError("must not execute while packaging")\n',encoding='utf-8')
            descriptor=root/'hardware.json'
            value=dict(schema='tg3_factory_bundle_v1',driver=dict(path=driver.name,
                sha256=hashlib.sha256(driver.read_bytes()).hexdigest(),factory='create_driver',
                evidence=dict(controller='TEST',version='fixture',interface_reference='fixture')),
                limits=asdict(limits()),hands={})
            descriptor.write_text(json.dumps(value),encoding='utf-8')
            digest=hashlib.sha256(descriptor.read_bytes()).hexdigest()
            result=reviewed_factory_overrides(descriptor,digest)
            self.assertEqual(set(result),{'config/factory.json','config/factory_driver.py'})
            rewritten=json.loads(result['config/factory.json'])
            self.assertEqual(rewritten['driver']['path'],'factory_driver.py')
            self.assertEqual(hashlib.sha256(result['config/factory_driver.py']).hexdigest(),rewritten['driver']['sha256'])
            self.assertFalse(marker.exists())
            descriptor.write_text(descriptor.read_text()+'\n',encoding='utf-8')
            with self.assertRaisesRegex(ValueError,'descriptor_changed'):reviewed_factory_overrides(descriptor,digest)

    def test_hardware_start_uses_explicit_factory_argument_and_never_enables_motion(self):
        import tg3_pico_dualarm
        for mode,backend in (('dry_run','manufacturer'),('hardware','manufacturer'),('hardware','xarm')):
            sent=[]
            event=dict(event='robot_station_started',version=tg3_pico_dualarm.__version__,protocol='tg3-native-1',listen_port=8878,runtime_mode=mode,hardware_motion_enabled=False)
            class Channel:
                closed=True
                def exec_command(self,value):sent.append(value)
                def sendall(self,value):self.received=getattr(self,'received',[])+[value]
                def recv_ready(self):return not getattr(self,'consumed',False)
                def recv(self,_):
                    self.consumed=True
                    return (json.dumps(event)+'\n').encode()
                def recv_stderr_ready(self):return False
                def recv_stderr(self,_):return b''
                def exit_status_ready(self):return True
                def recv_exit_status(self):return 0
            channel=Channel()
            deployment=RobotDeployment(ROOT,Path('unused'),lambda *args:False,lambda *args:None,runtime_mode=mode,hardware_backend=backend)
            deployment.remote='/home/ubuntu/release with space'
            deployment.ssh=NS(get_transport=lambda:NS(open_session=lambda:channel))
            with patch('tg3_pico_dualarm.deployment.Tunnel',return_value=NS(port=12345)):
                self.assertEqual(deployment.start('k'*40,administrator_password='private fixture password' if backend=='xarm' else None),12345)
            self.assertEqual('--hardware' in sent[0],mode=='hardware')
            self.assertEqual('--factory-config' in sent[0],mode=='hardware' and backend=='manufacturer')
            self.assertEqual('--xarm' in sent[0],mode=='hardware' and backend=='xarm')
            self.assertNotIn('enable',sent[0]);self.assertNotIn('k'*40,sent[0])
            self.assertNotIn('private fixture password',sent[0])
            managed=mode=='hardware' and backend=='xarm'
            self.assertEqual(sent[0].startswith("sudo -k -S -p '' /bin/bash "),managed)
            self.assertEqual(channel.received,([b'private fixture password\n'] if managed else [])+[b'k'*40+b'\n'])

    def test_managed_start_refuses_missing_admin_secret_before_opening_command_channel(self):
        deployment=RobotDeployment(ROOT,Path('unused'),lambda *a:False,lambda *a:None,runtime_mode='hardware',hardware_backend='xarm')
        deployment.remote='/home/ubuntu/reviewed';deployment.ssh=NS(get_transport=lambda:(_ for _ in ()).throw(AssertionError('opened channel')))
        with self.assertRaisesRegex(RuntimeError,'管理员密码'):deployment.start('k'*40)

    def test_start_refuses_unconfirmed_mode_or_non_disabled_hardware_state(self):
        import tg3_pico_dualarm
        base=dict(event='robot_station_started',version=tg3_pico_dualarm.__version__,protocol='tg3-native-1',
            listen_port=8878,runtime_mode='hardware',hardware_motion_enabled=False)
        for field,bad in (('runtime_mode',None),('runtime_mode','dry_run'),
                          ('hardware_motion_enabled',None),('hardware_motion_enabled',True)):
            event=dict(base);event[field]=bad
            class Channel:
                def exec_command(self,value):pass
                def sendall(self,value):pass
                def recv_ready(self):return True
                def recv(self,_):return (json.dumps(event)+'\n').encode()
                def recv_stderr_ready(self):return False
                def exit_status_ready(self):return False
            deployment=RobotDeployment(ROOT,Path('unused'),lambda *args:False,lambda *args:None,runtime_mode='hardware')
            deployment.remote='/reviewed/release';deployment.ssh=NS(get_transport=lambda:NS(open_session=lambda:Channel()))
            with self.subTest(field=field,value=bad),patch('tg3_pico_dualarm.deployment.Tunnel') as tunnel:
                with self.assertRaises(RuntimeError):deployment.start('k'*40)
                tunnel.assert_not_called()


class HeadsetInstallerTests(unittest.TestCase):
    def fixture(self,root):
        apk=Path(root)/'dist/TG3-Wireless-0.4.0.apk';apk.parent.mkdir();apk.write_bytes(b'explicit mock Android artifact')
        receipt=dict(version='0.4.0',native_build_executed=True,signature_verified=True,protocol='tg3-native-1',
            coordinates='openxr_rh_y_up',application_id='edu.scu.tg3.wireless',abi='arm64-v8a',min_sdk=30,
            sha256=hashlib.sha256(apk.read_bytes()).hexdigest())
        apk.with_suffix('.json').write_text(json.dumps(receipt),encoding='utf-8');return apk

    def test_installer_verifies_receipt_hash_device_and_starts_native_pair_activity(self):
        props={'ro.product.manufacturer':'PICO','ro.product.brand':'PICO','ro.product.model':'A9210',
               'ro.product.cpu.abilist':'arm64-v8a,armeabi-v7a','ro.build.version.sdk':'34'}
        def adb(command,**kwargs):
            return 'List of devices attached\nPICO_FIXTURE\tdevice\n' if command[-1]=='devices' else props[command[-1]]
        with tempfile.TemporaryDirectory() as folder:
            self.fixture(folder)
            with patch('install_headset.subprocess.check_output',side_effect=adb),patch('install_headset.subprocess.run') as run:
                result=install(folder,'mock_adb',version='0.4.0')
            self.assertIn('保持数据线连接',result)
            commands=[call.args[0] for call in run.call_args_list]
            self.assertIn('install',commands[0]);self.assertEqual(commands[-1][-1],'edu.scu.tg3.wireless/.PairActivity')
            self.assertEqual(commands[1],['mock_adb','-s','PICO_FIXTURE','reverse','tcp:8877','tcp:8877'])

    def test_usb_reverse_refuses_ambiguous_or_unauthorized_devices_without_modification(self):
        for devices in ('PICO1\tdevice\nPICO2\tdevice\n','PICO1\tunauthorized\n','PICO1\toffline\n'):
            with self.subTest(devices=devices),patch('install_headset.subprocess.check_output',return_value=devices),patch('install_headset.subprocess.run') as run:
                with self.assertRaisesRegex(RuntimeError,'恰好一台'):prepare_usb('mock_adb')
                run.assert_not_called()

    def test_usb_reverse_rechecks_expected_device_before_install_launch(self):
        props={'ro.product.manufacturer':'PICO','ro.product.brand':'PICO','ro.product.model':'A9210',
               'ro.product.cpu.abilist':'arm64-v8a','ro.build.version.sdk':'34'}
        def adb(command,**kwargs):
            return 'PICO_REPLACED\tdevice\n' if command[-1]=='devices' else props[command[-1]]
        with patch('install_headset.subprocess.check_output',side_effect=adb),patch('install_headset.subprocess.run') as run:
            with self.assertRaisesRegex(RuntimeError,'发生变化'):prepare_usb('mock_adb',expected_serial='ORIGINAL')
            run.assert_not_called()

    def test_modified_apk_or_non_pico_device_cannot_install(self):
        with tempfile.TemporaryDirectory() as folder:
            apk=self.fixture(folder);apk.write_bytes(b'changed')
            with self.assertRaisesRegex(RuntimeError,'校验不通过'):artifact_info(folder,'0.4.0')
        with tempfile.TemporaryDirectory() as folder:
            self.fixture(folder)
            def adb(command,**kwargs):return 'test\tdevice\n' if command[-1]=='devices' else 'other phone'
            with patch('install_headset.subprocess.check_output',side_effect=adb),patch('install_headset.subprocess.run') as run:
                with self.assertRaisesRegex(RuntimeError,'未识别为 PICO'):install(folder,'mock_adb','0.4.0')
                run.assert_not_called()

    def test_pico_with_incompatible_architecture_or_android_version_is_not_modified(self):
        for override in ({'ro.product.cpu.abilist':'armeabi-v7a'},
                         {'ro.build.version.sdk':'29'},{'ro.build.version.sdk':'unknown'}):
            props={'ro.product.manufacturer':'PICO','ro.product.brand':'PICO','ro.product.model':'A9210',
                'ro.product.cpu.abilist':'arm64-v8a,armeabi-v7a','ro.build.version.sdk':'34'}
            props.update(override)
            def adb(command,**kwargs):
                return 'test\tdevice\n' if command[-1]=='devices' else props[command[-1]]
            with self.subTest(properties=override),tempfile.TemporaryDirectory() as folder:
                self.fixture(folder)
                with patch('install_headset.subprocess.check_output',side_effect=adb),patch('install_headset.subprocess.run') as run:
                    with self.assertRaises(RuntimeError):install(folder,'mock_adb','0.4.0')
                    run.assert_not_called()

    @unittest.skipUnless((ROOT/'dist/TG3-Wireless-0.4.0.apk').exists(),'Built Android artifact absent in this checkout')
    def test_current_real_built_apk_matches_its_native_build_receipt(self):
        apk,receipt=artifact_info(ROOT,'0.4.0')
        self.assertGreater(apk.stat().st_size,1_000_000)
        self.assertTrue(receipt['native_build_executed']);self.assertTrue(receipt['signature_verified'])


if __name__=='__main__':unittest.main()
