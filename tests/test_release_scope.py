"""Public packaging with explicit fixtures, never scan real keys/VMs or freeze a release."""
import contextlib
import hashlib
import io
import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch
import zipfile

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'scripts'))
from release_scope import public_source,robot_bytes
from delivery_report import report
from build_release import build_release
from tg3_pico_dualarm import __version__


class ReleaseScopeTests(unittest.TestCase):
    def fixture(self,root):
        (root/'README.md').write_text('Explicit test project, not a real release',encoding='utf-8')
        source=root/'headset_native/cpp/main.cpp';source.parent.mkdir(parents=True);source.write_text('// fixture')
        (root/'src').mkdir();(root/'src/test.py').write_text('# fixture')
        artifacts=root/'artifacts';artifacts.mkdir()
        (artifacts/'test_result.json').write_text(json.dumps(dict(tests_run=1,passed=1,failures=0,errors=0,skipped=[])))
        (artifacts/'native_execution_result.json').write_text(json.dumps(dict(version='0.3.0',
            real_linux_execution=True,real_placo_executed=True,source_bundle_sha256='historical-fixture')))
        dist=root/'dist';dist.mkdir();apk=dist/('TG3-Wireless-'+__version__+'.apk')
        with zipfile.ZipFile(apk,'w') as z:
            for name in ('AndroidManifest.xml','classes.dex','lib/arm64-v8a/libtg3_openxr.so',
                         'lib/arm64-v8a/libopenxr_loader.so','assets/licenses/LICENSE.txt'):
                z.writestr(name,b'explicit mock artifact, not an Android binary')
        apk.with_suffix('.json').write_text(json.dumps(dict(version=__version__,native_build_executed=True,
            signature_verified=True,protocol='tg3-native-1',coordinates='openxr_rh_y_up',
            application_id='edu.scu.tg3.wireless',abi='arm64-v8a',
            sha256=hashlib.sha256(apk.read_bytes()).hexdigest(),
            source_files={'cpp/main.cpp':hashlib.sha256(source.read_bytes()).hexdigest()})))
        return source

    def test_walk_prunes_private_caches_and_does_not_read_unselected_files(self):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory).resolve();self.fixture(root)
            forbidden=['.headset_tools/sdk/huge.zip','manufacturer_research/secret.py','artifacts/native_environment/id_ed25519',
                '.desktop/credentials.json','.ssh/id_rsa','config/factory.json','config/factory_driver.py',
                'docs/private_key.pem','unselected_root/private.py','src/nested/.venv/secret.py']
            for name in forbidden:
                file=root/name;file.parent.mkdir(parents=True,exist_ok=True);file.write_text('PRIVATE FIXTURE')
            original=Path.read_bytes;reads=[]
            def read(file):
                name=file.relative_to(root).as_posix();reads.append(name)
                if name in forbidden:raise AssertionError('read excluded private fixture: '+name)
                return original(file)
            with patch.object(Path,'read_bytes',read):source=public_source(root)
            names={name for name,_ in source}
            self.assertIn('README.md',names);self.assertIn('headset_native/cpp/main.cpp',names)
            self.assertFalse(names.intersection(forbidden));self.assertEqual(names,set(reads))
            with zipfile.ZipFile(io.BytesIO(robot_bytes(source))) as z:
                self.assertNotIn('headset_native/cpp/main.cpp',z.namelist());self.assertIn('src/test.py',z.namelist())

    def test_ledger_retains_historical_native_scope_without_claiming_current_or_physical(self):
        with tempfile.TemporaryDirectory() as directory,contextlib.redirect_stdout(io.StringIO()):
            root=Path(directory);self.fixture(root);value=report(root)
            self.assertEqual(value['version'],__version__);self.assertTrue(value['headset_apk_built'])
            self.assertTrue(value['linux_native_ik_executed'])
            old=value['evidence_snapshots']['native_execution_result.json']
            self.assertEqual(old['recorded_version'],'0.3.0');self.assertFalse(old['same_project_version'])
            self.assertFalse(old['matches_current_robot_source']);self.assertFalse(value['hardware_ready'])
            self.assertFalse(value['physical_headset_acceptance_verified'])

    def test_release_contains_dynamic_apk_two_native_libraries_licenses_robot_and_selected_evidence(self):
        with tempfile.TemporaryDirectory() as directory,contextlib.redirect_stdout(io.StringIO()):
            root=Path(directory);self.fixture(root)
            (root/'artifacts/secret.json').write_text('{"PRIVATE_FIXTURE":true}')
            report(root);result=build_release(root,'fixture')
            with zipfile.ZipFile(result['archive']) as z:
                prefix='tg3_pico_dualarm/'
                self.assertIn(prefix+'robot/tg3_robot_fixture.zip',z.namelist())
                self.assertFalse(any('secret.json' in name for name in z.namelist()))
                apk=z.read(prefix+'dist/TG3-Wireless-'+__version__+'.apk')
                with zipfile.ZipFile(io.BytesIO(apk)) as a:
                    self.assertIn('lib/arm64-v8a/libtg3_openxr.so',a.namelist())
                    self.assertIn('lib/arm64-v8a/libopenxr_loader.so',a.namelist())
                    self.assertIn('assets/licenses/LICENSE.txt',a.namelist())
            with self.assertRaisesRegex(RuntimeError,'frozen'):build_release(root,'fixture')

    def test_code_change_after_ledger_prevents_freezing_and_headset_change_requires_rebuild(self):
        with tempfile.TemporaryDirectory() as directory,contextlib.redirect_stdout(io.StringIO()):
            root=Path(directory);source=self.fixture(root);report(root)
            (root/'src/test.py').write_text('# changed after ledger')
            with self.assertRaisesRegex(RuntimeError,'source changed'):build_release(root,'fixture')
            self.assertFalse((root/'dist/tg3_pico_dualarm_fixture.zip').exists())
            source.write_text('// changed native source')
            value=report(root)
            self.assertFalse(value['headset_apk_built'])
            self.assertIn('rebuild_required',value['blocked'][-1]['reason'])


if __name__=='__main__':unittest.main()
