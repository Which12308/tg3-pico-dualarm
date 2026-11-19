"""Device/SSH doubles and real localhost gateways. No physical ADB or robot access."""
import hashlib,json,os,subprocess,tempfile,time,unittest
from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import patch
from tg3_pico_dualarm.robot_usb import RobotHeadsetUSB,PACKAGE
from tg3_pico_dualarm.usb_preflight import verified_packages,install_missing_adb,wait_for_usb
from tg3_pico_dualarm.wireless import InputGateway,WirelessServer,GatewayClient
from tg3_pico_dualarm.timing import TimingStats

class AdbDouble:
    def __init__(self):
        self.calls=[];self.devices='PICO1\tdevice\n';self.brand='PICO';self.mapping='';self.app=True;self.version='0.4.0';self.fail=None
    def __call__(self,argv,**kw):
        args=argv[1:];self.calls.append(args)
        if self.fail and self.fail(args):raise subprocess.TimeoutExpired(argv,8)
        result=''
        if args==['devices']:result='List of devices attached\n'+self.devices
        elif args[-2]=='getprop':result=self.brand if args[-1]!='ro.product.model' else self.brand+(' A9210' if self.brand=='PICO' else '')
        elif args[-3:]==['pm','path',PACKAGE]:result='package:/data/app/ours/base.apk' if self.app else ''
        elif 'dumpsys' in args:result='versionName='+self.version+'\n'
        elif args[-2:]==['reverse','--list']:result=self.mapping
        elif len(args)>4 and args[2:5]==['reverse','--no-rebind','tcp:8877']:self.mapping='UsbFfs tcp:8877 '+args[5]
        elif '--remove' in args:self.mapping=''
        return NS(returncode=0,stdout=result,stderr='')

class USBTests(unittest.TestCase):
    def setUp(self):self.run=AdbDouble();self.usb=RobotHeadsetUSB('fake-adb',run=self.run)
    def test_missing_unauthorized_ambiguous_wrong_device_app_and_version_before_any_write(self):
        with patch('tg3_pico_dualarm.robot_usb.shutil.which',return_value=None):
            with self.assertRaisesRegex(RuntimeError,'adb_missing'):RobotHeadsetUSB()
        cases=[('devices','PICO1\tunauthorized\n'),('devices','PICO1\tdevice\nPICO2\tdevice\n'),
            ('devices','PICO1\toffline\n'),('brand','OTHER'),('app',False),('version','0.3.0')]
        for name,value in cases:
            run=AdbDouble();setattr(run,name,value)
            with self.subTest(name=name,value=value),self.assertRaises(RuntimeError):RobotHeadsetUSB('fake-adb',run=run).prepare(8878)
            self.assertFalse(any('force-stop' in c or ('reverse' in c and '--list' not in c) for c in run.calls))
    def test_prepare_own_app_and_cleanup_only_matching_reverse(self):
        identity=self.usb.prepare(49000)
        self.assertTrue(identity['device_authorized']);self.assertEqual(identity['app_version'],'0.4.0')
        self.assertIn(['-s','PICO1','shell','am','force-stop',PACKAGE],self.run.calls)
        self.assertEqual(self.run.calls[-1][-1],PACKAGE+'/.PairActivity')
        self.usb.close();self.assertEqual(self.run.mapping,'')
        self.assertTrue(all('kill-server' not in call and '--remove-all' not in call and 'uninstall' not in call for call in self.run.calls))
    def test_existing_reverse_is_not_overwritten_and_replaced_mapping_not_removed(self):
        self.run.mapping='UsbFfs tcp:8877 tcp:12345'
        with self.assertRaisesRegex(RuntimeError,'已有转发'):self.usb.prepare(49000)
        self.assertFalse(any('force-stop' in c for c in self.run.calls))
        self.run.mapping='';self.usb.prepare(49000);self.run.mapping='UsbFfs tcp:8877 tcp:55555'
        self.usb.close();self.assertIn('55555',self.run.mapping)
    def test_partial_launch_timeout_cleans_created_reverse(self):
        self.run.fail=lambda args:'start' in args
        with self.assertRaises(subprocess.TimeoutExpired):self.usb.prepare(49000)
        self.assertEqual(self.run.mapping,'');self.assertIsNone(self.usb.target)
    def test_wait_for_authorization_is_bounded_and_never_starts_hardware(self):
        clock=[0.];calls=[]
        def factory():
            def check():
                calls.append(1)
                if clock[0]<2:raise RuntimeError('请在头显允许USB调试授权')
                return {'ready':True}
            return NS(check_available=check)
        self.assertTrue(wait_for_usb(3,device_factory=factory,clock=lambda:clock[0],sleep=lambda n:clock.__setitem__(0,clock[0]+n),notify=lambda _:None)['ready'])
        self.assertEqual(len(calls),3)

class CacheTests(unittest.TestCase):
    def fixture(self,directory):
        data=b'!<arch>\nTEST_NOT_REAL_DEB';path=Path(directory)/'adb_fixture_amd64.deb';path.write_bytes(data)
        manifest=dict(schema='tg3_ubuntu_adb_cache_v1',distribution='ubuntu',version='24.04',architecture='amd64',
            packages=[dict(package='adb',version='fixture',file=path.name,bytes=len(data),sha256=hashlib.sha256(data).hexdigest())])
        file=Path(directory)/'manifest.json';file.write_text(json.dumps(manifest));return hashlib.sha256(file.read_bytes()).hexdigest()
    def test_validates_hashes_paths_architecture_and_corruption_before_install(self):
        with tempfile.TemporaryDirectory() as folder:
            sha=self.fixture(folder);self.assertEqual(verified_packages(folder,sha)[1],{'adb'})
            with self.assertRaises(RuntimeError):verified_packages(folder,'0'*64)
            (Path(folder)/'adb_fixture_amd64.deb').write_bytes(b'changed')
            with self.assertRaises(RuntimeError):verified_packages(folder,sha)
    def test_simulation_refuses_other_system_changes_and_no_install_runs(self):
        calls=[]
        def run(args,**kwargs):
            calls.append(args)
            if args[0]=='dpkg-query':return NS(returncode=1,stdout='',stderr='')
            return NS(returncode=0,stdout='Inst libc6 (unexpected system upgrade)\n',stderr='')
        with tempfile.TemporaryDirectory() as folder:
            sha=self.fixture(folder)
            with (patch('tg3_pico_dualarm.usb_preflight.os.geteuid',return_value=0,create=True),
                patch('tg3_pico_dualarm.usb_preflight.platform.freedesktop_os_release',return_value={'ID':'ubuntu','VERSION_ID':'24.04'}),
                patch('tg3_pico_dualarm.usb_preflight.platform.machine',return_value='x86_64')):
                with self.assertRaisesRegex(RuntimeError,'缓存以外'):install_missing_adb(folder,sha,run=run,which=lambda _:None)
        self.assertFalse(any('-y' in c for c in calls))
    def test_already_installed_does_not_need_packages_or_admin(self):
        self.assertFalse(install_missing_adb('/missing','bad',which=lambda _:'/usr/bin/adb')['installed_now'])

    def test_only_missing_cached_packages_install_with_no_download_upgrade_or_removal(self):
        calls=[];installed=[False]
        def run(args,**kwargs):
            calls.append(args)
            if args[0]=='dpkg-query':return NS(returncode=1,stdout='',stderr='')
            if '-s' in args:return NS(returncode=0,stdout='Inst adb (fixture local)\n',stderr='')
            installed[0]=True;return NS(returncode=0,stdout='',stderr='')
        with tempfile.TemporaryDirectory() as folder:
            sha=self.fixture(folder)
            with (patch('tg3_pico_dualarm.usb_preflight.os.geteuid',return_value=0,create=True),
                patch('tg3_pico_dualarm.usb_preflight.platform.freedesktop_os_release',return_value={'ID':'ubuntu','VERSION_ID':'24.04'}),
                patch('tg3_pico_dualarm.usb_preflight.platform.machine',return_value='x86_64')):
                result=install_missing_adb(folder,sha,run=run,which=lambda _: '/usr/bin/adb' if installed[0] else None)
        self.assertTrue(result['installed_now'])
        install=next(c for c in calls if '-y' in c)
        for flag in ('--no-download','--no-remove','--no-upgrade'):self.assertIn(flag,install)

class USBDeploymentTests(unittest.TestCase):
    def test_same_admin_identity_preflight_streams_password_not_argv_and_validates_upload(self):
        from tg3_pico_dualarm.deployment import RobotDeployment
        import io
        calls=[];uploaded=[]
        class Sink(io.BytesIO):
            def close(self):uploaded.append(self.getvalue());super().close()
        sftp=NS(stat=lambda _:object(),file=lambda *args:Sink(),close=lambda:None)
        with tempfile.TemporaryDirectory() as root:
            cache=Path(root)/'usb_dependencies';cache.mkdir();CacheTests().fixture(cache)
            deployment=RobotDeployment(root,Path(root)/'known_hosts',lambda *a:False,lambda _:None,
                runtime_mode='hardware',hardware_backend='xarm',headset_transport='robot_usb')
            deployment.ssh=NS(open_sftp=lambda:sftp);deployment.remote='/home/ubuntu/fixture-release'
            def command(value,timeout,stdin_data=None):
                calls.append((value,stdin_data));return json.dumps({'event':'robot_usb_preflight','ready':True,'operator_uid':0})
            deployment.command=command
            self.assertTrue(deployment.prepare_robot_usb('fixture-password')['ready'])
            self.assertIn('sudo -k -S',calls[0][0]);self.assertNotIn('fixture-password',calls[0][0])
            self.assertEqual(calls[0][1],b'fixture-password\n');self.assertEqual(len(uploaded),2)
            self.assertIn('--install-missing',calls[0][0]);self.assertIn('--manifest-sha',calls[0][0])
            deployment.command=lambda *a,**kw:json.dumps({'event':'robot_usb_preflight','ready':True,'operator_uid':1000})
            with self.assertRaisesRegex(RuntimeError,'身份'):deployment.prepare_robot_usb('fixture-password')


class DirectGatewayTests(unittest.TestCase):
    def test_real_http_direct_input_and_repair_no_relay_or_automatic_recording(self):
        from test_wireless import native
        key='x'*40;main=InputGateway(role='relay',key=key);peer=InputGateway();peer.tracking=main.tracking
        main.accept_frames=False;main.headset_gateway=peer;main.headset_pairing_allowed=lambda:True
        head_server=WirelessServer(peer,'127.0.0.1',0).start();server=WirelessServer(main,'127.0.0.1',0).start()
        reader=GatewayClient(*server.address,key,timeout=1.);head=GatewayClient(*head_server.address,timeout=1.)
        try:
            head.token=peer.pair(peer.open_pairing(),'direct-input-fixture');head.send(native())
            status=reader.request('GET','/v1/status');self.assertTrue(status['tracking']['right_tracked'])
            with self.assertRaisesRegex(ValueError,'exclusive'):reader.send({'protocol':'tg3-native-1','frame':native()['frame']})
            reader.close()  # Rejected POST closes its HTTP connection by design.
            self.assertIsNone(main.on_frame);self.assertIsNone(peer.on_frame);self.assertIsNone(main.recording_token)
            old=head.token;new=reader.reopen_headset_pairing()['pin']
            with self.assertRaises(ValueError):peer.challenge(old)
            head.token=peer.pair(new,'direct-input-fixture')
            held=native(0,session='repaired');held['frame']['right']['grip']=1.;head.send(held)
            self.assertTrue(main.tracking.sample().right.release_required)
            head.send(native(1,session='repaired'));self.assertFalse(main.tracking.sample().right.release_required)
            main.headset_pairing_allowed=lambda:False
            with self.assertRaisesRegex(ValueError,'disable_motion'):reader.reopen_headset_pairing()
        finally:head.close();reader.close();server.close();head_server.close()
    def test_timing_histograms_bounded_and_cached(self):
        clock=[1.];stats=TimingStats(capacity=5,clock=lambda:clock[0])
        for i in range(10):stats.add('cycle',i*.001)
        self.assertEqual(len(stats.values['cycle']),5);self.assertAlmostEqual(stats.snapshot()['cycle']['p99_ms'],9.)
        stats.add('cycle',.1);self.assertAlmostEqual(stats.snapshot()['cycle']['max_ms'],9.)
        clock[0]+=1;self.assertEqual(stats.snapshot()['cycle']['max_ms'],100.)

    def test_expiration_count_is_not_repeated_by_status_polls(self):
        from test_wireless import native
        clock=[1.];gateway=InputGateway(clock=lambda:clock[0]);key=gateway.pair(gateway.open_pairing(),'fixture')
        gateway.accept(key,gateway.challenge(key)['nonce'],native())
        clock[0]+=.16
        for _ in range(4):gateway.tracking.status()
        self.assertEqual(gateway.tracking.counters['expired'],1)
