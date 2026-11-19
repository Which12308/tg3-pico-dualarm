"""Real loopback control RPC and hidden UI; no robot connection or motion."""
import json
import os
from pathlib import Path
import sys
import tempfile
import threading
import time
from types import SimpleNamespace as NS
import unittest
from unittest.mock import patch

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'desktop'))
try:
    from start import Desk
    from preview import checked_chains,project
except ImportError:
    Desk=None
from tg3_pico_dualarm.model import DualArmModel,default_q
from tg3_pico_dualarm.wireless import InputGateway,WirelessServer


@unittest.skipUnless(Desk is not None and (os.name=='nt' or os.environ.get('DISPLAY')),'Hidden Tk checks require an available display/Tk runtime')
class DesktopControlsTests(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory()
        self.desk=Desk(visible=False,port=0,record_root=self.temp.name)
        self.servers=[]

    def tearDown(self):
        self.desk.close()
        for server in self.servers:server.close()
        self.temp.cleanup()

    def peer(self,callback):
        gateway=InputGateway(role='relay',key='x'*40);gateway.control_callback=callback
        server=WirelessServer(gateway,'127.0.0.1',0).start();self.servers.append(server)
        self.desk.remote_endpoint=('127.0.0.1',server.address[1],'x'*40)
        self.desk._runtime_mode_running='hardware'
        return gateway

    def spin_until(self,condition):
        deadline=time.monotonic()+2.
        while not condition() and time.monotonic()<deadline:
            self.desk.window.update();time.sleep(.01)
        self.desk.poll();self.assertTrue(condition())

    def test_enable_sends_remote_session_on_independent_connection_and_queue_is_not_enabled(self):
        received=[]
        self.peer(lambda value:(received.append(value) or dict(accepted=True,queued=True,pending=True)))
        self.desk.remote_tracking={'session':'remote-current-session','input_profile':'normal'}
        self.desk.remote_client=NS(close=lambda:None,request=lambda *args:self.fail('poll connection must not carry control'))
        self.desk.gateway.runtime=dict(readiness=dict(ready_to_request_motion=True,hardware_motion_enabled=False,
            hardware_ready=False,blockers=[]),motion=dict(state='disabled',reason='fixture'))
        self.desk.enable();self.spin_until(lambda:bool(received) and not self.desk._control_pending)
        self.assertEqual(received[0]['operation'],'enable');self.assertEqual(received[0]['session'],'remote-current-session')
        self.assertNotIn('输出已启用',self.desk.banner.get())
        self.desk.gateway.runtime['motion']['state']='armed'
        self.desk.gateway.runtime['readiness']['hardware_motion_enabled']=True
        self.desk.poll();self.assertIn('输出已启用',self.desk.banner.get())
        self.assertIn('尚未完成',self.desk.issues.get())

    def test_model_mode_enable_is_explained_without_any_rpc(self):
        with patch('start.messagebox.showinfo') as info,patch('start.GatewayClient') as client:
            self.desk.enable()
        info.assert_called_once();client.assert_not_called()
        self.assertEqual(self.desk.runtime_mode.get(),'dry_run')

    def test_wired_default_keeps_proxy_and_expired_queue_allows_retry(self):
        self.assertIs(self.desk.gateway.clock,time.perf_counter)
        self.assertEqual(self.desk._network_mode(),'wired')
        self.assertEqual(self.desk.connection_mode.get(),'USB')
        self.assertEqual(self.desk.input_profile.get(),'normal')
        self.desk._runtime_mode_running='hardware'
        self.desk._enable_queued=True;self.desk._enable_request_number=7
        self.desk.gateway.runtime=dict(motion=dict(state='disabled',attempt_id=0),
            readiness=dict(ready_to_request_motion=True),
            motion_request_result=dict(request_number=7,pending=False,accepted=False,reason='motion_request_expired'))
        self.desk.poll()
        self.assertFalse(self.desk._enable_queued)
        self.assertIn('队列中过期',self.desk.banner.get())

    def test_enable_checks_bound_route_before_sending_control_rpc(self):
        received=[]
        self.peer(lambda value:(received.append(value) or dict(accepted=True)))
        self.desk.deployment=NS(network_mode='wired',verify_network=lambda:(_ for _ in ()).throw(RuntimeError('wired interface changed')))
        with self.assertRaisesRegex(RuntimeError,'wired interface changed'):
            self.desk._rpc_control('enable','test-session',self.desk.remote_endpoint,self.desk._generation)
        self.assertEqual(received,[])
        self.desk.deployment=None

    def test_trial_profile_is_explicit_matches_deploy_and_resets_on_stop(self):
        self.assertEqual(self.desk.input_profile.get(),'normal')
        self.desk.connection_mode.set('USB')
        self.desk.input_profile.set('wifi_trial_200ms');self.desk._profile_changed()
        config=json.loads(self.desk._startup_overrides('dry_run')['config/vr.yaml'])
        self.assertEqual(config['tracking_timeout_s'],.2)
        self.assertEqual(config['source_age_limit_s'],.2)
        self.assertEqual(self.desk.gateway.tracking.policy.name,'wifi_trial_200ms')
        self.desk._runtime_mode_running='hardware'
        self.desk.remote_tracking={'session':'test','input_profile':'normal'}
        with patch('start.messagebox.showinfo') as info,patch.object(self.desk,'_control_worker') as control:
            self.desk.enable()
        info.assert_called_once();control.assert_not_called()
        self.desk.stop();self.desk.poll()
        self.assertEqual(self.desk.input_profile.get(),'normal')
        self.assertEqual(self.desk.gateway.tracking.policy.total_age_s,.15)
        self.assertFalse(self.desk.manual_recording.status['state']=='recording')

    def test_incident_button_only_reads_existing_logs_without_deployment(self):
        reader=NS(connect=lambda *args:None,close=lambda:None)
        calls=[]
        def command(*args,**kwargs):
            calls.append(args)
            if len(calls)==2:return json.dumps(dict(passive_only=True,robot_commands_sent=False,files=[]))
            return json.dumps(dict(scope='read_existing_project_fault_logs_only',control_services_called=False,files={}))
        reader.command=command
        self.desk.password.set('test-credential')
        with tempfile.TemporaryDirectory() as folder,patch('start.ROOT',Path(folder)),patch('start.RobotDeployment',return_value=reader):
            self.desk.read_incident_logs()
            self.spin_until(lambda:not self.desk.busy)
            reports=list((Path(folder)/'.desktop/robot_reports').glob('factory_incident_*.json'))
            self.assertEqual(len(reports),1)
        self.assertFalse(self.desk.password.get())
        self.assertIsNone(self.desk.deployment)
        self.assertIsNone(self.desk.relay)

    def test_fault_visible_with_live_diagnostics_never_shows_ready(self):
        self.desk.gateway.runtime=dict(station_fault=dict(reason='native_ik_worker_fault'),
            control=dict(state='fault'),readiness=dict(hardware_motion_enabled=False,
                ready_to_request_motion=False,blockers=['station_fault_restart_required']))
        self.desk.poll()
        self.assertIn('控制已停止',self.desk.banner.get())
        self.assertIn('native_ik_worker_fault',self.desk.banner.get())
        self.assertNotIn('输出已启用',self.desk.banner.get())

    def test_confirmed_station_exit_stops_polling_and_relay_and_preserves_cause(self):
        exit_status=dict(exit_code=2,reason='watchdog_expired_restart_required')
        client=NS(close=lambda:None,request=lambda *a:self.fail('must not poll an exited station'))
        self.desk.remote_client=client
        self.desk.deployment=NS(station_exit=exit_status,close=lambda:None)
        self.desk.relay=NS(stop=threading.Event(),close=lambda:None,status={'state':'forwarding'})
        self.desk._runtime_mode_running='dry_run'
        self.desk.remote_status();self.desk.poll()
        self.assertEqual(self.desk.gateway.runtime['station_exit'],exit_status)
        self.assertTrue(self.desk.relay.stop.is_set())
        self.assertIn('watchdog_expired_restart_required',self.desk.banner.get())
        self.assertIn('硬件输出关闭',self.desk.banner.get())
        self.assertNotIn('已尝试停用',self.desk.banner.get())

    def test_first_install_resolves_adb_off_the_ui_thread(self):
        entered=threading.Event();release=threading.Event();finished=threading.Event();threads=[]
        def ensure(root,notify):
            threads.append(threading.current_thread());entered.set();release.wait(2);return 'reviewed-adb'
        def installed(root,adb):
            self.assertEqual(adb,'reviewed-adb');finished.set();return 'fixture installation complete'
        with patch('start.artifact_info',return_value=(None,{})),patch.dict(sys.modules,{'adb_tools':NS(ensure_adb=ensure)}),patch('start.install',side_effect=installed):
            self.desk.install_headset()
            try:
                self.assertTrue(entered.wait(2));self.assertFalse(finished.is_set())
                self.assertIsNot(threads[0],threading.current_thread())
            finally:release.set()
            self.assertTrue(finished.wait(2))

    def test_usb_default_and_reverse_prepare_off_ui_thread_without_robot_control(self):
        entered=threading.Event();release=threading.Event();threads=[]
        def ensure(root,notify):
            threads.append(threading.current_thread());entered.set();release.wait(2);return 'private-adb'
        self.assertEqual(self.desk.connection_mode.get(),'USB')
        self.desk.connection_mode.set('USB');self.desk.pair()
        self.assertIn('127.0.0.1',self.desk.pairing.get())
        with patch.dict(sys.modules,{'adb_tools':NS(ensure_adb=ensure)}),patch('start.prepare_usb',return_value='fixture USB prepared') as prepare,patch('start.GatewayClient') as robot:
            self.desk.prepare_usb()
            try:
                self.assertTrue(entered.wait(2));self.assertIsNot(threads[0],threading.current_thread())
                self.assertFalse(prepare.called)
            finally:release.set()
            self.spin_until(lambda:not self.desk._usb_busy)
            prepare.assert_called_once_with('private-adb');robot.assert_not_called()
            self.assertIn('通道已设置',self.desk.usb_status.get())

    def test_queued_status_from_previous_connection_cannot_reenable_display_after_stop(self):
        previous=NS(close=lambda:None);self.desk.remote_client=previous
        generation=self.desk._generation
        self.desk.stop()
        self.desk.runtime_mode.set('hardware')
        stale=dict(tracking=dict(session='old'),runtime=dict(motion=dict(state='armed'),
            readiness=dict(hardware_motion_enabled=True,blockers=[])))
        self.desk.events.put(('remote',(previous,generation,stale)))
        self.desk.poll()
        self.assertEqual(self.desk.gateway.runtime,{})
        self.assertNotIn('输出已启用',self.desk.banner.get())

    def test_stop_requests_disable_before_closing_own_relay_and_ssh_and_generation_cancels_old_enable(self):
        trace=[]
        self.peer(lambda value:(trace.append(value['operation']) or dict(accepted=True,queued=True,pending=True)))
        endpoint=self.desk.remote_endpoint;old=self.desk._generation
        self.desk._generation+=1
        with self.assertRaisesRegex(RuntimeError,'cancelled'):
            self.desk._rpc_control('enable','old-session',endpoint,old)
        self.assertEqual(trace,[])
        self.desk.relay=NS(close=lambda:trace.append('relay.close'))
        self.desk.remote_client=NS(close=lambda:trace.append('poll.close'))
        self.desk.deployment=NS(close=lambda:trace.append('ssh.close'))
        self.desk.stop()
        self.assertEqual(trace[0],'disable')
        self.assertGreater(trace.index('ssh.close'),trace.index('disable'))
        self.assertIsNone(self.desk.remote_endpoint)

    def test_preview_draws_actual_supplied_urdf_link_coordinates_and_clears_missing_state(self):
        model=DualArmModel();q=default_q()
        links={side:{name:value.tolist() for name,value in model.arms[side].links(q[i*7:(i+1)*7]).items()}
            for i,side in enumerate(('left','right'))}
        control=dict(state='dry_run',model_links=links,hands=dict(closure=dict(left=.4,right=.7)))
        self.desk.preview.update_control(control);self.desk.window.update_idletasks()
        self.assertEqual(set(self.desk.preview.chains),{'left','right'})
        self.assertAlmostEqual(self.desk.preview.hand_values['right'].get(),70.)
        lines=[item for item in self.desk.preview.canvas.find_all() if self.desk.preview.canvas.type(item)=='line']
        self.assertGreaterEqual(len(lines),5)
        control['state']='hardware';self.desk.preview.update_control(control)
        self.assertIn('控制目标',self.desk.preview.source.get())
        self.desk.preview.update_control(dict(state='disarmed'))
        self.assertEqual(self.desk.preview.chains,{})

    def test_hardware_startup_configuration_is_disarmed_and_missing_provider_is_concrete(self):
        overrides=self.desk._startup_overrides('dry_run')
        robot=json.loads(overrides['config/robot.yaml'])
        self.assertEqual(robot['mode'],'dry_run');self.assertIs(robot['hardware_motion_enabled'],False)
        self.assertEqual(self.desk.hardware_backend.get(),'xarm')
        overrides=self.desk._startup_overrides('hardware')
        self.assertFalse(json.loads(overrides['config/robot.yaml'])['hardware_motion_enabled'])
        self.assertNotIn('config/factory_driver.py',overrides)
        self.desk.hardware_backend.set('manufacturer')
        with self.assertRaisesRegex(RuntimeError,'缺少实际厂家驱动'):
            self.desk._startup_overrides('hardware')

    def test_failed_readonly_capture_still_starts_dry_run_and_only_status_gets_long_timeout(self):
        trace=[]
        self.desk.connection_mode.set('USB');self.desk.pair()
        deployment=NS(known_hosts=Path(self.temp.name)/'known_hosts',remote='/tmp/fixture-release',
            closing=threading.Event(),connect=lambda *a:trace.append('connect'),
            deploy=lambda:trace.append('deploy'),capture_interfaces=lambda:(_ for _ in ()).throw(RuntimeError('fixture capture failure')),
            start=lambda key,**kwargs:(trace.append('start') or 12345),close=lambda:trace.append('close'))
        client=NS(close=lambda:None,control=lambda *a:dict(accepted=True,queued=True))
        relay=NS(close=lambda:None,status={'state':'fixture'})
        self.desk.remote_status=lambda:None
        with patch('start.RobotDeployment',return_value=deployment),patch('start.GatewayClient',return_value=client) as gateway,patch('start.LatestRelay',return_value=NS(start=lambda:relay)):
            self.desk.start();self.spin_until(lambda:self.desk.deployment is not None and not self.desk.busy)
            self.assertEqual(trace,['connect','deploy','start'])
            self.assertEqual(gateway.call_args_list[0].kwargs,{'timeout':1.0})
            self.assertEqual(gateway.call_args_list[1].kwargs,{'cache_challenge':False})
            self.desk.stop()

    def test_pairing_input_and_reconnection_never_create_trajectory_files_or_worker(self):
        from test_wireless import native
        self.desk.connection_mode.set('USB');self.desk.pair()
        token=self.desk.gateway.pair(self.desk._pair_pin,'loopback no recorder')
        for seq in range(3):
            nonce=self.desk.gateway.challenge(token)['nonce']
            self.desk.gateway.accept(token,nonce,native(seq))
        self.desk.poll();self.desk.pair();self.desk.poll()
        self.assertIsNone(self.desk.gateway.on_frame)
        self.assertIsNone(self.desk.manual_recording.process)
        self.assertEqual(list(Path(self.temp.name).iterdir()),[])
        self.assertEqual(str(self.desk.record_start['state']),'disabled')

    def test_recording_error_does_not_disable_motion_or_invalidate_tracking(self):
        from test_wireless import native
        self.desk.connection_mode.set('USB');self.desk.pair()
        token=self.desk.gateway.pair(self.desk._pair_pin,'loopback record failure')
        nonce=self.desk.gateway.challenge(token)['nonce'];self.desk.gateway.accept(token,nonce,native())
        before=self.desk.gateway.tracking.sample()
        self.desk.gateway.runtime=dict(motion=dict(state='armed'),readiness=dict(hardware_motion_enabled=True))
        self.desk.manual_recording.status=dict(state='recording')
        with (patch.object(self.desk.manual_recording,'poll',return_value=dict(state='error',incomplete=True,reason='USB removed')),
             patch.object(self.desk,'_control_worker') as control):
            self.desk.poll();control.assert_not_called()
        self.assertEqual(self.desk.gateway.tracking.sample(),before)
        self.assertEqual(self.desk.gateway.runtime['motion']['state'],'armed')
        self.assertIn('不完整',self.desk.record_state.get())

    def test_repair_cancels_pending_record_start_before_process_or_files_exist(self):
        began=threading.Event();release=threading.Event()
        def access(_):began.set();release.wait(2);return {'token':'fixture-readonly'}
        self.desk._session_credentials={};self.desk.record_destination=self.temp.name;self.desk.record_topics={}
        self.desk._session_credentials={'remote':'/tmp/fixture'};self.desk.remote_endpoint=('127.0.0.1',1,'fixture')
        with (patch('start.GatewayClient',return_value=NS(recording_access=access,close=lambda:None)),
              patch('start.messagebox.showerror')):
            self.desk.start_recording();self.assertTrue(began.wait(1))
            self.desk.pair();release.set();self.spin_until(lambda:not self.desk._record_pending)
        self.assertIsNone(self.desk.manual_recording.process)
        self.assertEqual(list(Path(self.temp.name).iterdir()),[])
        self.desk.remote_endpoint=None

    def test_direct_start_preflights_before_start_and_does_not_create_pc_relay(self):
        self.desk.connection_mode.set('USB（机器人直连）')
        trace=[]
        deployment=NS(known_hosts=Path(self.temp.name)/'known_hosts',remote='/tmp/direct-fixture',headset_transport='robot_usb',
            headset_pairing_code='12345678',closing=threading.Event(),connect=lambda *a:trace.append('connect'),
            deploy=lambda:trace.append('deploy'),prepare_robot_usb=lambda **kwargs:trace.append('usb'),
            capture_interfaces=lambda:dict(ok=True,path='fixture',summary={}),
            start=lambda *a,**kw:(trace.append('start') or 12345),close=lambda:trace.append('close'))
        client=NS(close=lambda:None,control=lambda *a:dict(accepted=True,queued=True))
        self.desk.remote_status=lambda:None
        with (patch('start.RobotDeployment',return_value=deployment),patch('start.GatewayClient',return_value=client),
              patch('start.LatestRelay') as relay):
            self.desk.start();self.spin_until(lambda:self.desk.deployment is not None and not self.desk.busy)
            self.assertEqual(trace,['connect','deploy','usb','start']);relay.assert_not_called()
            self.assertIn('12345678',self.desk.pairing.get());self.assertIsNone(self.desk.relay)
            self.desk.remote_tracking=dict(head_tracked=True,left_tracked=True,right_tracked=True,focused=True,sample_age_s=.01,counters={'accepted':12})
            self.desk.gateway.runtime=dict(usb={'reverse_ready':True},motion={'state':'disabled'},readiness={'hardware_motion_enabled':False})
            self.desk.poll();self.assertIn('数据新鲜：是',self.desk.usb_status.get())
            self.assertNotIn('输出已启用',self.desk.banner.get())
            self.desk.stop()

    def test_direct_usb_failure_does_not_start_factory_session(self):
        self.desk.connection_mode.set('USB（机器人直连）')
        trace=[]
        deployment=NS(closing=threading.Event(),connect=lambda *a:None,deploy=lambda:None,
            prepare_robot_usb=lambda **kwargs:(_ for _ in ()).throw(RuntimeError('USB unauthorized')),
            start=lambda *a,**kwargs:trace.append('UNSAFE_START'),close=lambda:trace.append('close'))
        with patch('start.RobotDeployment',return_value=deployment):
            self.desk.start();self.spin_until(lambda:not self.desk.busy)
        self.assertEqual(trace,['close']);self.assertIsNone(self.desk.remote_endpoint)


@unittest.skipUnless(Desk is not None,'Tk module is unavailable')
class PreviewMathTests(unittest.TestCase):
    def test_input_link_order_is_derived_from_actual_urdf_and_invalid_side_is_omitted(self):
        model=DualArmModel();q=default_q()
        links={side:{name:value.tolist() for name,value in reversed(list(model.arms[side].links(q[i*7:(i+1)*7]).items()))}
            for i,side in enumerate(('left','right'))}
        actual=checked_chains(model,links)
        self.assertEqual(len(actual['left']),9)
        self.assertEqual(actual['left'][-1],tuple(model.arms['left'].fk(q[:7])[:3,3]))
        links['right']['right_tcp_link'][0]=float('nan')
        self.assertEqual(set(checked_chains(model,links)),{'left'})
        self.assertEqual(project((.1,.2,.3),'正视 YZ'),(.2,.3))


if __name__=='__main__':unittest.main()

