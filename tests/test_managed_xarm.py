"""Managed factory and VR admission regression with explicit ROS doubles.

No installed vendor services or physical motion are exercised here.
"""
from dataclasses import replace
import json
import io
import os
from pathlib import Path
import tempfile
import threading
from types import SimpleNamespace as NS
import unittest
from unittest.mock import patch

from tg3_pico_dualarm.bounded_xarm_transport import BoundedXArmTransport
from tg3_pico_dualarm.xarm_preparation import XArmPreparation
from tg3_pico_dualarm.motion_control import MotionAdmission
from tg3_pico_dualarm.factory_runtime import FactoryBridgeHandoff
from tg3_pico_dualarm.supervised_station import FactoryObserver,station_environment
import test_xarm_adapter as fixtures
from test_mapping import frame
from test_manufacturer import mode_message


class Future:
    def __init__(self,value):self.value=value;self.completed=True
    def done(self):return self.completed
    def result(self):return self.value
    def cancel(self):self.completed=False


class Request:
    def __init__(self):
        self.activate_controllers=[];self.deactivate_controllers=[];self.timeout=NS(sec=0,nanosec=0)


class ModeFixture:
    """Hardware state is shared with the adapter's separately driven double."""
    def __init__(self,fixture):
        self.f=fixture;self.mode=0;self.calls=[];self.messages=[];self.delay=False;self.mode_writes=[]
        types={key:NS(Request=Request) for key in ('mode','list','switch')}
        types.update(string=lambda:NS(data=None),joy=object)
        self.node=NS(create_publisher=lambda *a:NS(publish=lambda m:self.messages.append(m.data)),
            create_subscription=lambda *a:object(),destroy_subscription=lambda *a:None,destroy_publisher=lambda *a:None,
            destroy_client=lambda *a:None,create_client=self.client)
        self.prep=XArmPreparation(self.node,fixture.feedback,clock=fixture.clock,types=types)
        fixture.robot.preparation=self.prep;self.refresh('SWR')

    def refresh(self,body=None):
        previous=self.f.feedback.mode['current_state'] if self.f.feedback.mode else 'SWR'
        self.f.refresh();message=mode_message()
        for item in message.values:
            if item.key in ('current_state','child_state'):item.value=body or previous
        self.f.feedback.fsm(message)
        self.prep._joy(NS(axes=[0.]*8+[-1.]*4))

    def client(self,typ,name):
        def call(request):
            self.calls.append((name,request))
            if name.endswith('/list_controllers'):reply=NS(controller=list(self.f.transport.controllers.values()))
            elif name.endswith('/switch_controller'):
                for controller in request.deactivate_controllers:
                    self.f.transport.controllers[controller].state='inactive'
                    self.f.transport.controllers[controller].claimed_interfaces=[]
                reply=NS(ok=True)
            elif name.endswith('/set_arm_mode'):
                self.mode_writes.append((request.mode,self.f.feedback.mode['current_state']))
                self.mode=request.mode;reply=NS(result=True,info='ack')
            else:reply=NS(result=True,info='arm_mode 为 '+str(self.mode))
            future=Future(reply);future.completed=not self.delay;return future
        return NS(service_is_ready=lambda:True,call_async=call)

    def step(self,body=None,dt=.02):
        self.f.now[0]+=dt;self.refresh(body);return self.prep.step()

    def ready(self):
        self.prep.begin()
        for _ in range(45):
            if self.step('HBWALK' if self.prep.phase=='body' else None):return
        raise AssertionError(self.prep.status())


class PreparationTests(unittest.TestCase):
    def setUp(self):
        self.f=fixtures.XArmAdapterTests();self.f.setUp();self.w=ModeFixture(self.f)

    def test_inactive_confirmation_precedes_mode_mutation_and_HBWALK_settles(self):
        self.w.prep.begin();self.assertEqual(len(self.w.calls),1)
        self.assertFalse(self.w.messages)
        for _ in range(10):
            if self.w.prep.phase=='body':break
            self.w.step()
        names=[name.rsplit('/',1)[1] for name,_ in self.w.calls]
        self.assertEqual(names[:3],['list_controllers','switch_controller','list_controllers'])
        self.assertNotIn('set_arm_mode',names)
        self.assertFalse(self.w.step('HBWALK'))
        self.assertFalse(self.w.step('HBWALK',dt=.1))
        self.assertFalse(self.w.step('HBWALK',dt=.21))
        self.assertEqual(self.w.prep.phase,'post_body_mode');self.assertFalse(self.w.mode_writes)
        for _ in range(4):
            if self.w.step('HBWALK'):break
        self.assertEqual(self.w.prep.phase,'ready')
        self.assertEqual(self.w.mode_writes,[(3,'HBWALK')])
        self.assertEqual(self.w.mode,3);self.assertEqual(self.f.transport.published,[])

    def test_body_reset_is_observed_before_position_mode_is_reestablished(self):
        self.w.mode=3;self.w.prep.begin()
        for _ in range(45):
            body='HBWALK' if self.w.prep.phase=='body' else None
            if body:self.w.mode=0 # Explicit fixture: body transition reset arm mode.
            if self.w.step(body):break
        self.assertEqual(self.w.prep.phase,'ready')
        self.assertEqual(self.w.prep.original_arm,3)
        self.assertEqual(self.w.prep.status()['arm_mode_after_body'],0)
        self.assertIsNotNone(self.w.prep.status()['position_mode_confirmed_at'])
        self.assertEqual(self.w.mode_writes,[(3,'HBWALK')]);self.assertEqual(self.w.mode,3)

    def test_losing_body_hold_before_mode_confirmation_prevents_arm_mode_write(self):
        self.w.prep.begin()
        for _ in range(40):
            if self.w.prep.phase=='post_body_mode':break
            self.w.step('HBWALK' if self.w.prep.phase=='body' else None)
        self.assertEqual(self.w.prep.phase,'post_body_mode')
        with self.assertRaisesRegex(RuntimeError,'HBWALK_lost'):
            self.w.step('SWR')
        self.assertFalse(self.w.mode_writes);self.assertFalse(self.f.transport.published)

    def test_output_fault_rejects_readmission_before_body_or_controller_mutations(self):
        self.f.transport.output_fault='real_output_feedback_lead';self.f.robot.stop_confirmed=True
        gate=MotionAdmission(self.f.robot,tools_calibrated=True,clock=self.f.clock)
        value=replace(frame(1,0.),source_time_s=self.f.clock(),received_at=self.f.clock(),scene_visible=True)
        result=gate.request('enable',value,expected_session=value.session)
        self.assertFalse(result['accepted']);self.assertIn('fault_latched',result['reason'])
        self.assertEqual(self.w.calls,[]);self.assertEqual(self.w.messages,[])
        self.assertTrue(self.w.prep.restored);self.assertTrue(self.f.robot.stop_confirmed)
        self.assertEqual(self.f.robot.state,'disabled')

    def test_restore_repeated_cancel_preserves_pending_service_and_can_rearm(self):
        self.w.ready();self.w.prep.cancel()
        for _ in range(15):
            self.f.now[0]+=.02;self.w.refresh('SWR' if self.w.messages and self.w.messages[-1]=='gotoSTART' else None)
            self.w.prep.cancel()
            if self.w.prep.restore_step(safe_to_restore=True):break
        self.assertTrue(self.w.prep.restored,self.w.prep.status());self.assertEqual(self.w.mode,0)
        self.w.ready();self.assertEqual(self.w.prep.phase,'ready')

    def test_physical_STOP_is_never_replaced_by_resume_request(self):
        self.w.ready();self.w.prep.cancel();self.w.refresh('STOP')
        for _ in range(5):self.w.prep.restore_step(safe_to_restore=True)
        self.assertNotIn('gotoSTART',self.w.messages)
        self.assertTrue(self.w.prep.restored)

    def test_stale_radio_nonfinite_axes_and_late_service_prevent_preparation(self):
        for bad in ('stale','nan','pressed'):
            self.setUp()
            if bad=='stale':self.f.now[0]+=.16
            else:self.w.prep._joy(NS(axes=([float('nan')] if bad=='nan' else [.5])+[0.]*7+[-1.]*4))
            with self.assertRaisesRegex(RuntimeError,'physical_remote'):self.w.prep.begin()
            self.assertEqual(self.w.calls,[])
        self.setUp();self.w.delay=True;self.w.prep.begin();self.f.now[0]+=.151;self.w.refresh()
        with self.assertRaisesRegex(RuntimeError,'service_timeout'):self.w.prep.step()
        self.assertFalse(any(n.endswith('/set_arm_mode') for n,_ in self.w.calls))

    def test_controller_with_body_resource_is_not_deactivated(self):
        self.f.transport.controllers['wholebody_controller']=fixtures.controller('wholebody_controller','active',
            ('waist_pitch_joint/position',))
        self.w.prep.begin()
        with self.assertRaisesRegex(RuntimeError,'nonarm_or_unknown'):self.w.step()
        self.assertFalse(any(n.endswith('/switch_controller') for n,_ in self.w.calls))

    def test_write_ahead_journal_retains_original_modes_before_mutation(self):
        with tempfile.TemporaryDirectory() as folder,patch.dict(os.environ,{'TG3_SUPERVISED_SESSION':folder}):
            self.w.ready();value=json.loads((Path(folder)/'preparation.json').read_text())
            self.assertEqual(value['original_arm_mode'],0);self.assertEqual(value['original_body'],'SWR')
            self.assertTrue(value['mode_change_requested']);self.assertFalse(value['restored'])

    def test_managed_admission_waits_for_unchanged_measured_hold_before_following(self):
        gate=MotionAdmission(self.f.robot,tools_calibrated=True,clock=self.f.clock)
        def input():return replace(frame(1,0.),source_time_s=self.f.clock(),received_at=self.f.clock(),scene_visible=True)
        value=input();self.assertTrue(gate.request('enable',value,expected_session=value.session)['pending'])
        for _ in range(100):
            self.f.now[0]+=.005
            self.w.refresh('HBWALK' if self.w.prep.phase in ('body','ready') else None)
            self.f.transport.complete();gate.tick(input())
            if gate.initial_hold_pending:break
        self.assertTrue(gate.initial_hold_pending,gate.status());self.assertFalse(gate.consume_enable_ack())
        self.assertEqual(self.f.transport.published,[])
        for _ in range(20):
            self.f.now[0]+=.02;self.w.refresh();self.f.transport.complete();gate.tick(input())
            if gate.just_enabled:break
        self.assertTrue(gate.consume_enable_ack(),gate.status());self.assertFalse(gate.consume_enable_ack())
        self.assertTrue(self.f.transport.published)
        self.assertTrue(all(q==self.f.robot.commanded_joint_positions for q in self.f.transport.published))


    def test_preparation_failure_survives_mode_restore_and_disabled_ticks(self):
        gate=MotionAdmission(self.f.robot,tools_calibrated=True,clock=self.f.clock)
        def input(focused=True):
            return replace(frame(1,0.),source_time_s=self.f.clock(),received_at=self.f.clock(),
                scene_visible=True,focused=focused)
        value=input();self.assertTrue(gate.request('enable',value,expected_session=value.session)['pending'])
        gate.tick(input(False))
        failure=gate.status()['last_failure']
        self.assertEqual(failure['reason'],'fresh_focused_head_tracking_required')
        self.assertEqual(failure['preparation']['phase'],'inactive')
        for _ in range(10):
            self.f.now[0]+=.02;self.w.refresh();gate.tick(input())
        self.assertEqual(gate.status()['last_failure'],failure)
        self.assertFalse(gate.status()['hardware_motion_enabled'])
        self.assertFalse(self.f.transport.published)

    def test_preparation_reads_current_provider_and_still_cancels_on_stale_or_changed_session(self):
        gate=MotionAdmission(self.f.robot,tools_calibrated=True,clock=self.f.clock)
        def fresh():return replace(frame(1,0.),source_time_s=self.f.clock(),received_at=self.f.clock(),scene_visible=True)
        latest=[fresh()];gate.frame_provider=lambda:latest[0]
        first=latest[0];self.assertTrue(gate.request('enable',first,expected_session=first.session)['pending'])
        self.f.now[0]+=.02;self.w.refresh();latest[0]=fresh();gate.tick(first)
        self.assertIs(gate.validated_frame,latest[0]);self.assertIsNone(gate.last_failure)
        latest[0]=replace(fresh(),received_at=self.f.clock()-.084768778,source_age_s=.075289274)
        gate.tick(fresh());self.assertEqual(gate.last_failure['stage'],'preparation_input')
        self.assertIn('stale',gate.last_failure['reason']);self.assertFalse(self.f.transport.published)


class BoundedRawTests(unittest.TestCase):
    def fixture(self):
        relay=object.__new__(BoundedXArmTransport);relay.output_mutex=threading.RLock();relay.output_fault=None
        relay.actual_count=0;relay.maximum_step=0.;relay.last_actual=(0.,)*14;relay.last_actual_time=9.99
        messages=[];relay.actual_publisher=NS(publish=messages.append)
        relay.adapter=NS(state='armed',_fresh_status=lambda:None,_feedback_ok=lambda:NS(positions=(0.,)*14))
        relay.settings=NS(lower=(-1.,)*14,upper=(1.,)*14,velocity_cap=(.02,)*14,feedback_lead=(.03,)*14)
        relay.clock=lambda:10.;relay.node=NS(destroy_publisher=lambda pub:None)
        message=NS(mode=0,label=108,reserved=0,ctrl=[NS(name=i,pos=.5,spd=3.14,kp=0.,kd=0.) for i in tuple(range(11,18))+tuple(range(21,28))])
        return relay,messages,message

    def test_initial_output_is_seeded_and_speed_bounded_without_changing_input_or_protocol(self):
        relay,messages,message=self.fixture();relay.raw(message)
        self.assertEqual(len(messages),1);self.assertLessEqual(max(m.pos for m in messages[0].ctrl),.0002+1e-12)
        self.assertTrue(all(m.spd==.02 for m in messages[0].ctrl))
        self.assertEqual(messages[0].label,108);self.assertEqual(messages[0].reserved,0)
        self.assertTrue(all(m.pos==.5 and m.spd==3.14 for m in message.ctrl))
        relay,messages,message=self.fixture();relay.settings.velocity_cap=(.005,)*14;relay.raw(message)
        self.assertLessEqual(messages[0].ctrl[0].pos,.00005+1e-12);self.assertEqual(messages[0].ctrl[0].spd,.005)

    def test_feedback_fault_retains_one_joint_snapshot_without_publishing_or_chasing_measurement(self):
        relay,messages,message=self.fixture();relay.last_actual=(.04,)*14
        original=relay.last_actual
        relay.raw(message)
        self.assertEqual(relay.output_fault,'real_output_feedback_lead')
        self.assertFalse(messages);self.assertEqual(relay.last_actual,original)
        self.assertEqual(relay.feedback_fault_details['motor_id'],11)
        self.assertEqual(relay.feedback_fault_details['error_rad'],.04)
        self.assertEqual(relay.feedback_fault_details['limit_rad'],.03)
        self.assertEqual(relay.feedback_fault_details['actual_frames_before_fault'],0)

    def test_stopped_duplicate_timing_mode_lead_and_nonfinite_data_cannot_reach_actuators(self):
        for fail in ('closed','disabled','duplicate','timing','mode','lead','nan'):
            relay,messages,message=self.fixture()
            if fail=='closed':relay.actual_publisher=None
            elif fail=='disabled':relay.adapter.state='disabled'
            elif fail=='duplicate':message.ctrl[-1].name=message.ctrl[0].name
            elif fail=='timing':relay.last_actual_time=9.
            elif fail=='mode':message.mode=1
            elif fail=='lead':relay.last_actual=(.04,)*14
            else:message.ctrl[0].pos=float('nan')
            relay.raw(message);self.assertFalse(messages,fail)
            if fail not in ('closed','disabled'):self.assertIsNone(relay.actual_publisher);self.assertTrue(relay.output_fault)


class SupervisorBoundaryTests(unittest.TestCase):
    def test_usb_precheck_failure_precedes_any_factory_or_bridge_access(self):
        import tg3_pico_dualarm.supervised_station as supervisor
        with (patch.object(supervisor.sys,'platform','linux'),patch.object(supervisor.os,'geteuid',return_value=0,create=True),
            patch('tg3_pico_dualarm.robot_usb.RobotHeadsetUSB') as usb,
            patch.object(supervisor,'FactoryObserver') as observer,patch('tg3_pico_dualarm.factory_runtime.FactoryBridgeHandoff') as bridge):
            usb.return_value.check_available.side_effect=RuntimeError('USB unauthorized')
            with self.assertRaisesRegex(RuntimeError,'unauthorized'):supervisor.run(['--hardware','--xarm','--headset-usb-local'])
            observer.assert_not_called();bridge.assert_not_called()

    def lifecycle(self,*,fail_factory=False):
        import tg3_pico_dualarm.supervised_station as supervisor
        trace=[];release=threading.Event()
        stdin=NS(readline=lambda:'k'*40+'\n',read=lambda:release.wait(2.))
        child=NS(stdin=io.BytesIO(),poll=lambda:0,returncode=0,wait=lambda **kw:trace.append('station_wait'))
        factory=NS(process=NS(poll=lambda:None),model_sha='fixture',start=lambda:trace.append('factory_start'),close=lambda:trace.append('factory_close'))
        bridge=NS(restored=True,start=lambda:trace.append('bridge_start'),close=lambda:trace.append('bridge_restore'))
        def ready(*args):
            trace.append('factory_ready')
            if fail_factory:raise RuntimeError('fixture startup failure')
        observer=NS(observe=lambda **kw:None,wait_ready=lambda:None,no_existing_factory=lambda:None,wait_disabled=ready,
            spin=lambda *a:None,fallback_release=lambda *a:(trace.append('release_check') or {}),close=lambda:trace.append('observer_close'))
        def spawn(command,**kwargs):
            trace.append('station_start');self.assertNotIn('k'*40,' '.join(command));return child
        with tempfile.TemporaryDirectory() as folder:
            app=Path(folder)
            with patch.object(supervisor,'RECOVERY_ROOT',app/'recovery'),patch.object(supervisor,'__file__',str(app/'src/tg3_pico_dualarm/supervised_station.py')),patch.object(supervisor.sys,'stdin',stdin),\
                 patch.object(supervisor.sys,'platform','linux'),patch.object(supervisor.os,'geteuid',return_value=0,create=True),\
                 patch.object(supervisor.signal,'signal'),patch.object(supervisor.signal,'SIGHUP',1,create=True),patch.object(supervisor,'FactoryObserver',return_value=observer),\
                 patch('tg3_pico_dualarm.factory_runtime.FactoryBridgeHandoff',return_value=bridge),\
                 patch('tg3_pico_dualarm.factory_runtime.DisabledFactoryProcess',return_value=factory),\
                 patch.object(supervisor.subprocess,'Popen',side_effect=spawn),patch.object(supervisor,'event'),\
                 patch('tg3_pico_dualarm.holding_profile.load_holding_profile',return_value=NS(status=lambda:{})):
                try:
                    if fail_factory:
                        with self.assertRaisesRegex(RuntimeError,'fixture startup failure'):supervisor.run(['--hardware','--xarm'])
                    else:self.assertEqual(supervisor.run(['--hardware','--xarm']),0)
                finally:release.set()
        return trace

    def test_station_exit_cleans_owned_factory_then_restores_original_bridge(self):
        trace=self.lifecycle()
        self.assertEqual(trace,['bridge_start','factory_start','factory_ready','station_start','station_wait','release_check','factory_close','bridge_restore','observer_close'])

    def test_failed_factory_startup_does_not_start_station_but_still_restores_bridge(self):
        trace=self.lifecycle(fail_factory=True)
        self.assertNotIn('station_start',trace)
        self.assertLess(trace.index('factory_close'),trace.index('bridge_restore'))

    def test_private_placo_environment_does_not_modify_global_factory_library_path(self):
        with patch.dict(os.environ,{'LD_LIBRARY_PATH':'factory-pin-3.9','PYTHONPATH':'/opt/ros/jazzy/site-packages'}):
            value=station_environment(Path('/app'),Path('/session'))
            self.assertTrue(value['LD_LIBRARY_PATH'].startswith(str(Path('/app')/'.venv')))
            self.assertEqual(os.environ['LD_LIBRARY_PATH'],'factory-pin-3.9')
            self.assertTrue(value['PYTHONPATH'].endswith(':/opt/ros/jazzy/site-packages'))
            self.assertEqual(value['TG3_SUPERVISED_SESSION'],str(Path('/session')))

    def test_factory_restore_requires_original_argument_identity_and_verified_backup(self):
        handoff=FactoryBridgeHandoff(Path('.'),observe=lambda **kw:None)
        handoff.original=dict(argv=['/usr/bin/python3','/factory','--params-file','/tmp/original'],parameter_backups=[])
        self.assertTrue(handoff._original_identity(handoff.original['argv']))
        self.assertFalse(handoff._original_identity(['/usr/bin/python3','/factory','--params-file','/tmp/foreign']))
        self.assertFalse(handoff._original_identity(handoff.original['argv']+['-r','/arm/cmd:=/other']))

    def test_supervisor_never_sends_cleanup_services_without_a_recorded_admission(self):
        observer=object.__new__(FactoryObserver);observer.call=lambda *a:(_ for _ in ()).throw(AssertionError('unexpected mutation'))
        with tempfile.TemporaryDirectory() as folder:
            self.assertFalse(observer.fallback_release(folder)['required'])
            (Path(folder)/'preparation.json').write_text(json.dumps(dict(restored=True)))
            self.assertFalse(observer.fallback_release(folder)['required'])


if __name__=='__main__':unittest.main()
