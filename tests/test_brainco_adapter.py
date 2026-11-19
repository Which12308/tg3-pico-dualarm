"""Explicit ROS/XARM doubles; no hardware, hand calibration or ownership claims."""
from dataclasses import asdict
import json
from pathlib import Path
import tempfile
import threading
from types import SimpleNamespace as NS
import unittest

from tg3_pico_dualarm.brainco_adapter import BrainCoHandAdapter,hand_schema,encode_hand,validate_brainco_config
from tg3_pico_dualarm.hands import HandCalibration
from tg3_pico_dualarm.manufacturer_feedback import ManufacturerFeedback
import test_manufacturer as fixtures


class HandPublisher:
    def __init__(self,topic,gid):self.topic=topic;self.gid=gid;self.messages=[];self.closed=False
    def publish(self,message):
        if self.closed:raise RuntimeError('closed fixture publisher')
        self.messages.append(message)


class HandNode:
    def __init__(self):self.publishers=[];self.external={};self.next_gid=0;self.fail_create=None
    def get_name(self):return 'explicit_test_xarm'
    def get_namespace(self):return '/test'
    def get_publishers_info_by_topic(self,topic):
        return self.external.get(topic,[])+[NS(node_name=self.get_name(),node_namespace=self.get_namespace(),
            topic_type='brainco_hand_msgs/msg/SetMotorMulti',endpoint_gid=value.gid)
            for value in self.publishers if value.topic==topic and not value.closed]
    def create_publisher(self,cls,topic,qos):
        if topic==self.fail_create:raise RuntimeError('fixture publisher creation failed')
        self.next_gid+=1;publisher=HandPublisher(topic,bytes([self.next_gid])*24)
        self.publishers.append(publisher);return publisher
    def destroy_publisher(self,publisher):publisher.closed=True;return True


class XArmParent:
    """Test double for audited parent; does not establish real control."""
    def __init__(self,feedback):
        self.lock=threading.RLock();self.state='armed';self.hardware_writes_enabled=True
        self.audit_fresh=True;self.feedback=feedback
    def _fresh_status(self):
        if not self.audit_fresh:raise RuntimeError('fixture_controller_audit_stale')
    def _feedback_ok(self):
        value=self.feedback.read_state()
        if value.quality!='measured' or value.estop is not False:raise RuntimeError('fixture_parent_feedback_not_ready')
        return value


class BrainCoHandAdapterTests(unittest.TestCase):
    def setUp(self):
        self.now=[10.];self.clock=lambda:self.now[0]
        self.feedback=ManufacturerFeedback(clock=self.clock);self.refresh()
        self.node=HandNode();self.parent=XArmParent(self.feedback)
        self.profile=HandCalibration((10,)*6,(900,)*6,(1,)*6,(1000,)*6,(20,)*6,(30,)*6,'TEST ONLY measured fixture')
        self.adapter=self.make()

    def refresh(self,position=100):
        self.feedback.arm(fixtures.arm_message());self.feedback.fsm(fixtures.mode_message())
        self.feedback.key_status(fixtures.power_key_message())
        for side in ('left','right'):self.feedback.hand(side,fixtures.hand_message(position))

    def make(self,**options):
        defaults=dict(parent=self.parent,profiles={'right':self.profile},hand_raw_rate={'right':(100,)*6},
            hand_raw_lead={'right':(20,)*6},message_type=fixtures.HandMessage,clock=self.clock)
        defaults.update(options)
        return BrainCoHandAdapter(self.node,self.feedback,**defaults)

    def submit(self,closure=1.,operation='close',follow=True):
        self.now[0]+=.02;self.refresh(self.adapter.hand_positions.get('right',(100,)*6)[0] if follow else 100)
        return self.adapter.submit_hand('right',closure,intent_stamp=self.clock(),operation=operation)

    def test_missing_calibration_and_independent_limits_are_disabled_and_specific(self):
        for options,reason in ((dict(profiles=None),'calibration_missing'),
                               (dict(hand_raw_rate=None),'hand_raw_rate'),(dict(hand_raw_lead=None),'hand_raw_lead')):
            with self.subTest(options=options):
                adapter=self.make(**options)
                self.assertEqual(adapter.profiles,{})
                self.assertIn(reason,adapter.status()['disabled_sides']['right'])
                self.assertEqual(adapter.measured_hand_initialization(),{})
                self.assertFalse(adapter.submit_hand('right',1.,intent_stamp=self.clock()).accepted)
        self.assertEqual(self.node.publishers,[])

    def test_json_load_requires_all_measured_fields_and_does_not_invent_limits(self):
        with tempfile.TemporaryDirectory() as folder:
            path=Path(folder)/'hand.json'
            path.write_text(json.dumps(dict(hands={'right':asdict(self.profile)},
                hand_raw_rate={'right':[100]*6},hand_raw_lead={'right':[20]*6})),encoding='utf-8')
            adapter=self.make(profiles=None,hand_raw_rate=None,hand_raw_lead=None,config_path=path)
            self.assertEqual(adapter.profiles['right'],self.profile)
            path.write_text(json.dumps(dict(hands={'right':asdict(self.profile)})),encoding='utf-8')
            adapter=self.make(profiles=None,hand_raw_rate=None,hand_raw_lead=None,config_path=path)
            self.assertEqual(adapter.profiles,{});self.assertIn('hand_raw_rate',adapter.disabled_reasons['right'])

    def test_strict_config_validator_accepts_complete_independent_sides_without_mutating_input(self):
        for sides in (('right',),('left','right')):
            value=dict(hands={side:asdict(self.profile) for side in sides},
                hand_raw_rate={side:[100]*6 for side in sides},hand_raw_lead={side:[20]*6 for side in sides})
            before=json.dumps(value)
            self.assertEqual(validate_brainco_config(value),sorted(sides))
            self.assertEqual(json.dumps(value),before)

    def test_strict_config_validator_rejects_wrong_files_and_partial_selected_sides(self):
        valid=dict(hands={'right':asdict(self.profile)},hand_raw_rate={'right':[100]*6},hand_raw_lead={'right':[20]*6})
        cases=[None,[],{},dict(valid,extra=True),{name:item for name,item in valid.items() if name!='hand_raw_lead'},
               dict(valid,hands={}),dict(valid,hands={'center':asdict(self.profile)}),dict(valid,hand_raw_rate={}),
               dict(valid,hand_raw_lead={'left':[20]*6}),dict(valid,hand_raw_rate=[])]
        for field,bad in (('speed',[-1]*6),('opened',[1]*5),('evidence',''),('unexpected',True)):
            cases.append(dict(valid,hands={'right':dict(asdict(self.profile),**{field:bad})}))
        incomplete=asdict(self.profile);incomplete.pop('speed');cases.append(dict(valid,hands={'right':incomplete}))
        cases.append(dict(valid,hands={'right':dict(asdict(self.profile),closed=self.profile.opened)}))
        for field in ('hand_raw_rate','hand_raw_lead'):
            for bad in ([0]*6,[1]*5,[True]*6,[float('nan')]*6,[-1]*6,['1']*6):
                cases.append(dict(valid,**{field:{'right':bad}}))
        for index,value in enumerate(cases):
            with self.subTest(case=index),self.assertRaises(ValueError):validate_brainco_config(value)
        self.assertEqual(self.node.publishers,[])

    def test_initialization_uses_actual_six_motor_pose_and_never_opens_or_publishes(self):
        initial=self.adapter.measured_hand_initialization()['right']
        self.assertEqual(initial['positions'],(100.,)*6);self.assertTrue(initial['closure_is_projection'])
        self.assertEqual(self.adapter.hand_positions['right'],(100,)*6)
        self.assertEqual(len(self.node.publishers),1);self.assertEqual(self.node.publishers[0].messages,[])
        self.assertTrue(self.adapter.hardware_writes_enabled)
        self.assertTrue(self.submit().accepted)
        value=self.node.publishers[0].messages[-1]
        self.assertEqual(value.positions,[102]*6);self.assertEqual(value.mode,1)
        self.assertEqual(value.speeds,[20]*6);self.assertEqual(value.currents,[30]*6)
        self.assertEqual(value.pwms,[0]*6);self.assertEqual(value.durations,[0]*6)

    def test_any_existing_command_publisher_including_same_node_is_rejected_before_creation(self):
        for name,namespace in (('factory_hand','/'),(self.node.get_name(),self.node.get_namespace())):
            self.node.external['/right_hand/set_motor_multi']=[NS(node_name=name,node_namespace=namespace,
                topic_type='brainco_hand_msgs/msg/SetMotorMulti',endpoint_gid=bytes(24))]
            with self.subTest(name=name),self.assertRaisesRegex(RuntimeError,'competing_command_publisher'):
                self.adapter.measured_hand_initialization()
            self.assertEqual(self.node.publishers,[])

    def test_competitor_after_admission_closes_own_writer_without_any_open_command(self):
        self.adapter.measured_hand_initialization();self.assertTrue(self.submit().accepted)
        publisher=self.node.publishers[0];before=len(publisher.messages)
        self.node.external[publisher.topic]=[NS(node_name='factory_hand',node_namespace='/',
            topic_type='brainco_hand_msgs/msg/SetMotorMulti',endpoint_gid=bytes(24))]
        self.assertFalse(self.submit().accepted);self.assertTrue(publisher.closed)
        self.assertEqual(len(publisher.messages),before)
        self.assertFalse(self.adapter.initialized)

    def test_parent_disarmed_stale_controller_estop_and_hand_feedback_never_send_or_auto_resume(self):
        for condition in ('parent','audit','estop','hand','hand_state'):
            with self.subTest(condition=condition):
                self.setUp();self.adapter.measured_hand_initialization();publisher=self.node.publishers[0]
                self.now[0]+=.02;self.refresh()
                if condition=='parent':self.parent.state='disabled'
                elif condition=='audit':self.parent.audit_fresh=False
                elif condition=='estop':self.feedback.key_status(fixtures.power_key_message(estop=True))
                elif condition=='hand':self.feedback.hands['right']=None
                else:
                    value=fixtures.hand_message();value.states[0]=7;self.feedback.hand('right',value)
                self.assertTrue(self.adapter.check_watchdog());self.assertTrue(publisher.closed)
                self.assertEqual(publisher.messages,[])
                self.parent.state='armed';self.parent.audit_fresh=True;self.refresh()
                self.assertFalse(self.adapter.submit_hand('right',1.,intent_stamp=self.clock()).accepted)

    def test_arm_disarmed_and_stale_hand_are_rejected_before_publishers_open(self):
        self.parent.state='disabled'
        with self.assertRaisesRegex(RuntimeError,'parent_not_armed'):self.adapter.measured_hand_initialization()
        self.parent.state='armed';self.feedback.hands['right']=None
        with self.assertRaisesRegex(RuntimeError,'hand_feedback_missing'):self.adapter.measured_hand_initialization()
        self.assertEqual(self.node.publishers,[])

    def test_duplicate_stale_and_future_intents_do_not_publish(self):
        self.adapter.measured_hand_initialization();self.assertTrue(self.submit().accepted)
        publisher=self.node.publishers[0]
        duplicate=self.adapter.submit_hand('right',1.,intent_stamp=self.clock(),operation='close')
        self.assertEqual(duplicate.reason,'no_new_hand_intent');self.assertEqual(len(publisher.messages),1)
        self.assertTrue(self.adapter.hardware_writes_enabled)
        for stamp in (self.clock()-1,self.clock()+1,True,float('nan')):
            self.adapter.measured_hand_initialization()
            self.assertFalse(self.adapter.submit_hand('right',1.,intent_stamp=stamp).accepted)
        self.assertEqual(len(publisher.messages),1)

    def test_feedback_corridor_saturates_without_claiming_contact_and_bad_old_target_faults(self):
        self.adapter.measured_hand_initialization()
        for _ in range(40):self.assertTrue(self.submit(follow=False).accepted)
        self.assertEqual(self.adapter.hand_positions['right'],(120,)*6)
        diagnostics=self.adapter.hand_diagnostics['right']
        self.assertTrue(diagnostics['feedback_limited']);self.assertEqual(diagnostics['limit_interpretation'],'feedback_corridor_only')
        self.assertNotIn('contact',diagnostics)
        self.now[0]+=.02;self.refresh(70)
        result=self.adapter.submit_hand('right',1.,intent_stamp=self.clock(),operation='close')
        self.assertFalse(result.accepted);self.assertIn('feedback_lead_violation',result.reason)

    def test_fractional_rate_accumulates_and_trigger_decrease_never_opens(self):
        self.adapter.hand_raw_rate['right']=(1.,)*6;self.adapter.measured_hand_initialization()
        for _ in range(80):self.assertTrue(self.submit().accepted)
        self.assertEqual(self.adapter.hand_positions['right'],(102,)*6)
        before=self.adapter.hand_positions['right']
        for _ in range(20):self.assertTrue(self.submit(.01).accepted)
        self.assertEqual(self.adapter.hand_positions['right'],before)

    def test_close_honors_mixed_calibrated_directions_and_constant_channels(self):
        profile=HandCalibration((10,900,500,10,900,400),(900,10,500,900,10,400),
            (1,)*6,(1000,)*6,(20,)*6,(30,)*6,'TEST ONLY directions')
        self.adapter=self.make(profiles={'right':profile});self.adapter.measured_hand_initialization()
        self.assertTrue(self.submit(.1111111111).accepted)
        self.assertEqual(self.adapter.hand_positions['right'],(102,100,100,102,100,100))

    def test_invalidate_and_close_destroy_writer_retain_last_target_and_never_open(self):
        self.adapter.measured_hand_initialization();self.assertTrue(self.submit().accepted)
        publisher=self.node.publishers[0];before=self.adapter.hand_positions.copy();count=len(publisher.messages)
        self.assertTrue(self.adapter.invalidate('tracking_lost').accepted)
        self.assertTrue(publisher.closed);self.assertEqual(len(publisher.messages),count)
        self.assertEqual(self.adapter.hand_positions,before);self.assertTrue(self.adapter.close().accepted)
        self.assertFalse(self.adapter.submit_hand('right',0.,intent_stamp=self.clock(),operation='open').accepted)

    def test_message_schema_supports_documented_pwm_and_sampled_pwms_and_rejects_drift(self):
        class PdfHand(fixtures.HandMessage):
            @classmethod
            def get_fields_and_field_types(cls):
                fields=super().get_fields_and_field_types();fields['pwm']='int16[6]';fields.pop('pwms');return fields
        fields=hand_schema(PdfHand);message=encode_hand(PdfHand,fields,self.profile,(100,)*6)
        self.assertEqual(message.pwm,[0]*6);self.assertFalse(hasattr(message,'pwms'))
        for mutation in (dict(pwms='uint16[6]'),dict(extra='uint8'),dict(positions='uint16[7]')):
            class Drift(PdfHand):
                @classmethod
                def get_fields_and_field_types(cls):return dict(super().get_fields_and_field_types(),**mutation)
            with self.subTest(mutation=mutation),self.assertRaises(ValueError):hand_schema(Drift)

    def test_partial_creation_failure_destroys_first_side_writer_without_messages(self):
        adapter=self.make(profiles={'left':self.profile,'right':self.profile},
            hand_raw_rate=dict(left=(100,)*6,right=(100,)*6),hand_raw_lead=dict(left=(20,)*6,right=(20,)*6))
        self.node.fail_create='/right_hand/set_motor_multi'
        with self.assertRaisesRegex(RuntimeError,'creation failed'):adapter.measured_hand_initialization()
        self.assertEqual(len(self.node.publishers),1);self.assertTrue(self.node.publishers[0].closed)
        self.assertEqual(self.node.publishers[0].messages,[]);self.assertFalse(adapter.initialized)


if __name__=='__main__':unittest.main()
