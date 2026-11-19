"""Documented board feedback with explicit doubles; no ROS or robot acceptance."""
from types import SimpleNamespace as NS
import sys
import unittest
from unittest.mock import patch

from tg3_pico_dualarm.manufacturer_feedback import ManufacturerFeedback,POWER_KEY_FIELDS,POWER_KEY_SOURCE
from test_manufacturer import arm_message,mode_message,power_key_message
from test_manufacturer_session import fake_ros


class ManufacturerFeedbackEstopTests(unittest.TestCase):
    def setUp(self):
        self.now=[10.]
        self.feedback=ManufacturerFeedback(clock=lambda:self.now[0])
        self.feedback.arm(arm_message());self.feedback.fsm(mode_message())

    def test_both_documented_estops_are_active_high_and_combined_with_or(self):
        for local,remote in ((False,False),(True,False),(False,True),(True,True)):
            with self.subTest(local=local,remote=remote):
                self.feedback.key_status(power_key_message(local,remote))
                snapshot=self.feedback.snapshot(require_hands=False)
                self.assertIs(snapshot['physical_estop'],local or remote)
                self.assertIs(self.feedback.read_state().estop,local or remote)
                self.assertEqual(snapshot['physical_estop_stamp'],10.)
                self.assertEqual(snapshot['physical_estop_source'],POWER_KEY_SOURCE)
                self.assertIs(snapshot['power_key_status']['is_estop'],local)
                self.assertIs(snapshot['power_key_status']['is_remote_estop'],remote)
                self.assertIs(snapshot['ready'],not (local or remote))
                self.assertTrue(snapshot['physical_estop_fresh'])
                self.assertFalse(snapshot['robot_commands_sent'])

    def test_fsm_and_external_false_cannot_fill_missing_or_expired_board_estop(self):
        self.assertIsNone(self.feedback.read_state().estop)
        self.assertIsNone(self.feedback.read_state(False).estop)
        self.assertFalse(self.feedback.snapshot(False)['ready'])
        self.feedback.key_status(power_key_message())
        self.assertIs(self.feedback.read_state(False).estop,False)
        self.now[0]+=.251
        self.feedback.arm(arm_message());self.feedback.fsm(mode_message())
        value=self.feedback.snapshot(False)
        self.assertIsNone(value['physical_estop']);self.assertFalse(value['physical_estop_fresh'])
        self.assertEqual(value['physical_estop_stamp'],10.)
        self.assertIn('physical_estop_feedback_missing_or_stale',value['issues'])
        self.assertIsNone(self.feedback.read_state(False).estop)
        self.assertIs(self.feedback.read_state(True).estop,True)

    def test_nonboolean_or_missing_raw_fields_immediately_revoke_previous_release(self):
        for field in POWER_KEY_FIELDS:
            for bad in (0,1,0.,None,'false',NS(data=False)):
                with self.subTest(field=field,bad=bad):
                    self.feedback.key_status(power_key_message())
                    message=power_key_message();getattr(message,field).data=bad
                    self.feedback.key_status(message)
                    value=self.feedback.snapshot(False)
                    self.assertIsNone(value['physical_estop']);self.assertIsNone(value['physical_estop_stamp'])
                    self.assertIsNone(value['power_key_status']);self.assertFalse(value['ready'])
                    self.assertIn('nonboolean',value['errors']['power_key_status'])
                    self.assertIsNone(self.feedback.read_state(False).estop)
            self.feedback.key_status(power_key_message())
            message=power_key_message();delattr(message,field)
            self.feedback.key_status(message)
            self.assertIsNone(self.feedback.read_state().estop)
        self.feedback.key_status(power_key_message())
        self.assertIs(self.feedback.read_state().estop,False)
        self.assertNotIn('power_key_status',self.feedback.snapshot(False)['errors'])

    def test_clock_rollback_does_not_make_feedback_fresh_and_timeout_is_bounded(self):
        self.feedback.key_status(power_key_message());self.now[0]-=1.
        self.assertIsNone(self.feedback.read_state().estop)
        for invalid in (True,0,-1,.501,float('nan'),float('inf')):
            with self.subTest(timeout=invalid),self.assertRaises(ValueError):
                ManufacturerFeedback(estop_timeout=invalid)

    def test_real_message_class_is_subscribed_and_no_publishers_are_created(self):
        modules,calls=fake_ros()
        with patch.dict(sys.modules,modules):
            node=modules['rclpy.node'].Node('test-only')
            self.feedback.subscribe(node)
            self.assertIn('/power/board/key_status',[sub.topic for sub in self.feedback.subscriptions])
            self.assertIs(self.feedback.read_state().estop,False)
            self.assertEqual(list(node.publishers),[])
            self.feedback.close()
            self.assertEqual(node._subscriptions,[])

    def test_absent_vendor_package_keeps_other_feedback_observable_but_unknown(self):
        modules,calls=fake_ros();modules.update({'bodyctrl_msgs':None,'bodyctrl_msgs.msg':None})
        with patch.dict(sys.modules,modules):
            node=modules['rclpy.node'].Node('test-only')
            self.feedback.subscribe(node)
            value=self.feedback.snapshot(False)
            self.assertEqual(len(node._subscriptions),4)
            self.assertIsNone(value['physical_estop']);self.assertFalse(value['ready'])
            self.assertIn('ModuleNotFoundError',value['errors']['power_key_status'])
            self.assertEqual(self.feedback.read_state().quality,'measured')
            self.assertEqual(list(node.publishers),[])
            self.feedback.close()

    def test_incompatible_installed_schema_or_type_support_never_subscribes_board(self):
        for failure in ('field','missing','nested_bool','support'):
            modules,calls=fake_ros()
            base=modules['bodyctrl_msgs.msg'].PowerBoardKeyStatus
            if failure in ('field','missing'):
                fields=base.get_fields_and_field_types()
                if failure=='field':fields['is_remote_estop']='boolean'
                else:fields.pop('is_estop')
                class Changed:
                    @classmethod
                    def get_fields_and_field_types(cls):return fields
                modules['bodyctrl_msgs.msg'].PowerBoardKeyStatus=Changed
            elif failure=='nested_bool':
                class ChangedBool:
                    @classmethod
                    def get_fields_and_field_types(cls):return {'data':'uint8'}
                modules['std_msgs.msg'].Bool=ChangedBool
            else:
                def unsupported(cls):
                    if cls is base:raise RuntimeError('fixture native support unavailable')
                modules['rclpy.type_support'].check_for_type_support=unsupported
            with self.subTest(failure=failure),patch.dict(sys.modules,modules):
                feedback=ManufacturerFeedback(clock=lambda:10.)
                node=modules['rclpy.node'].Node('test-only');feedback.subscribe(node)
                self.assertEqual(len(node._subscriptions),4)
                self.assertIsNone(feedback.read_state(False).estop)
                self.assertIn('PowerBoardKeyStatus_subscription_unavailable',feedback.snapshot(False)['errors']['power_key_status'])
                feedback.close()


if __name__=='__main__':unittest.main()
