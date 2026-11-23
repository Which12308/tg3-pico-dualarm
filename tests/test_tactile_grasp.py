"""Exercise the ROS hand command path with synthetic tactile data."""
from dataclasses import asdict
from pathlib import Path
import tempfile
from types import SimpleNamespace as NS
import unittest
from unittest.mock import Mock, patch

from tg3_pico_dualarm import safety
from tg3_pico_dualarm.brainco_adapter import BrainCoHandAdapter, validate_brainco_config
from tg3_pico_dualarm.hands import HandCalibration
from tg3_pico_dualarm.manufacturer_feedback import ManufacturerFeedback
from tg3_pico_dualarm.manufacturer_grasp import ManufacturerGrasp
from tg3_pico_dualarm.recording_writer import DatasetWriter, write_outcome
from tg3_pico_dualarm.rosbag_record import select_topics
from tg3_pico_dualarm.xarm_adapter import XArmAdapter
from test_brainco_adapter import HandNode, XArmParent
import test_manufacturer as fixtures


class TactileGraspTests(unittest.TestCase):
    def setUp(self):
        policy = patch.object(safety, 'LIMITS_MANAGED_EXTERNALLY', True)
        policy.start()
        self.addCleanup(policy.stop)
        self.now = 10.
        self.feedback = ManufacturerFeedback(clock=lambda: self.now)
        self.node = HandNode()
        self.parent = XArmParent(self.feedback)
        self.opened = (10,) * 6
        self.profile = HandCalibration(self.opened, (700,) * 6, (0,) * 6, (1000,) * 6,
                                       (0,) * 6, (0,) * 6, 'Fixture only')
        self.tactile = dict(current_mode=4, thumb_position=300, position_tolerance=5,
                            contact_threshold=[20] * 5, target_force=[40] * 5, force_deadband=[5] * 5,
                            current_min=[30] * 5, current_max=[40] * 5, current_step=[5] * 5,
                            closing_sign=[1, 1, -1, 1, 1], opposition_current=9,
                            support_fingers=[0, 1, 2], evidence='Fixture only')
        self.refresh(self.opened)
        self.adapter = BrainCoHandAdapter(self.node,self.feedback,parent=self.parent,
            profiles={'right':self.profile}, hand_raw_rate={'right':[100]*6},
            hand_raw_lead={'right':[20]*6},right_tactile=self.tactile,
            message_type=fixtures.HandMessage,clock=lambda:self.now)
        self.adapter.measured_hand_initialization()
        self.commands = self.node.publishers[0].messages

    def refresh(self, positions, force=None, sequence=0):
        self.now += .02
        self.feedback.arm(fixtures.arm_message())
        self.feedback.fsm(fixtures.mode_message())
        self.feedback.key_status(fixtures.power_key_message())
        motor = fixtures.hand_message()
        motor.positions = positions
        self.feedback.hand('right', motor)
        if force is not None:
            self.feedback.touch(NS(data=[NS(normal_force1=load,normal_force2=0,normal_force3=0,
                tangential_force1=1,tangential_force2=0,tangential_force3=0,status=sequence<<8) for load in force]))

    def submit(self, operation, closure=1.):
        return self.adapter.submit_hand('right',closure,operation=operation,intent_stamp=self.now)

    def test_real_command_shape_thumb_during_close_current_after_contact_and_position_on_release(self):
        self.assertFalse(self.submit('close').accepted)
        self.assertEqual(self.commands, [])
        self.refresh(self.opened,[0]*5,1)
        self.assertTrue(self.submit('close').accepted)
        self.assertEqual(self.commands[-1].positions, [10,300,10,10,10,10])
        self.assertEqual(self.commands[-1].mode, 1)
        self.refresh((140,300,10,10,10,10),[25,0,0,0,0],2)
        self.assertTrue(self.submit('close').accepted)
        self.assertEqual(self.commands[-1].positions, [140,300,700,700,700,700])
        pose = (140,300,550,600,690,700)
        self.refresh(pose, [25,25,25,0,0],3)
        self.assertTrue(self.submit('close').accepted)
        self.assertEqual(self.commands[-1].mode, 4)
        self.assertEqual(self.commands[-1].currents, [30,9,30,-30,30,30])
        self.assertFalse(self.submit('hold').accepted)
        self.refresh(pose, [20,40,55,0,80],4)
        self.assertTrue(self.submit('hold',0.).accepted)
        self.assertEqual(self.commands[-1].currents, [35,9,30,-30,35,30])
        self.refresh(pose)
        self.assertTrue(self.submit('open',0.).accepted)
        self.assertEqual(self.commands[-1].mode, 1)
        self.assertEqual(self.commands[-1].positions, [10,300,10,10,10,10])
        self.refresh((10,300,10,10,10,10))
        self.assertTrue(self.submit('open',0.).accepted)
        self.assertEqual(self.commands[-1].positions, list(self.opened))

    def test_position_at_endpoint_without_contact_never_selects_current(self):
        self.refresh((700,300,700,700,700,700),[0]*5,1)
        self.assertTrue(self.submit('close').accepted)
        self.assertEqual(self.commands[-1].mode, 1)
        self.assertFalse(self.submit('hold',0.).accepted)

    def test_tracking_trigger_release_continues_current_and_button_opens(self):
        robot = Mock(spec=XArmAdapter)
        robot.lock = self.parent.lock
        robot.profiles = self.adapter.profiles
        robot.hand_positions = self.adapter.hand_positions
        robot.hand_diagnostics = self.adapter.hand_diagnostics
        robot.hands = self.adapter
        robot.feedback = self.feedback
        robot.hardware_writes_enabled = True
        robot.measured_hand_initialization = self.adapter.measured_hand_initialization
        robot.submit_hand = self.adapter.submit_hand
        grasp = ManufacturerGrasp(robot,clock=lambda:self.now)
        grasp.initialize()

        def step(sequence, trigger=0., buttons=()):
            item = NS(connected=True,tracked=True,pose=object(),source_id='controller',neutral=not (trigger or buttons),
                      trigger=trigger,buttons=buttons,release_required=False)
            frame = NS(session='headset',sequence=sequence,source_time_s=self.now,
                       right=item,received_at=self.now,focused=True)
            with patch('tg3_pico_dualarm.manufacturer_grasp.validate_frame'):
                return grasp.step(frame)

        pose = (100,300,300,300,300,300)
        step(1)
        self.refresh(pose,[25]*5,1)
        active = step(2,1.)
        self.assertEqual(active['motor_diagnostics']['right']['grasp_phase'],'holding')
        self.assertIsNone(active['commanded_motor_positions']['right'])
        self.refresh(pose,[25]*5,2)
        holding = step(3)
        self.assertTrue(holding['hardware_hand_commands_sent'])
        self.assertEqual(self.commands[-1].currents,[35,9,35,-35,35,35])
        self.refresh(pose)
        self.assertTrue(step(4,1.,('B',))['hardware_hand_commands_sent'])
        self.assertEqual(self.commands[-1].mode,1)

    def test_recording_selects_actual_tactile_type_and_saves_episode_outcome(self):
        channel='/right_hand/touch_status'
        schema='brainco_hand_msgs/msg/TouchStatus'
        self.assertEqual(select_topics({channel:[schema]}),{channel:schema})
        with tempfile.TemporaryDirectory() as root:
            writer=DatasetWriter(root,{},reserve=0)
            with self.assertRaisesRegex(ValueError,'not_complete'):
                write_outcome(writer.directory,'success')
            writer.close()
            self.assertIn('unlabeled',(writer.directory/'outcome.json').read_text())
            write_outcome(writer.directory,'success')
            self.assertIn('success',(writer.directory/'outcome.json').read_text())
            self.assertTrue((writer.directory/'ros_0001.mcap').is_file())

    def test_existing_hand_config_and_tactile_extension_share_entry_point(self):
        profile=asdict(self.profile)
        value=dict(hands={'right':profile},hand_raw_rate={'right':[100]*6},hand_raw_lead={'right':[20]*6})
        self.assertEqual(validate_brainco_config(value),['right'])
        self.assertEqual(validate_brainco_config(dict(value,right_tactile=self.tactile)),['right'])
