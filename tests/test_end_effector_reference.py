from copy import deepcopy
from dataclasses import replace
import json
from pathlib import Path
import unittest
import numpy as np
from tg3_pico_dualarm.tool_setup import tcp_reference_tools,reference_status,measured_tools
from tg3_pico_dualarm.mapping import Calibration,RelativeMapper
from tg3_pico_dualarm.motion_control import MotionAdmission
from tg3_pico_dualarm.readiness import readiness
from tg3_pico_dualarm.hands import HandCalibration
from tg3_pico_dualarm.brainco_adapter import encode_hand,hand_schema
from tg3_pico_dualarm.hand_observe import summarize
from test_mapping import frame
import test_manufacturer as factory


ROOT=Path(__file__).resolve().parents[1]


class ReferenceTests(unittest.TestCase):
    def setUp(self):self.config=json.loads((ROOT/'config/calibration.yaml').read_text())

    def test_model_reference_is_a_definition_and_never_a_fake_palm_measurement(self):
        status=reference_status(self.config)
        self.assertEqual(status['kind'],'urdf_tcp');self.assertTrue(status['ready'])
        self.assertFalse(status['offsets_measured']);self.assertFalse(self.config['tool_offsets_calibrated'])
        for mutation in ('model','offset','measurement','source'):
            value=deepcopy(self.config)
            if mutation=='model':value['reference_model_sha256']='0'*64
            elif mutation=='offset':value['tcp_to_hand']['right'][0][3]=.1
            elif mutation=='measurement':value['tool_offsets_calibrated']=True
            else:value['controller_reference']='unknown_aim_pose'
            with self.subTest(mutation=mutation),self.assertRaises(ValueError):reference_status(value)

    def test_measured_grasp_mode_still_needs_measurements_and_old_configs_do_not_silently_enable(self):
        old=deepcopy(self.config);old.pop('tool_reference')
        self.assertFalse(reference_status(old)['ready'])
        entries={name:{side:[0]*6 for side in ('left','right')} for name in ('controller_to_hand','tcp_to_hand')}
        measured=measured_tools(self.config,entries,'explicit test measurement',True)
        self.assertEqual(reference_status(measured)['kind'],'measured_grasp_point')
        self.assertTrue(reference_status(measured)['ready'])
        self.assertFalse(tcp_reference_tools(measured)['tool_offsets_calibrated'])

    def test_tcp_clutch_initialization_and_regrasp_are_continuous(self):
        options={k:self.config[k] for k in ('controller_to_hand','tcp_to_hand','tool_offsets_calibrated','tool_reference')}
        calibration=Calibration.from_head(frame(0).head,**options);mapper=RelativeMapper(calibration)
        poses=dict(left=np.eye(4),right=np.eye(4))
        mapper.update(frame(0),poses,1.)
        np.testing.assert_allclose(mapper.update(frame(1,1.),poses,1.02).targets['right'],np.eye(4))
        self.assertEqual(calibration.as_dict()['tool_calibration_source'],'defined_openxr_grip_to_URDF_TCP')
        mapper.update(frame(2,0.,position=(0.,1.,-.1)),poses,1.04)
        poses['right'][0,3]=.04
        value=mapper.update(frame(3,1.,position=(0.,1.,-.1)),poses,1.06)
        np.testing.assert_allclose(value.targets['right'],poses['right'])

    def test_explicit_tcp_reference_allows_admission_but_neutral_tracking_and_estop_still_gate(self):
        fixture=factory.ManufacturerTests();fixture.setUp();f=fixture
        gate=MotionAdmission(f.adapter,calibration_config=self.config,clock=f.clock)
        value=replace(frame(1),session='test',source_time_s=f.clock(),received_at=f.clock(),scene_visible=True)
        held=replace(value,right=replace(value.right,grip=1.))
        self.assertFalse(gate.request('enable',held,expected_session=held.session)['accepted'])
        self.assertEqual(f.driver.events,[])
        self.assertTrue(gate.request('enable',value,expected_session=value.session)['accepted'])
        self.assertFalse(gate.tools_calibrated)
        self.assertEqual(gate.status()['end_effector_reference']['kind'],'urdf_tcp')

    def test_readiness_distinguishes_target_reference_from_palm_measurement(self):
        tracking=dict(head_tracked=True,left_tracked=True,right_tracked=True,left_release_required=False,right_release_required=False)
        maker=dict(available=True,feedback_ready=True,physical_estop=False,state='disabled')
        result=readiness(tracking=tracking,native_ik=True,ros_feedback=True,headset_build=True,
            manufacturer=maker,end_effector=reference_status(self.config))
        self.assertTrue(result['ready_to_request_motion']);self.assertFalse(result['tool_offsets_measured'])
        self.assertFalse(result['hardware_motion_enabled'])


class ReadOnlyHandTests(unittest.TestCase):
    def test_stable_feedback_is_not_a_verified_grasp_or_mechanical_limit(self):
        rows=[dict(received_at=1+i*.04,positions=(400,400,50,50,50,50),speeds=(0,)*6,states=(0,)*6) for i in range(20)]
        result=summarize(rows,1.8)
        self.assertTrue(result['ready']);self.assertFalse(result['grasp_verified']);self.assertFalse(result['mechanical_limits_verified'])
        self.assertFalse(summarize(rows,2.1)['ready'])
        moving=deepcopy(rows);moving[-1]['positions']=(400,400,80,50,50,50)
        self.assertFalse(summarize(moving,1.8)['ready'])
        moving=deepcopy(rows);moving[-1]['states']=(1,0,0,0,0,0)
        self.assertFalse(summarize(moving,1.8)['ready'])

    def test_documented_position_only_message_can_use_zero_inactive_fields(self):
        profile=HandCalibration((50,)*6,(600,)*6,(1,)*6,(1000,)*6,(0,)*6,(0,)*6,'SDK 26.06 PDF37 mode1 fixture')
        # The simple ROS schema double is reused only for formatter validation.
        from test_manufacturer import HandMessage as Message
        message=encode_hand(Message,hand_schema(Message),profile,(100,)*6)
        self.assertEqual(message.mode,1);self.assertEqual(list(message.speeds),[0]*6)
        self.assertEqual(list(message.currents),[0]*6)


if __name__=='__main__':unittest.main()
