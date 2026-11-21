"""XARM target window inside the fixed feedback lead gate. Offline doubles only.

These tests do not establish physical tracking, stopping or any real-arm
behaviour. The 0.03 rad submit check is asserted unchanged.
"""
import json
from pathlib import Path
import tempfile
import unittest
import numpy as np
from tg3_pico_dualarm.model import DualArmModel,default_q
from tg3_pico_dualarm.robot import JointCommand,SubmitResult
from tg3_pico_dualarm.safety import JointSafetyFilter,SafetyError,SafetyLimits
from tg3_pico_dualarm.xarm_config import load_xarm_settings
import test_xarm_adapter as xarm_fixtures
import test_controller


ROOT=Path(__file__).resolve().parents[1]
GATE=.03;RESERVE=.005;WINDOW=GATE-RESERVE


def xarm_filter(model,boundary_hold=True):
    return JointSafetyFilter(model,SafetyLimits(joint_speed=.02,feedback_lead=WINDOW),
                             boundary_hold=boundary_hold,feedback_gate=GATE)


class XArmFeedbackWindowTests(unittest.TestCase):
    def setUp(self):self.model=DualArmModel();self.q=default_q()

    def test_checked_profile_keeps_gate_and_adds_reserve(self):
        settings=load_xarm_settings(ROOT/'config/xarm.yaml')
        self.assertEqual(settings.feedback_lead,(GATE,)*14)
        self.assertEqual(settings.feedback_lead_reserve,RESERVE)
        value=json.loads((ROOT/'config/xarm.yaml').read_text(encoding='utf-8'))
        with tempfile.TemporaryDirectory() as folder:
            path=Path(folder)/'xarm.yaml'
            legacy=dict(value);legacy.pop('feedback_lead_reserve_rad')
            path.write_text(json.dumps(legacy),encoding='utf-8')
            self.assertEqual(load_xarm_settings(path).feedback_lead_reserve,RESERVE)
            for bad in (-.001,.011,float('nan'),True,'0.005'):
                candidate=dict(value);candidate['feedback_lead_reserve_rad']=bad
                path.write_text(json.dumps(candidate),encoding='utf-8')
                with self.subTest(bad=bad),self.assertRaises(ValueError):load_xarm_settings(path)
            candidate=dict(value,feedback_lead_rad=.004,feedback_lead_reserve_rad=.005)
            path.write_text(json.dumps(candidate),encoding='utf-8')
            with self.assertRaises(ValueError):load_xarm_settings(path)
        with self.assertRaises(ValueError):xarm_fixtures.settings(feedback_lead_reserve=.2)

    def test_gate_cannot_be_narrower_than_window_or_wider_than_envelope(self):
        for gate in (.02,.036,float('nan'),True):
            with self.subTest(gate=gate),self.assertRaises(ValueError):
                JointSafetyFilter(self.model,SafetyLimits(joint_speed=.02,feedback_lead=WINDOW),feedback_gate=gate)
        self.assertEqual(JointSafetyFilter(self.model).feedback_gate,.035)

    def test_targets_stay_inside_measured_window_and_survive_feedback_moving_by_reserve(self):
        guarded=xarm_filter(self.model)
        previous=self.q.copy();previous[8]+=WINDOW-.0002;velocity=np.zeros(14);velocity[8]=.01
        proposed=previous.copy();proposed[8]+=.05
        target,v=guarded.step(proposed,self.q,previous,velocity,.02)
        self.assertLessEqual(target[8]-self.q[8],WINDOW+1e-12)
        self.assertTrue(any(d['joint_index']==8 for d in guarded.last_diagnostics['window_limited']))
        # Submit re-reads feedback; it may have moved away by up to the reserve.
        drifted=self.q.copy();drifted[8]-=RESERVE-1e-6
        self.assertLessEqual(abs(target[8]-drifted[8]),GATE)
        # The previous 0.03 window would put the same target on the gate edge.
        old=JointSafetyFilter(self.model,SafetyLimits(joint_speed=.02,feedback_lead=GATE),boundary_hold=True)
        previous_old=self.q.copy();previous_old[8]+=GATE-.0002
        old_target,_=old.step(previous_old+.05*(np.arange(14)==8),self.q,previous_old,velocity,.02)
        self.assertGreater(abs(old_target[8]-drifted[8]),GATE)

    def test_target_between_window_and_gate_is_held_or_pulled_in_not_rejected(self):
        guarded=xarm_filter(self.model)
        previous=self.q.copy();previous[8]+=.028
        proposed=previous.copy();proposed[8]+=.01;proposed[1]+=.001
        target,v=guarded.step(proposed,self.q,previous,np.zeros(14),.02)
        self.assertLessEqual(target[8],previous[8]);self.assertLessEqual(v[8],0.)
        self.assertGreater(target[1],self.q[1])
        outside=guarded.last_diagnostics['outside_window']
        self.assertEqual([d['joint_index'] for d in outside],[8])
        self.assertAlmostEqual(outside[0]['difference_rad'],.028)
        pulled,_=guarded.step(self.q,self.q,previous,np.zeros(14),.02)
        self.assertLess(pulled[8],previous[8])

    def test_target_beyond_absolute_gate_still_rejected(self):
        guarded=xarm_filter(self.model)
        previous=self.q.copy();previous[8]+=GATE+.0001
        with self.assertRaisesRegex(SafetyError,'previous_target_outside_feedback_window'):
            guarded.step(previous,self.q,previous,np.zeros(14),.02)


class XArmSubmitRejectionRecordTests(unittest.TestCase):
    def setUp(self):
        self.fixture=xarm_fixtures.XArmAdapterTests();self.fixture.setUp();self.f=self.fixture

    def test_lead_rejection_records_joint_target_measured_and_feedback_time(self):
        self.f.robot.limits.feedback_lead=(GATE,)*14;self.f.arm()
        command=self.f.command();measured=list(default_q())
        q=list(command.positions);q[8]=measured[8]+GATE+.001
        self.f.robot._target=tuple(q)
        result=self.f.robot.submit(JointCommand(tuple(q),command.velocities,command.stamp))
        self.assertFalse(result.accepted);self.assertIn('xarm_feedback_lead_exceeded',result.reason)
        record=self.f.robot.last_submit_rejection
        self.assertEqual((record['joint_index'],record['motor_id'],record['side']),(8,22,'right'))
        self.assertAlmostEqual(record['target_rad'],q[8]);self.assertAlmostEqual(record['measured_rad'],measured[8])
        self.assertAlmostEqual(record['difference_rad'],GATE+.001);self.assertEqual(record['limit_rad'],GATE)
        self.assertEqual(record['command_stamp'],command.stamp)
        self.assertGreaterEqual(record['feedback_age_s'],0.);self.assertIsNotNone(record['feedback_stamp'])
        self.assertEqual(self.f.robot.status()['last_submit_rejection'],record)
        self.assertEqual(self.f.transport.published,[])

    def test_controller_disarm_carries_rejection_details_for_that_command(self):
        tests=test_controller.ControllerTests();tests.setUp();tests.tick();tests.tick(1.)
        robot=tests.robot;original=robot.submit
        def reject(command):
            robot.last_submit_rejection=dict(reason='xarm_feedback_lead_exceeded',joint_index=8,motor_id=22,
                target_rad=1.,measured_rad=.97,feedback_stamp=command.stamp,command_stamp=command.stamp)
            return SubmitResult(False,'xarm_feedback_lead_exceeded')
        robot.submit=reject
        result=tests.tick(1.)
        self.assertEqual(result['reason'],'robot_rejected:xarm_feedback_lead_exceeded')
        self.assertEqual(result['failure_details']['submit_rejection']['motor_id'],22)
        robot.submit=original


if __name__=='__main__':unittest.main()
