from dataclasses import replace
import math
import unittest
import numpy as np
from tg3_pico_dualarm.controller import Teleoperation
from tg3_pico_dualarm.model import DualArmModel,default_q
from tg3_pico_dualarm.robot import DryRunRobot,ReadOnlyRobot,RobotState
from tg3_pico_dualarm.tracking import Pose,ControllerState,TrackingFrame


class StubIK:
    """Output-boundary tests only. This is not evidence of native IK solving."""
    def __init__(self):self.fail=False;self.anchors={};self.last_diagnostics={'backend':'test_stub'}
    def reset(self,side=None):self.anchors={}
    def latch_posture(self,side,q):self.anchors[side]=q.copy()
    def solve(self,q,targets,active,dt):
        if self.fail:raise RuntimeError('injected_native_failure')
        result=q.copy()
        for side,offset in (('left',0),('right',7)):
            if active[side]:result[offset]+=.001
        return result


class ControllerTests(unittest.TestCase):
    def setUp(self):
        self.time=1.;self.seq=0;self.model=DualArmModel()
        self.robot=DryRunRobot(default_q(),monotonic=lambda:self.time)
        self.ik=StubIK();self.c=Teleoperation(self.robot,self.model,self.ik,monotonic=lambda:self.time)

    def tick(self,grip=0.,focused=True):
        self.time+=.02;self.seq+=1
        hand=ControllerState(Pose((0.,1.,-.2),(0.,0.,0.,1.)),True,True,grip,0.,(),False)
        f=TrackingFrame('t',self.seq,self.time,self.time,Pose((0.,1.6,0.),(0.,0.,0.,1.)),hand,hand,focused)
        return self.c.tick(f)

    def test_scheduling_jitter_does_not_create_catchup_motion(self):
        self.tick()
        self.tick(1.)
        before=np.asarray(self.robot.read_state().joint_positions)
        observed=[]
        original=self.ik.solve
        def capture(q,targets,active,dt):
            observed.append(dt)
            return original(q,targets,active,dt)
        self.ik.solve=capture
        self.time+=.08;self.seq+=1
        hand=ControllerState(Pose((0.,1.,-.2),(0.,0.,0.,1.)),True,True,1.,0.,(),False)
        frame=TrackingFrame('t',self.seq,self.time,self.time,Pose((0.,1.6,0.),(0.,0.,0.,1.)),hand,hand)
        result=self.c.tick(frame,.08)
        self.assertEqual(result['state'],'dry_run',result)
        self.assertEqual(observed,[.02])
        after=np.asarray(self.robot.read_state().joint_positions)
        self.assertLessEqual(float(np.max(abs(after-before))),self.c.filter.config.joint_speed*.02+1e-10)
        self.assertEqual(result['cycle_timing']['observed_interval_s'],.08)

    def test_pause_beyond_original_watchdog_still_stops_without_command(self):
        self.tick();self.tick(1.)
        count=len(self.robot.commands)
        result=self.c.tick(None,.151)
        self.assertEqual(result['reason'],'control_tick_overrun')
        self.assertEqual(result['failure_details']['tick_interval_s'],.151)
        self.assertEqual(len(self.robot.commands),count)

    def test_jitter_does_not_renew_stale_tracking(self):
        self.tick();self.tick(1.)
        self.time+=.08;self.seq+=1
        hand=ControllerState(Pose((0.,1.,-.2),(0.,0.,0.,1.)),True,True,1.,0.,(),False)
        frame=TrackingFrame('t',self.seq,self.time-.25,self.time-.25,Pose((0.,1.6,0.),(0.,0.,0.,1.)),hand,hand)
        result=self.c.tick(frame,.08)
        self.assertFalse(any(result['active'].values()))

    def test_loss_holds_and_requires_release_before_reentry(self):
        self.tick();self.tick(1.);before=self.robot.read_state().joint_positions
        self.tick(1.,False)
        np.testing.assert_allclose(self.robot.read_state().joint_positions,before)
        self.assertFalse(any(self.tick(1.)['active'].values()))
        self.tick();self.assertTrue(all(self.tick(1.)['active'].values()))

    def test_failed_solver_does_not_submit_previous_command(self):
        self.tick();self.tick(1.);count=len(self.robot.commands)
        self.ik.fail=True;result=self.tick(1.)
        self.assertEqual(result['state'],'disarmed');self.assertEqual(len(self.robot.commands),count)

    def test_receipt_clock_read_after_state_prevents_microsecond_future_false_alarm(self):
        clock=[30.]
        def advancing():clock[0]+=.000001;return clock[0]
        robot=DryRunRobot(default_q(),monotonic=advancing)
        controller=Teleoperation(robot,self.model,StubIK(),monotonic=advancing)
        for sequence in range(5):
            now=advancing()
            hand=ControllerState(Pose((0.,1.,-.2),(0.,0.,0.,1.)),True,True,0.,0.,(),False)
            f=TrackingFrame('real_receipt',sequence,now,now,Pose((0.,1.6,0.),(0.,0.,0.,1.)),hand,hand)
            status=controller.tick(f)
            self.assertEqual(status['state'],'dry_run',status)

    def test_readonly_hardware_cannot_be_armed_by_clutch(self):
        self.c=Teleoperation(ReadOnlyRobot(lambda:RobotState(tuple(default_q()),self.time,'HBWALK',None,'observed')),
                             self.model,self.ik,monotonic=lambda:self.time)
        self.assertEqual(self.tick(1.)['reason'],'verified_dual_arm_hardware_adapter_required')

    def test_new_reference_requires_neutral_and_recalibrates_head_heading(self):
        self.tick();self.tick(1.)
        self.time+=.02
        hand=ControllerState(Pose((0.,1.,-.2),(0.,0.,0.,1.)),True,True,1.,0.,(),False)
        head=Pose((0.,1.6,0.),(0.,math.sin(math.pi/4),0.,math.cos(math.pi/4)))
        f=TrackingFrame('new-reference',1,self.time,self.time,head,hand,hand)
        result=self.c.tick(f)
        self.assertEqual(result['reason'],'release_inputs_for_heading_calibration')
        self.time+=.02
        neutral=replace(hand,grip=0.)
        self.c.tick(replace(f,sequence=2,source_time_s=self.time,received_at=self.time,left=neutral,right=neutral))
        np.testing.assert_allclose(self.c.mapper.calibration.xr_to_robot@[-1.,0.,0.],[1.,0.,0.],atol=1e-12)
