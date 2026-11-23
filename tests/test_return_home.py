from unittest import TestCase
from unittest.mock import Mock, patch
import numpy as np

from tg3_pico_dualarm import safety
from tg3_pico_dualarm.controller import Teleoperation, RETURN_SPEED_RAD_S
from tg3_pico_dualarm.model import DualArmModel, default_q
from tg3_pico_dualarm.robot import RobotState, SubmitResult
from tg3_pico_dualarm.tracking import ControllerState, Pose, TrackingFrame
from tg3_pico_dualarm.wireless import GatewayClient, InputGateway, WirelessServer
from tg3_pico_dualarm.xarm_adapter import XArmAdapter
from test_controller import StubIK


class ReturnHomeTests(TestCase):
    def setUp(self):
        policy=patch.object(safety,'LIMITS_MANAGED_EXTERNALLY',True)
        policy.start()
        self.addCleanup(policy.stop)
        self.now=10.
        self.sequence=0
        self.home=np.asarray(default_q(),dtype=float)
        self.measured=self.home.copy()
        self.commands=[]
        self.robot=Mock(spec=XArmAdapter)
        self.robot.profiles={}
        self.robot.hardware_writes_enabled=True
        self.robot.output_interval_s=None
        self.robot.verified_velocity=None
        self.robot.commanded_joint_positions=tuple(self.home)
        self.robot.read_state.side_effect=lambda:RobotState(tuple(self.measured),self.now,'test',False,'measured')
        self.robot.submit.side_effect=self.submit
        self.robot.hold.return_value=SubmitResult(True,'test hold')
        self.robot.disarm.return_value=SubmitResult(True,'test disarm')
        self.c=Teleoperation(self.robot,DualArmModel(),StubIK(),allow_hardware=True,monotonic=lambda:self.now)
        self.c.reseed_after_hardware_enable()
        self.measured=self.home.copy()
        self.measured[8]+=.12
        self.robot.commanded_joint_positions=tuple(self.measured)
        self.c.target=self.measured.copy()

    def submit(self,command):
        self.commands.append(command)
        self.measured=np.asarray(command.positions)
        self.robot.commanded_joint_positions=command.positions
        return SubmitResult(True,'test command')

    def frame(self,grip=0.,session='test'):
        self.now+=.02
        self.sequence+=1
        pose=Pose((0.,1.,-.2),(0.,0.,0.,1.))
        hand=ControllerState(pose,True,True,grip,0.,(),False)
        return TrackingFrame(session,self.sequence,self.now,self.now,pose,hand,hand)

    def test_smooth_return_targets_the_snapshot_and_confirms_measured_arrival(self):
        self.assertTrue(self.c.request_home(self.frame(),'test')['accepted'])
        for _ in range(200):
            status=self.c.tick(self.frame())
            if status.get('return_home')=='arrived':break
        self.assertEqual(status.get('return_home'),'arrived')
        self.assertGreater(len(self.commands),1)
        self.assertLessEqual(max(max(map(abs,item.velocities)) for item in self.commands),RETURN_SPEED_RAD_S)
        np.testing.assert_allclose(self.commands[-1].positions,self.home)
        self.assertEqual(self.c.tick(self.frame())['return_home'],'arrived')

    def test_grip_cancels_return_until_neutral(self):
        self.assertTrue(self.c.request_home(self.frame(),'test')['accepted'])
        self.c.tick(self.frame())
        count=len(self.commands)
        self.assertEqual(self.c.tick(self.frame(grip=1.))['return_home'],'waiting_for_release')
        self.assertEqual(self.c.tick(self.frame(grip=1.))['return_home'],'waiting_for_release')
        self.assertEqual(len(self.commands),count)
        self.assertEqual(self.c.tick(self.frame())['return_home'],'cancelled')

    def test_old_frame_cancels_return_and_disarm_clears_reference(self):
        frame=self.frame()
        self.assertTrue(self.c.request_home(frame,'test')['accepted'])
        self.now+=1.
        self.assertEqual(self.c.tick(frame)['return_home'],'waiting_for_release')
        self.assertEqual(self.commands,[])
        self.c.disarm('test stop')
        self.assertFalse(self.c.request_home(self.frame(),'test')['accepted'])
        self.assertIsNone(self.c.home_joints)

    def test_request_rejects_held_input_and_other_sessions(self):
        self.assertFalse(self.c.request_home(self.frame(grip=1.),'test')['accepted'])
        self.assertFalse(self.c.request_home(self.frame(session='old'),'test')['accepted'])
        self.assertEqual(self.commands,[])

    def test_wireless_accepts_only_authenticated_return_requests(self):
        def receive(value):
            return dict(accepted=True,operation=value['operation'],session=value['session'])
        key='return-home-fixture-key-0000000001'
        gateway=InputGateway(role='relay',key=key)
        gateway.control_callback=receive
        listener=WirelessServer(gateway,host='127.0.0.1',port=0).start()
        try:
            port=listener.address[1]
            client=GatewayClient('127.0.0.1',port,key)
            unauthorized=GatewayClient('127.0.0.1',port,'invalid')
            try:
                self.assertEqual(client.control('return_home','test')['session'],'test')
                with self.assertRaises(ValueError):
                    unauthorized.control('return_home','test')
            finally:
                client.close()
                unauthorized.close()
        finally:listener.close()
