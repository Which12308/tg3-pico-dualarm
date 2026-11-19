"""Before-actuator alignment gate. All messages/ROS objects are test doubles."""
from types import SimpleNamespace as NS
from unittest.mock import patch
import threading
import unittest
from tg3_pico_dualarm.initial_alignment import InitialOutputAlignment
from tg3_pico_dualarm.bounded_xarm_transport import BoundedXArmTransport
from tg3_pico_dualarm.xarm_adapter import XArmRosTransport
import test_xarm_adapter as fixtures


class InitialAlignmentTests(unittest.TestCase):
    def test_old_native_target_never_becomes_ready_and_timeout_has_no_io(self):
        gate=InitialOutputAlignment((.2,)*14,1.)
        for n in range(1,40):gate.observe((0.,)*14,(.2,)*14,1.+n*.01)
        self.assertFalse(gate.ready(1.4))
        with self.assertRaisesRegex(RuntimeError,'timeout'):gate.ready(1.501)
        self.assertAlmostEqual(gate.status()['raw_error_rad'],.2)

    def test_only_fresh_continuously_matching_frames_pass(self):
        gate=InitialOutputAlignment((.2,)*14,1.)
        for n in range(1,14):gate.observe((.2,)*14,(.2,)*14,1.+n*.01)
        self.assertTrue(gate.ready(1.135))
        self.assertFalse(gate.ready(1.151))
        gate.observe((.2,)*14,(.2,)*14,1.2)
        self.assertFalse(gate.ready(1.2))
        with self.assertRaises(ValueError):gate.observe((.2,)*14,(.2,)*14,1.2)

    def test_measured_movement_is_rejected_not_used_as_new_seed(self):
        gate=InitialOutputAlignment((.2,)*14,1.)
        with self.assertRaisesRegex(RuntimeError,'measured_pose_moved'):
            gate.observe((.2,)*14,(.205,)*14,1.01)
        self.assertEqual(gate.seed,(.2,)*14)

    def transport(self):
        t=object.__new__(BoundedXArmTransport);t.output_mutex=threading.RLock()
        t.publisher=t.actual_publisher=None;t.alignment=t.last_alignment=None;t.output_fault=None
        t.actual_count=0;t.feedback_fault_details=None;t.arm_type=NS
        t.last_actual=t.last_actual_time=None;t.now=[1.];t.clock=lambda:t.now[0]
        t.state=NS(positions=(.2,)*14)
        t.adapter=NS(state='acquiring',_fresh_status=lambda:None,_feedback_ok=lambda:t.state)
        t.settings=NS(lower=(-1.,)*14,upper=(1.,)*14)
        t.cache=[];t.publish=lambda q:t.cache.append(tuple(q));t.check_publishers=lambda:None
        t.real=[];t.node=NS(create_publisher=lambda *args:(t.real.append(args) or NS(publish=lambda _:None)))
        return t

    def message(self,value=.2):
        return NS(mode=0,reserved=0,ctrl=[NS(name=i,pos=value,spd=3.14,kp=0.,kd=0.)
            for i in tuple(range(11,18))+tuple(range(21,28))])

    def test_native_raw_is_observed_without_forwarding_and_commit_is_explicit(self):
        t=self.transport()
        with patch.object(XArmRosTransport,'open_publisher',lambda obj:setattr(obj,'publisher',NS())):
            t.open_publisher()
        self.assertIsNone(t.actual_publisher);self.assertFalse(t.real)
        t.begin_initial_alignment((.2,)*14)
        for n in range(1,14):t.now[0]=1.+n*.01;t.raw(self.message())
        self.assertEqual(t.actual_count,0);self.assertFalse(t.real)
        self.assertTrue(t.poll_initial_alignment((.2,)*14))
        self.assertFalse(t.real)
        self.assertEqual(t.commit_initial_alignment(),(.2,)*14)
        self.assertEqual(len(t.real),1)
        self.assertEqual(t.real[0][1],'/arm/cmd')
        self.assertEqual(t.last_alignment['phase'],'confirmed_before_output')

    def test_invalid_or_unaligned_raw_never_opens_actual_output(self):
        t=self.transport();t.begin_initial_alignment((.2,)*14)
        for n in range(1,40):t.now[0]=1.+n*.01;t.raw(self.message(0.))
        self.assertFalse(t.real);self.assertEqual(t.actual_count,0)
        with self.assertRaises(RuntimeError):t.commit_initial_alignment()
        self.assertFalse(t.real)
        t.now[0]=1.51
        with self.assertRaisesRegex(RuntimeError,'timeout'):t.poll_initial_alignment((.2,)*14)

    def test_adapter_stays_acquiring_until_alignment_is_confirmed(self):
        f=fixtures.XArmAdapterTests();f.setUp();t=f.transport
        t.initial_alignment_required=True;t.aligned=False;t.committed=False;t.seed=None
        t.begin_initial_alignment=lambda q:setattr(t,'seed',tuple(q))
        t.poll_initial_alignment=lambda q:t.aligned
        def commit():t.committed=True;return t.seed
        t.commit_initial_alignment=commit
        f.robot.request_enable(True)
        for _ in range(30):
            f.step()
            if f.robot._stage=='initial_alignment':break
        self.assertEqual(f.robot.state,'acquiring');self.assertFalse(f.robot.hardware_writes_enabled)
        self.assertFalse(t.committed);self.assertFalse(t.published)
        f.step();self.assertFalse(f.robot.hardware_writes_enabled)
        t.aligned=True;f.step()
        self.assertEqual(f.robot.state,'armed');self.assertTrue(t.committed)

    def test_commit_rechecks_feedback_and_refuses_late_pose_change(self):
        t=self.transport();t.begin_initial_alignment((.2,)*14)
        for n in range(1,14):t.now[0]=1.+n*.01;t.raw(self.message())
        t.state.positions=(.21,)*14
        with self.assertRaisesRegex(RuntimeError,'measured_pose_moved'):t.commit_initial_alignment()
        self.assertFalse(t.real)
