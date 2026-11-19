import unittest
from tg3_pico_dualarm.operator_status import motion_view,timing_summary


class OperatorStatusTests(unittest.TestCase):
    def test_factory_fault_and_latched_output_never_look_like_ordinary_disabled(self):
        runtime=dict(motion=dict(state='fault',reason='xarm_stop_unconfirmed:real_output_feedback_lead'),
            manufacturer=dict(state='fault',stop_confirmed=False))
        phase,text=motion_view(runtime,'hardware',queued=True)
        self.assertEqual(phase,'fault');self.assertIn('禁止再次启用',text)
        runtime=dict(motion=dict(state='disabled'),manufacturer=dict(state='disabled',stop_confirmed=True,
            xarm_status=dict(bounded_output=dict(fault='real_output_feedback_lead'))))
        phase,text=motion_view(runtime,'hardware')
        self.assertEqual(phase,'fault');self.assertIn('已确认释放',text)
        from tg3_pico_dualarm.readiness import readiness
        result=readiness(tracking=dict(head_tracked=True,left_tracked=True,right_tracked=True,
            left_release_required=False,right_release_required=False),native_ik=True,ros_feedback=True,
            tools_calibrated=True,headset_build=True,manufacturer=dict(runtime['manufacturer'],
            available=True,physical_estop=False,feedback_ready=True))
        self.assertFalse(result['ready_to_request_motion'])
        self.assertIn('fault_latched',result['motion_enable_reason'])

    def test_queue_and_factory_acquisition_never_display_enabled(self):
        for state in ('disabled','preparing','acquiring','settling'):
            runtime=dict(motion=dict(state=state),readiness=dict(hardware_motion_enabled=False))
            phase,text=motion_view(runtime,'hardware',queued=True)
            self.assertNotIn('输出已启用',text)
        phase,text=motion_view(dict(motion=dict(state='acquiring',initial_output_alignment=dict(samples=3))),'hardware')
        self.assertEqual(phase,'aligning')

    def test_only_confirmed_current_state_enables_following(self):
        runtime=dict(motion=dict(state='armed',hardware_motion_enabled=True),readiness=dict(hardware_motion_enabled=True))
        self.assertEqual(motion_view(runtime,'hardware')[0],'enabled')
        runtime['control']=dict(active=dict(right=True,left=False))
        self.assertIn('右臂',motion_view(runtime,'hardware')[1])
        self.assertEqual(motion_view(runtime,'hardware',stale=True)[0],'unknown')

    def test_actual_initial_output_is_not_misreported_as_never_enabled(self):
        runtime=dict(motion=dict(state='disabled',attempt_id=1,last_failure=dict(attempt_id=1,
            actual_output_count=696,reason='initial_hold_did_not_settle')),manufacturer=dict(stop_confirmed=True))
        phase,text=motion_view(runtime,'hardware',request_error='initial_hold_did_not_settle')
        self.assertEqual(phase,'failed');self.assertIn('696帧',text);self.assertIn('已接管',text)
        self.assertIn('已确认停用',text);self.assertNotIn('运动未确认启用',text)

    def test_previous_attempt_failure_not_reported_as_current_failure(self):
        runtime=dict(motion=dict(state='disabled',attempt_id=2,last_failure=dict(attempt_id=1,reason='old')))
        self.assertEqual(motion_view(runtime,'hardware')[0],'disabled')
        runtime['motion']['last_failure']=dict(attempt_id=2,reason="tracking frame is stale or from the host clock's future",input_timing=dict(total_age_s=.2079))
        phase,text=motion_view(runtime,'hardware')
        self.assertEqual(phase,'failed');self.assertIn('207.9ms',text)

    def test_transport_timing_is_not_labelled_as_mechanical_response(self):
        text=timing_summary(dict(sample_age_s=.04),dict(timing=dict(cycle_processing=dict(p95_ms=3))),dict(transfer_ms=5))
        self.assertIn('40.0ms',text);self.assertIn('5.0ms',text);self.assertIn('不等于',text)
