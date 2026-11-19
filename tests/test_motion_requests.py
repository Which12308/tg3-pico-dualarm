"""Operator queue fixtures; requests are not manufacturer enable acknowledgements."""
import unittest
from tg3_pico_dualarm.motion_requests import MotionRequests
from tg3_pico_dualarm.operator_status import motion_view


class MotionRequestTests(unittest.TestCase):
    def test_single_pending_request_and_disable_preemption_preserve_identity(self):
        requests=MotionRequests(lambda:False)
        first=requests.enqueue(dict(operation='enable',requested_at=1.))
        self.assertTrue(first['queued']);self.assertFalse(first['hardware_motion_enabled'])
        with self.assertRaisesRegex(ValueError,'already_pending'):
            requests.enqueue(dict(operation='enable'))
        stop=requests.enqueue(dict(operation='disable',requested_at=2.))
        self.assertGreater(stop['request_number'],first['request_number'])
        action=requests.pop()
        self.assertEqual(action['operation'],'disable')
        self.assertEqual(action['request_number'],stop['request_number'])
        self.assertIsNone(requests.pop())

    def test_expired_request_can_be_displayed_without_an_admission_attempt(self):
        runtime=dict(motion=dict(state='disabled',attempt_id=0))
        phase,text=motion_view(runtime,'hardware',request_error='motion_request_expired')
        self.assertEqual(phase,'failed');self.assertIn('队列中过期',text)
        self.assertNotIn('输出已启用',text)
