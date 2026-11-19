"""Control-session heartbeat scope; no robot or wall-clock timing fixture."""
import unittest
from tg3_pico_dualarm.watchdog import CommandWatchdog


class WatchdogSessionTests(unittest.TestCase):
    def setUp(self):
        self.now=0.;self.expirations=[]
        self.guard=CommandWatchdog(lambda:self.expirations.append(self.now),
            monotonic=lambda:self.now,active=False)

    def test_idle_wait_then_explicit_acquisition_has_the_same_150ms_deadline(self):
        self.now=300.
        self.assertFalse(self.guard.check());self.assertEqual(self.expirations,[])
        self.guard.activate();self.now+=.149
        self.assertFalse(self.guard.check())
        self.now+=.002
        self.assertTrue(self.guard.check());self.assertEqual(len(self.expirations),1)

    def test_duplicate_enable_cannot_renew_an_active_lease(self):
        self.guard.activate();self.now=.14;self.guard.activate();self.now=.151
        self.assertTrue(self.guard.check())

    def test_deadline_crossed_before_deactivation_is_terminal(self):
        self.guard.activate();self.now=.151
        self.assertFalse(self.guard.deactivate());self.guard.beat()
        self.assertTrue(self.guard.check());self.assertEqual(len(self.expirations),1)
        with self.assertRaisesRegex(RuntimeError,'requires_new_station'):self.guard.activate()

    def test_confirmed_disabled_interval_can_wait_until_next_explicit_enable(self):
        self.guard.activate();self.now=.04;self.guard.beat();self.now=.05
        self.assertTrue(self.guard.deactivate());self.now=30.
        self.assertFalse(self.guard.check());self.guard.activate();self.now+=.151
        self.assertTrue(self.guard.check())

    def test_existing_default_behavior_is_active(self):
        guard=CommandWatchdog(lambda:self.expirations.append(self.now),monotonic=lambda:self.now)
        self.now=.151
        self.assertTrue(guard.check());self.assertEqual(len(self.expirations),1)


if __name__=='__main__':unittest.main()
