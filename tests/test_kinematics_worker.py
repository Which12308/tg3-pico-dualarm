"""Process-boundary tests with explicit doubles; these do not validate Placo IK."""
import importlib.util
import multiprocessing.process
import os
import threading
import time
import unittest
from functools import partial
from unittest.mock import patch

import numpy as np

from tg3_pico_dualarm.kinematics import KinematicsError
from tg3_pico_dualarm.kinematics_worker import ProcessKinematics, MAX_WAIT_S
from tg3_pico_dualarm.model import DualArmModel, default_q


class BoundarySolverDouble:
    """Deliberate transport fixture, never selected by production startup."""
    def __init__(self, options, behavior='success'):
        self.model = DualArmModel(options['modelroot'])
        self.solve_budget_s = options['solve_budget_s']
        self.behavior = behavior
        self.calls = 0
        self.anchors = dict(left=None, right=None)
        self.last_diagnostics = dict(test_double=True)

    def reset(self, side=None):
        for item in ('left', 'right') if side is None else (side,):
            self.anchors[item] = None

    def latch_posture(self, side, values):
        self.anchors[side] = np.array(values, copy=True)

    def solve(self, q, targets, active, dt):
        self.calls += 1
        if self.behavior == 'warmup_failure':
            raise KinematicsError('injected_warmup_failure')
        # Three startup calls must complete before request fault injection.
        if self.calls > 3:
            if self.behavior == 'delay':
                time.sleep(.25)
            elif self.behavior == 'hang':
                while True:
                    time.sleep(1.)
            elif self.behavior == 'failure':
                raise KinematicsError('injected_native_failure')
            elif self.behavior == 'invalid':
                return np.full(14, np.nan)
        self.last_diagnostics = dict(test_double=True, child_calls=self.calls,
            anchors=self.anchors.copy(), active=dict(active))
        return np.array(q, copy=True)


def _never_ready(connection, options, warmup_iterations, solver_factory):
    while True:
        time.sleep(1.)


def _duplicate_reply(connection, options, warmup_iterations, solver_factory):
    """Reply for request one twice: request two must reject replayed joints."""
    connection.send(dict(kind='ready', diagnostics=dict(test_double=True), startup={}))
    request = connection.recv()
    response = dict(kind='result', id=request['id'], proposed=request['seed'], diagnostics={})
    connection.send(response)
    connection.send(response)
    while True:
        connection.recv()


def _wrong_id(connection, options, warmup_iterations, solver_factory):
    connection.send(dict(kind='ready', diagnostics=dict(test_double=True), startup={}))
    request = connection.recv()
    connection.send(dict(kind='result', id=request['id']+1, proposed=request['seed'], diagnostics={}))
    while True:
        time.sleep(1.)


def _exits_on_solve(connection, options, warmup_iterations, solver_factory):
    connection.send(dict(kind='ready', diagnostics=dict(test_double=True), startup={}))
    connection.recv()
    os._exit(23)


class ProcessBoundaryTests(unittest.TestCase):
    """These tests certify isolation behavior, not physical IK correctness."""
    def make_engine(self, behavior='success', **options):
        engine = ProcessKinematics(_solver_factory=partial(BoundarySolverDouble, behavior=behavior),
            startup_timeout_s=10., **options)
        self.addCleanup(engine.close)
        return engine

    def solve(self, engine, q=None):
        q = default_q() if q is None else q
        return engine.solve(q, engine.model.fk(q), {'left': True, 'right': True}, .02)

    def test_spawn_startup_finishes_three_warmups_before_return(self):
        engine = self.make_engine()
        report = engine.last_diagnostics
        self.assertNotEqual(report['worker_pid'], os.getpid())
        self.assertFalse(engine.faulted)
        self.assertIsNone(engine.fault_reason)
        self.assertEqual(report['startup']['warmup_iterations'], 3)
        self.assertEqual(len(report['startup']['warmup_tick_s']), 3)
        self.assertEqual(report['child_calls'], 3)
        q = default_q()
        np.testing.assert_array_equal(self.solve(engine, q), q)
        self.assertEqual(engine.last_diagnostics['request_id'], 1)
        self.assertEqual(engine.last_diagnostics['child_calls'], 4)
        self.assertIn('solve_total_s', engine.last_diagnostics['worker_timing'])
        self.assertIn('wait_s', engine.last_diagnostics['parent_timing'])

    def test_wait_budget_cannot_be_relaxed_above_40ms(self):
        for bad in (0., -.1, MAX_WAIT_S+.001, np.nan, np.inf):
            with self.assertRaises(ValueError):
                ProcessKinematics(wait_budget_s=bad)

    def test_slow_native_call_times_out_and_worker_is_terminally_quarantined(self):
        engine = self.make_engine('delay', wait_budget_s=.015)
        started = time.perf_counter()
        with self.assertRaisesRegex(KinematicsError, 'ik_worker_deadline_exceeded'):
            self.solve(engine)
        elapsed = time.perf_counter()-started
        self.assertLess(elapsed, .040)
        report = engine.last_diagnostics
        self.assertTrue(report['worker_faulted'])
        self.assertTrue(engine.faulted)
        self.assertEqual(engine.fault_reason, 'ik_worker_deadline_exceeded')
        self.assertEqual(report['request_id'], 1)
        self.assertLess(report['parent_timing']['wait_s'], .040)
        self.assertEqual(report['parent_timing']['worker_progress']['phase'], 'solve')
        # Even after the old result could have arrived, there is no retry or
        # cached target. Cleanup may have killed it before it could send.
        self.assertTrue(engine._cleanup_done.wait(1.))
        self.assertFalse(engine._process.is_alive())
        time.sleep(.26)
        started = time.perf_counter()
        with self.assertRaisesRegex(KinematicsError, 'ik_worker_unavailable'):
            self.solve(engine, default_q()+.0001)
        self.assertLess(time.perf_counter()-started, .020)
        self.assertEqual(engine._sequence, 1)

    def test_hung_worker_close_has_bounded_parent_wait_and_is_reaped(self):
        engine = self.make_engine('hang', wait_budget_s=.015)
        with self.assertRaisesRegex(KinematicsError, 'deadline_exceeded'):
            self.solve(engine)
        started = time.perf_counter()
        engine.close(timeout_s=.015)
        self.assertLess(time.perf_counter()-started, .040)
        self.assertTrue(engine._cleanup_done.wait(1.))
        self.assertFalse(engine._process.is_alive())
        self.assertFalse(engine.cleanup_diagnostics['alive'])
        engine.close(timeout_s=0.)
        self.assertTrue(engine.faulted)
        self.assertEqual(engine.fault_reason, 'ik_worker_deadline_exceeded')
        with self.assertRaisesRegex(KinematicsError, 'unavailable'):
            self.solve(engine)

    def test_actual_late_reply_cannot_be_consumed_by_subsequent_call(self):
        engine = self.make_engine('delay', wait_budget_s=.015)
        # Deliberately delay termination in the cleanup thread so this test
        # observes an actual late reply, instead of merely assuming one could
        # have arrived. Neither the worker solver nor production policy changes.
        release_cleanup = threading.Event()
        real_terminate = engine._process.terminate
        def delayed_terminate():
            release_cleanup.wait(1.)
            real_terminate()
        try:
            with patch.object(engine._process, 'terminate', side_effect=delayed_terminate):
                with patch.object(engine._requests, 'put_nowait', wraps=engine._requests.put_nowait) as dispatch:
                    with self.assertRaisesRegex(KinematicsError, 'deadline_exceeded'):
                        self.solve(engine)
                exchange = dispatch.call_args.args[0]
                self.assertTrue(exchange.done.wait(.7), 'test worker did not deliver its late reply')
                self.assertEqual(exchange.response['id'], 1)
                np.testing.assert_array_equal(exchange.response['proposed'], default_q())
                with self.assertRaisesRegex(KinematicsError, 'unavailable'):
                    self.solve(engine, default_q()+.0001)
                self.assertEqual(engine._sequence, 1)
                release_cleanup.set()
        finally:
            release_cleanup.set()

    def test_old_duplicate_reply_is_not_accepted_for_next_seed(self):
        engine = self.make_engine(_worker_target=_duplicate_reply)
        q = default_q()
        np.testing.assert_array_equal(self.solve(engine, q), q)
        with self.assertRaisesRegex(KinematicsError, 'response_id_mismatch'):
            self.solve(engine, q+.0001)
        with self.assertRaisesRegex(KinematicsError, 'unavailable'):
            self.solve(engine)

    def test_reply_for_wrong_request_is_rejected(self):
        engine = self.make_engine(_worker_target=_wrong_id)
        with self.assertRaisesRegex(KinematicsError, 'response_id_mismatch'):
            self.solve(engine)

    def test_child_exit_fails_closed_without_waiting_for_watchdog(self):
        engine = self.make_engine(_worker_target=_exits_on_solve)
        started = time.perf_counter()
        with self.assertRaisesRegex(KinematicsError, 'transport_failed|deadline_exceeded'):
            self.solve(engine)
        self.assertLess(time.perf_counter()-started, .050)
        self.assertTrue(engine.faulted)

    def test_close_healthy_worker_latches_terminal_fault(self):
        engine = self.make_engine()
        engine.close(timeout_s=.015)
        self.assertTrue(engine.faulted)
        self.assertEqual(engine.fault_reason, 'ik_worker_closed')
        with self.assertRaisesRegex(KinematicsError, 'unavailable'):
            self.solve(engine)

    def test_solver_error_is_reported_and_never_returns_previous_solution(self):
        engine = self.make_engine('failure')
        with self.assertRaisesRegex(KinematicsError, 'injected_native_failure'):
            self.solve(engine)
        self.assertTrue(engine.last_diagnostics['worker_faulted'])
        with self.assertRaisesRegex(KinematicsError, 'unavailable'):
            self.solve(engine)

    def test_invalid_result_is_rejected_and_quarantined(self):
        engine = self.make_engine('invalid')
        with self.assertRaisesRegex(KinematicsError, 'invalid_result'):
            self.solve(engine)
        self.assertTrue(engine.last_diagnostics['worker_faulted'])

    def test_posture_and_reset_are_local_until_next_solve_and_keep_order(self):
        engine = self.make_engine()
        q = default_q()
        anchor = q[:7].copy()
        engine.latch_posture('left', anchor)
        anchor[0] += .1
        self.solve(engine)
        np.testing.assert_array_equal(engine.last_diagnostics['anchors']['left'], q[:7])
        engine.reset('left')
        self.solve(engine)
        self.assertIsNone(engine.last_diagnostics['anchors']['left'])
        engine.reset('right')
        engine.latch_posture('right', q[7:])
        self.solve(engine)
        np.testing.assert_array_equal(engine.last_diagnostics['anchors']['right'], q[7:])

    def test_released_arms_do_not_dispatch_a_solver_request(self):
        engine = self.make_engine()
        q = default_q()
        engine.latch_posture('left', q[:7])
        result = engine.solve(q, {}, {'left': False, 'right': False}, .02)
        np.testing.assert_array_equal(result, q)
        self.assertIsNot(result, q)
        self.assertEqual(engine._sequence, 0)
        self.assertFalse(engine.last_diagnostics['solve_called'])
        self.solve(engine)
        self.assertIsNone(engine.last_diagnostics['anchors']['left'])

    def test_invalid_local_input_does_not_fault_a_healthy_worker(self):
        engine = self.make_engine()
        with self.assertRaisesRegex(KinematicsError, 'invalid_seed'):
            engine.solve(np.full(14, np.nan), {}, {'left': True, 'right': True}, .02)
        with self.assertRaisesRegex(KinematicsError, 'missing_target'):
            engine.solve(default_q(), {}, {'left': True, 'right': False}, .02)
        self.assertIsNone(engine._fault)
        self.assertEqual(engine._sequence, 0)
        self.solve(engine)

    def test_startup_timeout_fails_before_runtime_admission(self):
        # Windows CreateProcess / URDF loading are OS work, not an
        # interruptible native-worker wait. Measure the bounded wait after
        # spawn returns, and verify quarantine and actual child cleanup.
        after_spawn=[];invalidated=[]
        real_start=multiprocessing.process.BaseProcess.start
        real_invalidate=ProcessKinematics._invalidate
        def start(process):
            real_start(process)
            after_spawn.append(time.perf_counter())
        def invalidate(engine,reason):
            invalidated.append(engine)
            real_invalidate(engine,reason)
        with patch.object(multiprocessing.process.BaseProcess,'start',new=start), \
                patch.object(ProcessKinematics,'_invalidate',new=invalidate):
            with self.assertRaisesRegex(KinematicsError, 'startup_timeout'):
                ProcessKinematics(startup_timeout_s=.05, _worker_target=_never_ready)
        self.assertEqual(len(after_spawn),1)
        self.assertLess(time.perf_counter()-after_spawn[0], .3)
        self.assertEqual(len(invalidated),1)
        engine=invalidated[0]
        self.assertTrue(engine.faulted)
        self.assertEqual(engine.fault_reason,'ik_worker_startup_timeout')
        self.assertTrue(engine._cleanup_done.wait(1.))
        self.assertFalse(engine._process.is_alive())

    def test_warmup_failure_prevents_ready_and_runtime_admission(self):
        with self.assertRaisesRegex(KinematicsError, 'injected_warmup_failure'):
            ProcessKinematics(startup_timeout_s=10.,
                _solver_factory=partial(BoundarySolverDouble, behavior='warmup_failure'))

    @unittest.skipIf(importlib.util.find_spec('placo') is not None, 'requires an environment without native Placo')
    def test_missing_native_placo_fails_startup_without_any_fallback(self):
        with self.assertRaisesRegex(KinematicsError, 'native_placo_unavailable'):
            ProcessKinematics(startup_timeout_s=10.)


@unittest.skipUnless(importlib.util.find_spec('placo') is not None,
    'Real native Placo unavailable; process doubles do not replace native acceptance')
class NativeProcessAcceptanceTests(unittest.TestCase):
    def test_native_worker_warms_up_then_solves_without_actuators(self):
        # This test must run in the actual Linux/Placo environment as a separate
        # acceptance check. OS timing overruns remain possible and fail closed.
        with ProcessKinematics(warmup_iterations=12) as engine:
            q = default_q()
            targets = engine.model.fk(q)
            for index, side in enumerate(('left', 'right')):
                engine.latch_posture(side, q[index*7:(index+1)*7])
            proposed = engine.solve(q, targets, {'left': True, 'right': True}, .02)
            np.testing.assert_allclose(proposed, q, atol=1e-6)
            self.assertEqual(engine.last_diagnostics['backend'], 'placo')
            self.assertEqual(engine.last_diagnostics['isolation'], 'process')


if __name__ == '__main__':
    unittest.main()
