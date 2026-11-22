"""Fail-closed process boundary for non-interruptible native Placo calls.

Only the spawned child imports/constructs Placo. Its startup and real solver
warmup finish before this constructor returns. The control thread never reads
or writes a Pipe: a daemon transport thread does that work, and every request
has its own event and sequence number. A timeout permanently invalidates this
instance; no old or late joint target can be returned by a later request.

This bounds the parent's *wait*, not native execution time or OS scheduling.
It is not a hard real-time guarantee. The worker has no ROS or actuator access.
"""
from __future__ import annotations

from . import safety as motion_policy
from dataclasses import dataclass, field
import multiprocessing as mp
import queue
import threading
import time

import numpy as np

from .kinematics import KinematicsError, checked_pose
from .model import DualArmModel, SIDES, default_q, finite_vector

MAX_WAIT_S = .040
DEFAULT_WAIT_S = .035


def _native_factory(options):
    from .kinematics import Kinematics
    motion_policy.LIMITS_MANAGED_EXTERNALLY=options.pop('_external_limits',True)
    return Kinematics(**options)


def _worker_main(connection, options, warmup_iterations, solver_factory):
    """Child entry point; test factories exercise transport, never native IK."""
    started = time.perf_counter()
    try:
        setup_started = time.perf_counter()
        engine = solver_factory(options)
        setup_s = time.perf_counter()-setup_started
        q = np.clip(default_q(), engine.model.limits[:, 0], engine.model.limits[:, 1])
        goals = engine.model.fk(q)
        warmup_times = []
        # Startup may take longer than a control tick. Never run this warmup
        # after starting the command watchdog or admitting hardware motion.
        original_budget = engine.solve_budget_s
        engine.solve_budget_s = 1.
        patterns = ({'left': True, 'right': False},
                    {'left': False, 'right': True}, {'left': True, 'right': True})
        for index in range(warmup_iterations):
            tick_started = time.perf_counter()
            engine.reset()
            for i, side in enumerate(SIDES):
                engine.latch_posture(side, q[i*7:(i+1)*7])
            engine.solve(q, goals, patterns[index % len(patterns)], .02)
            warmup_times.append(time.perf_counter()-tick_started)
        engine.solve_budget_s = original_budget
        engine.reset()
        connection.send(dict(kind='ready', diagnostics=engine.last_diagnostics,
            startup=dict(setup_s=setup_s, warmup_s=sum(warmup_times),
                         warmup_iterations=warmup_iterations,
                         warmup_tick_s=warmup_times, total_s=time.perf_counter()-started)))
        while True:
            request = connection.recv()
            request_started = time.perf_counter()
            request_id = request['id']
            try:
                connection.send(dict(kind='phase', id=request_id, phase='prepare', elapsed_s=0.))
                for side in request['resets']:
                    engine.reset(side)
                for side, anchor in request['anchors'].items():
                    if anchor is not None:
                        engine.latch_posture(side, anchor)
                prepared_s = time.perf_counter()-request_started
                solve_started = time.perf_counter()
                connection.send(dict(kind='phase', id=request_id, phase='solve', elapsed_s=prepared_s))
                proposed = engine.solve(request['seed'], request['targets'], request['active'], request['dt'])
                response = dict(kind='result', id=request_id, proposed=proposed,
                    diagnostics=engine.last_diagnostics,
                    worker_timing=dict(prepare_s=prepared_s,
                        solve_total_s=time.perf_counter()-solve_started,
                        total_s=time.perf_counter()-request_started))
            except Exception as exc:
                response = dict(kind='error', id=request_id,
                    reason=getattr(exc, 'reason', 'ik_worker_solve_failed: '+str(exc)),
                    diagnostics=engine.last_diagnostics,
                    worker_timing=dict(total_s=time.perf_counter()-request_started))
            connection.send(response)
    except (EOFError, BrokenPipeError, OSError):
        pass
    except BaseException as exc:
        try:
            connection.send(dict(kind='startup_error',
                reason=getattr(exc, 'reason', 'ik_worker_startup_failed: '+str(exc))))
        except (EOFError, BrokenPipeError, OSError):
            pass
    finally:
        connection.close()


@dataclass
class _Exchange:
    request: dict
    done: threading.Event = field(default_factory=threading.Event)
    response: object = None
    error: str | None = None
    progress: dict = field(default_factory=dict)


class ProcessKinematics:
    """Kinematics-compatible, terminally fail-closed native worker.

    ``wait_budget_s`` covers validation, queueing, transport and waiting for a
    complete response, and cannot exceed 40 ms. ``close`` also waits at most
    40 ms; cleanup continues on a prestarted reaper thread. Construct before
    starting the watchdog. Recreate the station to recover a faulted worker.

    The private factory/entry-point arguments are explicit transport test seams,
    not a runtime fake-IK fallback. Production always uses native Placo.
    """
    def __init__(self, modelroot=None, mode='pose', velocity_limit=.24,
                 posture_weight=1e-4, solve_budget_s=.05,
                 position_weight=1., orientation_weight=1.,
                 torso_waist_locked=True, joint_limits=None, *,
                 wait_budget_s=DEFAULT_WAIT_S, startup_timeout_s=30.,
                 warmup_iterations=3, _solver_factory=None, _worker_target=None):
        if not np.isfinite(wait_budget_s) or not 0 < wait_budget_s <= MAX_WAIT_S:
            raise ValueError('IK worker wait budget must be in (0, .040] seconds')
        if not np.isfinite(startup_timeout_s) or not 0 < startup_timeout_s <= 120.:
            raise ValueError('invalid IK worker startup timeout')
        if not isinstance(warmup_iterations, int) or not 3 <= warmup_iterations <= 100:
            raise ValueError('IK worker requires 3 to 100 native warmup iterations')
        if torso_waist_locked is not True:
            raise KinematicsError('unlocked_torso_waist_not_supported')
        self.model = DualArmModel(modelroot)
        if joint_limits is not None:
            self.model.restrict_joint_limits(*joint_limits)
        self.wait_budget_s = None if motion_policy.LIMITS_MANAGED_EXTERNALLY else float(wait_budget_s)
        self._anchors = {side: None for side in SIDES}
        self._resets = set(SIDES)
        self._sequence = 0
        self._fault = None
        self._request_lock = threading.Lock()
        self._requests = queue.Queue(maxsize=1)
        self._stop = threading.Event()
        self._cleanup_requested = threading.Event()
        self._cleanup_done = threading.Event()
        self._startup = _Exchange({})
        self.cleanup_diagnostics = {}
        self.last_diagnostics = dict(backend='placo', isolation='process', initialized=False)
        options = dict(modelroot=modelroot, mode=mode, velocity_limit=velocity_limit,
            posture_weight=posture_weight, solve_budget_s=solve_budget_s,
            position_weight=position_weight, orientation_weight=orientation_weight,
            torso_waist_locked=torso_waist_locked, joint_limits=joint_limits,
            _external_limits=motion_policy.LIMITS_MANAGED_EXTERNALLY)
        context = mp.get_context('spawn')
        self._connection, child_connection = context.Pipe(duplex=True)
        self._process = context.Process(target=_worker_target or _worker_main,
            args=(child_connection, options, warmup_iterations, _solver_factory or _native_factory),
            name='tg3-native-ik', daemon=True)
        started = time.perf_counter()
        try:
            self._process.start()
        except BaseException:
            child_connection.close()
            self._connection.close()
            raise
        child_connection.close()
        self._reaper_thread = threading.Thread(target=self._reap, name='tg3-ik-reaper', daemon=True)
        self._transport_thread = threading.Thread(target=self._transport, name='tg3-ik-transport', daemon=True)
        self._reaper_thread.start()
        self._transport_thread.start()
        if not self._startup.done.wait(max(0., startup_timeout_s-(time.perf_counter()-started))):
            self._invalidate('ik_worker_startup_timeout')
            raise KinematicsError(self._fault)
        response = self._startup.response
        if self._startup.error or not isinstance(response, dict) or response.get('kind') != 'ready':
            reason = (response.get('reason') if isinstance(response, dict) else None)
            self._invalidate(reason or self._startup.error or 'ik_worker_invalid_startup_response')
            raise KinematicsError(self._fault)
        self.last_diagnostics = dict(response.get('diagnostics', {}), backend='placo',
            isolation='process', initialized=True, worker_pid=self._process.pid,
            startup=response.get('startup', {}), startup_parent_s=time.perf_counter()-started)

    def _transport(self):
        exchange = self._startup
        try:
            exchange.response = self._connection.recv()
            exchange.done.set()
            while not self._stop.is_set():
                try:
                    exchange = self._requests.get(timeout=.01)
                except queue.Empty:
                    continue
                if self._stop.is_set():
                    break
                self._connection.send(exchange.request)
                while True:
                    response = self._connection.recv()
                    if (isinstance(response, dict) and response.get('kind') == 'phase'
                            and response.get('id') == exchange.request['id']):
                        exchange.progress = dict(response, received_at_parent=time.perf_counter())
                        continue
                    exchange.response = response
                    break
                exchange.done.set()
        except Exception as exc:
            exchange.error = 'ik_worker_transport_failed: '+str(exc)
            exchange.done.set()

    def _reap(self):
        self._cleanup_requested.wait()
        started = time.perf_counter()
        try:
            if self._process.is_alive():
                self._process.terminate()
                self._process.join(timeout=.05)
            if self._process.is_alive():
                self._process.kill()
                self._process.join(timeout=.05)
            self.cleanup_diagnostics = dict(elapsed_s=time.perf_counter()-started,
                alive=self._process.is_alive(), exitcode=self._process.exitcode)
        except Exception as exc:
            self.cleanup_diagnostics = dict(elapsed_s=time.perf_counter()-started, error=str(exc))
        finally:
            # Closing a Pipe while another thread is blocked in I/O is kept
            # off the control thread, too.
            self._connection.close()
            self._cleanup_done.set()

    def _invalidate(self, reason):
        self._fault = self._fault or reason
        self._stop.set()
        self._cleanup_requested.set()

    @property
    def faulted(self):
        """Terminal failure/closure latch; recovery requires a new instance."""
        return self._fault is not None

    @property
    def fault_reason(self):
        return self._fault

    def latch_posture(self, side, q7):
        if side not in SIDES:
            raise ValueError('side must be left or right')
        value = finite_vector(q7, 7)
        limits = self.model.arms[side].limits
        if not motion_policy.LIMITS_MANAGED_EXTERNALLY and (np.any(value < limits[:, 0]) or np.any(value > limits[:, 1])):
            raise KinematicsError('posture_reference_outside_soft_limits')
        self._anchors[side] = value.copy()

    def reset(self, side=None):
        sides = SIDES if side is None else (side,)
        for item in sides:
            if item not in SIDES:
                raise ValueError('side must be left or right')
            self._anchors[item] = None
            self._resets.add(item)

    def solve(self, measured14, targets, active, dt):
        started = time.perf_counter()
        deadline = float('inf') if self.wait_budget_s is None else started+self.wait_budget_s
        if self._fault:
            raise KinematicsError('ik_worker_unavailable: '+self._fault)
        if not self._request_lock.acquire(blocking=False):
            raise KinematicsError('ik_worker_concurrent_request')
        request_id = None
        exchange = None
        timing = {}
        try:
            try:
                q = finite_vector(measured14, 14).copy()
            except (TypeError, ValueError) as exc:
                raise KinematicsError('invalid_seed') from exc
            if not np.isfinite(dt) or dt<=0 or (not motion_policy.LIMITS_MANAGED_EXTERNALLY and dt>.05):
                raise KinematicsError('invalid_tick_interval')
            if set(active) != set(SIDES) or any(not isinstance(active[s], (bool, np.bool_)) for s in SIDES):
                raise KinematicsError('invalid_active_arms')
            if set(targets)-set(SIDES):
                raise KinematicsError('unknown_target_arm')
            if not motion_policy.LIMITS_MANAGED_EXTERNALLY and (np.any(q < self.model.limits[:, 0]) or np.any(q > self.model.limits[:, 1])):
                raise KinematicsError('seed_outside_soft_limits')
            goals = {side: checked_pose(targets[side]) for side in SIDES if active[side] and side in targets}
            if any(active[side] and side not in goals for side in SIDES):
                raise KinematicsError('active_arm_missing_target')
            if not any(active.values()):
                self.reset()
                self.last_diagnostics = dict(backend='placo', isolation='process',
                    solve_called=False, reason='both_released', parent_total_s=time.perf_counter()-started)
                return q
            self._sequence += 1
            request_id = self._sequence
            exchange = _Exchange(dict(id=request_id, seed=q, targets=goals,
                active=dict(active), dt=float(dt), resets=tuple(self._resets),
                anchors={s: None if a is None else a.copy() for s, a in self._anchors.items()}))
            timing['prepare_s'] = time.perf_counter()-started
            if time.perf_counter() >= deadline:
                raise KinematicsError('ik_worker_deadline_exceeded')
            self._requests.put_nowait(exchange)
            wait_started = time.perf_counter()
            received = exchange.done.wait(None if self.wait_budget_s is None else max(0., deadline-time.perf_counter()))
            timing['wait_s'] = time.perf_counter()-wait_started
            if not received or time.perf_counter() >= deadline:
                raise KinematicsError('ik_worker_deadline_exceeded')
            if exchange.error:
                raise KinematicsError(exchange.error)
            response = exchange.response
            if not isinstance(response, dict) or response.get('id') != request_id:
                raise KinematicsError('ik_worker_response_id_mismatch')
            if response.get('kind') == 'error':
                self.last_diagnostics = dict(response.get('diagnostics', {}))
                timing['worker'] = response.get('worker_timing', {})
                raise KinematicsError(response.get('reason', 'ik_worker_solve_failed'))
            if response.get('kind') != 'result':
                raise KinematicsError('ik_worker_invalid_response')
            proposed = finite_vector(response.get('proposed'), 14).copy()
            if time.perf_counter() >= deadline:
                raise KinematicsError('ik_worker_deadline_exceeded')
            self._resets.clear()
            self.last_diagnostics = dict(response.get('diagnostics', {}),
                backend='placo', isolation='process', worker_pid=self._process.pid,
                request_id=request_id, worker_timing=response.get('worker_timing', {}),
                parent_timing=dict(timing, total_s=time.perf_counter()-started),
                wait_budget_s=self.wait_budget_s)
            return proposed
        except (KinematicsError, ValueError, TypeError, queue.Full) as exc:
            reason = getattr(exc, 'reason', 'ik_worker_invalid_result: '+str(exc))
            # Invalid caller input is rejected locally. Once dispatched, any
            # failure quarantines the process and its changed solver state.
            if request_id is not None and not motion_policy.LIMITS_MANAGED_EXTERNALLY:
                self._invalidate(reason)
                if exchange is not None and exchange.progress:
                    progress = dict(exchange.progress)
                    progress['parent_time_since_phase_s'] = max(0.,
                        time.perf_counter()-progress.pop('received_at_parent'))
                    timing['worker_progress'] = progress
                self.last_diagnostics = dict(self.last_diagnostics, backend='placo',
                    isolation='process', request_id=request_id, reason=reason,
                    worker_faulted=True, parent_timing=dict(timing, total_s=time.perf_counter()-started),
                    wait_budget_s=self.wait_budget_s)
            if isinstance(exc, KinematicsError):
                raise
            raise KinematicsError(reason) from exc
        finally:
            self._request_lock.release()

    def close(self, timeout_s=MAX_WAIT_S):
        """Invalidate immediately; wait at most 40 ms for asynchronous cleanup."""
        if not np.isfinite(timeout_s) or not 0 <= timeout_s <= MAX_WAIT_S:
            raise ValueError('IK worker close wait must be in [0, .040] seconds')
        self._invalidate('ik_worker_closed')
        self._cleanup_done.wait(float(timeout_s))

    def __enter__(self):
        return self

    def __exit__(self, *_exc):
        self.close()
