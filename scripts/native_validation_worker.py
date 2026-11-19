"""Exercise the real process-isolated Placo worker with moving arm targets.

This is synthetic native acceptance in the owned Linux guest, not robot or
headset acceptance. It creates no ROS node and has no actuator interface.
"""
from __future__ import annotations

from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import sys
import time
import traceback

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'src'))

import numpy as np

from tg3_pico_dualarm.kinematics_worker import ProcessKinematics
from tg3_pico_dualarm.model import default_q
from tg3_pico_dualarm.recording import serializable


def main():
    result = dict(passed=False, started=datetime.now(timezone.utc).isoformat(),
        input='SYNTHETIC_SMALL_MOVING_TARGETS', physical_robot_tested=False,
        headset_tested=False, robot_commands_sent=False, real_native_placo=True,
        wait_budget_s=.035, wait_hard_cap_s=.040, control_tick_cap_s=.050,
        target_period_s=.020, requested_active_frames=300)
    from release_scope import public_source
    result['source_file_sha256']={name:hashlib.sha256(data).hexdigest() for name,data in public_source(ROOT)}
    source = Path('/home/tg3/project.zip')
    if source.exists():
        result['source_bundle_sha256'] = hashlib.file_digest(source.open('rb'), 'sha256').hexdigest()
    directory = ROOT/'artifacts'
    directory.mkdir(exist_ok=True)
    samples = []
    engine = None
    try:
        if sys.platform != 'linux':
            raise RuntimeError('actual Linux native acceptance required')
        engine = ProcessKinematics(warmup_iterations=12)
        result['startup'] = engine.last_diagnostics
        q = default_q()
        for label, active in (('right_only', dict(left=False, right=True)),
                              ('left_only', dict(left=True, right=False)),
                              ('both', dict(left=True, right=True))):
            engine.reset()
            baseline = q.copy()
            goals = engine.model.fk(q)
            for index, side in enumerate(('left', 'right')):
                if active[side]:
                    engine.latch_posture(side, q[index*7:(index+1)*7])
            stage_q = []
            for sequence in range(100):
                began = time.perf_counter()
                targets = {}
                for side in ('left', 'right'):
                    if active[side]:
                        target = goals[side].copy()
                        axis = 0 if side == 'left' else 1
                        target[axis, 3] += .002*math.sin(2*math.pi*sequence/100.)
                        targets[side] = target
                prior = q.copy()
                q = engine.solve(q, targets, active, .020)
                elapsed = time.perf_counter()-began
                if elapsed >= .050:
                    raise AssertionError(f'control tick exceeded 50ms: {elapsed}')
                if not np.all(np.isfinite(q)):
                    raise AssertionError('nonfinite joint target')
                if np.max(np.abs(q-prior)) > .24*.020+1e-7:
                    raise AssertionError('velocity bound exceeded')
                for index, side in enumerate(('left', 'right')):
                    if not active[side]:
                        np.testing.assert_array_equal(q[index*7:(index+1)*7],
                                                     baseline[index*7:(index+1)*7])
                diagnostic = engine.last_diagnostics
                assert diagnostic['backend'] == 'placo' and diagnostic['isolation'] == 'process'
                assert diagnostic['parent_timing']['total_s'] < .035
                samples.append(dict(stage=label, sequence=sequence, wall_s=elapsed,
                    joint_positions=q.tolist(), diagnostics=diagnostic))
                stage_q.append(q.copy())
                time.sleep(max(0., .020-(time.perf_counter()-began)))
            for index, side in enumerate(('left', 'right')):
                if active[side]:
                    movement = np.max(np.ptp(np.array(stage_q)[:, index*7:(index+1)*7], axis=0))
                    if movement <= 1e-5:
                        raise AssertionError(f'{label}:{side} did not move')
        released = q.copy()
        for _ in range(20):
            q = engine.solve(q, {}, dict(left=False, right=False), .020)
            np.testing.assert_array_equal(q, released)
        durations = [row['wall_s'] for row in samples]
        result.update(passed=True, actual_active_frames=len(samples),
            released_hold_frames=20, phases=['right_only', 'left_only', 'both'],
            inactive_arm_held=True, real_worker_pid=engine._process.pid,
            parent_pid=__import__('os').getpid(), released_targets_held=True,
            max_tick_wall_s=max(durations), p95_tick_wall_s=float(np.percentile(durations, 95)),
            max_parent_total_s=max(row['diagnostics']['parent_timing']['total_s'] for row in samples),
            max_worker_total_s=max(row['diagnostics']['worker_timing']['total_s'] for row in samples),
            max_native_solve_s=max(row['diagnostics'].get('solve_s', 0.) for row in samples))
    except Exception as exc:
        result.update(error=str(exc), traceback=traceback.format_exc(),
            completed_active_frames=len(samples), last_diagnostics=engine.last_diagnostics if engine else None)
    finally:
        if engine:
            engine.close()
            result['worker_cleanup'] = engine.cleanup_diagnostics
        result['finished'] = datetime.now(timezone.utc).isoformat()
        (directory/'native_worker_samples.jsonl').write_text(''.join(
            json.dumps(serializable(row), ensure_ascii=False)+'\n' for row in samples), encoding='utf-8')
        (directory/'native_worker_result.json').write_text(
            json.dumps(serializable(result), ensure_ascii=False, indent=2), encoding='utf-8')
    print(json.dumps(serializable(result), ensure_ascii=False, indent=2))
    return 0 if result['passed'] else 2


if __name__ == '__main__':
    raise SystemExit(main())
