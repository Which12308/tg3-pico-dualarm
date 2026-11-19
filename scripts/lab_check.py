"""One command for real native offline validation and optional read-only ROS."""
import argparse
from datetime import datetime,timezone
import json
from pathlib import Path
import subprocess
import sys
import uuid

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src'))
from tg3_pico_dualarm import __version__
from tg3_pico_dualarm.cli import demo
from tg3_pico_dualarm.recording import serializable


def main(argv=None):
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--ros',action='store_true',help='Read existing robot feedback and graph; never publish motion')
    args=p.parse_args(argv)
    directory=ROOT/'artifacts'/('lab_'+datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')+'_'+uuid.uuid4().hex[:6])
    directory.mkdir(parents=True,exist_ok=False)
    report=dict(version=__version__,hardware_motion_enabled=False,robot_commands_sent=False,
                native_placo_passed=False,portable_and_native_tests_passed=False,
                ros_observation_requested=args.ros,ros_native_build_executed=False)
    status=1
    try:
        report['native_demo']=demo(record=directory/'recordings')
        report['native_placo_passed']=True
        with (directory/'tests.log').open('w',encoding='utf-8') as log:
            result=subprocess.run([sys.executable,'-s','-B',str(ROOT/'scripts/run_tests.py')],cwd=ROOT,
                                  stdout=log,stderr=subprocess.STDOUT)
        tests=json.loads((ROOT/'artifacts/test_result.json').read_text(encoding='utf-8'))
        report['tests']=tests
        if result.returncode or tests['failures'] or tests['errors']:
            raise RuntimeError('regression_tests_failed_see_tests_log')
        if any('test_kinematics.NativePlacoTests' in item['test'] for item in tests['skipped']):
            raise RuntimeError('native_placo_tests_were_skipped')
        report['portable_and_native_tests_passed']=True
        if args.ros:
            from tg3_pico_dualarm.preflight import run
            report['ros_observation']=run(10.)
        status=0
    except (RuntimeError,ValueError,OSError,ImportError) as exc:
        report['error']=str(exc)
    finally:
        path=directory/'report.json'
        path.write_text(json.dumps(serializable(report),ensure_ascii=False,indent=2),encoding='utf-8')
        print(json.dumps(dict(report=str(path),native_placo_passed=report['native_placo_passed'],
            tests_passed=report['portable_and_native_tests_passed'],robot_commands_sent=False,
            error=report.get('error')),ensure_ascii=False,indent=2))
    return status


if __name__=='__main__':raise SystemExit(main())
