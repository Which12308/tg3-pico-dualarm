"""Run portable tests and explicitly report native solver skips."""
import json
import hashlib
from pathlib import Path
import sys
import unittest

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src'))
def main():
    # Spawned IK children import __main__; never discover tests in a child.
    suite=unittest.defaultTestLoader.discover(str(ROOT/'tests'))
    result=unittest.TextTestRunner(verbosity=2).run(suite)
    summary=dict(tests_run=result.testsRun,passed=result.testsRun-len(result.skipped)-len(result.errors)-len(result.failures),
                 skipped=[dict(test=str(test),reason=reason) for test,reason in result.skipped],
                 failures=len(result.failures),errors=len(result.errors),hardware_commands_sent=False)
    from release_scope import public_source
    summary['source_file_sha256']={name:hashlib.sha256(data).hexdigest() for name,data in public_source(ROOT)}
    output=ROOT/'artifacts'/'test_result.json';output.parent.mkdir(exist_ok=True)
    output.write_text(json.dumps(summary,ensure_ascii=False,indent=2),encoding='utf-8')
    print(json.dumps({k:v for k,v in summary.items() if k!='source_file_sha256'},ensure_ascii=False,indent=2))
    return not result.wasSuccessful()


if __name__=='__main__':raise SystemExit(main())
