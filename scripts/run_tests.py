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
    from tg3_pico_dualarm import safety
    def cases(suite):
        for item in suite:
            if isinstance(item,unittest.TestSuite):yield from cases(item)
            else:yield item
    all_cases=list(cases(unittest.defaultTestLoader.discover(str(ROOT/'tests'))))
    external=[test for test in all_cases if test.__class__.__module__=='test_external_limits']
    legacy=[test for test in all_cases if test.__class__.__module__!='test_external_limits']
    profiles={}
    results=[]
    original_policy=safety.LIMITS_MANAGED_EXTERNALLY
    try:
        for name,policy,tests in (('legacy_local_guards',False,legacy),('original_chain_external_limits',True,external)):
            safety.LIMITS_MANAGED_EXTERNALLY=policy
            print('\nTEST PROFILE: '+name,flush=True)
            result=unittest.TextTestRunner(verbosity=2).run(unittest.TestSuite(tests))
            results.append(result)
            profiles[name]=dict(tests_run=result.testsRun,
                passed=result.testsRun-len(result.skipped)-len(result.errors)-len(result.failures),
                skipped=len(result.skipped),failures=len(result.failures),errors=len(result.errors))
    finally:safety.LIMITS_MANAGED_EXTERNALLY=original_policy
    summary=dict(tests_run=sum(r.testsRun for r in results),
                 passed=sum(r.testsRun-len(r.skipped)-len(r.errors)-len(r.failures) for r in results),
                 skipped=[dict(test=str(test),reason=reason) for r in results for test,reason in r.skipped],
                 failures=sum(len(r.failures) for r in results),errors=sum(len(r.errors) for r in results),
                 profiles=profiles,hardware_commands_sent=False)
    from release_scope import public_source
    summary['source_file_sha256']={name:hashlib.sha256(data).hexdigest() for name,data in public_source(ROOT)}
    output=ROOT/'artifacts'/'test_result.json';output.parent.mkdir(exist_ok=True)
    output.write_text(json.dumps(summary,ensure_ascii=False,indent=2),encoding='utf-8')
    print(json.dumps({k:v for k,v in summary.items() if k!='source_file_sha256'},ensure_ascii=False,indent=2))
    return not all(result.wasSuccessful() for result in results)


if __name__=='__main__':raise SystemExit(main())
