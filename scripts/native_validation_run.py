"""Execute real native acceptance in the isolated task-owned Ubuntu guest."""
import argparse
from datetime import datetime,timezone
import json
from pathlib import Path
import sys
import traceback
import urllib.request
import urllib.parse
import shlex

from native_validation_vm import ROOT,ENV,connect,upload,command,load


def fetch(client,remote,local):
    sftp=client.open_sftp()
    try:sftp.get(remote,str(local))
    finally:sftp.close()


def native(client,install_system=True):
    digest=upload(client)
    sftp=client.open_sftp()
    try:previous=sftp.listdir('/home/tg3/project/artifacts')
    except OSError:previous=[]
    finally:sftp.close()
    (ENV/'source_bundle.json').write_text(json.dumps(dict(source_bundle_sha256=digest,previous_lab_reports=previous),indent=2),encoding='utf-8')
    proxy=''
    # Reuse an existing unauthenticated loopback proxy only in this task guest.
    # VirtualBox's 10.0.2.2 is the host-loopback alias. No credentials are read,
    # copied, logged, or accepted here; other proxy forms are deliberately ignored.
    candidates=urllib.request.getproxies()
    for name in ('https','all'):
        value=urllib.parse.urlparse(candidates.get(name,''))
        if value.hostname in ('127.0.0.1','localhost') and value.username is None and value.password is None and value.port:
            endpoint='http://10.0.2.2:'+str(value.port)
            proxy='HTTPS_PROXY='+shlex.quote(endpoint)+' HTTP_PROXY='+shlex.quote(endpoint)+' '
            break
    system='sudo cloud-init status --wait\nsudo apt-get update -qq\nsudo DEBIAN_FRONTEND=noninteractive apt-get install -y python3-venv python3-tk nodejs xvfb xauth\n' if install_system else ''
    script='''set -eo pipefail
SYSTEM
cd /home/tg3/project
PROXYpython3 scripts/robot_environment.py
export LD_LIBRARY_PATH="$PWD/.venv/lib/python3.12/site-packages/cmeel.prefix/lib:${LD_LIBRARY_PATH:-}"
xvfb-run -a .venv/bin/python -s scripts/lab_check.py
.venv/bin/python -s -m pip freeze > artifacts/native_freeze.txt
'''.replace('PROXY',proxy).replace('SYSTEM',system)
    command(client,'bash -c '+__import__('shlex').quote(script),timeout=1800,log_name='native_run.log')
    fetch(client,'/home/tg3/project/artifacts/test_result.json',ROOT/'artifacts/native_test_result.json')
    fetch(client,'/home/tg3/project/artifacts/native_freeze.txt',ROOT/'artifacts/native_freeze.txt')
    fetch(client,'/home/tg3/project/.venv/tg3_environment.json',ROOT/'artifacts/native_environment_result.json')
    sftp=client.open_sftp()
    folders=sorted(n for n in sftp.listdir('/home/tg3/project/artifacts') if n.startswith('lab_'))
    if not folders:raise RuntimeError('No actual Linux lab-check report')
    directory='/home/tg3/project/artifacts/'+folders[-1]
    sftp.get(directory+'/report.json',str(ROOT/'artifacts/native_lab_result.json'))
    sftp.get(directory+'/tests.log',str(ROOT/'artifacts/native_tests.log'));sftp.close()
    report=json.loads((ROOT/'artifacts/native_lab_result.json').read_text(encoding='utf-8'))
    tests=report['tests']
    # The no-Placo startup test is intentionally the opposite environment of
    # this native acceptance. Keep its real skip in the receipt; no native
    # solver/worker acceptance or other skipped case is exempted.
    expected_skips=[item for item in tests['skipped']
        if 'test_missing_native_placo_fails_startup_without_any_fallback' in item['test']
        and item['reason']=='requires an environment without native Placo']
    unexpected_skips=[item for item in tests['skipped'] if item not in expected_skips]
    report['expected_environment_skips']=expected_skips
    report['unexpected_skips']=unexpected_skips
    if tests['failures'] or tests['errors'] or unexpected_skips:
        raise RuntimeError('Native test acceptance has failures, errors, or unexpected skips')
    report['source_bundle_sha256']=digest
    report['execution_environment']='Task-owned VirtualBox Ubuntu 24.04 x86_64 guest, not robot hardware'
    report['native_kernel']=True
    (ROOT/'artifacts/native_lab_result.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
    return report


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--skip-system-install',action='store_true',help='Reuse this owned guest after system packages were installed')
    args=parser.parse_args()
    client=connect(20)
    result=dict(started=datetime.now(timezone.utc).isoformat(),real_linux_execution=True,
                real_placo_executed=False,ros2_build_executed=False,robot_commands_sent=False)
    try:
        report=native(client,install_system=not args.skip_system_install)
        result.update(real_linux_execution=True,real_placo_executed=report['native_placo_passed'],
                      tests=report['tests'],source_bundle_sha256=report['source_bundle_sha256'],version=report['version'])
    except Exception as exc:
        result['error']=str(exc);result['traceback']=traceback.format_exc()
    finally:
        try:
            sftp=client.open_sftp()
            receipt='/home/tg3/project/.venv/tg3_environment.json'
            try:
                sftp.get(receipt,str(ROOT/'artifacts/native_environment_result.json'))
                environment=json.loads((ROOT/'artifacts/native_environment_result.json').read_text(encoding='utf-8'))
                current=datetime.fromisoformat(environment['time'])>=datetime.fromisoformat(result['started'])
                result['real_placo_executed']=(result.get('real_placo_executed') is True
                    or (environment.get('real_placo_selftest') is True and current))
            except OSError:pass
            context=json.loads((ENV/'source_bundle.json').read_text(encoding='utf-8'))
            previous=context.get('previous_lab_reports',[])
            folders=sorted(n for n in sftp.listdir('/home/tg3/project/artifacts') if n.startswith('lab_') and n not in previous)
            if folders:
                directory='/home/tg3/project/artifacts/'+folders[-1]
                sftp.get(directory+'/report.json',str(ROOT/'artifacts/native_lab_result.json'))
                sftp.get(directory+'/tests.log',str(ROOT/'artifacts/native_tests.log'))
                report=json.loads((ROOT/'artifacts/native_lab_result.json').read_text(encoding='utf-8'))
                if report.get('tests'):
                    sftp.get('/home/tg3/project/artifacts/test_result.json',str(ROOT/'artifacts/native_test_result.json'))
                result['tests']=report.get('tests')
                result['version']=report.get('version')
                result['all_native_tests_passed']=report.get('portable_and_native_tests_passed') is True
            sftp.close()
            result['source_bundle_sha256']=context['source_bundle_sha256']
        except Exception as exc:result['evidence_collection_error']=str(exc)
        client.close();result['finished']=datetime.now(timezone.utc).isoformat()
        (ROOT/'artifacts/native_execution_result.json').write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf-8')
    print(json.dumps(result,ensure_ascii=False,indent=2))
    return 0 if result.get('all_native_tests_passed') and not result.get('error') else 2


if __name__=='__main__':raise SystemExit(main())
