"""Compile/run the actual Java transport with a real local HTTP fixture.

The framework JSON parser is replaced by host org.json only for JVM execution;
no Android API behavior, headset render, or robot control is claimed verified.
"""
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import urllib.request

ROOT=Path(__file__).resolve().parents[2]
CACHE=ROOT.parent/'native_tools'/'java_test_dependencies'
URL='https://repo.maven.apache.org/maven2/org/json/json/20240303/json-20240303.jar'
EXPECTED_JSON_SHA256='3cf6cd6892e32e2b4c1c39e0f52f5248a2f5b37646fdfbb79a66b46b618414ed'


def main():
    CACHE.mkdir(parents=True,exist_ok=True)
    jar=CACHE/'json-20240303.jar'
    if not jar.exists():
        with urllib.request.urlopen(URL,timeout=30) as stream:jar.write_bytes(stream.read())
    if hashlib.sha256(jar.read_bytes()).hexdigest()!=EXPECTED_JSON_SHA256:
        raise RuntimeError('Pinned host JSON test dependency hash mismatch')
    output=ROOT/'artifacts/headset_java_http_test';output.mkdir(parents=True,exist_ok=True)
    java=shutil.which('java');javac=shutil.which('javac')
    if not java or not javac:raise RuntimeError('JDK is required')
    sources=[ROOT/'headset_native/java/edu/scu/tg3/wireless/WirelessLink.java',
             ROOT/'headset_native/tests/WirelessLinkJvmTest.java']
    subprocess.run([javac,'--release','8','-Xlint:-options','-encoding','UTF-8','-classpath',str(jar),
                    '-d',str(output),*map(str,sources)],check=True)
    host='127.0.0.1'  # Same HTTP host used by native Android through ADB reverse.
    process=subprocess.run([java,'--add-modules','jdk.httpserver','-Dtg3.test.host='+host,
        '-classpath',str(output)+os.pathsep+str(jar),'edu.scu.tg3.wireless.WirelessLinkJvmTest'],
        capture_output=True,text=True,encoding='utf-8',timeout=25)
    (output/'stdout.log').write_text(process.stdout,encoding='utf-8')
    (output/'stderr.log').write_text(process.stderr,encoding='utf-8')
    if process.returncode:raise RuntimeError(process.stdout+process.stderr)
    report=json.loads(process.stdout.strip().splitlines()[-1])
    report['host_json_dependency']=dict(url=URL,sha256=hashlib.sha256(jar.read_bytes()).hexdigest())
    report['java_source_sha256']=hashlib.sha256(sources[0].read_bytes()).hexdigest()
    (ROOT/'artifacts/headset_java_transport_result.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
    print(json.dumps(report,indent=2))


if __name__=='__main__':main()
