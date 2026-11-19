"""Create a pinned XRoboToolkit project with the TG3 native wireless component.

Only modifies the dedicated generated checkout, never the original project.
Unity 2022.3.16f1 plus Android support and a valid Unity license are required.
"""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import xml.etree.ElementTree as ET

UPSTREAM='https://github.com/XR-Robotics/XRoboToolkit-Unity-Client.git'
COMMIT='cdc53166b0bf412efae71046c6a225eb5091605f'
HERE=Path(__file__).resolve().parent
ROOT=HERE.parent


def prepare(target):
    target=Path(target).resolve()
    if target == ROOT or ROOT in target.parents and target.name not in ('generated_client',):
        raise ValueError('use a separate generated_client directory')
    if not target.exists():
        subprocess.run(['git','clone','--no-checkout',UPSTREAM,str(target)],check=True)
        subprocess.run(['git','-C',str(target),'checkout',COMMIT],check=True)
    actual=subprocess.check_output(['git','-C',str(target),'rev-parse','HEAD'],text=True).strip()
    if actual!=COMMIT:
        raise ValueError('unexpected_upstream_commit_do_not_overwrite')
    marker=target/'.tg3-generated'
    status=subprocess.check_output(['git','-C',str(target),'status','--porcelain'],text=True)
    if status.strip() and not marker.exists():
        raise ValueError('checkout_has_user_changes_do_not_overwrite')
    for source in (HERE/'Assets').rglob('*.cs'):
        destination=target/'Assets'/source.relative_to(HERE/'Assets')
        destination.parent.mkdir(parents=True,exist_ok=True)
        shutil.copyfile(source,destination)
    # Standard system passthrough is sufficient. Remove the upstream enterprise
    # camera capture and boundary-disable calls from our dedicated client.
    (target/'Assets/Scripts/Main.cs').write_text('''using UnityEngine;
using Unity.XR.PXR;
public class Main : MonoBehaviour {
 void Awake() { PXR_Manager.EnableVideoSeeThrough = true; }
 void OnApplicationPause(bool paused) { if (!paused) PXR_Manager.EnableVideoSeeThrough = true; }
}
''',encoding='utf-8')
    manifest=target/'Assets/Plugins/Android/AndroidManifest.xml'
    tree=ET.parse(manifest); android='{http://schemas.android.com/apk/res/android}'
    tree.getroot().find('application').set(android+'usesCleartextTraffic','true')
    tree.write(manifest,encoding='utf-8',xml_declaration=True)
    marker.write_text(json.dumps(dict(upstream=UPSTREAM,commit=COMMIT,protocol='tg3-native-1',
        component_sha256=hashlib.sha256((HERE/'Assets/TG3/TG3Wireless.cs').read_bytes()).hexdigest())),encoding='utf-8')
    return target


def main():
    p=argparse.ArgumentParser();p.add_argument('--project',type=Path,default=HERE/'generated_client')
    p.add_argument('--unity',type=Path);p.add_argument('--output',type=Path,default=ROOT/'dist/TG3-Wireless-0.3.0.apk')
    args=p.parse_args();target=prepare(args.project)
    if not args.unity:
        print('源码已准备；APK 尚未构建。需要 Unity 2022.3.16f1、Android 模块及有效许可证。')
        return 2
    args.output.parent.mkdir(exist_ok=True,parents=True)
    result=subprocess.run([str(args.unity),'-batchmode','-quit','-projectPath',str(target),'-buildTarget','Android',
        '-executeMethod','TG3Build.Android','-tg3Output',str(args.output.resolve()),
        '-logFile',str(args.output.with_suffix('.build.log').resolve())])
    if result.returncode or not args.output.exists():return 2
    args.output.with_suffix('.json').write_text(json.dumps(dict(version='0.3.0',protocol='tg3-native-1',
        upstream_commit=COMMIT,sha256=hashlib.sha256(args.output.read_bytes()).hexdigest(),
        native_build_executed=True,headset_wireless_acceptance=False,hardware_acceptance=False),indent=2),encoding='utf-8')
    return 0


if __name__=='__main__':raise SystemExit(main())
