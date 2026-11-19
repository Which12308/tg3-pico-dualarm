"""Package selected public source/evidence/APK; do not include machine environments."""
import argparse
import hashlib
import io
import json
from pathlib import Path
import sys
import zipfile

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src'))
from tg3_pico_dualarm import __version__
from release_scope import PUBLIC_RESULTS,headset_artifact,public_source,read_public_result,robot_bytes


def build_release(root,label):
    root=Path(root)
    if not label or not all(c.isalnum() or c in '_-' for c in label):raise ValueError('invalid_release_label')
    output=root/'dist'
    report=read_public_result(root,'test_result.json')
    if not report or report.get('failures')!=0 or report.get('errors')!=0:
        raise RuntimeError('Tests missing or have unresolved failures')
    delivery=read_public_result(root,'delivery_result.json')
    if not delivery or delivery.get('version')!=__version__:raise RuntimeError('Generate matching delivery report first')
    apk,receipt,apk_info=headset_artifact(root,__version__)
    if delivery.get('headset_apk_built') is not True:raise RuntimeError('Delivery report has no verified current APK')
    if (delivery.get('headset_apk_receipt') or {}).get('sha256')!=apk_info['sha256']:
        raise RuntimeError('APK changed after delivery report; regenerate evidence ledger')
    archive=output/('tg3_pico_dualarm_'+label+'.zip')
    robot_name='tg3_robot_'+label+'.zip';robot_archive=output/robot_name
    info_path=output/('release_info_'+label+'.json')
    if any(p.exists() for p in (archive,robot_archive,info_path)):
        raise RuntimeError('Keep frozen release artifacts; use a new label')
    source=public_source(root);robot_data=robot_bytes(source)
    robot_sha=hashlib.sha256(robot_data).hexdigest()
    if delivery.get('current_robot_source_sha256')!=robot_sha:
        raise RuntimeError('Public source changed after delivery report; regenerate ledger before freezing')
    files=list(source)
    for name in PUBLIC_RESULTS+('delivery_result.json',):
        value=read_public_result(root,name)
        if value is not None:
            files.append(('validation/'+name,json.dumps(value,ensure_ascii=False,indent=2).encode('utf-8')))
    files.extend((('robot/'+robot_name,robot_data),('dist/'+apk.name,apk.read_bytes()),
                  ('dist/'+receipt.name,receipt.read_bytes())))
    names=[name for name,_ in files]
    if len(names)!=len(set(names)):raise RuntimeError('Duplicate release member')
    manifest=dict(version=__version__,tests=report,status='development_candidate',
        default_transport='usb_adb_reverse',optional_transport='wifi',
        hardware_motion_enabled=False,headset_apk_built=True,hardware_ready=False,one_click_hardware_ready=False,
        pending=delivery.get('pending',[]),blocked=delivery.get('blocked',[]),
        native_placo_executed=delivery.get('linux_native_ik_executed',False),
        ros2_jazzy_native_build_executed=delivery.get('ros2_native_build_executed',False),
        evidence_snapshots=delivery.get('evidence_snapshots',{}),
        execution_flags_scope=delivery.get('execution_flags_scope'),
        current_robot_source_sha256=robot_sha,
        scope='Native OpenXR APK, USB-first desktop, native IK, managed XARM startup/handoff/bounded output/restore and calibrated BrainCo adapters. Actual default-disabled entry and separate small elbow trial passed; integrated headset motion and grasp pending.',
        files={name:hashlib.sha256(data).hexdigest() for name,data in files})
    buffer=io.BytesIO()
    with zipfile.ZipFile(buffer,'w',zipfile.ZIP_DEFLATED) as zipped:
        for name,data in files:zipped.writestr('tg3_pico_dualarm/'+name,data)
        zipped.writestr('tg3_pico_dualarm/release_manifest.json',json.dumps(manifest,ensure_ascii=False,indent=2))
    data=buffer.getvalue()
    with zipfile.ZipFile(io.BytesIO(data)) as zipped:
        if zipped.testzip() is not None:raise RuntimeError('Release CRC mismatch')
        for name,expected in manifest['files'].items():
            if hashlib.sha256(zipped.read('tg3_pico_dualarm/'+name)).hexdigest()!=expected:
                raise RuntimeError('Release member hash mismatch:'+name)
    with zipfile.ZipFile(io.BytesIO(robot_data)) as zipped:
        if zipped.testzip() is not None:raise RuntimeError('Robot archive CRC mismatch')
    output.mkdir(exist_ok=True)
    # No real release is written until all selected content and hashes validate.
    with robot_archive.open('xb') as stream:stream.write(robot_data)
    with archive.open('xb') as stream:stream.write(data)
    summary=dict(archive=str(archive),sha256=hashlib.sha256(data).hexdigest(),bytes=len(data),
        files=len(files)+1,hardware_motion_enabled=False,headset_apk_built=True,
        robot_archive=str(robot_archive),robot_sha256=robot_sha)
    with info_path.open('x',encoding='utf-8') as stream:json.dump(summary,stream,ensure_ascii=False,indent=2)
    print(json.dumps(summary,ensure_ascii=False,indent=2))
    return summary


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--label',default='20261114')
    args=parser.parse_args();build_release(ROOT,args.label)


if __name__=='__main__':main()
