"""Summarize executed evidence without upgrading a snapshot to physical acceptance."""
import hashlib
import json
from pathlib import Path
import platform
import sys

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src'))
from tg3_pico_dualarm import __version__
from release_scope import PUBLIC_RESULTS,headset_artifact,public_source,read_public_result,robot_bytes


def report(root=ROOT):
    root=Path(root)
    executed={name:read_public_result(root,name) for name in PUBLIC_RESULTS}
    tests=executed['test_result.json']
    if not tests or tests.get('failures')!=0 or tests.get('errors')!=0:
        raise RuntimeError('portable_tests_missing_or_not_passing')
    source=public_source(root);robot_hash=hashlib.sha256(robot_bytes(source)).hexdigest()
    source_manifest={name:hashlib.sha256(data).hexdigest() for name,data in source}
    snapshots={}
    for name,value in executed.items():
        if value is None:continue
        version=value.get('version',value.get('project_version'))
        snapshot=value.get('source_bundle_sha256')
        snapshots[name]=dict(recorded_version=version,same_project_version=version==__version__,
            source_bundle_sha256=snapshot,
            matches_current_robot_source=bool(snapshot and snapshot==robot_hash))
        recorded_files=value.get('source_file_sha256')
        if isinstance(recorded_files,dict):
            changed=[name for name,digest in recorded_files.items() if source_manifest.get(name)!=digest]
            snapshots[name]['recorded_source_files_changed']=changed
            snapshots[name]['recorded_source_files_match_current']=not changed
    built=False;apk_receipt=None;apk_error=None
    try:
        _,_,apk_receipt=headset_artifact(root,__version__);built=True
    except (OSError,ValueError,RuntimeError) as exc:apk_error=str(exc)
    native=executed.get('native_execution_result.json') or {}
    ros=executed.get('native_ros_result.json') or {}
    pipeline=executed.get('native_pipeline_result.json') or {}
    station=executed.get('native_station_result.json') or {}
    usb=executed.get('headset_usb_install_result.json') or {}
    usb_current=bool(built and usb.get('version')==__version__ and usb.get('installed_matches_build') is True
        and usb.get('installed_apk_sha256')==apk_receipt['sha256'])
    arm=executed.get('robot_arm_acceptance_20261115.json') or {}
    integration=executed.get('managed_vr_integration_result.json') or {}
    managed_native=executed.get('managed_xarm_native_result.json') or {}
    blocked=[dict(item='full_VR_grasp_acceptance_and_hand_pose_validation',
        reason='Model-TCP relative control uses verified reference definitions with automatic heading/clutch calibration; measured palm offsets are optional. Integrated headset motion, hand grasp-pose validation and fixed-bottle demonstrations remain field work')]
    if not built:blocked.append(dict(item='current_native_headset_apk',reason=apk_error))
    result=dict(version=__version__,status='development_candidate',default_transport='usb_adb_reverse',
        optional_transport='wifi',hardware_ready=False,one_click_hardware_ready=False,
        hardware_motion_enabled=False,platform=platform.platform(),executed=executed,
        evidence_snapshots=snapshots,current_robot_source_sha256=robot_hash,
        public_source_manifest_sha256=hashlib.sha256(json.dumps(source_manifest,sort_keys=True).encode()).hexdigest(),
        headset_apk_built=built,headset_apk_receipt=apk_receipt,
        linux_native_ik_executed=native.get('real_linux_execution') is True and native.get('real_placo_executed') is True,
        ros2_native_build_executed=ros.get('ros2_build_executed') is True,
        ros2_native_runtime_executed=ros.get('ros2_runtime_executed') is True,
        native_pipeline_executed=pipeline.get('passed') is True and pipeline.get('real_native_placo') is True,
        native_station_executed=station.get('passed') is True,
        native_worker_executed=(executed.get('native_worker_result.json') or {}).get('passed') is True,
        one_click_entry_implemented=integration.get('software_integrated') is True,
        managed_entry_start_stop_tested=integration.get('passed') is True,
        managed_native_regression_passed=managed_native.get('passed') is True,
        supported_small_right_arm_motion_verified=arm.get('passed') is True and arm.get('physical_robot_tested') is True,
        physical_headset_usb_apk_matches_current=usb_current,
        physical_headset_usb_input_executed=bool(usb_current and usb.get('physical_pico_usb_detected') is True
            and usb.get('actual_headset_to_pc_frames',0)>0),
        physical_passthrough_operator_confirmed=bool(usb_current and usb.get('passthrough_reported_by_operator') is True),
        execution_flags_scope='Recorded snapshots; inspect version and source matches before attributing them to this release',
        physical_headset_acceptance_verified=False,physical_robot_acceptance_verified=False,
        manufacturer_execution_scope='Actual supported small right-elbow motion and actual managed default-disabled factory startup/restore; full VR/grasp acceptance pending. Portable tests use explicit doubles.',
        blocked=blocked,pending=[
            'Refresh final Windows/Linux evidence for the final source snapshot; retain skips and mismatches explicitly',
            'Verify the new PC SSH/TCP_NODELAY/status-timeout and automatic read-only collector on the actual live link; local regression is not field acceptance',
            'Complete PICO single-controller occlusion, USB disconnect/reconnect, released reengagement and Y/B tests; Wi-Fi remains untested and optional',
            'Validate headset-driven enable, clutch hold/reengage, tracking loss and physical stop with the integrated managed entry; the separate supported elbow trial does not replace it',
            'Validate the selected model-TCP reference and hand grasp poses; supply measured palm offsets only when that operation point is desired',
            'Integrated low-speed VR single/dual arm and five consecutive fixed-bottle trials',
            'D455 recording and camera/control clock alignment at the laboratory'])
    path=root/'artifacts/delivery_result.json';path.parent.mkdir(exist_ok=True)
    path.write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf-8')
    print(json.dumps(dict(report=str(path),version=__version__,status=result['status'],
        headset_apk_built=built,hardware_ready=False,tests=tests.get('tests_run'),
        passed=tests.get('passed'),skipped=len(tests.get('skipped',[]))),ensure_ascii=False,indent=2))
    return result


if __name__=='__main__':report()
