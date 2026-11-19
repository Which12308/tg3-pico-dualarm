"""Explicit public release scope; prune caches before walking or reading files."""
import hashlib
import io
import json
import os
from pathlib import Path
import stat
import zipfile

PUBLIC_DIRS={'.github','config','desktop','docs','headset','headset_native','launch',
             'models','resource','scripts','src','tests','web'}
ROOT_FILES={'.gitignore','LICENSE','NOTICE','README.md','package.xml','pyproject.toml',
            'requirements-linux.txt','requirements.txt','setup_linux.sh','setup.cfg','setup.py',
            '启动无线遥操作.cmd','启动有线遥操作.cmd'}
EXCLUDED_DIRS={'.git','.venv','venv','.desktop','.headset_tools','manufacturer_research',
               'generated_client','__pycache__','dist','build','artifacts','recordings',
               '.test_tmp','.pytest_cache','.mypy_cache','.ruff_cache','.ssh','credentials',
               'secrets','node_modules','android-sdk','native_tools','native_environment',
               'native_snapshots'}
TEXT_SUFFIXES={'.py','.toml','.cfg','.md','.txt','.yaml','.yml','.json','.xml','.urdf',
               '.xacro','.cmake','.cpp','.c','.h','.hpp','.java','.cs','.html','.js','.css',
               '.sh','.cmd','.ps1','.rviz','.glsl','.vert','.frag','.msg','.srv','.action',
               '.obj','.dae','.stl','.svg','.png','.jpg','.jpeg','.meta'}
# Only these generated, reviewed reports may enter validation/. Never scan
# artifacts: it also holds virtual disks, SSH keys, recordings and raw logs.
PUBLIC_RESULTS=(
    'test_result.json','install_result.json','desktop_result.json','desktop_integration_result.json',
    'desktop_transport_result.json',
    'headset_java_transport_result.json','headset_bootstrap_result.json','usb_transport_result.json',
    'headset_usb_install_result.json',
    'manufacturer_result.json','manufacturer_grasp_result.json','manufacturer_arm_clutch_result.json',
    'native_execution_result.json','native_test_result.json','native_environment_result.json',
    'native_lab_result.json','native_ros_result.json','native_ros_runtime_result.json',
    'native_packaging_result.json','native_pipeline_result.json','native_station_result.json',
    'native_final_result.json',
    'native_worker_result.json',
    'robot_arm_acceptance_20261115.json','managed_vr_integration_result.json','managed_xarm_native_result.json',
    'headless_preparation_result.json',
)


def regular_file(path):
    try:return stat.S_ISREG(path.lstat().st_mode) and not path.is_symlink()
    except OSError:return False


def public_files(root):
    root=Path(root).resolve()
    result=[]
    for directory,dirs,names in os.walk(root,followlinks=False):
        base=Path(directory)
        dirs[:]=sorted(name for name in dirs
            if name.casefold() not in EXCLUDED_DIRS and not name.casefold().endswith('.egg-info')
            and not (base/name).is_symlink() and not getattr(base/name,'is_junction',lambda:False)()
            and (base!=root or name in PUBLIC_DIRS))
        for name in sorted(names):
            path=base/name;relative=path.relative_to(root);key=relative.as_posix()
            if not regular_file(path):continue
            if base==root:
                if name not in ROOT_FILES:continue
            else:
                lower=name.lower()
                if (lower.startswith(('.env','id_rsa','id_ed25519','credentials','private_key'))
                    or relative.parts[0] not in PUBLIC_DIRS
                    or key.casefold() in ('config/factory.json','config/factory_driver.py')):continue
                if path.suffix.lower() not in TEXT_SUFFIXES and not (
                    key=='headset_native/vendor/libopenxr_loader.so'
                    or name.startswith(('LICENSE','NOTICE','CMakeLists'))
                    or relative.parts[0]=='resource' and path.suffix==''):continue
            result.append((key,path))
    return sorted(result)


def public_source(root):
    # Selection precedes reads. Unselected private/cache paths are never read.
    return [(name,path.read_bytes()) for name,path in public_files(root)]


def robot_bytes(source):
    """Same deterministic project zip layout, excluding desktop/headset code."""
    buffer=io.BytesIO()
    with zipfile.ZipFile(buffer,'w',zipfile.ZIP_DEFLATED) as archive:
        for name,data in source:
            if name.split('/',1)[0] in ('desktop','headset','headset_native'):continue
            info=zipfile.ZipInfo(name,date_time=(2026,11,14,0,0,0))
            info.compress_type=zipfile.ZIP_DEFLATED;info.external_attr=0o100644<<16
            archive.writestr(info,data)
    return buffer.getvalue()


def read_public_result(root,name):
    if name not in PUBLIC_RESULTS and name!='delivery_result.json':raise ValueError('unapproved_validation_name')
    path=Path(root)/'artifacts'/name
    if not regular_file(path):return None
    if path.stat().st_size>8*1024*1024:raise ValueError('oversized_validation_report:'+name)
    value=json.loads(path.read_text(encoding='utf-8-sig'))
    if not isinstance(value,dict):raise ValueError('validation_report_object_required:'+name)
    return value


def headset_artifact(root,version):
    root=Path(root);apk=root/'dist'/('TG3-Wireless-'+version+'.apk');receipt=apk.with_suffix('.json')
    if not regular_file(apk) or not regular_file(receipt):raise RuntimeError('matching_native_headset_artifact_missing')
    value=json.loads(receipt.read_text(encoding='utf-8'))
    required=dict(version=version,native_build_executed=True,signature_verified=True,
        protocol='tg3-native-1',coordinates='openxr_rh_y_up',application_id='edu.scu.tg3.wireless',abi='arm64-v8a')
    if any(value.get(k)!=v or isinstance(v,bool) and value.get(k) is not v for k,v in required.items()):
        raise RuntimeError('native_headset_receipt_incompatible')
    if hashlib.sha256(apk.read_bytes()).hexdigest()!=value.get('sha256'):raise RuntimeError('native_headset_apk_hash_mismatch')
    with zipfile.ZipFile(apk) as archive:
        required_members={'AndroidManifest.xml','classes.dex','lib/arm64-v8a/libtg3_openxr.so',
                          'lib/arm64-v8a/libopenxr_loader.so'}
        if not required_members<=set(archive.namelist()) or archive.testzip() is not None:
            raise RuntimeError('native_headset_members_or_crc_invalid')
        if any(archive.getinfo(n).file_size==0 for n in required_members):raise RuntimeError('empty_native_headset_member')
        if not any(n.startswith('assets/licenses/') and not n.endswith('/') for n in archive.namelist()):
            raise RuntimeError('native_headset_licenses_missing')
    sources=value.get('source_files')
    if not isinstance(sources,dict) or not sources:raise RuntimeError('native_headset_source_receipt_missing')
    for name,digest in sources.items():
        relative=Path(name)
        if (relative.is_absolute() or '\\' in name or ':' in name or '..' in relative.parts
            or not (name=='AndroidManifest.xml' or name.startswith(('java/','cpp/','res/')))):
            raise RuntimeError('unsafe_native_headset_source_receipt')
        path=root/'headset_native'/relative
        if not regular_file(path) or hashlib.sha256(path.read_bytes()).hexdigest()!=digest:
            raise RuntimeError('native_headset_sources_changed_rebuild_required:'+name)
    return apk,receipt,value
