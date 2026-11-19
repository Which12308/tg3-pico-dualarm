"""Build and verify the actual ARM64 APK with Android SDK/NDK; no Unity needed."""
import argparse
from datetime import datetime,timezone
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import zipfile

HERE=Path(__file__).resolve().parent
ROOT=HERE.parent
VERSION='0.4.0'


def build(tools):
    tools=Path(tools).resolve()
    sdk=tools/'android-sdk'
    ndk=sdk/'android-ndk-r26d'
    build_tools=sdk/'android-14'
    java_home=Path(os.environ.get('JAVA_HOME','D:/JavaDevelop/jdk'))
    def java_tool(name):
        p=java_home/'bin'/(name+'.exe')
        if p.exists():return str(p)
        value=shutil.which(name)
        if not value:raise RuntimeError('Missing JDK 17+ tool: '+name)
        return value
    android=sdk/'android-34/android.jar'
    compiler=ndk/'toolchains/llvm/prebuilt/windows-x86_64/bin/clang++.exe'
    if not compiler.exists() or not android.exists():
        raise RuntimeError('Run headset_native/bootstrap.py --tools-dir <path> first; Android 34 and NDK r26d required')
    vendor=HERE/'vendor'
    receipt=json.loads((vendor/'dependencies.json').read_text(encoding='utf-8'))
    for name,digest in receipt['files'].items():
        if hashlib.sha256((vendor/name).read_bytes()).hexdigest()!=digest:raise RuntimeError('Vendor hash mismatch: '+name)
    output=ROOT/'dist'/('TG3-Wireless-'+VERSION+'.apk')
    work=ROOT/'artifacts/native_apk_build';work.mkdir(exist_ok=True,parents=True)
    output.parent.mkdir(exist_ok=True)
    report=dict(version=VERSION,protocol='tg3-native-1',coordinates='openxr_rh_y_up',
        implementation='Khronos native OpenXR / PICO controller profiles',native_build_executed=False,
        headset_wireless_acceptance=False,hardware_acceptance=False,
        robot_commands_sent=False,steps=[],dependency_receipt=receipt)
    def run(label,args):
        with (work/(label+'.log')).open('w',encoding='utf-8') as log:
            result=subprocess.run([str(v) for v in args],stdout=log,stderr=subprocess.STDOUT,cwd=ROOT)
        report['steps'].append(dict(step=label,returncode=result.returncode))
        if result.returncode:
            raise RuntimeError(label+' failed: '+(work/(label+'.log')).read_text(encoding='utf-8',errors='replace')[-6000:])
        print('PASS',label,flush=True)
    try:
        common=['--target=aarch64-linux-android30','--sysroot='+str(ndk/'toolchains/llvm/prebuilt/windows-x86_64/sysroot'),'-fPIC','-O2']
        glue=ndk/'sources/android/native_app_glue'
        run('native_glue',[compiler.with_name('clang.exe'),*common,'-c',glue/'android_native_app_glue.c','-o',work/'glue.o'])
        run('native_openxr',[compiler,*common,'-std=c++17','-shared','-static-libstdc++',
            '-Wl,--no-undefined','-Wl,-z,max-page-size=16384','-Wl,-u,ANativeActivity_onCreate',
            '-I'+str(glue),'-I'+str(vendor),'-I'+str(HERE/'cpp'),HERE/'cpp/main.cpp',work/'glue.o',
            '-L'+str(vendor),'-lopenxr_loader','-llog','-landroid','-lEGL','-lGLESv3','-o',work/'libtg3_openxr.so'])
        run('strip_native',[compiler.with_name('llvm-strip.exe'),'--strip-unneeded',work/'libtg3_openxr.so'])
        readelf=compiler.with_name('llvm-readelf.exe')
        run('inspect_native_elf',[readelf,'-h','-l','-d',work/'libtg3_openxr.so'])
        classes=work/'classes';classes.mkdir(exist_ok=True)
        sources=list((HERE/'java').rglob('*.java'))
        run('compile_java',[java_tool('javac'),'--release','8','-encoding','UTF-8','-classpath',android,'-d',classes,*sources])
        dex=work/'dex';dex.mkdir(exist_ok=True)
        run('android_dex',[java_tool('java'),'-cp',build_tools/'lib/d8.jar','com.android.tools.r8.D8',
            '--release','--min-api','30','--lib',android,'--output',dex,*classes.rglob('*.class')])
        base=work/'base.apk'
        run('android_resources',[build_tools/'aapt2.exe','compile','--dir',(HERE/'res').relative_to(ROOT),
            '-o',(work/'resources.zip').relative_to(ROOT)])
        run('android_manifest',[build_tools/'aapt2.exe','link','-I',android,'--manifest',HERE/'AndroidManifest.xml',
            '--min-sdk-version','30','--target-sdk-version','34','--version-code','400','--version-name',VERSION,
            '-o',base,work/'resources.zip'])
        unsigned=work/'unsigned.apk';shutil.copyfile(base,unsigned)
        with zipfile.ZipFile(unsigned,'a') as archive:
            archive.write(dex/'classes.dex','classes.dex',compress_type=zipfile.ZIP_DEFLATED)
            archive.write(work/'libtg3_openxr.so','lib/arm64-v8a/libtg3_openxr.so',compress_type=zipfile.ZIP_STORED)
            archive.write(vendor/'libopenxr_loader.so','lib/arm64-v8a/libopenxr_loader.so',compress_type=zipfile.ZIP_STORED)
            for path in (vendor/'LICENSES').glob('*'):
                if path.is_file():archive.write(path,'assets/licenses/'+path.name)
        aligned=work/'aligned.apk'
        run('zipalign',[build_tools/'zipalign.exe','-f','-p','4',unsigned,aligned])
        keystore=tools/'tg3-development.keystore'
        if not keystore.exists():
            run('development_key',[java_tool('keytool'),'-genkeypair','-keystore',keystore,'-alias','androiddebugkey',
                '-storepass','android','-keypass','android','-keyalg','RSA','-keysize','2048','-sigalg','SHA256withRSA',
                '-validity','3650','-dname','CN=TG3 Laboratory Development,O=SCU Student Project,C=CN','-noprompt'])
        signer=[java_tool('java'),'-jar',build_tools/'lib/apksigner.jar']
        run('sign_apk',[*signer,'sign','--ks',keystore,'--ks-key-alias','androiddebugkey','--ks-pass','pass:android',
            '--key-pass','pass:android','--out',output,aligned])
        run('verify_signature',[*signer,'verify','--verbose','--print-certs',output])
        run('inspect_apk',[build_tools/'aapt.exe','dump','badging',output.relative_to(ROOT)])
        with zipfile.ZipFile(output) as archive:
            if archive.testzip() is not None:raise RuntimeError('APK CRC failed')
            if not {'AndroidManifest.xml','classes.dex','lib/arm64-v8a/libtg3_openxr.so','lib/arm64-v8a/libopenxr_loader.so'}<=set(archive.namelist()):
                raise RuntimeError('APK missing required runtime code')
        report.update(native_build_executed=True,signature_verified=True,sha256=hashlib.sha256(output.read_bytes()).hexdigest(),
            bytes=output.stat().st_size,application_id='edu.scu.tg3.wireless',abi='arm64-v8a',min_sdk=30,target_sdk=34,
            development_signed=True,built_at=datetime.now(timezone.utc).isoformat(),
            source_files={p.relative_to(HERE).as_posix():hashlib.sha256(p.read_bytes()).hexdigest()
                          for folder in ('java','cpp','res') for p in (HERE/folder).rglob('*') if p.is_file()})
        report['source_files']['AndroidManifest.xml']=hashlib.sha256((HERE/'AndroidManifest.xml').read_bytes()).hexdigest()
        output.with_suffix('.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
        print(json.dumps(dict(apk=str(output),bytes=report['bytes'],sha256=report['sha256'],signature_verified=True,
            headset_tested=False,robot_commands_sent=False),ensure_ascii=False,indent=2))
    except (RuntimeError,OSError) as exc:
        report['error']=str(exc)
        raise
    finally:
        (work/'build_result.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--tools-dir',type=Path,default=ROOT/'.headset_tools')
    args=parser.parse_args()
    build(args.tools_dir)
