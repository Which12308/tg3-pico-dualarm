"""First-install helper. Requires a genuine custom APK and matching build receipt."""
import hashlib
import json
from pathlib import Path
import subprocess
from tg3_pico_dualarm import __version__


def artifact_info(root,version=None):
    root=Path(root)
    version=version or __version__
    apk=root/('dist/TG3-Wireless-'+version+'.apk')
    receipt=apk.with_suffix('.json')
    if not apk.exists() or not receipt.exists():
        raise RuntimeError('交付目录缺少匹配版本的头显 APK 或构建回执：'+apk.name)
    value=json.loads(receipt.read_text(encoding='utf-8'))
    if (value.get('version')!=version or value.get('native_build_executed') is not True
        or value.get('signature_verified') is not True or value.get('protocol')!='tg3-native-1'
        or value.get('coordinates')!='openxr_rh_y_up' or value.get('application_id')!='edu.scu.tg3.wireless'
        or value.get('abi')!='arm64-v8a'):
        raise RuntimeError('头显构建回执不兼容')
    if hashlib.sha256(apk.read_bytes()).hexdigest()!=value.get('sha256'):
        raise RuntimeError('头显安装包校验不通过')
    return apk,value


def select_pico(adb,min_sdk=30):
    """Read identity before any install/reverse; never select a phone by position."""
    devices=subprocess.check_output([str(adb),'devices'],text=True,timeout=8)
    selected=[line.split()[0] for line in devices.splitlines() if len(line.split())==2 and line.split()[1]=='device']
    if len(selected)!=1:
        raise RuntimeError('首次安装需恰好一台已授权 USB 头显；请在头显允许 USB 调试。')
    def prop(name):
        return subprocess.check_output([str(adb),'-s',selected[0],'shell','getprop',name],text=True,timeout=8).strip()
    identity=' '.join(prop(name) for name in ('ro.product.manufacturer','ro.product.brand','ro.product.model'))
    if 'pico' not in identity.lower() and 'a9210' not in identity.lower():
        raise RuntimeError('当前授权设备未识别为 PICO，请确认 USB 连接的是实验室头显。')
    if 'arm64-v8a' not in prop('ro.product.cpu.abilist').split(','):
        raise RuntimeError('头显 CPU 架构与 ARM64 安装包不匹配。')
    try:sdk=int(prop('ro.build.version.sdk'))
    except ValueError as exc:raise RuntimeError('无法读取头显 Android API 版本。') from exc
    if sdk<max(30,min_sdk):raise RuntimeError('头显系统版本低于安装包要求。')
    return selected[0]


def prepare_usb(adb,expected_serial=None):
    """Prepare the native HTTP endpoint through ADB, not USB network sharing."""
    serial=select_pico(adb)
    if expected_serial is not None and serial!=expected_serial:
        raise RuntimeError('头显连接在安装期间发生变化；USB 通道未配置，请重新准备。')
    subprocess.run([str(adb),'-s',serial,'reverse','tcp:8877','tcp:8877'],check=True,timeout=10)
    return 'USB 通道已设置；请保持数据线连接，在头显选择 USB 并填写电脑显示的配对码。'


def install(root, adb,version=None):
    apk,value=artifact_info(root,version)
    serial=select_pico(adb,value['min_sdk'])
    subprocess.run([str(adb),'-s',serial,'install','-r',str(apk)],check=True,timeout=120)
    prepare_usb(adb,expected_serial=serial)
    subprocess.run([str(adb),'-s',serial,'shell','am','start','-n',
        'edu.scu.tg3.wireless/.PairActivity'],check=True,timeout=15)
    return '安装完成，USB 通道已设置；请保持数据线连接，在头显默认 USB 模式填写配对码。Wi-Fi 是可选模式。'
