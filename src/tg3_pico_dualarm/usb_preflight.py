"""Robot-local USB prerequisite checks, before any factory controller startup."""
import hashlib,json,os,platform,re,shutil,subprocess,time
from pathlib import Path
from .robot_usb import RobotHeadsetUSB

def verified_packages(directory,expected_manifest_sha):
    directory=Path(directory).resolve(strict=True);manifest=directory/'manifest.json';raw=manifest.read_bytes()
    if hashlib.sha256(raw).hexdigest()!=expected_manifest_sha:raise RuntimeError('ADB缓存清单校验失败')
    value=json.loads(raw)
    if (value.get('schema'),value.get('distribution'),value.get('version'),value.get('architecture'))!=('tg3_ubuntu_adb_cache_v1','ubuntu','24.04','amd64'):
        raise RuntimeError('ADB缓存不适用于Ubuntu24.04/amd64')
    result=[];names=set()
    for item in value['packages']:
        filename=item['file'];name=item['package']
        if not isinstance(name,str) or not re.fullmatch(r'[a-z0-9][a-z0-9+.-]*',name):raise ValueError('invalid_deb_package')
        if not isinstance(filename,str) or Path(filename).name!=filename or '/' in filename or '\\' in filename or not filename.endswith('.deb'):
            raise ValueError('invalid_deb_path')
        path=directory/filename
        if path.is_symlink() or path.resolve().parent!=directory:raise ValueError('invalid_deb_path')
        data=path.read_bytes()
        if len(data)!=item['bytes'] or hashlib.sha256(data).hexdigest()!=item['sha256'] or not data.startswith(b'!<arch>\n'):
            raise RuntimeError('ADB安装包校验失败:'+name)
        names.add(name);result.append(str(path))
    if 'adb' not in names or len(names)!=len(result):raise ValueError('invalid_adb_package_set')
    return result,names

def install_missing_adb(directory,manifest_sha,*,run=subprocess.run,which=shutil.which):
    if which('adb'):return dict(installed_now=False)
    if os.geteuid()!=0:raise RuntimeError('ADB安装和USB检查需要与hardware/xarm相同的管理员身份')
    release=platform.freedesktop_os_release()
    if release.get('ID')!='ubuntu' or release.get('VERSION_ID')!='24.04' or platform.machine()!='x86_64':
        raise RuntimeError('离线ADB安装仅支持已核实Ubuntu24.04/amd64')
    paths,names=verified_packages(directory,manifest_sha)
    missing=[]
    for path in paths:
        name=next(item['package'] for item in json.loads((Path(directory)/'manifest.json').read_text(encoding='utf-8'))['packages'] if item['file']==Path(path).name)
        check=run(['dpkg-query','-W','-f=${db:Status-Abbrev}',name],capture_output=True,text=True,timeout=10)
        if check.returncode or not check.stdout.startswith('ii'):missing.append(path)
    paths=missing  # Never downgrade already-installed dependencies from a base-release cache.
    if not paths:raise RuntimeError('ADB包状态与可执行文件不一致，请检查本机安装')
    base=['apt-get','--no-download','--no-remove','--no-upgrade']
    checked=run(base+['-s','install',*paths],capture_output=True,text=True,timeout=45)
    if checked.returncode:raise RuntimeError('ADB离线依赖检查失败：'+(checked.stdout+checked.stderr)[-1800:])
    changes=set(re.findall(r'^Inst ([a-z0-9+.-]+)(?::\w+)?\s',checked.stdout,re.M))
    if changes-names or re.search(r'^Remv ',checked.stdout,re.M):raise RuntimeError('ADB安装要求改变缓存以外系统包，已停止')
    done=run(base+['-y','install',*paths],capture_output=True,text=True,timeout=180,
        env=dict(os.environ,DEBIAN_FRONTEND='noninteractive'))
    if done.returncode or not which('adb'):raise RuntimeError('ADB离线安装失败：'+(done.stdout+done.stderr)[-1800:])
    return dict(installed_now=True,packages=sorted(changes))

def wait_for_usb(seconds=60,*,device_factory=RobotHeadsetUSB,clock=time.monotonic,sleep=time.sleep,notify=print):
    deadline=clock()+seconds;last=None
    while True:
        try:return device_factory().check_available()
        except (RuntimeError,OSError,subprocess.TimeoutExpired) as exc:
            reason=str(exc)
            if reason!=last:notify('USB预检：'+reason);last=reason
            # Only an absent device / permission dialog is expected to resolve
            # while the user puts on the headset. Other faults fail immediately.
            retry=('USB调试授权' in reason or '恰好一台' in reason)
            if not retry or clock()>=deadline:raise RuntimeError(reason) from exc
            sleep(min(1.,max(0.,deadline-clock())))
