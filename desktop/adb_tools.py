"""Private, fixed official Platform Tools for first-install; no global PATH change."""
import hashlib
from pathlib import Path
import threading
import urllib.request
import zipfile

URL='https://dl.google.com/android/repository/platform-tools_r37.0.1-win.zip'
SIZE=8044989
SHA1='e03e78b1d80b396f1c3358e31251cb31740e1110'
LOCK=threading.RLock()
REQUIRED=('adb.exe','AdbWinApi.dll','AdbWinUsbApi.dll')
OPTIONAL=('NOTICE.txt','source.properties')


def extract_checked(archive,destination):
    destination=Path(destination).resolve();destination.mkdir(parents=True,exist_ok=True)
    with zipfile.ZipFile(archive) as source:
        entries=source.infolist()
        if sum(v.file_size for v in entries)>128*1024*1024:raise ValueError('platform_tools_zip_too_large')
        names=[v.filename for v in entries]
        if len(set(name.casefold() for name in names))!=len(names):raise ValueError('ambiguous_platform_tools_zip')
        for name in REQUIRED:
            if 'platform-tools/'+name not in names:raise ValueError('platform_tools_missing_'+name)
        if source.testzip() is not None:raise ValueError('platform_tools_crc_failed')
        for name in REQUIRED+OPTIONAL:
            key='platform-tools/'+name
            if key not in names:continue
            info=source.getinfo(key)
            if (info.external_attr>>16)&0o170000 not in (0,0o100000):raise ValueError('platform_tools_special_file')
            target=destination/name
            if target.resolve().parent!=destination:raise ValueError('platform_tools_cache_redirected')
            data=source.read(info)
            if not target.exists() or target.read_bytes()!=data:target.write_bytes(data)
    return destination/'adb.exe'


def ensure_adb(root,notify=lambda message:None):
    root=Path(root).resolve();cache=(root/'.desktop/platform-tools').resolve()
    if root not in cache.parents:raise ValueError('platform_tools_cache_outside_project')
    with LOCK:
        cache.mkdir(parents=True,exist_ok=True)
        archive=cache/'platform-tools_r37.0.1-win.zip'
        if not archive.exists() or archive.stat().st_size!=SIZE or hashlib.sha1(archive.read_bytes()).hexdigest()!=SHA1:
            notify('首次安装：正在下载 Google 官方 ADB 到项目私有目录。')
            partial=archive.with_suffix('.part')
            with urllib.request.urlopen(URL,timeout=45) as response,partial.open('wb') as stream:
                total=0
                while True:
                    data=response.read(256*1024)
                    if not data:break
                    total+=len(data)
                    if total>SIZE:raise ValueError('unexpected_platform_tools_size')
                    stream.write(data)
            if total!=SIZE or hashlib.sha1(partial.read_bytes()).hexdigest()!=SHA1:
                raise ValueError('Google_platform_tools_checksum_failed')
            partial.replace(archive)
        return extract_checked(archive,cache)
