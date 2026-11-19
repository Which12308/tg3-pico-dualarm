"""Install pinned Android build inputs into a project-private Windows directory.

Official downloads are resumed using confirmed byte ranges, checked by SHA-1
and SHA-256, and extracted only after archive path validation. No registry,
global Python, Android Studio, system PATH or original robot file is modified.
"""
import argparse
from contextlib import contextmanager
from datetime import datetime, timezone
import hashlib
import http.client
import json
import os
from pathlib import Path
import platform
import re
import shutil
import subprocess
import sys
import time
import urllib.error
import urllib.parse
import urllib.request
import zipfile

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
CHUNK_SIZE = 32 * 1024 * 1024


class IntegrityError(RuntimeError):
    pass


def hashes(path):
    sha1, sha256 = hashlib.sha1(), hashlib.sha256()
    with Path(path).open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            sha1.update(chunk)
            sha256.update(chunk)
    return sha1.hexdigest(), sha256.hexdigest()


def verify_archive(path, item):
    path = Path(path)
    if path.stat().st_size != item['size']:
        raise IntegrityError('archive size mismatch: ' + item['id'])
    sha1, sha256 = hashes(path)
    if sha1 != item['sha1'] or sha256 != item['sha256']:
        raise IntegrityError('official archive checksum mismatch: ' + item['id'])


def _contained(base, path):
    base, path = Path(base).resolve(), Path(path).resolve()
    if base not in path.parents:
        raise ValueError('path leaves intended tools directory: ' + str(path))
    return path


def _quarantine(path):
    """Keep corrupt private-cache files for diagnosis, without recursive moves."""
    path = Path(path).resolve()
    target = path.with_name(path.name + '.corrupt.' + str(time.time_ns()))
    _contained(path.parent, path)
    _contained(path.parent, target)
    path.replace(target)
    return target


def _private_child(tools, name):
    tools = Path(tools).resolve()
    child = tools / name
    if child.is_symlink() or (hasattr(child, 'is_junction') and child.is_junction()):
        raise ValueError('private cache directory must not be redirected by a link: ' + name)
    return _contained(tools, child)


def _allowed_url(url, allow_local_http=False):
    value = urllib.parse.urlsplit(url)
    if value.username or value.password or value.query or value.fragment:
        raise ValueError('tool download URL cannot contain credentials or parameters')
    if value.scheme == 'https' and value.hostname == 'dl.google.com':
        return
    if allow_local_http and value.scheme == 'http' and value.hostname in ('127.0.0.1', 'localhost', '::1'):
        return
    raise ValueError('tool download requires the pinned official Google HTTPS source')


def download(item, directory, *, chunk_size=CHUNK_SIZE, retries=5, timeout=45,
             allow_local_http=False, progress=print):
    """Return a fully verified archive. Failed transfers retain the .part file."""
    _allowed_url(item['url'], allow_local_http)
    directory = Path(directory).resolve()
    directory.mkdir(parents=True, exist_ok=True)
    name = item['archive']
    if Path(name).name != name or not re.fullmatch(r'[A-Za-z0-9_.-]+\.zip', name):
        raise ValueError('invalid cache archive name')
    destination_name = directory / name
    partial_name = destination_name.with_suffix('.part')
    for candidate in (destination_name, partial_name):
        if candidate.is_symlink() or (hasattr(candidate, 'is_junction') and candidate.is_junction()):
            raise ValueError('tool archive cache file must not be a link')
        if candidate.exists() and not candidate.is_file():
            raise ValueError('tool archive cache path is not a regular file')
    destination = _contained(directory, destination_name)
    partial = _contained(directory, partial_name)
    if destination.exists():
        try:
            verify_archive(destination, item)
            return destination
        except IntegrityError:
            progress('Corrupt cached archive preserved: ' + str(_quarantine(destination)), flush=True)
    if partial.exists() and partial.stat().st_size > item['size']:
        progress('Oversize partial archive preserved: ' + str(_quarantine(partial)), flush=True)
    failures = 0
    while not partial.exists() or partial.stat().st_size < item['size']:
        offset = partial.stat().st_size if partial.exists() else 0
        end = min(item['size'] - 1, offset + chunk_size - 1)
        request = urllib.request.Request(item['url'], headers={
            'User-Agent': 'TG3-native-build/0.4', 'Accept-Encoding': 'identity',
            'Range': f'bytes={offset}-{end}',
        })
        try:
            with urllib.request.urlopen(request, timeout=timeout) as response:
                _allowed_url(response.geturl(), allow_local_http)
                matched = re.fullmatch(r'bytes (\d+)-(\d+)/(\d+)', response.headers.get('Content-Range', ''))
                if response.status != 206 or matched is None:
                    raise IntegrityError('server did not confirm resumable byte range; partial file retained')
                start, actual_end, total = map(int, matched.groups())
                if start != offset or not start <= actual_end <= end or total != item['size']:
                    raise IntegrityError('unexpected Content-Range; refused to append response')
                encoding = response.headers.get('Content-Encoding', 'identity')
                length = response.headers.get('Content-Length')
                remaining = actual_end - start + 1
                if encoding != 'identity' or (length is not None and int(length) != remaining):
                    raise IntegrityError('byte range encoding or length mismatch')
                with partial.open('ab') as output:
                    while remaining:
                        chunk = response.read(min(1024 * 1024, remaining))
                        if not chunk:
                            raise OSError('truncated range response; partial bytes preserved')
                        output.write(chunk)
                        remaining -= len(chunk)
            failures = 0
            progress(f"{item['id']}: {partial.stat().st_size / 1048576:.1f} / {item['size'] / 1048576:.1f} MiB", flush=True)
        except (OSError, http.client.HTTPException, urllib.error.URLError) as error:
            failures += 1
            if failures >= retries:
                raise RuntimeError('download interrupted; run the same command to resume: ' + item['id']) from error
            progress(f"{item['id']}: connection interrupted; preserving progress, retry {failures}/{retries}", flush=True)
            time.sleep(min(2., failures * .25))
    try:
        verify_archive(partial, item)
    except IntegrityError:
        preserved = _quarantine(partial)
        raise IntegrityError('completed download checksum mismatch; preserved at ' + str(preserved))
    # Both resolved absolute paths were checked to be inside this private cache.
    partial.replace(destination)
    return destination


def _member_path(sdk, name, roots):
    if not name or '\\' in name or '\x00' in name or name.startswith('/'):
        raise ValueError('unsafe archive path')
    parts = name.rstrip('/').split('/')
    reserved = {'CON', 'PRN', 'AUX', 'NUL', *(f'COM{i}' for i in range(1, 10)), *(f'LPT{i}' for i in range(1, 10))}
    if any(p in ('', '.', '..') or ':' in p or p.endswith((' ', '.'))
           or p.split('.')[0].upper() in reserved for p in parts):
        raise ValueError('unsafe archive path component')
    if parts[0] not in roots:
        raise ValueError('archive member is outside its declared package root')
    target = _contained(sdk, Path(sdk).joinpath(*parts))
    current = Path(sdk)
    for part in parts:
        current = current / part
        if current.is_symlink() or (hasattr(current, 'is_junction') and current.is_junction()):
            raise ValueError('existing links cannot redirect a tools installation')
    return target


def safe_extract(archive_path, sdk, roots, *, expected_uncompressed=None):
    """Validate the full zip before writing; never delete or move directories."""
    sdk = Path(sdk)
    if sdk.is_symlink() or (hasattr(sdk, 'is_junction') and sdk.is_junction()):
        raise ValueError('SDK extraction root must not be a link')
    sdk = sdk.resolve()
    sdk.mkdir(parents=True, exist_ok=True)
    with zipfile.ZipFile(archive_path) as archive:
        members = []
        entries = {}
        if expected_uncompressed is not None and sum(i.file_size for i in archive.infolist()) != expected_uncompressed:
            raise IntegrityError('archive uncompressed size differs from pinned manifest')
        for info in archive.infolist():
            mode = (info.external_attr >> 16) & 0o170000
            if mode not in (0, 0o100000, 0o040000):
                raise ValueError('archive contains links or special files')
            target = _member_path(sdk, info.orig_filename, roots)
            key = str(target).casefold()
            if key in entries:
                raise ValueError('duplicate or case-colliding archive path')
            entries[key] = info.is_dir()
            members.append((info, target))
        for info, target in members:
            if any(entries.get(str(parent).casefold()) is False for parent in target.parents if parent != sdk):
                raise ValueError('archive file conflicts with a parent directory')
        for info, target in members:
            if info.is_dir():
                target.mkdir(parents=True, exist_ok=True)
                continue
            target.parent.mkdir(parents=True, exist_ok=True)
            temporary = _contained(sdk, target.with_name(target.name + '.tg3-part'))
            if temporary.is_symlink() or (hasattr(temporary, 'is_junction') and temporary.is_junction()):
                raise ValueError('temporary extraction path is a link')
            with archive.open(info) as source, temporary.open('wb') as output:
                shutil.copyfileobj(source, output, length=1024 * 1024)
            # Reading through ZipFile.open verifies each entry's CRC first.
            temporary.replace(target)
    return len(members)


def _marker(sdk, item):
    return Path(sdk) / (item['id'].replace(';', '_') + '.verified.json')


def cache_ready(item, sdk):
    path = _marker(sdk, item)
    if not path.exists():
        return False
    try:
        value = json.loads(path.read_text(encoding='utf-8'))
        if value.get('sha256') != item['sha256'] or value.get('size') != item['size']:
            return False
        for name, expected in item['installed_files'].items():
            target = _member_path(sdk, name, item['allowed_roots'])
            if not target.is_file() or hashes(target)[1] != expected:
                return False
        return True
    except (OSError, ValueError):
        return False


@contextmanager
def package_lock(tools, item):
    folder = _private_child(tools, '.locks')
    folder.mkdir(parents=True, exist_ok=True)
    name = item['id'].replace(';', '_') + '.lock'
    with (folder / name).open('a+b') as stream:
        stream.seek(0, 2)
        if stream.tell() == 0:
            stream.write(b'0'); stream.flush()
        stream.seek(0)
        try:
            if os.name == 'nt':
                import msvcrt
                msvcrt.locking(stream.fileno(), msvcrt.LK_NBLCK, 1)
            else:
                import fcntl
                fcntl.flock(stream.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
        except OSError as error:
            raise RuntimeError('another bootstrap process is using this package cache') from error
        try:
            yield
        finally:
            stream.seek(0)
            if os.name == 'nt':
                msvcrt.locking(stream.fileno(), msvcrt.LK_UNLCK, 1)
            else:
                fcntl.flock(stream.fileno(), fcntl.LOCK_UN)


def install(item, tools, *, check_only=False):
    tools = Path(tools).resolve()
    sdk = _private_child(tools, 'android-sdk')
    if cache_ready(item, sdk):
        return dict(package=item['id'], status='verified_cache_reused', downloaded=False)
    if check_only:
        raise RuntimeError('missing or changed installed tool package: ' + item['id'])
    with package_lock(tools, item):
        if cache_ready(item, sdk):
            return dict(package=item['id'], status='verified_cache_reused', downloaded=False)
        archive = download(item, _private_child(tools, 'downloads'))
        count = safe_extract(archive, sdk, item['allowed_roots'], expected_uncompressed=item['uncompressed_size'])
        for name, digest in item['installed_files'].items():
            if hashes(_member_path(sdk, name, item['allowed_roots']))[1] != digest:
                raise IntegrityError('installed file checksum mismatch: ' + name)
        receipt = dict(name=item['id'], url=item['url'], size=item['size'], checksum=item['sha1'], sha256=item['sha256'],
                       files_extracted=count, anchors=item['installed_files'],
                       verified_at=datetime.now(timezone.utc).isoformat())
        marker = _marker(sdk, item)
        temporary = marker.with_suffix('.tmp')
        temporary.write_text(json.dumps(receipt, indent=2), encoding='utf-8')
        temporary.replace(marker)
        return dict(package=item['id'], status='download_verified_and_installed', files=count)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--tools-dir', type=Path, default=ROOT / '.headset_tools')
    parser.add_argument('--check-only', action='store_true', help='Verify a pre-existing private tool cache without downloads')
    parser.add_argument('--build', action='store_true', help='After bootstrap, compile and sign the native APK')
    args = parser.parse_args(argv)
    if os.name != 'nt' or platform.machine().lower() not in ('amd64', 'x86_64'):
        raise RuntimeError('This pinned tool manifest targets Windows x86_64; it must not install Windows NDK tools on Linux')
    javac = shutil.which('javac')
    if not javac:
        raise RuntimeError('JDK 17+ is required; javac was not found')
    version = subprocess.run([javac, '-version'], capture_output=True, text=True, check=True)
    match = re.search(r'javac\s+(\d+)', version.stdout + version.stderr)
    if not match or int(match.group(1)) < 17:
        raise RuntimeError('JDK 17+ is required; observed ' + version.stdout + version.stderr)
    manifest = json.loads((HERE / 'tools_manifest.json').read_text(encoding='utf-8'))
    if manifest['schema'] != 'tg3-android-tools-v1' or manifest['host'] != 'windows-x86_64':
        raise ValueError('unsupported tools manifest')
    results = [install(item, args.tools_dir, check_only=args.check_only) for item in manifest['packages']]
    report = dict(tools_dir=str(args.tools_dir.resolve()), jdk=version.stdout.strip() or version.stderr.strip(),
                  packages=results, global_system_modified=False, apk_build_executed=False, robot_commands_sent=False)
    if args.build:
        subprocess.run([sys.executable, str(HERE / 'build.py'), '--tools-dir', str(args.tools_dir.resolve())], check=True)
        report['apk_build_executed'] = True
    output = ROOT / 'artifacts/headset_tools_result.json'
    output.parent.mkdir(exist_ok=True)
    output.write_text(json.dumps(report, indent=2), encoding='utf-8')
    print(json.dumps(report, ensure_ascii=False, indent=2))
    return 0


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except (OSError, RuntimeError, ValueError, subprocess.CalledProcessError) as error:
        print('Native headset tools not ready: ' + str(error), file=sys.stderr)
        raise SystemExit(2)
