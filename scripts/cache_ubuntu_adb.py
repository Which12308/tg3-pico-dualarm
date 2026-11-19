"""Download Ubuntu Noble amd64 ADB dependencies; no installation or device access."""
import argparse,hashlib,json,lzma,re,urllib.request
from pathlib import Path

BASE='https://archive.ubuntu.com/ubuntu/'
ROOT=Path(__file__).resolve().parents[1]
SYSTEM_BASE={'libc6','libstdc++6','libgcc-s1','libusb-1.0-0','libzstd1','zlib1g','libssl3t64','libudev1','libsystemd0'}

def download(url):
    with urllib.request.urlopen(url,timeout=90) as source:return source.read(100*1024*1024)

def paragraphs(text):
    for block in text.split('\n\n'):
        fields={};key=None
        for line in block.splitlines():
            if line.startswith(' ') and key:fields[key]+=' '+line.strip()
            elif ': ' in line:key,value=line.split(': ',1);fields[key]=value
        if 'Package' in fields:yield fields

def main():
    parser=argparse.ArgumentParser();parser.add_argument('--cache',type=Path,default=ROOT/'.desktop/ubuntu_adb_noble_amd64');args=parser.parse_args()
    args.cache.mkdir(parents=True,exist_ok=True)
    release=download(BASE+'dists/noble/InRelease').decode();packages={};indices=[];provides={}
    for component in ('main','universe'):
        relative=component+'/binary-amd64/Packages.xz'
        match=re.search(r'^ ([a-f0-9]{64})\s+\d+\s+'+re.escape(relative)+r'$',release,re.M)
        if not match:raise RuntimeError('Ubuntu_release_index_hash_missing')
        cached=args.cache/(component+'_Packages.xz')
        raw=cached.read_bytes() if cached.exists() else download(BASE+'dists/noble/'+relative)
        if hashlib.sha256(raw).hexdigest()!=match[1]:raise RuntimeError('Ubuntu_index_hash_mismatch')
        if not cached.exists():cached.write_bytes(raw)
        indices.append(dict(path=relative,sha256=match[1]))
        for fields in paragraphs(lzma.decompress(raw).decode()):
            packages[fields['Package']]=fields
            for term in fields.get('Provides','').split(','):
                if term.strip():provides[re.split(r'[ (:]',term.strip())[0]]=fields['Package']
    chosen={};pending=['adb']
    while pending:
        name=pending.pop()
        if name in chosen or name in SYSTEM_BASE:continue
        item=packages[name];chosen[name]=item
        for term in item.get('Depends','').split(','):
            if not term.strip():continue
            # First supported archive alternative; apt performs final compatibility checks.
            options=[re.split(r'[ (:]',part.strip())[0] for part in term.split('|')]
            candidate=next((v if v in packages or v in SYSTEM_BASE else provides.get(v) for v in options
                if v in packages or v in SYSTEM_BASE or v in provides),None)
            if candidate is None:raise RuntimeError('unsupported_adb_dependency:'+term)
            pending.append(candidate)
    output=[]
    for name,item in sorted(chosen.items()):
        relative=item['Filename'];filename=Path(relative).name;path=args.cache/filename
        if not re.fullmatch(r'[A-Za-z0-9_.+~%-]+\.deb',filename):raise ValueError('invalid_package_filename')
        raw=path.read_bytes() if path.exists() else download(BASE+relative)
        if hashlib.sha256(raw).hexdigest()!=item['SHA256'] or len(raw)!=int(item['Size']):raise RuntimeError('Ubuntu_deb_hash_mismatch:'+name)
        if not raw.startswith(b'!<arch>\n'):raise RuntimeError('invalid_deb_archive')
        if not path.exists():path.write_bytes(raw)
        output.append(dict(package=name,version=item['Version'],file=filename,sha256=item['SHA256'],
            bytes=len(raw),url=BASE+relative,depends=item.get('Depends','')))
    manifest=dict(schema='tg3_ubuntu_adb_cache_v1',distribution='ubuntu',version='24.04',architecture='amd64',
        index_sources=indices,inrelease_sha256=hashlib.sha256(release.encode()).hexdigest(),
        verification='Ubuntu HTTPS InRelease index hashes and package hashes; no independent GPG verification',
        base_dependencies_checked_by_target_apt=sorted(SYSTEM_BASE),packages=output)
    (args.cache/'manifest.json').write_text(json.dumps(manifest,indent=2),encoding='utf-8')
    print(json.dumps(dict(cached=True,packages=[v['package'] for v in output],bytes=sum(v['bytes'] for v in output),directory=str(args.cache))))

if __name__=='__main__':main()
