import hashlib
import importlib.util
import io
import json
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
import re
import socket
import tempfile
import threading
import unittest
from unittest.mock import patch
import zipfile

BOOTSTRAP=Path(__file__).resolve().parents[1]/'bootstrap.py'
spec=importlib.util.spec_from_file_location('tg3_native_bootstrap_test',BOOTSTRAP)
bootstrap=importlib.util.module_from_spec(spec);spec.loader.exec_module(bootstrap)


def archive_bytes(entries):
    output=io.BytesIO()
    with zipfile.ZipFile(output,'w',zipfile.ZIP_STORED) as archive:
        for name,value in entries:archive.writestr(name,value)
    return output.getvalue()


class RangeFixture:
    def __init__(self,data,*,drop=False,wrong_range=False,ignore_range=False):
        self.requests=[];owner=self;self.drop=drop
        class Handler(BaseHTTPRequestHandler):
            protocol_version='HTTP/1.1'
            def log_message(self,*args):pass
            def do_GET(self):
                start,end=map(int,re.fullmatch(r'bytes=(\d+)-(\d+)',self.headers['Range']).groups())
                owner.requests.append((start,end))
                if ignore_range:
                    self.send_response(200);self.send_header('Content-Length',str(len(data)));self.end_headers()
                    self.wfile.write(data);return
                end=min(end,len(data)-1)
                self.send_response(206)
                self.send_header('Content-Length',str(end-start+1))
                self.send_header('Content-Range',f'bytes {start+int(wrong_range)}-{end}/{len(data)}')
                self.end_headers()
                if owner.drop:
                    owner.drop=False
                    self.wfile.write(data[start:start+67]);self.wfile.flush()
                    self.connection.shutdown(socket.SHUT_RDWR);self.connection.close();return
                self.wfile.write(data[start:end+1])
        self.server=ThreadingHTTPServer(('127.0.0.1',0),Handler)
        self.thread=threading.Thread(target=self.server.serve_forever,daemon=True);self.thread.start()
        self.item=dict(id='fixture',archive='fixture.zip',url='http://127.0.0.1:'+str(self.server.server_address[1])+'/fixture.zip',
                      size=len(data),sha1=hashlib.sha1(data).hexdigest(),sha256=hashlib.sha256(data).hexdigest())

    def close(self):self.server.shutdown();self.server.server_close();self.thread.join(2)


class DownloadTests(unittest.TestCase):
    def make_fixture(self,**options):
        data=archive_bytes([('android-34/android.jar',bytes(range(256))*15)])
        fixture=RangeFixture(data,**options);self.addCleanup(fixture.close)
        return data,fixture

    def download(self,fixture,folder):
        return bootstrap.download(fixture.item,folder,chunk_size=256,retries=3,timeout=2,
            allow_local_http=True,progress=lambda *args,**kwargs:None)

    def test_existing_partial_file_resumes_from_exact_byte_then_verifies(self):
        data,fixture=self.make_fixture()
        with tempfile.TemporaryDirectory() as folder:
            Path(folder,'fixture.part').write_bytes(data[:91])
            result=self.download(fixture,folder)
            self.assertEqual(fixture.requests[0][0],91)
            self.assertEqual(result.read_bytes(),data)
            self.assertFalse(Path(folder,'fixture.part').exists())

    def test_dropped_real_http_range_preserves_progress_and_resumes(self):
        data,fixture=self.make_fixture(drop=True)
        with tempfile.TemporaryDirectory() as folder:
            result=self.download(fixture,folder)
            self.assertEqual(result.read_bytes(),data)
            self.assertEqual(fixture.requests[1][0],67)

    def test_wrong_range_cannot_append_to_partial_file(self):
        data,fixture=self.make_fixture(wrong_range=True)
        with tempfile.TemporaryDirectory() as folder:
            partial=Path(folder,'fixture.part');partial.write_bytes(data[:31])
            with self.assertRaises(bootstrap.IntegrityError):self.download(fixture,folder)
            self.assertEqual(partial.read_bytes(),data[:31])
            self.assertFalse(Path(folder,'fixture.zip').exists())

    def test_server_ignoring_range_cannot_duplicate_a_partial_prefix(self):
        data,fixture=self.make_fixture(ignore_range=True)
        with tempfile.TemporaryDirectory() as folder:
            partial=Path(folder,'fixture.part');partial.write_bytes(data[:31])
            with self.assertRaises(bootstrap.IntegrityError):self.download(fixture,folder)
            self.assertEqual(partial.read_bytes(),data[:31])

    def test_bad_checksum_is_quarantined_and_never_promoted_as_verified_archive(self):
        data,fixture=self.make_fixture();fixture.item['sha256']='0'*64
        with tempfile.TemporaryDirectory() as folder:
            with self.assertRaises(bootstrap.IntegrityError):self.download(fixture,folder)
            self.assertFalse(Path(folder,'fixture.zip').exists())
            preserved=list(Path(folder).glob('fixture.part.corrupt.*'))
            self.assertEqual(len(preserved),1)
            self.assertEqual(preserved[0].read_bytes(),data)

    def test_verified_complete_archive_is_reused_without_network(self):
        data,fixture=self.make_fixture()
        with tempfile.TemporaryDirectory() as folder:
            Path(folder,'fixture.zip').write_bytes(data)
            with patch.object(bootstrap.urllib.request,'urlopen',side_effect=AssertionError('network must not run')):
                result=self.download(fixture,folder)
            self.assertEqual(result.read_bytes(),data)


class ExtractionTests(unittest.TestCase):
    def test_valid_archive_extracts_files_and_counts_directories(self):
        with tempfile.TemporaryDirectory() as folder:
            root=Path(folder);archive=root/'fixture.zip'
            archive.write_bytes(archive_bytes([('android-34/',b''),('android-34/android.jar',b'valid SDK fixture')]))
            count=bootstrap.safe_extract(archive,root/'sdk',['android-34'],expected_uncompressed=17)
            self.assertEqual(count,2)
            self.assertEqual((root/'sdk/android-34/android.jar').read_bytes(),b'valid SDK fixture')

    def test_path_traversal_and_windows_special_names_rejected_before_any_write(self):
        for unsafe in ('../escape','/absolute','android-34/../../escape','android-34\\escape',
                       'C:/escape','android-34/NUL','android-34/.. /escape','android-34/file:stream',
                       'unrelated/file'):
            with self.subTest(path=unsafe),tempfile.TemporaryDirectory() as folder:
                root=Path(folder);archive=root/'fixture.zip'
                data=archive_bytes([('android-34/good',b'good'),(unsafe,b'bad')])
                if '\\' in unsafe:
                    # Windows ZipInfo normalizes separators when writing.
                    # Put the unsafe original name into both ZIP headers.
                    data=data.replace(unsafe.replace('\\','/').encode(),unsafe.encode())
                archive.write_bytes(data)
                with self.assertRaises(ValueError):bootstrap.safe_extract(archive,root/'sdk',['android-34'])
                self.assertFalse((root/'sdk/android-34/good').exists())
                self.assertFalse((root/'escape').exists())

    def test_symlink_entry_and_case_collisions_rejected_before_extract(self):
        link=zipfile.ZipInfo('android-34/link');link.create_system=3;link.external_attr=0o120777<<16
        for entries in ([('android-34/good',b'good'),(link,b'../outside')],
                        [('android-34/a',b'first'),('android-34/A',b'second')],
                        [('android-34/parent',b'file'),('android-34/parent/child',b'bad')]):
            with self.subTest(entries=str(entries)),tempfile.TemporaryDirectory() as folder:
                root=Path(folder);archive=root/'fixture.zip';archive.write_bytes(archive_bytes(entries))
                with self.assertRaises(ValueError):bootstrap.safe_extract(archive,root/'sdk',['android-34'])
                self.assertFalse((root/'sdk/android-34/good').exists())

    def test_bad_entry_crc_does_not_replace_a_previous_installed_file(self):
        name='android-34/android.jar'
        data=bytearray(archive_bytes([(name,b'known SDK content')]))
        data[30+len(name.encode())]^=1
        with tempfile.TemporaryDirectory() as folder:
            root=Path(folder);archive=root/'fixture.zip';archive.write_bytes(data)
            existing=root/'sdk'/name;existing.parent.mkdir(parents=True);existing.write_bytes(b'previous')
            with self.assertRaises(zipfile.BadZipFile):bootstrap.safe_extract(archive,root/'sdk',['android-34'])
            self.assertEqual(existing.read_bytes(),b'previous')

    def test_installed_cache_uses_fixed_anchor_hashes_and_does_not_download(self):
        content=b'verified tool'
        item=dict(id='platforms;android-34',size=123,sha256='a'*64,allowed_roots=['android-34'],
                  installed_files={'android-34/android.jar':hashlib.sha256(content).hexdigest()})
        with tempfile.TemporaryDirectory() as folder:
            root=Path(folder);sdk=root/'android-sdk';(sdk/'android-34').mkdir(parents=True)
            (sdk/'android-34/android.jar').write_bytes(content)
            (sdk/'platforms_android-34.verified.json').write_text(json.dumps(dict(size=123,sha256='a'*64)))
            with patch.object(bootstrap,'download',side_effect=AssertionError('network must not run')):
                result=bootstrap.install(item,root,check_only=True)
            self.assertEqual(result['status'],'verified_cache_reused')
            (sdk/'android-34/android.jar').write_bytes(b'changed')
            with self.assertRaises(RuntimeError):bootstrap.install(item,root,check_only=True)


if __name__=='__main__':unittest.main(verbosity=2)
