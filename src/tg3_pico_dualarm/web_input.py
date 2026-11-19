"""Loopback WebXR acquisition service; no robot SDK, ROS, or actuator output."""
from __future__ import annotations

import argparse
import ipaddress
import json
from pathlib import Path
import secrets
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlsplit

from .tracking import LatestTracking, strict_json

WEB_ROOT = Path(__file__).resolve().parent / 'web_assets'
MAX_BODY = 16384
NONCE_TTL_S = .120


class InputServer:
    def __init__(self, host: str = '127.0.0.1', port: int = 8890, *,
                 tracking: LatestTracking | None = None, clock=time.monotonic,
                 web_root: str | Path = WEB_ROOT):
        if not ipaddress.ip_address(host).is_loopback:
            raise ValueError('headset input server must bind a loopback address')
        if type(port) is not int or not 0 <= port <= 65535:
            raise ValueError('invalid input port')
        self.clock, self.tracking = clock, tracking if tracking is not None else LatestTracking(clock)
        self.web_root = Path(web_root).resolve()
        self.lock = threading.Lock()
        self.challenges = {}
        self.runtime_status = {'robot_commands_sent': False, 'state': 'waiting_for_controller'}
        owner = self

        class Handler(BaseHTTPRequestHandler):
            server_version = 'PicoDualArmInput/1'
            def log_message(self, *_args):
                pass

            def valid_origin(self):
                def local_url(value):
                    try:
                        parsed = urlsplit(value)
                        return (parsed.scheme == 'http' and parsed.hostname in ('localhost', '127.0.0.1', '::1')
                                and parsed.port == owner.server_address[1] and not parsed.username and not parsed.password)
                    except (TypeError, ValueError):
                        return False
                host = self.headers.get('Host', '')
                if not local_url('http://' + host):
                    return False
                origin = self.headers.get('Origin')
                if origin is not None and not local_url(origin):
                    return False
                return self.headers.get('Sec-Fetch-Site', 'same-origin') in ('same-origin', 'none')

            def reply(self, status, value, mime='application/json; charset=utf-8'):
                if isinstance(value, bytes):
                    data = value
                else:
                    data = json.dumps(value, ensure_ascii=False, allow_nan=False).encode('utf-8')
                self.send_response(status)
                self.send_header('Content-Type', mime)
                self.send_header('Content-Length', str(len(data)))
                self.send_header('Cache-Control', 'no-store')
                self.send_header('X-Content-Type-Options', 'nosniff')
                self.send_header('Referrer-Policy', 'no-referrer')
                self.send_header('Permissions-Policy', 'xr-spatial-tracking=(self)')
                self.send_header('Content-Security-Policy', "default-src 'self'; connect-src 'self'; script-src 'self'; style-src 'self'; img-src 'self' data:; object-src 'none'; base-uri 'none'; frame-ancestors 'none'")
                self.end_headers()
                self.wfile.write(data)

            def do_GET(self):
                if not self.valid_origin():
                    return self.reply(403, {'error': 'loopback origin required'})
                if self.path == '/api/challenge':
                    try:
                        return self.reply(200, {'nonce': owner.challenge(), 'expires_in_s': NONCE_TTL_S})
                    except ValueError as error:
                        return self.reply(429, {'error': str(error)})
                if self.path == '/api/status':
                    status = owner.tracking.status()
                    status['source'] = 'webxr'
                    status['address'] = list(owner.server_address)
                    with owner.lock:
                        status['runtime'] = json.loads(json.dumps(owner.runtime_status))
                    return self.reply(200, status)
                if self.path == '/api/frame':
                    frame = owner.tracking.sample()
                    return self.reply(200, {'frame': None if frame is None else frame.to_payload(),
                                           'robot_commands_sent': False})
                files = {'/': ('index.html', 'text/html; charset=utf-8'),
                         '/app.js': ('app.js', 'text/javascript; charset=utf-8'),
                         '/style.css': ('style.css', 'text/css; charset=utf-8')}
                item = files.get(self.path)
                if item is None:
                    return self.reply(404, {'error': 'not found'})
                try:
                    return self.reply(200, (owner.web_root/item[0]).read_bytes(), item[1])
                except OSError:
                    return self.reply(404, {'error': 'web assets not installed'})

            def do_POST(self):
                if not self.valid_origin():
                    return self.reply(403, {'error': 'loopback origin required'})
                if self.path != '/api/frame':
                    return self.reply(404, {'error': 'not found'})
                if self.headers.get('Content-Type', '').split(';')[0].strip().lower() != 'application/json':
                    return self.reply(415, {'error': 'application/json required'})
                size = self.headers.get('Content-Length', '')
                if not size.isdecimal() or not 1 <= int(size) <= MAX_BODY:
                    return self.reply(413, {'error': 'invalid request size'})
                try:
                    data = strict_json(self.rfile.read(int(size)))
                except (ValueError, UnicodeError):
                    return self.reply(400, {'error': 'invalid JSON'})
                if not isinstance(data, dict):
                    return self.reply(400, {'error': 'frame must be an object'})
                nonce = data.pop('nonce', None)
                nonce_age = owner.consume(nonce)
                if nonce_age is None:
                    # A page without a live nonce cannot revoke another page's
                    # valid stream; expiry covers interrupted authorized senders.
                    return self.reply(409, {'error': 'expired or reused challenge'})
                frame = owner.tracking.accept(data, additional_age_s=nonce_age)
                if frame is None:
                    return self.reply(422, {'error': owner.tracking.last_error, 'next_nonce': owner.challenge()})
                return self.reply(200, {'accepted': True, 'sequence': frame.sequence,
                    'next_nonce': owner.challenge(),
                    'left_release_required': frame.left.release_required,
                    'right_release_required': frame.right.release_required})

        self.httpd = ThreadingHTTPServer((host, port), Handler)
        self.httpd.daemon_threads = True
        self.server_address = self.httpd.server_address

    def set_runtime_status(self, value: dict):
        if not isinstance(value, dict):
            raise ValueError('runtime status must be an object')
        serialized = json.dumps(value, ensure_ascii=False, allow_nan=False)
        if len(serialized.encode('utf-8')) > 65536:
            raise ValueError('runtime status too large')
        with self.lock:
            self.runtime_status = json.loads(serialized)

    def challenge(self) -> str:
        with self.lock:
            now = self.clock()
            self.challenges = {nonce: stamp for nonce, stamp in self.challenges.items()
                               if 0 <= now-stamp <= NONCE_TTL_S}
            if len(self.challenges) >= 128:
                raise ValueError('too many pending input challenges')
            nonce = secrets.token_hex(16)
            self.challenges[nonce] = now
            return nonce

    def consume(self, nonce) -> float | None:
        with self.lock:
            issued = self.challenges.pop(nonce, None) if isinstance(nonce, str) else None
            age = None if issued is None else self.clock()-issued
            return age if age is not None and 0 <= age <= NONCE_TTL_S else None

    def serve_forever(self):
        self.httpd.serve_forever(poll_interval=.1)

    def shutdown(self):
        self.httpd.shutdown()
        self.httpd.server_close()
        self.tracking.invalidate('input_server_stopped')


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--host', default='127.0.0.1')
    parser.add_argument('--port', type=int, default=8890)
    args = parser.parse_args(argv)
    server = InputServer(args.host, args.port)
    print(f'双手柄输入页面：http://localhost:{server.server_address[1]}；此服务不控制机器人。', flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        server.tracking.invalidate('input_server_stopped')
        server.httpd.server_close()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
