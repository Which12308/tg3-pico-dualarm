"""Versioned native Wi-Fi receiver and latest-only workstation relay.

Every hop uses a one-use receiver-clock challenge. Source and workstation
clocks are never subtracted. Pairing is local-LAN trust, not Internet TLS.
The server has no robot publishers, enable-motion endpoint or action queue.
"""
from collections import OrderedDict,deque
from dataclasses import replace
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import http.client
import ipaddress
import json
import math
import os
import secrets
import socket
import threading
import time
from .tracking import LatestTracking, TrackingFrame
from .input_policy import NORMAL

PROTOCOL = 'tg3-native-1'
COORDINATES = 'unity_lh_y_up'
BODY_LIMIT = 16384
# Read-only diagnostics include the discovered ROS graph and feedback metrics.
# Keep control/frame messages under the original cap and transport deadline.
STATUS_REPLY_LIMIT = 1024 * 1024


class ExclusiveInputServer(ThreadingHTTPServer):
    # Windows SO_REUSEADDR otherwise permits two desktops to share 8877,
    # leaving headset authentication/session selection nondeterministic.
    allow_reuse_address = os.name != 'nt'

    def server_bind(self):
        if os.name == 'nt':
            self.socket.setsockopt(socket.SOL_SOCKET,socket.SO_EXCLUSIVEADDRUSE,1)
        super().server_bind()


def parse_json(data):
    def unique(items):
        result={}
        for key,value in items:
            if key in result:raise ValueError('duplicate_json_field')
            result[key]=value
        return result
    def invalid_constant(value):raise ValueError('nonfinite_json_number')
    return json.loads(data,object_pairs_hook=unique,parse_constant=invalid_constant)


def canonical_frame(value, policy=NORMAL):
    """Unity XR local poses: reflect Z; quaternions map (-x,-y,z,w).

    These are Unity InputDevice poses, NOT the upstream raw PXR right-handed
    poses. Require the declaration so old/browser packets cannot mix axes.
    """
    if not isinstance(value, dict) or set(value) != {'protocol', 'coordinates', 'frame'}:
        raise ValueError('invalid_native_envelope')
    sources={COORDINATES:'xrobotoolkit','openxr_rh_y_up':'openxr'}
    if value['protocol'] != PROTOCOL or value['coordinates'] not in sources:
        raise ValueError('unsupported_native_protocol_or_coordinates')
    # Decode into independent containers. TrackingFrame then validates every
    # finite numeric value, range, required field and per-side validity flag.
    frame = json.loads(json.dumps(value['frame'], allow_nan=False))
    if frame.get('source') != sources[value['coordinates']]:
        raise ValueError('native_source_required')
    TrackingFrame.from_payload(frame,0.,policy)
    if value['coordinates']=='openxr_rh_y_up':return frame
    def pose(item):
        if item is None:
            return
        p, q = item['position'], item['orientation_xyzw']
        if len(p) != 3 or len(q) != 4:
            raise ValueError('invalid_native_pose')
        item['position'] = [p[0], p[1], -p[2]]
        item['orientation_xyzw'] = [-q[0], -q[1], q[2], q[3]]
    pose(frame['head'])
    for side in ('left', 'right'):
        pose(frame[side]['pose'])
    return frame


class InputGateway:
    def __init__(self, *, role='headset', key=None, clock=time.monotonic, on_frame=None, policy=NORMAL):
        if role not in ('headset', 'relay'):
            raise ValueError('invalid gateway role')
        if role == 'relay' and (not isinstance(key, str) or len(key) < 32):
            raise ValueError('relay requires an independent strong key')
        self.role, self.clock = role, clock
        self.tracking = LatestTracking(clock=clock, policy=policy)
        self.lock = threading.RLock()
        self.token = key if role == 'relay' else None
        self.pin = None
        self.pair_until = 0.
        self.challenges = OrderedDict()
        self.pair_failures = 0
        self.peer = None
        self.runtime = {}
        self.response_disconnects = 0
        self.accept_frames=True
        self.headset_gateway=None
        self.headset_pairing_allowed=lambda:False
        self.recording_token=None
        self.recording_frames=deque();self.recording_events=deque();self.recording_fault=None
        self.recording_capture_active=False
        self.on_frame = on_frame
        self.control_callback=None

    def open_pairing(self):
        if self.role != 'headset':
            raise ValueError('relay has no headset pairing')
        with self.lock:
            self.tracking.invalidate('new_pairing_requires_release')
            self.token = None
            self.pin = f'{secrets.randbelow(100000000):08d}'
            self.pair_until = self.clock() + 120
            self.pair_failures = 0
            self.challenges.clear()
            return self.pin

    def pair(self, pin, device):
        with self.lock:
            if self.role != 'headset' or self.pin is None or self.clock() > self.pair_until:
                raise ValueError('pairing_closed')
            if self.pair_failures >= 8:
                raise ValueError('pairing_attempt_limit_reopen_locally')
            if not isinstance(pin, str) or not secrets.compare_digest(pin, self.pin):
                self.pair_failures += 1
                raise ValueError('incorrect_pairing_code')
            if not isinstance(device, str) or not 1 <= len(device) <= 128:
                raise ValueError('invalid_device')
            self.token = secrets.token_urlsafe(32)
            self.pin = None
            self.peer = device
            self.challenges.clear()
            self.tracking.invalidate('paired_release_both_inputs')
            return self.token

    def authenticate(self, token):
        if not isinstance(token, str) or not self.token or not secrets.compare_digest(token, self.token):
            raise ValueError('authentication_required')

    def challenge(self, token):
        with self.lock:
            self.authenticate(token)
            now = self.clock()
            self.challenges = OrderedDict((k, t) for k, t in self.challenges.items() if now-t <= .12)
            if len(self.challenges) >= 8:
                raise ValueError('too_many_pending_frames')
            nonce = secrets.token_urlsafe(24)
            self.challenges[nonce] = now
            return dict(protocol=PROTOCOL, nonce=nonce, expires_ms=120)

    def accept(self, token, nonce, value):
        with self.lock:
            self.authenticate(token)
            if not self.accept_frames:raise ValueError('robot_usb_is_the_exclusive_tracking_source')
            issued = self.challenges.pop(nonce, None) if isinstance(nonce, str) else None
            now = self.clock()
            if issued is None or not 0 <= now-issued <= .12:
                self.tracking.invalidate('missing_or_expired_transport_challenge')
                raise ValueError('missing_or_expired_transport_challenge')
            try:
                if self.role == 'headset':
                    frame = canonical_frame(value,self.tracking.policy)
                else:
                    if not isinstance(value, dict) or set(value) != {'protocol', 'frame'} or value['protocol'] != PROTOCOL:
                        raise ValueError('invalid_relay_envelope')
                    frame = value['frame']
                result = self.tracking.accept(frame, now, additional_age_s=now-issued)
                if result is None:
                    raise ValueError(self.tracking.last_error)
                if self.on_frame:
                    self.on_frame(result, value)
                if self.recording_capture_active and not self.recording_fault:
                    if len(self.recording_frames)>=128 or (self.recording_frames and now-self.recording_frames[0][0]>1.):
                        self.recording_fault='recording_input_backlog';self.recording_frames.clear()
                    else:self.recording_frames.append((now,result))
                return result
            except (ValueError, TypeError, KeyError, OverflowError, OSError) as exc:
                self.tracking.invalidate('native_frame_rejected')
                raise ValueError(str(exc)) from exc

    def status(self):
        with self.lock:
            return dict(protocol=PROTOCOL, role=self.role, paired=bool(self.token),
                        pairing_open=bool(self.pin and self.clock() <= self.pair_until),
                        tracking=self.tracking.status(), runtime=self.runtime.copy(),
                        transport=dict(response_disconnects=self.response_disconnects),
                        hardware_motion_enabled=bool(self.runtime.get('readiness',{}).get('hardware_motion_enabled',False)))

    def request_control(self,token,nonce,value):
        with self.lock:
            self.authenticate(token)
            issued=self.challenges.pop(nonce,None) if isinstance(nonce,str) else None
            if issued is None or not 0<=self.clock()-issued<=.12:raise ValueError('expired_control_challenge')
            if self.role!='relay' or self.control_callback is None:raise ValueError('hardware_control_not_configured')
            if (not isinstance(value,dict) or set(value)!={'protocol','operation','session'}
                    or value['protocol']!=PROTOCOL or value['operation'] not in ('enable','disable','return_home')
                    or (value['session'] is not None and not isinstance(value['session'],str))):
                raise ValueError('invalid_control_request')
            result=self.control_callback(dict(value,requested_at=self.clock()))
            if self.recording_capture_active and not self.recording_fault:
                if len(self.recording_events)>=128:self.recording_fault='recording_event_backlog'
                else:self.recording_events.append(dict(operation=value['operation'],timestamp_ns=time.time_ns(),result=result))
            return result

    def reopen_headset_pairing(self,token,nonce,value):
        with self.lock:
            self.authenticate(token)
            issued=self.challenges.pop(nonce,None) if isinstance(nonce,str) else None
            if issued is None or not 0<=self.clock()-issued<=.12:raise ValueError('expired_pairing_challenge')
            if self.role!='relay' or self.headset_gateway is None:raise ValueError('robot_usb_headset_not_configured')
            if value!={'protocol':PROTOCOL}:raise ValueError('invalid_headset_pairing_request')
            if not self.headset_pairing_allowed():raise ValueError('disable_motion_before_headset_pairing')
            return dict(pin=self.headset_gateway.open_pairing(),expires_ms=120000,transport='robot_usb')

    def recording_access(self,token,nonce,value):
        with self.lock:
            self.authenticate(token)
            issued=self.challenges.pop(nonce,None) if isinstance(nonce,str) else None
            if issued is None or not 0<=self.clock()-issued<=.12:raise ValueError('expired_recording_request')
            if self.role!='relay' or value not in ({'protocol':PROTOCOL,'operation':'start'},
                                                  {'protocol':PROTOCOL,'operation':'stop'}):
                raise ValueError('invalid_recording_access')
            self.recording_token=secrets.token_urlsafe(32) if value['operation']=='start' else None
            self.recording_frames.clear();self.recording_events.clear();self.recording_fault=None
            self.recording_capture_active=False
            return dict(token=self.recording_token,scope='read_only_recording')

    def recording_sample(self,token):
        with self.lock:
            if not self.recording_token or not isinstance(token,str) or not secrets.compare_digest(token,self.recording_token):
                raise ValueError('recording_authentication_required')
            if self.recording_fault:raise ValueError(self.recording_fault)
            self.recording_capture_active=True
            frame=self.tracking.sample()
            frames=[dict(received_at=stamp,frame=frame.to_payload()) for stamp,frame in self.recording_frames]
            events=list(self.recording_events);self.recording_frames.clear();self.recording_events.clear()
            return dict(timestamp_ns=time.time_ns(),monotonic_s=self.clock(),tracking_frames=frames,events=events,
                tracking=frame.to_payload() if frame else None,
                control=self.runtime.get('control',{}),actual_feedback=self.runtime.get('actual_feedback',{}),
                hands=self.runtime.get('manufacturer_feedback',{}).get('hands',{}))

    def recording_end(self,token):
        with self.lock:
            if not self.recording_token or not secrets.compare_digest(str(token),self.recording_token):
                raise ValueError('recording_authentication_required')
            fault=self.recording_fault
            final=None if fault else self.recording_sample(token)
            self.recording_token=None;self.recording_capture_active=False
            self.recording_frames.clear();self.recording_events.clear()
            return dict(ended=True,final_sample=final,error=fault)


class WirelessServer:
    """Bind explicitly to LAN or loopback; pairing can only be opened locally."""
    def __init__(self, gateway, host='0.0.0.0', port=8877):
        self.gateway = gateway
        outer = self
        class Handler(BaseHTTPRequestHandler):
            protocol_version = 'HTTP/1.1'
            def setup(self):
                super().setup()
                self.connection.setsockopt(socket.IPPROTO_TCP,socket.TCP_NODELAY,1)
            def log_message(self, *args):
                pass  # Never log pairing codes, bearer tokens or packet bodies.
            def handle(self):
                try:super().handle()
                except (ConnectionResetError,BrokenPipeError,TimeoutError):
                    with gateway.lock:gateway.response_disconnects+=1
                    self.close_connection=True
            def reply(self, code, value):
                data = json.dumps(value, allow_nan=False).encode()
                try:
                    self.send_response(code)
                    self.send_header('Content-Type', 'application/json')
                    self.send_header('Content-Length', str(len(data)))
                    self.send_header('Cache-Control', 'no-store')
                    # ADB/SSH may forward the two writes through a different
                    # socket whose Nagle setting we do not own. Keep each
                    # small JSON response in one write, as for requests.
                    self._headers_buffer.append(b'\r\n')
                    packet=b''.join(self._headers_buffer)+data
                    self._headers_buffer.clear()
                    self.wfile.write(packet)
                except (BrokenPipeError, ConnectionResetError, TimeoutError):
                    # A client abandoning a reply is an ordinary link loss.
                    # Keep it observable without repeated traceback output.
                    # A disconnected status reader has no input authority.
                    with gateway.lock:gateway.response_disconnects += 1
                    self.close_connection = True
            def token(self):
                auth = self.headers.get('Authorization', '')
                return auth[7:] if auth.startswith('Bearer ') else ''
            def do_GET(self):
                self.connection.settimeout(1.)
                if self.path == '/v1/info':
                    return self.reply(200, dict(protocol=PROTOCOL, role=gateway.role,
                        version=__import__('tg3_pico_dualarm').__version__,
                        hardware_motion_enabled=gateway.status()['hardware_motion_enabled']))
                try:
                    if self.path=='/v1/recording/sample':
                        return self.reply(200,gateway.recording_sample(self.token()))
                    if self.path == '/v1/challenge':
                        return self.reply(200, gateway.challenge(self.token()))
                    gateway.authenticate(self.token())
                    if self.path == '/v1/status':
                        return self.reply(200, gateway.status())
                    self.reply(404, {'error': 'unknown_endpoint'})
                except ValueError as exc:
                    self.reply(403, {'error': str(exc)})
            def do_POST(self):
                self.connection.settimeout(1.)
                try:
                    # No browser cross-origin LAN writes or chunked unbounded input.
                    if self.headers.get('Origin') or self.headers.get('Transfer-Encoding'):
                        raise ValueError('native_client_required')
                    size = int(self.headers.get('Content-Length', '0'))
                    if not 0 < size <= BODY_LIMIT:
                        raise ValueError('invalid_content_length')
                    value = parse_json(self.rfile.read(size))
                    if self.path == '/v1/pair':
                        if set(value) != {'pin', 'device', 'protocol'} or value['protocol'] != PROTOCOL:
                            raise ValueError('invalid_pairing_request')
                        token = gateway.pair(value['pin'], value['device'])
                        return self.reply(200, dict(token=token, protocol=PROTOCOL))
                    if self.path == '/v1/headset/pairing':
                        return self.reply(200,gateway.reopen_headset_pairing(
                            self.token(),self.headers.get('X-TG3-Nonce'),value))
                    if self.path == '/v1/recording/access':
                        return self.reply(200,gateway.recording_access(
                            self.token(),self.headers.get('X-TG3-Nonce'),value))
                    if self.path == '/v1/recording/end':
                        if value!={'protocol':PROTOCOL}:raise ValueError('invalid_recording_end')
                        return self.reply(200,gateway.recording_end(self.token()))
                    if self.path == '/v1/frame':
                        frame = gateway.accept(self.token(), self.headers.get('X-TG3-Nonce'), value)
                        reply=dict(accepted=True, sequence=frame.sequence)
                        if gateway.role=='relay' and self.headers.get('X-TG3-Next-Challenge')=='1':
                            # Return the next one-use receiver-clock challenge
                            # with the ACK: no extra round trip per relay frame.
                            reply['next_challenge']=gateway.challenge(self.token())
                        return self.reply(200, reply)
                    if self.path == '/v1/control':
                        return self.reply(200,gateway.request_control(self.token(),self.headers.get('X-TG3-Nonce'),value))
                    self.reply(404, {'error': 'unknown_endpoint'})
                except (ValueError, TypeError, KeyError, OSError) as exc:
                    self.close_connection = True
                    self.reply(400, {'error': str(exc)})
        self.httpd = ExclusiveInputServer((host, port), Handler)
        self.httpd.daemon_threads = True
        self.thread = None

    @property
    def address(self):
        return self.httpd.server_address

    def start(self):
        self.thread = threading.Thread(target=self.httpd.serve_forever, daemon=True)
        self.thread.start()
        return self

    def close(self):
        with self.gateway.lock:
            self.gateway.token=None
            self.gateway.pin=None
            self.gateway.challenges.clear()
            self.gateway.recording_token=None
            self.gateway.tracking.invalidate('receiver_stopped')
        if self.thread:
            self.httpd.shutdown()
            self.thread.join(2)
        self.httpd.server_close()


class _PackedHTTPConnection(http.client.HTTPConnection):
    """Keep a small JSON request together across SSH's TCP forwarding hop.

    Separate header/body writes can trigger Nagle + delayed ACK at the
    forwarding socket, even when both HTTP endpoints disable Nagle.
    Streaming/chunked bodies retain the standard library implementation.
    """
    def _send_output(self,message_body=None,encode_chunked=False):
        if isinstance(message_body,bytes) and not encode_chunked:
            self._buffer.extend((b'',b''))
            packet=b'\r\n'.join(self._buffer)+message_body
            self._buffer.clear()
            self.send(packet)
        else:super()._send_output(message_body,encode_chunked=encode_chunked)


class GatewayClient:
    def __init__(self, host, port, token='', timeout=.12, *, cache_challenge=True):
        self.connection = _PackedHTTPConnection(host, port, timeout=timeout)
        self.token = token
        self._next_challenge=None
        self.cache_challenge=cache_challenge is True

    def request(self, method, path, value=None, nonce=None, *, next_challenge=False):
        headers = {'Content-Type': 'application/json', 'Authorization': 'Bearer '+self.token}
        if next_challenge:headers['X-TG3-Next-Challenge']='1'
        if nonce:
            headers['X-TG3-Nonce'] = nonce
        data = json.dumps(value, allow_nan=False).encode('utf-8') if value is not None else None
        self.connection.request(method, path, data, headers)
        result = self.connection.getresponse()
        limit = STATUS_REPLY_LIMIT if ((method == 'GET' and path in ('/v1/status','/v1/recording/sample'))
            or (method == 'POST' and path=='/v1/recording/end')) else BODY_LIMIT
        body = result.read(limit+1)
        if len(body) > limit:
            self.close()  # Do not reuse a connection with an unread response.
            raise ValueError('oversize_server_reply')
        value = json.loads(body)
        if result.status != 200:
            raise ValueError(value.get('error', 'gateway_request_failed'))
        return value

    def send(self, value, *, on_challenge=None):
        challenge=self._next_challenge if self.cache_challenge else None
        self._next_challenge=None
        if challenge is None:challenge = self.request('GET', '/v1/challenge')
        # Relay obtains the latest frame AFTER requesting a challenge, rather
        # than forwarding a queued sample captured before a network stall.
        if on_challenge:
            value = on_challenge()
        reply=self.request('POST', '/v1/frame', value, nonce=challenge['nonce'],next_challenge=self.cache_challenge)
        if self.cache_challenge:self._next_challenge=reply.get('next_challenge')
        return reply

    def control(self,operation,session):
        challenge=self.request('GET','/v1/challenge')
        return self.request('POST','/v1/control',dict(protocol=PROTOCOL,operation=operation,session=session),nonce=challenge['nonce'])

    def reopen_headset_pairing(self):
        challenge=self.request('GET','/v1/challenge')
        return self.request('POST','/v1/headset/pairing',{'protocol':PROTOCOL},nonce=challenge['nonce'])

    def recording_access(self,operation):
        challenge=self.request('GET','/v1/challenge')
        return self.request('POST','/v1/recording/access',{'protocol':PROTOCOL,'operation':operation},nonce=challenge['nonce'])

    def close(self):
        self._next_challenge=None
        self.connection.close()


class LatestRelay:
    def __init__(self, tracking, client, clock=time.monotonic):
        self.tracking, self.client, self.clock = tracking, client, clock
        self.stop = threading.Event()
        self.last = None
        self.status = dict(state='waiting_for_input', forwarded=0)
        self.last_payload_age=None
        self.thread = None

    def step(self):
        current_frame = self.tracking.sample(self.clock())
        if current_frame is None:
            self.status=dict(state='waiting_for_input',forwarded=self.status['forwarded'])
            return
        if not current_frame.focused:
            self.status=dict(state='input_paused_or_expired',forwarded=self.status['forwarded'])
        if (current_frame.session,current_frame.sequence) == self.last:
            return
        def payload():
            now = self.clock()
            frame = self.tracking.sample(now)
            if frame is None:
                raise ValueError('no_input')
            age = frame.source_age_s + now-frame.received_at
            if not 0 <= age <= self.tracking.policy.source_age_s:
                raise ValueError('stale_relay_input')
            self.last_payload_age=age
            current = frame.session, frame.sequence
            if current == self.last:
                raise ValueError('no_new_frame')
            frame=replace(frame,source_age_s=age)
            for side in ('left','right'):
                controller=getattr(frame,side)
                if controller.release_required:
                    frame=replace(frame,**{side:replace(controller,tracked=False,pose=None)})
            value = frame.to_input_payload()
            self.last = current
            return dict(protocol=PROTOCOL, frame=value)
        started=self.clock();self.last_payload_age=None
        try:
            self.client.send(None, on_challenge=payload)
            self.status = dict(state='forwarding', forwarded=self.status['forwarded']+1)
        except (OSError, http.client.HTTPException) as exc:
            self.tracking.invalidate('relay_network_reset_release_inputs')
            self.client.close()
            self.status = dict(state='waiting_or_disconnected', reason=str(exc), forwarded=self.status['forwarded'])

        except ValueError as exc:
            self.client.close()
            if str(exc) not in ('no_input','no_new_frame','stale_relay_input'):
                self.tracking.invalidate('relay_rejected_release_inputs')
            self.status = dict(state='waiting_or_disconnected', reason=str(exc), forwarded=self.status['forwarded'])
        finally:
            self.status['transfer_ms']=max(0.,self.clock()-started)*1000
            self.status['input_age_at_send_ms']=None if self.last_payload_age is None else self.last_payload_age*1000

    def start(self):
        def run():
            while not self.stop.is_set():
                self.step()
                self.stop.wait(.01)
        self.thread = threading.Thread(target=run, daemon=True)
        self.thread.start()
        return self

    def close(self):
        self.stop.set()
        if self.thread:
            self.thread.join(2)
        self.client.close()
