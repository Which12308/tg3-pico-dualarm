import copy
import json
from pathlib import Path
import shutil
import subprocess
import threading
import unittest
import urllib.error
import urllib.request

from tg3_pico_dualarm.web_input import InputServer, WEB_ROOT
from test_tracking import payload


class WebInputTests(unittest.TestCase):
    def setUp(self):
        self.now = 5.
        self.server = InputServer(port=0, clock=lambda: self.now)
        self.thread = threading.Thread(target=self.server.serve_forever, daemon=True)
        self.thread.start()
        self.base = f'http://127.0.0.1:{self.server.server_address[1]}'

    def tearDown(self):
        self.server.shutdown()
        self.thread.join(timeout=2)

    def get(self, path, headers=None):
        request = urllib.request.Request(self.base+path, headers=headers or {})
        try:
            with urllib.request.urlopen(request, timeout=2) as response:
                return response.status, response.read(), dict(response.headers)
        except urllib.error.HTTPError as error:
            return error.code, error.read(), dict(error.headers)

    def send(self, frame, *, nonce=None, headers=None):
        if nonce is None:
            _, raw, _ = self.get('/api/challenge')
            nonce = json.loads(raw)['nonce']
        value = copy.deepcopy(frame)
        value['nonce'] = nonce
        request = urllib.request.Request(self.base+'/api/frame', data=json.dumps(value).encode(),
            headers={'Content-Type':'application/json', **(headers or {})}, method='POST')
        try:
            with urllib.request.urlopen(request, timeout=2) as response:
                return response.status, json.load(response), nonce
        except urllib.error.HTTPError as error:
            return error.code, json.load(error), nonce

    def test_actual_http_receives_head_and_two_hands_without_robot_interface(self):
        status, reply, _ = self.send(payload())
        self.assertEqual(status, 200)
        self.assertTrue(reply['accepted'])
        frame = self.server.tracking.sample()
        self.assertTrue(frame.left.tracked and frame.right.tracked)
        self.assertIsNotNone(frame.head)
        code, raw, _ = self.get('/api/status')
        self.assertEqual(code, 200)
        result = json.loads(raw)
        self.assertFalse(result['robot_commands_sent'])
        self.assertTrue(result['left_tracked'])
        self.assertEqual(result['runtime']['state'], 'waiting_for_controller')

    def test_reused_or_expired_nonce_is_rejected(self):
        _, _, nonce = self.send(payload())
        self.assertEqual(self.send(payload(2), nonce=nonce)[0], 409)
        _, raw, _ = self.get('/api/challenge')
        nonce = json.loads(raw)['nonce']
        self.now += .121
        self.assertEqual(self.send(payload(3), nonce=nonce)[0], 409)

    def test_delayed_nonce_does_not_grant_a_fresh_full_frame_lease(self):
        _, raw, _ = self.get('/api/challenge')
        nonce = json.loads(raw)['nonce']
        self.now += .119
        self.assertEqual(self.send(payload(), nonce=nonce)[0], 200)
        self.assertAlmostEqual(self.server.tracking.sample().source_age_s, .119)
        self.now += .032
        self.assertFalse(self.server.tracking.sample().right.tracked)
        self.assertTrue(self.server.tracking.sample().right.release_required)

    def test_delayed_nonce_plus_client_age_is_rejected_before_eligibility(self):
        _, raw, _ = self.get('/api/challenge')
        nonce = json.loads(raw)['nonce']
        frame = payload(); frame['source_age_s'] = .02
        self.now += .11
        self.assertEqual(self.send(frame, nonce=nonce)[0], 422)

    def test_cross_origin_page_cannot_feed_or_revoke_tracking(self):
        self.send(payload())
        status, _, _ = self.send(payload(2), headers={'Origin':'http://unrelated.example'})
        self.assertEqual(status, 403)
        self.assertTrue(self.server.tracking.sample().right.tracked)
        self.assertEqual(self.get('/api/challenge', {'Host':'unrelated.example'})[0], 403)

    def test_authenticated_malformed_pose_revokes_old_target(self):
        self.send(payload())
        frame = payload(2)
        frame['right']['pose']['orientation_xyzw'] = [0., 0., 0., 0.]
        self.assertEqual(self.send(frame)[0], 422)
        self.assertFalse(self.server.tracking.sample().right.tracked)
        self.assertFalse(self.server.tracking.sample().left.tracked)

    def test_http_read_returns_expired_invalid_frame(self):
        self.send(payload())
        self.now += .151
        code, raw, _ = self.get('/api/frame')
        self.assertEqual(code, 200)
        result = json.loads(raw)
        self.assertFalse(result['frame']['right']['tracked'])
        self.assertTrue(result['frame']['right']['release_required'])

    def test_runtime_telemetry_is_copied_and_json_safe(self):
        runtime = {'robot_commands_sent':False, 'state':'dry_run', 'left':{'joints':[1., 2.]}}
        self.server.set_runtime_status(runtime)
        runtime['left']['joints'][0] = 99.
        _, raw, _ = self.get('/api/status')
        self.assertEqual(json.loads(raw)['runtime']['left']['joints'][0], 1.)
        with self.assertRaises(ValueError):
            self.server.set_runtime_status({'joints':[float('nan')]})

    def test_packaged_assets_and_csp_are_served(self):
        for path, text in (('/', '双手柄采集'), ('/app.js', 'getViewerPose'), ('/style.css', ':root')):
            code, raw, headers = self.get(path)
            self.assertEqual(code, 200)
            self.assertIn(text, raw.decode('utf-8'))
            self.assertIn("connect-src 'self'", headers['Content-Security-Policy'])
        self.assertEqual(self.get('/../../README.md')[0], 404)

    def test_bind_is_loopback_only(self):
        with self.assertRaises(ValueError): InputServer(host='0.0.0.0', port=0)

    def test_challenge_storage_is_bounded(self):
        for _ in range(128): self.server.challenge()
        with self.assertRaises(ValueError): self.server.challenge()
        self.now += .121
        self.assertIsInstance(self.server.challenge(), str)


class WebReadersTests(unittest.TestCase):
    @unittest.skipUnless(shutil.which('node'), 'Node is optional for JavaScript reader checks')
    def test_real_js_readers_validate_independent_controller_and_head_poses(self):
        source = r'''
const assert = require('node:assert/strict');
const input = require(process.argv[1]);
const validPose = {emulatedPosition:false, transform:{position:{x:0,y:1,z:-.4},orientation:{x:0,y:0,z:0,w:1}}};
function source() {return {gripSpace:{},targetRayMode:'tracked-pointer',gamepad:{connected:true,mapping:'xr-standard',
 buttons:Array.from({length:6},()=>({pressed:false,value:0})), axes:[0,0,.2,-.3]}};}
const left=source(),right=source();
left.gamepad.buttons[4]={pressed:true,value:1};right.gamepad.buttons[5]={pressed:true,value:1};
const frame={getPose:()=>validPose};
assert.deepEqual(input.controller(frame,{},left,'left').buttons,['X']);
assert.deepEqual(input.controller(frame,{},right,'right').buttons,['B']);
left.gamepad.connected=false;
assert.equal(input.controller(frame,{},left,'left').tracked,false);
assert.equal(input.controller(frame,{},right,'right').tracked,true);
assert.equal(input.poseOf({...validPose,emulatedPosition:true}),null);
assert.equal(input.poseOf({...validPose,transform:{position:{x:NaN,y:0,z:0},orientation:{x:0,y:0,z:0,w:1}}}),null);
assert.deepEqual(input.poseOf(validPose).orientation_xyzw,[0,0,0,1]);
right.gamepad.mapping='';assert.equal(input.controller(frame,{},right,'right').tracked,false);
console.log('head/left/right readers passed');
'''
        result = subprocess.run([shutil.which('node'), '-e', source, str(WEB_ROOT/'app.js')],
                                capture_output=True, text=True, timeout=10)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn('passed', result.stdout)

    def test_editable_and_packaged_web_assets_match(self):
        editable = Path(__file__).resolve().parents[1]/'web'
        for name in ('index.html', 'app.js', 'style.css'):
            self.assertEqual((editable/name).read_bytes(), (WEB_ROOT/name).read_bytes())


if __name__ == '__main__':
    unittest.main()
