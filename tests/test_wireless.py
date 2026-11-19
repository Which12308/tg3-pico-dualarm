from dataclasses import replace
import copy
import math
import unittest
import numpy as np
from tg3_pico_dualarm.mapping import Calibration,RelativeMapper,rotation
from tg3_pico_dualarm.wireless import InputGateway,WirelessServer,GatewayClient,LatestRelay,canonical_frame,parse_json,BODY_LIMIT,STATUS_REPLY_LIMIT,_PackedHTTPConnection


def native(seq=0,session='native-a',**overrides):
    def hand(side):
        return dict(pose=dict(position=[-.25 if side=='left' else .25,1.,.3],orientation_xyzw=[0.,0.,0.,1.]),
                    connected=True,tracked=True,grip=0.,trigger=0.,buttons=[],axes=[0.,0.],source_id=side+'1')
    frame=dict(session=session,sequence=seq,source_time_s=100+seq*.02,source_age_s=0.,source='xrobotoolkit',
               focused=True,scene_visible=True,head=dict(position=[0.,1.6,0.],orientation_xyzw=[0.,0.,0.,1.]),
               left=hand('left'),right=hand('right'))
    frame.update(overrides)
    return dict(protocol='tg3-native-1',coordinates='unity_lh_y_up',frame=frame)


class NativeGatewayTests(unittest.TestCase):
    def test_second_receiver_cannot_share_the_first_receivers_input_port(self):
        server=WirelessServer(InputGateway(),'127.0.0.1',0).start()
        self.addCleanup(server.close)
        with self.assertRaises(OSError):WirelessServer(InputGateway(),*server.address)

    def setUp(self):
        self.clock=[1.]
        self.gateway=InputGateway(clock=lambda:self.clock[0])
        pin=self.gateway.open_pairing();self.token=self.gateway.pair(pin,'PICO test')

    def send(self,value):
        challenge=self.gateway.challenge(self.token)
        return self.gateway.accept(self.token,challenge['nonce'],value)

    def test_native_handedness_is_not_browser_axis_copy_and_preserves_rotations(self):
        value=native();q=[math.sin(.3),0.,0.,math.cos(.3)]
        value['frame']['right']['pose']['orientation_xyzw']=q
        original=copy.deepcopy(value)
        actual=self.send(value)
        np.testing.assert_allclose(actual.right.pose.position,[.25,1.,-.3])
        reflection=np.diag([1,1,-1])
        np.testing.assert_allclose(rotation(actual.right.pose.orientation_xyzw),reflection@rotation(q)@reflection)
        self.assertEqual(value,original)
        self.assertFalse(actual.right.release_required)
        value['coordinates']='webxr'
        with self.assertRaisesRegex(ValueError,'coordinates'):self.send(value)
        self.assertFalse(self.gateway.tracking.sample().right.tracked)

    def test_occluded_left_does_not_revoke_valid_right_and_left_needs_release(self):
        self.send(native())
        first=native(1);first['frame']['left']['grip']=first['frame']['right']['grip']=1.
        self.send(first)
        lost=native(2);lost['frame']['left'].update(tracked=False,pose=None,grip=1.)
        lost['frame']['right']['grip']=1.
        frame=self.send(lost)
        self.assertTrue(frame.left.release_required);self.assertFalse(frame.right.release_required)
        held=native(3);held['frame']['left']['grip']=held['frame']['right']['grip']=1.
        self.assertTrue(self.send(held).left.release_required)
        released=native(4);released['frame']['right']['grip']=1.
        self.assertFalse(self.send(released).left.release_required)

    def test_each_hop_challenge_is_one_use_and_delays_do_not_refresh_a_lease(self):
        nonce=self.gateway.challenge(self.token)['nonce'];self.clock[0]+=.10
        value=native();value['frame']['source_age_s']=.04
        with self.assertRaisesRegex(ValueError,'expired_source'):self.gateway.accept(self.token,nonce,value)
        with self.assertRaisesRegex(ValueError,'challenge'):self.gateway.accept(self.token,nonce,native(1))
        nonce=self.gateway.challenge(self.token)['nonce'];self.clock[0]+=.121
        with self.assertRaisesRegex(ValueError,'challenge'):self.gateway.accept(self.token,nonce,native(2))

    def test_sleep_resume_relocalization_and_restart_require_released_inputs(self):
        self.send(native())
        paused=native(1,focused=False)
        self.assertFalse(self.send(paused).right.tracked)
        resumed=native(2);resumed['frame']['right']['grip']=1.
        self.assertTrue(self.send(resumed).right.release_required)
        reset=native(0,session='new-origin');reset['frame']['right']['grip']=1.
        self.assertTrue(self.send(reset).right.release_required)
        with self.assertRaisesRegex(ValueError,'retired'):self.send(native(99))
        self.assertFalse(self.send(native(1,session='new-origin')).right.release_required)
        old_token=self.token
        self.gateway.open_pairing()
        with self.assertRaisesRegex(ValueError,'authentication'):self.gateway.challenge(old_token)

    def test_stationary_valid_pose_is_allowed_but_replay_is_not(self):
        self.send(native())
        self.send(native(1))
        self.assertTrue(self.gateway.tracking.sample().right.tracked)
        with self.assertRaisesRegex(ValueError,'replayed'):self.send(native(1))

    def test_axis_conversion_cannot_turn_boolean_pose_values_into_valid_numbers(self):
        value=native();value['frame']['right']['pose']['position'][2]=True
        with self.assertRaisesRegex(ValueError,'finite number'):self.send(value)

    def test_openxr_input_keeps_right_handed_pose_and_rejects_source_coordinate_mix(self):
        value=native();value['coordinates']='openxr_rh_y_up';value['frame']['source']='openxr'
        value['frame']['right']['pose']['position']=[.25,1.,-.3]
        actual=self.send(value)
        self.assertEqual(actual.source,'openxr')
        np.testing.assert_allclose(actual.right.pose.position,[.25,1.,-.3])
        value['coordinates']='unity_lh_y_up'
        with self.assertRaisesRegex(ValueError,'native_source_required'):self.send(value)

    def test_pairing_is_local_opened_expiring_and_attempt_limited(self):
        gateway=InputGateway(clock=lambda:self.clock[0]);pin=gateway.open_pairing()
        for _ in range(8):
            with self.assertRaises(ValueError):gateway.pair('bad','other')
        with self.assertRaisesRegex(ValueError,'attempt_limit'):gateway.pair(pin,'PICO')
        pin=gateway.open_pairing();self.clock[0]+=121
        with self.assertRaisesRegex(ValueError,'closed'):gateway.pair(pin,'PICO')
        self.assertNotIn('token',self.gateway.status())
        self.assertNotIn('pin',self.gateway.status())


class WirelessSocketTests(unittest.TestCase):
    def test_real_small_json_response_has_one_complete_wire_write(self):
        import json,socketserver
        from unittest.mock import patch
        gateway=InputGateway();server=WirelessServer(gateway,'127.0.0.1',0).start()
        self.addCleanup(server.close)
        client=GatewayClient(*server.address,timeout=1);self.addCleanup(client.close)
        writes=[];original=socketserver._SocketWriter.write
        def write(stream,data):
            writes.append(bytes(data));return original(stream,data)
        with patch.object(socketserver._SocketWriter,'write',write):value=client.request('GET','/v1/info')
        self.assertEqual(len(writes),1)
        headers,body=writes[0].split(b'\r\n\r\n',1)
        self.assertEqual(json.loads(body),value)
        self.assertIn(('Content-Length: '+str(len(body))).encode(),headers)

    def test_wired_fresh_challenge_excludes_idle_time_but_never_renews_old_input(self):
        from unittest.mock import patch
        now=[1.];gateway=InputGateway(role='relay',key='n'*40,clock=lambda:now[0])
        server=WirelessServer(gateway,'127.0.0.1',0).start();self.addCleanup(server.close)
        client=GatewayClient(*server.address,token='n'*40,timeout=1,cache_challenge=False)
        self.addCleanup(client.close)
        def payload(seq,age=0.):return dict(protocol='tg3-native-1',frame=canonical_frame(native(seq,source_age_s=age)))
        client.send(payload(0));now[0]+=.2  # Legitimately new sample after sender idle.
        with patch.object(client,'request',wraps=client.request) as requests:
            client.send(payload(1));self.assertEqual(requests.call_count,2)
        self.assertEqual(gateway.tracking.sample().sequence,1)
        self.assertIsNone(client._next_challenge);self.assertFalse(gateway.challenges)
        with self.assertRaises(ValueError):client.send(payload(2,.121))
        client.close();now[0]+=.02
        def delayed():
            now[0]+=.121;return payload(3)
        with self.assertRaisesRegex(ValueError,'expired_transport_challenge'):
            client.send(None,on_challenge=delayed)

    def test_small_json_request_has_one_complete_wire_write(self):
        connection=_PackedHTTPConnection('127.0.0.1',1)
        writes=[];connection.send=writes.append
        body=b'{"protocol":"tg3-native-1","test":"no robot"}'
        connection.request('POST','/v1/frame',body,{'Content-Type':'application/json'})
        self.assertEqual(len(writes),1)
        headers,payload=writes[0].split(b'\r\n\r\n',1)
        self.assertEqual(payload,body)
        self.assertIn(('Content-Length: '+str(len(body))).encode(),headers)

    def test_relay_ack_pipelines_single_use_challenge_without_extending_its_expiry(self):
        from unittest.mock import patch
        now=[1.]
        gateway=InputGateway(role='relay',key='n'*40,clock=lambda:now[0])
        server=WirelessServer(gateway,'127.0.0.1',0).start();self.addCleanup(server.close)
        client=GatewayClient(*server.address,token='n'*40,timeout=1);self.addCleanup(client.close)
        def payload(seq):return dict(protocol='tg3-native-1',frame=canonical_frame(native(seq)))
        with patch.object(client,'request',wraps=client.request) as requests:
            client.send(payload(0))
            self.assertEqual(requests.call_count,2)
            requests.reset_mock();now[0]+=.02
            client.send(payload(1))
            self.assertEqual(requests.call_count,1)
            self.assertEqual(requests.call_args.args[:2],('POST','/v1/frame'))
            self.assertEqual(gateway.tracking.sample().sequence,1)
            # A cached challenge cannot refresh itself, even if its payload
            # reports a fresh source. Receiver-clock 120 ms still applies.
            now[0]+=.121
            with self.assertRaisesRegex(ValueError,'expired_transport_challenge'):
                client.send(payload(2))
            self.assertFalse(gateway.tracking.sample().focused)
            client.close()
            client.send(payload(3))
            self.assertFalse(gateway.tracking.sample().right.release_required)

    def test_large_readonly_status_does_not_expand_control_or_input_limits(self):
        gateway=InputGateway(role='relay',key='s'*40)
        gateway.runtime={'diagnostics':'x'*(BODY_LIMIT*3)}
        server=WirelessServer(gateway,'127.0.0.1',0).start();self.addCleanup(server.close)
        client=GatewayClient(*server.address,token='s'*40,timeout=1);self.addCleanup(client.close)
        self.assertEqual(client.request('GET','/v1/status')['runtime'],gateway.runtime)
        with self.assertRaisesRegex(ValueError,'invalid_content_length'):
            client.request('POST','/v1/frame',{'oversize':'x'*BODY_LIMIT})
        client.close()  # Invalid input closes the server connection by design.
        gateway.control_callback=lambda value: {'oversize':'x'*BODY_LIMIT}
        with self.assertRaisesRegex(ValueError,'oversize_server_reply'):
            client.control('disable',None)
        # A rejected oversized reply must not corrupt the next HTTP response.
        self.assertEqual(client.request('GET','/v1/info')['role'],'relay')
        gateway.runtime={'diagnostics':'x'*STATUS_REPLY_LIMIT}
        with self.assertRaisesRegex(ValueError,'oversize_server_reply'):
            client.request('GET','/v1/status')

    def test_real_two_hop_sockets_and_calibrated_one_to_one_forward_hold_reclutch(self):
        pc=InputGateway();pin=pc.open_pairing()
        server=WirelessServer(pc,'127.0.0.1',0).start();self.addCleanup(server.close)
        headset=GatewayClient(*server.address,timeout=1);self.addCleanup(headset.close)
        headset.token=headset.request('POST','/v1/pair',dict(pin=pin,device='PICO test',protocol='tg3-native-1'))['token']
        robot=InputGateway(role='relay',key='x'*40)
        remote=WirelessServer(robot,'127.0.0.1',0).start();self.addCleanup(remote.close)
        client=GatewayClient(*remote.address,token='x'*40,timeout=1);self.addCleanup(client.close)
        relay=LatestRelay(pc.tracking,client)
        def send(value):
            headset.send(value);relay.step();return robot.tracking.sample()
        frame=send(native());self.assertTrue(frame.scene_visible)
        mapper=RelativeMapper(Calibration.from_head(frame.head))
        positions=dict(left=np.eye(4),right=np.eye(4))
        mapper.update(frame,positions,frame.received_at)
        value=native(1);value['frame']['right']['grip']=1.
        frame=send(value);mapper.update(frame,positions,frame.received_at)
        value=native(2);value['frame']['right']['grip']=1.
        value['frame']['right']['pose']['position'][2]+=.01
        frame=send(value);result=mapper.update(frame,positions,frame.received_at)
        np.testing.assert_allclose(result.targets['right'][:3,3],[.01,0,0],atol=1e-8)
        self.assertFalse(result.active['left']);self.assertTrue(result.active['right'])
        release=send(native(3));result=mapper.update(release,positions,release.received_at)
        self.assertFalse(result.active['right'])
        positions['right'][:3,3]=[.01,0,0]
        held=native(4);held['frame']['right']['grip']=1.;held['frame']['right']['pose']['position'][2]=1.
        frame=send(held);result=mapper.update(frame,positions,frame.received_at)
        np.testing.assert_allclose(result.targets['right'],positions['right'])
        self.assertEqual(relay.status['forwarded'],5)
        self.assertFalse(robot.status()['hardware_motion_enabled'])

    def test_transport_error_revokes_pc_input_and_grip_cannot_auto_reconnect(self):
        pc=InputGateway();token=pc.pair(pc.open_pairing(),'PICO')
        pc.accept(token,pc.challenge(token)['nonce'],native())
        class Failed:
            def send(self,*args,**kwargs):raise OSError('link failed')
            def close(self):pass
        relay=LatestRelay(pc.tracking,Failed());relay.step()
        self.assertFalse(pc.tracking.sample().right.tracked)
        value=native(1);value['frame']['right']['grip']=1.
        pc.accept(token,pc.challenge(token)['nonce'],value)
        self.assertTrue(pc.tracking.sample().right.release_required)

    def test_bad_bearer_cannot_revoke_a_valid_paired_session(self):
        gateway=InputGateway();token=gateway.pair(gateway.open_pairing(),'PICO')
        gateway.accept(token,gateway.challenge(token)['nonce'],native())
        with self.assertRaises(ValueError):gateway.accept('wrong','bad',{})
        self.assertTrue(gateway.tracking.sample().right.tracked)

    def test_short_relay_reconnect_propagates_release_gate_even_before_remote_expiry(self):
        pc=InputGateway();token=pc.pair(pc.open_pairing(),'PICO')
        robot=InputGateway(role='relay',key='r'*40)
        class Direct:
            def send(self,value,on_challenge):
                nonce=robot.challenge('r'*40)['nonce']
                robot.accept('r'*40,nonce,on_challenge())
            def close(self):pass
        relay=LatestRelay(pc.tracking,Direct())
        def send(value):
            pc.accept(token,pc.challenge(token)['nonce'],value);relay.step()
        send(native())
        value=native(1);value['frame']['right']['grip']=1.;send(value)
        self.assertFalse(robot.tracking.sample().right.release_required)
        pc.tracking.invalidate('brief_wifi_interruption')
        value=native(2);value['frame']['right']['grip']=1.;send(value)
        frame=robot.tracking.sample()
        self.assertFalse(frame.right.tracked);self.assertTrue(frame.right.release_required)
        send(native(3));self.assertFalse(robot.tracking.sample().right.release_required)

    def test_duplicate_fields_and_nonfinite_json_are_rejected(self):
        for value in ('{"tracked":true,"tracked":false}','{"source_age_s":NaN}'):
            with self.assertRaises(ValueError):parse_json(value)


if __name__=='__main__':unittest.main()
