"""Temporary freshness policy tests. No physical robot commands."""
import json
import sys
import unittest
from dataclasses import replace
from pathlib import Path
import numpy as np
from tg3_pico_dualarm.input_policy import NORMAL,WIFI_TRIAL,from_config
from tg3_pico_dualarm.input_validation import validate_frame
from tg3_pico_dualarm.tracking import TrackingFrame,LatestTracking
from tg3_pico_dualarm.wireless import InputGateway,WirelessServer,GatewayClient,LatestRelay
from tg3_pico_dualarm.mapping import RelativeMapper,Calibration
from tg3_pico_dualarm.motion_control import MotionAdmission
from tg3_pico_dualarm.controller import Teleoperation
from tg3_pico_dualarm.model import DualArmModel,default_q
from tg3_pico_dualarm.robot import DryRunRobot
from test_wireless import native
import test_manufacturer as factory_fixtures


class WifiTrialTests(unittest.TestCase):
    def test_exact_observed_age_only_changes_freshness_not_motion_authority(self):
        frame=TrackingFrame.from_payload(native()['frame'],10.)
        frame=replace(frame,received_at=10.-.084768778,source_age_s=.075289274)
        frame=replace(frame,left=replace(frame.left,release_required=False),right=replace(frame.right,release_required=False))
        with self.assertRaisesRegex(ValueError,'stale'):validate_frame(frame,10.)
        validate_frame(frame,10.,WIFI_TRIAL)
        self.assertAlmostEqual(10.-frame.received_at+frame.source_age_s,.160058052)
        fixture=factory_fixtures.ManufacturerTests();fixture.setUp();fixture.refresh(tuple(default_q()))
        gate=MotionAdmission(fixture.adapter,tools_calibrated=True,clock=lambda:10.,policy=WIFI_TRIAL)
        gate._input(frame,neutral=True)
        self.assertFalse(fixture.adapter.hardware_writes_enabled)
        self.assertFalse(fixture.driver.events)

    def test_source_plus_receipt_is_one_budget_never_two_200ms_leases(self):
        p=native()['frame'];p['source_age_s']=.19
        with self.assertRaises(ValueError):TrackingFrame.from_payload(p,10.)
        frame=TrackingFrame.from_payload(p,10.,WIFI_TRIAL)
        validate_frame(frame,10.009,WIFI_TRIAL)
        with self.assertRaises(ValueError):validate_frame(frame,10.011,WIFI_TRIAL)
        cache=LatestTracking(clock=lambda:10.,policy=WIFI_TRIAL)
        self.assertIsNone(cache.accept(p,additional_age_s=.011))
        self.assertIsNotNone(cache.accept(p,additional_age_s=.009))
        self.assertFalse(cache.sample(10.002).focused)

    def test_real_http_two_hops_use_same_budget_and_remain_disabled(self):
        pc=InputGateway(policy=WIFI_TRIAL);token=pc.pair(pc.open_pairing(),'synthetic PICO')
        server=WirelessServer(pc,'127.0.0.1',0).start();self.addCleanup(server.close)
        head=GatewayClient(*server.address,token=token,timeout=1);self.addCleanup(head.close)
        robot=InputGateway(role='relay',key='r'*40,policy=WIFI_TRIAL)
        remote=WirelessServer(robot,'127.0.0.1',0).start();self.addCleanup(remote.close)
        client=GatewayClient(*remote.address,token='r'*40,timeout=1);self.addCleanup(client.close)
        head.send(native(source_age_s=.16))
        LatestRelay(pc.tracking,client).step()
        actual=robot.tracking.sample()
        self.assertIsNotNone(actual)
        self.assertTrue(actual.focused)
        self.assertGreaterEqual(actual.source_age_s+1e-9,.16)
        self.assertLessEqual(actual.source_age_s,.2)
        self.assertEqual(robot.status()['tracking']['input_profile'],WIFI_TRIAL.name)
        self.assertFalse(robot.status()['hardware_motion_enabled'])
        self.assertFalse(robot.recording_capture_active)

    def test_one_use_challenge_still_120ms_and_replay_rejected_in_trial(self):
        now=[1.];g=InputGateway(clock=lambda:now[0],policy=WIFI_TRIAL)
        key=g.pair(g.open_pairing(),'PICO');nonce=g.challenge(key)['nonce']
        now[0]+=.121
        with self.assertRaisesRegex(ValueError,'challenge'):g.accept(key,nonce,native())
        g.accept(key,g.challenge(key)['nonce'],native())
        with self.assertRaisesRegex(ValueError,'replayed'):g.accept(key,g.challenge(key)['nonce'],native())
        self.assertFalse(g.status()['hardware_motion_enabled'])

    def test_loss_reconnect_requires_release_mapper_and_controller_share_policy(self):
        now=[1.];cache=LatestTracking(clock=lambda:now[0],policy=WIFI_TRIAL)
        frame=cache.accept(native(source_age_s=.16)['frame'])
        mapper=RelativeMapper(Calibration.from_head(frame.head),policy=WIFI_TRIAL)
        poses=dict(left=np.eye(4),right=np.eye(4))
        mapper.update(frame,poses,now[0])
        now[0]+=.01;p=native(1,source_age_s=.16)['frame'];p['right']['grip']=1.
        frame=cache.accept(p)
        self.assertTrue(mapper.update(frame,poses,now[0]).active['right'])
        robot=DryRunRobot(default_q(),monotonic=lambda:now[0])
        control=Teleoperation(robot,DualArmModel(),None,monotonic=lambda:now[0],policy=WIFI_TRIAL)
        neutral=replace(frame,right=replace(frame.right,grip=0.))
        result=control.tick(neutral)
        self.assertEqual(result['state'],'cartesian_preview_only')
        self.assertIs(control.mapper.policy,WIFI_TRIAL)
        now[0]+=.05;self.assertFalse(cache.sample().focused)
        p=native(2,source_age_s=.16)['frame'];p['right']['grip']=1.
        self.assertTrue(cache.accept(p).right.release_required)

    def test_only_two_explicit_profiles_default_normal_no_arbitrary_relaxation(self):
        for name,total,source in [('normal',.15,.12),('wifi_trial_200ms',.2,.2)]:
            self.assertEqual(from_config(dict(input_profile=name,tracking_timeout_s=total,source_age_limit_s=source)).name,name)
        with self.assertRaises(ValueError):from_config(dict(input_profile='wifi_trial_200ms',tracking_timeout_s=.3,source_age_limit_s=.2))
        self.assertEqual(InputGateway().tracking.policy,NORMAL)
        self.assertEqual(LatestTracking(policy=WIFI_TRIAL,max_age_s=.5).max_age_s,.2)
