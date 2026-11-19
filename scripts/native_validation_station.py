"""Run the actual station CLI as a child, with synthetic native wireless input.

Linux/Jazzy only. No private manufacturer messages or control provider are
created. The child receives its relay secret through stdin, never argv/logs.
"""
from __future__ import annotations
import argparse
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import queue
import secrets
import signal
import socket
import subprocess
import sys
import threading
import time
import traceback
import uuid

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src'))
from tg3_pico_dualarm import __version__
from tg3_pico_dualarm.wireless import InputGateway,WirelessServer,GatewayClient,LatestRelay


def sample(sequence,session):
    def hand(side):
        active=(30<=sequence<95) if side=='right' else (60<=sequence<95)
        progress=min(max(sequence-(35 if side=='right' else 65),0)/20,1.)
        distance=.006*progress
        angle=.006*progress if side=='right' else 0.
        return dict(pose=dict(position=[-.25 if side=='left' else .25,
                        1.1+(distance if side=='left' else 0),-.3-(distance if side=='right' else 0)],
                    orientation_xyzw=[0.,math.sin(angle/2),0.,math.cos(angle/2)]),
            connected=True,tracked=True,grip=1. if active else 0.,
            trigger=.55 if side=='right' and 75<=sequence<90 else 0.,buttons=[],axes=[0.,0.],
            source_id=side+'-SYNTHETIC-station-test')
    return dict(protocol='tg3-native-1',coordinates='openxr_rh_y_up',frame=dict(session=session,
        sequence=sequence,source_time_s=1000.+sequence*.02,source_age_s=0.,source='openxr',focused=True,
        scene_visible=True,head=dict(position=[0.,1.6,0.],orientation_xyzw=[0.,0.,0.,1.]),
        left=hand('left'),right=hand('right')))


def validate(config,frames):
    if sys.platform!='linux':raise RuntimeError('actual Linux/Jazzy required; no station child started')
    import rclpy
    from rclpy.context import Context
    from rclpy.node import Node
    from rclpy.executors import SingleThreadedExecutor
    from sensor_msgs.msg import JointState
    from std_msgs.msg import String
    from tg3_pico_dualarm.ros_bridge import DRY_RUN_JOINT_TOPIC,DIAGNOSTIC_TOPIC
    from tg3_pico_dualarm.preflight import CONTROL_TOPICS

    result=dict(passed=False,version=__version__,test='actual_station_cli_process',
        started=datetime.now(timezone.utc).isoformat(),input='SYNTHETIC_OPENXR',
        headset_tested=False,physical_robot_tested=False,robot_commands_sent=False)
    from release_scope import public_source
    result['source_file_sha256']={name:hashlib.sha256(data).hexdigest() for name,data in public_source(ROOT)}
    source=Path('/home/tg3/project.zip')
    if source.exists():result['source_bundle_sha256']=hashlib.file_digest(source.open('rb'),'sha256').hexdigest()
    directory=ROOT/'artifacts'/('native_station_'+uuid.uuid4().hex[:8]);directory.mkdir(parents=True)
    result['artifact_directory']=str(directory)
    (directory/'input_manifest.json').write_text(json.dumps(dict(source='SYNTHETIC_OPENXR',headset_used=False,
        physical_robot_used=False,frames=frames,scope='station CLI lifecycle and software integration'),indent=2),encoding='utf-8')
    context=Context();rclpy.init(args=[],context=context)
    observer=Node('tg3_station_native_observer_'+uuid.uuid4().hex[:8],context=context,enable_rosout=False)
    executor=SingleThreadedExecutor(context=context);executor.add_node(observer)
    ros_counts={'joints':0,'diagnostics':0};joint_values=[];diagnostics=[]
    def joints(message):
        if len(message.name)==14 and len(message.position)==14:
            ros_counts['joints']+=1;joint_values.append(tuple(message.position))
    def diagnostic(message):
        payload=json.loads(message.data)
        if payload.get('mode')=='dry_run':
            ros_counts['diagnostics']+=1;diagnostics.append(dict(payload,received_monotonic=time.monotonic()))
    subscriptions=[observer.create_subscription(JointState,DRY_RUN_JOINT_TOPIC,joints,10),
                   observer.create_subscription(String,DIAGNOSTIC_TOPIC,diagnostic,10)]
    spin=threading.Thread(target=executor.spin,daemon=True);spin.start()
    process=None;pc_server=None;headset=None;relay=None;status_client=None;rpc_client=None
    stdout_queue=queue.Queue();readers=[];child_recording=None;samples=[]
    def drain(stream,path,lines=None):
        with path.open('w',encoding='utf-8') as output:
            for line in stream:
                output.write(line);output.flush()
                if lines is not None:lines.put(line)
    try:
        key=secrets.token_urlsafe(32)
        environment=os.environ.copy()
        environment.update(ROS_LOCALHOST_ONLY='1',PYTHONUNBUFFERED='1')
        command=[sys.executable,'-s','-u','-m','tg3_pico_dualarm.station','--key-stdin','--port','0',
                 '--config',str(config.resolve()),'--record',str(directory/'recordings'),'--ros']
        process=subprocess.Popen(command,cwd=ROOT,env=environment,stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,stderr=subprocess.PIPE,text=True,encoding='utf-8',bufsize=1,
            start_new_session=True)
        for stream,path,lines in ((process.stdout,directory/'stdout.log',stdout_queue),
                                  (process.stderr,directory/'stderr.log',None)):
            thread=threading.Thread(target=drain,args=(stream,path,lines),daemon=True)
            thread.start();readers.append(thread)
        process.stdin.write(key+'\n');process.stdin.flush()
        deadline=time.monotonic()+25;startup=None
        while time.monotonic()<deadline:
            if process.poll() is not None:raise RuntimeError('station CLI exited before startup; inspect stdout/stderr logs')
            try:line=stdout_queue.get(timeout=.1)
            except queue.Empty:continue
            try:value=json.loads(line)
            except ValueError:continue
            if value.get('event')=='robot_station_started':startup=value;break
        if startup is None:raise RuntimeError('station CLI startup timeout')
        assert startup['version']==__version__ and startup['native_placo'] is True,startup
        assert startup['runtime_mode']=='dry_run' and startup['hardware_motion_enabled'] is False,startup
        assert 0<startup['listen_port']<=65535,startup
        result['startup']=startup;assert startup['recording'] is None,startup
        child_recording=directory/'recordings'
        endpoint=('127.0.0.1',startup['listen_port'])
        pc=InputGateway();pc_server=WirelessServer(pc,'127.0.0.1',0).start()
        headset=GatewayClient(*pc_server.address,timeout=.12)
        headset.token=pc.pair(pc.open_pairing(),'SYNTHETIC-native-station-validation')
        relay=LatestRelay(pc.tracking,GatewayClient(*endpoint,token=key,timeout=.12))
        status_client=GatewayClient(*endpoint,token=key,timeout=1.)
        rpc_client=GatewayClient(*endpoint,token=key,timeout=1.)
        session='SYNTHETIC-native-station-'+uuid.uuid4().hex
        active_seen=dict(left=False,right=False);closures=[];right_only=[];left_moving=[];holds=[];warmup_states=[];ages=[]
        # The deployed desktop polls this full JSON endpoint at 2 Hz. Keep
        # input at 33 Hz; checking status on every frame artificially loads the
        # station. ROS software-disarmed observations are checked separately;
        # this check does not claim a complete per-tick control archive.
        poll_sequences={0,29,45,59,75,94,100,frames-1}
        active_started=None;active_finished=None
        for sequence in range(frames):
            began=time.monotonic()
            if sequence==30:active_started=began
            if sequence==95:active_finished=began
            if process.poll() is not None:raise RuntimeError('station CLI stopped during synthetic input')
            headset.send(None,on_challenge=lambda:sample(sequence,session));relay.step()
            if sequence not in poll_sequences:
                time.sleep(max(0.,.03-(time.monotonic()-began)))
                continue
            status=status_client.request('GET','/v1/status')
            runtime=status.get('runtime',{});control=runtime.get('control',{})
            samples.append(dict(sequence=sequence,status=status))
            assert status['hardware_motion_enabled'] is False,status
            assert runtime.get('readiness',{}).get('hardware_ready') is False,runtime
            if sequence<30 and control.get('state')!='dry_run':
                warmup_states.append(dict(sequence=sequence,state=control.get('state'),reason=control.get('reason')))
            if sequence>=30:
                assert control.get('state')=='dry_run',control
                assert control.get('ik',{}).get('backend')=='placo',control.get('ik')
                assert set(control.get('model_links',{}))=={'left','right'},control
                assert len(control.get('joint_positions',[]))==14,control
                assert all(math.isfinite(value) for value in control['joint_positions'])
                assert 0<=status['tracking']['sample_age_s']<=.15,status['tracking']
                ages.append(status['tracking']['sample_age_s'])
                assert status['tracking']['input_source']=='openxr'
            for side,active in control.get('active',{}).items():active_seen[side]|=active
            if 38<=sequence<60 and control.get('active',{}).get('right'):
                assert control['active']['left'] is False
                right_only.append(control['joint_positions'])
            if 68<=sequence<95 and control.get('active',{}).get('left'):
                left_moving.append(control['joint_positions'])
            if sequence>=100:
                closures.append(control.get('hands',{}).get('closure',{}).get('right',0))
                holds.append(control['joint_positions'])
                assert not any(control.get('active',{}).values()),control['active']
            time.sleep(max(0.,.03-(time.monotonic()-began)))
        assert relay.status['forwarded']==frames,relay.status
        assert all(active_seen.values()),active_seen
        assert len(right_only)>=2,'right-only stage not observed'
        assert max(abs(a-b) for a,b in zip(right_only[0][7:],right_only[-1][7:]))>1e-5
        assert max(abs(a-b) for a,b in zip(right_only[0][:7],right_only[-1][:7]))<1e-8
        assert len(left_moving)>=2,'left movement stage not observed'
        assert max(abs(a-b) for a,b in zip(left_moving[0][:7],left_moving[-1][:7]))>1e-5
        assert max(abs(a-b) for a,b in zip(holds[0],holds[-1]))<1e-10,'released joints did not hold'
        assert min(closures)>.03 and max(closures)-min(closures)<1e-8,closures
        final=samples[-1]['status']['runtime'];warnings=final.get('warnings',[])
        assert any('manufacturer_observation_unavailable' in warning for warning in warnings),warnings
        assert final['readiness']['manufacturer_adapter_available'] is False,final['readiness']
        assert final['readiness']['native_ik_ready'] is True,final['readiness']
        assert final['recording'] is None
        assert final['rosbag']['state']=='off' and not final['rosbag']['ready']
        assert min(ros_counts.values())>=3,ros_counts
        assert all(payload['hardware_writes_enabled'] is False for payload in diagnostics)
        assert all(payload['robot_commands_sent'] is False for payload in diagnostics)
        forbidden=set(CONTROL_TOPICS)|{'/left_hand/set_motor_multi','/waist/cmd','/head/cmd'}
        created={topic:len(observer.get_publishers_info_by_topic(topic)) for topic in sorted(forbidden)}
        assert not any(created.values()),created
        publishers=[name for name,_ in observer.get_publisher_names_and_types_by_node('tg3_pico_dualarm_bridge','/')]
        assert set(publishers)<={'/parameter_events',DRY_RUN_JOINT_TOPIC,DIAGNOSTIC_TOPIC},publishers
        challenge=rpc_client.request('GET','/v1/challenge')
        try:
            rpc_client.request('POST','/v1/control',dict(protocol='tg3-native-1',operation='enable',session=session),nonce=challenge['nonce'])
        except ValueError as exc:
            assert 'hardware_control_not_configured' in str(exc),str(exc)
            result['enable_denied_reason']=str(exc)
        else:raise AssertionError('dry-run station accepted hardware enable')
        for client in (status_client,rpc_client):client.close()
        relay.close();relay=None;headset.close();headset=None;pc_server.close();pc_server=None
        process.stdin.close() # The requested actual CLI parent-disconnect path.
        exited=time.monotonic()
        code=process.wait(timeout=35)
        result['child_exit_code']=code;result['stdin_close_exit_s']=time.monotonic()-exited
        assert code==0,code
        for thread in readers:thread.join(2)
        assert not child_recording.exists(),'station unexpectedly created data recording directory'
        active_disarms=[row for row in diagnostics if row['software_disarmed']
                        and active_started<=row['received_monotonic']<active_finished]
        assert not active_disarms,active_disarms
        active_rows=[row for row in samples if 30<=row['sequence']<95]
        for row in active_rows:
            control=row['status']['runtime']['control'];sequence=row['sequence']
            assert control['state']=='dry_run' and control['active']['right'] is True,(sequence,control)
            assert control['active']['left'] is (sequence>=60),(sequence,control)
        try:
            socket.create_connection(endpoint,timeout=.3).close()
        except OSError:listener_closed=True
        else:listener_closed=False
        assert listener_closed,'station listener remained open after process exit'
        result.update(passed=True,actual_cli_process=True,actual_two_hop_frames=frames,
            neutral_startup_warmup_frames=30,startup_warmup_states=warmup_states,
            status_poll_samples=len(samples),status_poll_production_rate_hz=2,
            active_sampled_control_rows=len(active_rows),active_disarms=active_disarms,
            both_arms_activated=active_seen,both_arms_moved=True,right_only_left_held=True,
            released_joints_hold=True,trigger_release_retains_grasp=True,max_source_age_s=max(ages),
            actual_ros_messages=ros_counts,no_motion_publishers=created,manufacturer_fallback_warnings=warnings,
            station_bridge_publishers=publishers,
            automatic_recording=False,recorded_control_rows=0,listener_closed=listener_closed,
            readiness=final['readiness'],rosbag_status=final.get('rosbag',{}))
        (directory/'runtime_samples.json').write_text(json.dumps(samples,indent=2),encoding='utf-8')
    except Exception as exc:
        result.update(error=str(exc),traceback=traceback.format_exc())
        if samples:result['last_runtime']=samples[-1]
    finally:
        for client in (status_client,rpc_client,headset):
            if client:
                try:client.close()
                except OSError:pass
        if relay:relay.close()
        if pc_server:pc_server.close()
        if process and process.poll() is None:
            if process.stdin and not process.stdin.closed:process.stdin.close()
            try:process.wait(timeout=35)
            except subprocess.TimeoutExpired:
                # Only the child process/session this test just created.
                os.killpg(process.pid,signal.SIGTERM)
                try:process.wait(timeout=3)
                except subprocess.TimeoutExpired:os.killpg(process.pid,signal.SIGKILL);process.wait(timeout=3)
        for thread in readers:thread.join(2)
        executor.shutdown(timeout_sec=2);observer.destroy_node()
        if context.ok():context.shutdown()
        spin.join(2)
        result['finished']=datetime.now(timezone.utc).isoformat()
        (ROOT/'artifacts/native_station_result.json').write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf-8')
    return result


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config',type=Path,default=ROOT/'config')
    parser.add_argument('--frames',type=int,default=120)
    args=parser.parse_args()
    if args.frames<120:parser.error('at least 120 frames required')
    result=validate(args.config,args.frames)
    print(json.dumps(result,ensure_ascii=False,indent=2))
    return 0 if result['passed'] else 2


if __name__=='__main__':raise SystemExit(main())
