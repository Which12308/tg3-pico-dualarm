"""Real Linux HTTP -> native Placo -> native Jazzy -> JSONL integration.

Input is explicitly SYNTHETIC OpenXR. No headset, camera or manufacturer
actuator is used. DDS must be restricted to guest loopback by the caller.
"""
from dataclasses import asdict
import json
import hashlib
from pathlib import Path
import time
import traceback

from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from tg3_pico_dualarm import __version__
from tg3_pico_dualarm.controller import Teleoperation
from tg3_pico_dualarm.kinematics import Kinematics
from tg3_pico_dualarm.model import DualArmModel,MODEL_SHA256
from tg3_pico_dualarm.recording import SessionRecorder
from tg3_pico_dualarm.ros_bridge import RosDryRunSession,DRY_RUN_JOINT_TOPIC,DIAGNOSTIC_TOPIC
from tg3_pico_dualarm.wireless import InputGateway,WirelessServer,GatewayClient,LatestRelay


def frame(sequence):
    def hand(side):
        active=10<=sequence<80
        forward=.006*min(max(sequence-15,0)/50,1)
        return dict(pose=dict(position=[-.25 if side=='left' else .25,1.1+(forward if side=='left' else 0),-.3-(forward if side=='right' else 0)],
                    orientation_xyzw=[0.,0.,0.,1.]),connected=True,tracked=True,grip=1. if active else 0.,
                    trigger=.55 if side=='right' and 40<=sequence<60 else 0.,buttons=[],axes=[0.,0.],source_id=side+'-synthetic')
    return dict(protocol='tg3-native-1',coordinates='openxr_rh_y_up',frame=dict(session='native-full-pipeline',
        sequence=sequence,source_time_s=1000.+sequence*.02,source_age_s=0.,source='openxr',focused=True,scene_visible=True,
        head=dict(position=[0.,1.6,0.],orientation_xyzw=[0.,0.,0.,1.]),left=hand('left'),right=hand('right')))


def main():
    session=RosDryRunSession(observe_robot=False)
    observer=Node('tg3_full_native_pipeline_observer',context=session.context,enable_rosout=False)
    counts={'joints':0,'diagnostics':0}
    def joints(message):
        assert len(message.name)==14 and len(message.position)==14
        counts['joints']+=1
    def diagnostics(message):
        value=json.loads(message.data)
        assert value['hardware_writes_enabled'] is False and value['robot_commands_sent'] is False
        counts['diagnostics']+=1
    subscriptions=[observer.create_subscription(JointState,DRY_RUN_JOINT_TOPIC,joints,10),
                   observer.create_subscription(String,DIAGNOSTIC_TOPIC,diagnostics,10)]
    session.executor.add_node(observer)
    recorder=SessionRecorder('artifacts/native_pipeline_sessions',dict(version=__version__,
        source='SYNTHETIC_OPENXR_NATIVE_INTEGRATION',native_placo_used=True,native_ros2_used=True,
        physical_robot_commands_sent=False,headset_used=False,model_sha256=MODEL_SHA256))
    pc=InputGateway();server=WirelessServer(pc,'127.0.0.1',0).start()
    client=GatewayClient(*server.address,timeout=.12)
    client.token=pc.pair(pc.open_pairing(),'synthetic-validation-only')
    robot_gateway=InputGateway(role='relay',key='native-test-only-'+('r'*32))
    remote=WirelessServer(robot_gateway,'127.0.0.1',0).start()
    relay_client=GatewayClient(*remote.address,token=robot_gateway.token,timeout=.12)
    relay=LatestRelay(pc.tracking,relay_client)
    control=Teleoperation(session.robot,DualArmModel(),Kinematics(),recorder=recorder)
    active_seen=dict(left=False,right=False);released=None
    previous=time.monotonic()-.02
    try:
        for sequence in range(100):
            started=time.monotonic()
            client.send(frame(sequence));relay.step()
            sample=robot_gateway.tracking.sample()
            assert sample is not None and sample.focused,relay.status
            now=time.monotonic();status=control.tick(sample,now-previous);previous=now
            assert status['state']=='dry_run',status
            for side,active in status['active'].items():active_seen[side]|=active
            if sequence==82:released=session.robot.read_state().joint_positions
            time.sleep(max(0,.02-(time.monotonic()-started)))
        assert all(active_seen.values()),active_seen
        assert min(counts.values())>=3,counts
        assert session.robot.read_state().joint_positions==released
        assert status['hands']['closure']['right']>0 and status['hands']['closure']['left']==0
        assert relay.status['forwarded']==100,relay.status
        names=[name for name,_ in session.node.get_publisher_names_and_types_by_node(session.node.get_name(),session.node.get_namespace())]
        assert '/arm/cmd' not in names and not any('set_motor_multi' in name for name in names)
        result=dict(passed=True,version=__version__,real_native_placo=True,real_native_ros2=True,
            source_bundle_sha256=hashlib.file_digest(Path('/home/tg3/project.zip').open('rb'),'sha256').hexdigest(),
            actual_two_hop_http_frames=relay.status['forwarded'],actual_ros_messages=counts,
            both_arms_activated=active_seen,release_hold_verified=True,continuous_grasp_retained=True,
            recorded_steps=recorder.steps,recording=str(recorder.directory),publisher_topics=names,
            input='SYNTHETIC_OPENXR',headset_tested=False,physical_robot_tested=False,robot_commands_sent=False)
        Path('artifacts/native_pipeline_result.json').write_text(json.dumps(result,indent=2),encoding='utf-8')
        print(json.dumps(result,indent=2))
    finally:
        client.close();relay_client.close();remote.close();server.close()
        control.disarm('native_integration_finished');recorder.close()
        session.executor.remove_node(observer);observer.destroy_node();session.close()


if __name__=='__main__':
    try:main()
    except Exception as exc:
        result=dict(passed=False,version=__version__,error=str(exc),traceback=traceback.format_exc(),
            headset_tested=False,physical_robot_tested=False,robot_commands_sent=False)
        source=Path('/home/tg3/project.zip')
        if source.exists():result['source_bundle_sha256']=hashlib.file_digest(source.open('rb'),'sha256').hexdigest()
        Path('artifacts').mkdir(exist_ok=True)
        Path('artifacts/native_pipeline_result.json').write_text(json.dumps(result,indent=2),encoding='utf-8')
        raise
