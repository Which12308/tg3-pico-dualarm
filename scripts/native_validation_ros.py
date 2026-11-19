"""Build and run the real Jazzy dry-run bridge in the owned Linux guest.

This does not install proprietary robot messages, authorize hardware or claim
robot feedback. DDS is restricted to the guest's own loopback interface.
"""
from datetime import datetime,timezone
import argparse
import json
import shlex
import traceback
import urllib.request
import urllib.parse
from native_validation_vm import ROOT,ENV,connect,command

APT_SOURCE_URL='https://github.com/ros-infrastructure/ros-apt-source/releases/download/1.3.0/ros2-apt-source_1.3.0.noble_all.deb'
APT_SOURCE_SHA='f31d84adf5054c7d60ded0e82c0f776a77ea33af53d08f40b9ad7c94cca55296'


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--skip-install',action='store_true')
    parser.add_argument('--pipeline',action='store_true',help='Also run real HTTP/IK/ROS/recording integration with synthetic input')
    args=parser.parse_args()
    proxy=''
    for name,value in urllib.request.getproxies().items():
        url=urllib.parse.urlparse(value)
        if name in ('https','all') and url.hostname in ('127.0.0.1','localhost') and url.username is None and url.password is None and url.port:
            proxy='http://10.0.2.2:'+str(url.port)
            break
    environment=''
    if proxy:environment='https_proxy='+shlex.quote(proxy)+' http_proxy='+shlex.quote(proxy)+' HTTPS_PROXY='+shlex.quote(proxy)+' HTTP_PROXY='+shlex.quote(proxy)+' '
    install='''set -eo pipefail
PROXYpython3 - <<'PY'
import urllib.request,hashlib,pathlib
p=pathlib.Path('/home/tg3/ros2-apt-source.deb')
url=URL
if not p.exists():p.write_bytes(urllib.request.urlopen(url,timeout=60).read())
assert hashlib.sha256(p.read_bytes()).hexdigest()==SHA
PY
sudo dpkg -i /home/tg3/ros2-apt-source.deb
sudo PROXYDEBIAN_FRONTEND=noninteractive apt-get update -qq
sudo PROXYDEBIAN_FRONTEND=noninteractive apt-get install -y ros-jazzy-ros-base python3-colcon-common-extensions
'''.replace('PROXY',environment).replace('URL',repr(APT_SOURCE_URL)).replace('SHA',repr(APT_SOURCE_SHA))
    # Probe and build are separate so a build failure never masquerades as a
    # package-install failure or a successful manufacturer's hardware check.
    build='''set -eo pipefail
source /opt/ros/jazzy/setup.bash
export ROS_LOCALHOST_ONLY=1
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
cd /home/tg3/project
mkdir -p artifacts/native_ros_build
colcon --log-base artifacts/native_ros_build/log build --base-paths /home/tg3/project --packages-select tg3_pico_dualarm --build-base artifacts/native_ros_build/build --install-base artifacts/native_ros_build/install
source artifacts/native_ros_build/install/setup.bash
ros2 pkg prefix tg3_pico_dualarm
export LD_LIBRARY_PATH="$PWD/.venv/lib/python3.12/site-packages/cmeel.prefix/lib:${LD_LIBRARY_PATH:-}"
.venv/bin/python -s - <<'PY'
import json,pathlib,time,hashlib
from dataclasses import asdict
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from tg3_pico_dualarm.ros_bridge import RosDryRunSession,DRY_RUN_JOINT_TOPIC,DIAGNOSTIC_TOPIC
from tg3_pico_dualarm.robot import JointCommand
from tg3_pico_dualarm import __version__
session=RosDryRunSession(observe_robot=False)
observer=Node('tg3_actual_jazzy_validation',context=session.context,enable_rosout=False)
counts={'joints':0,'diagnostics':0};payload=[]
def joints(message):
 assert len(message.name)==14 and len(message.position)==14
 counts['joints']+=1
def diagnostics(message):
 value=json.loads(message.data)
 assert value['hardware_writes_enabled'] is False and value['robot_commands_sent'] is False
 counts['diagnostics']+=1;payload[:]=[value]
subscriptions=[observer.create_subscription(JointState,DRY_RUN_JOINT_TOPIC,joints,10),observer.create_subscription(String,DIAGNOSTIC_TOPIC,diagnostics,10)]
session.executor.add_node(observer)
try:
 state=session.robot.read_state();now=time.monotonic()
 result=session.robot.submit(JointCommand(state.joint_positions,(0.,)*14,now))
 assert result.accepted
 deadline=time.monotonic()+8
 while min(counts.values())<3 and time.monotonic()<deadline:time.sleep(.05)
 assert min(counts.values())>=3,counts
 topics=session.node.get_publisher_names_and_types_by_node(session.node.get_name(),session.node.get_namespace())
 names=[name for name,_ in topics]
 assert '/arm/cmd' not in names and not any('set_motor_multi' in name for name in names)
 output=dict(passed=True,version=__version__,ros_distro='jazzy',actual_rclpy_file=rclpy.__file__,actual_messages=counts,publisher_topics=names,
   source_bundle_sha256=hashlib.file_digest(pathlib.Path('/home/tg3/project.zip').open('rb'),'sha256').hexdigest(),
   hardware_motion_enabled=False,manufacturer_messages_installed=False,robot_commands_sent=False,
   physical_robot_tested=False,mode='actual_ROS2_native_dry_run',DDS='guest_loopback_only')
 pathlib.Path('artifacts/native_ros_runtime_result.json').write_text(json.dumps(output,indent=2),encoding='utf-8')
 print(json.dumps(output,indent=2))
finally:
 session.executor.remove_node(observer);observer.destroy_node();session.close()
PY
'''
    result=dict(started=datetime.now(timezone.utc).isoformat(),ros2_build_executed=False,ros2_runtime_executed=False,
                ros2_build_attempted=False,
                pipeline_requested=args.pipeline,pipeline_passed=False,
                manufacturer_messages_installed=False,hardware_motion_enabled=False,robot_commands_sent=False)
    client=connect(20)
    try:
        if not args.skip_install:command(client,'bash -c '+shlex.quote(install),timeout=1800,log_name='ros_install.log')
        result['ros2_build_attempted']=True
        command(client,'bash -c '+shlex.quote(build),timeout=600,log_name='ros_build_run.log')
        sftp=client.open_sftp()
        sftp.get('/home/tg3/project/artifacts/native_ros_runtime_result.json',str(ROOT/'artifacts/native_ros_runtime_result.json'))
        sftp.close()
        runtime=json.loads((ROOT/'artifacts/native_ros_runtime_result.json').read_text(encoding='utf-8'))
        result.update(ros2_build_executed=True,ros2_runtime_executed=runtime['passed'],runtime=runtime,
                      version=runtime['version'],source_bundle_sha256=runtime['source_bundle_sha256'])
        if args.pipeline:
            pipeline='''set -eo pipefail
source /opt/ros/jazzy/setup.bash
export ROS_LOCALHOST_ONLY=1
cd /home/tg3/project
export LD_LIBRARY_PATH="$PWD/.venv/lib/python3.12/site-packages/cmeel.prefix/lib:${LD_LIBRARY_PATH:-}"
.venv/bin/python -s scripts/native_validation_pipeline.py
'''
            try:command(client,'bash -c '+shlex.quote(pipeline),timeout=45,log_name='native_pipeline.log')
            finally:
                sftp=client.open_sftp()
                try:sftp.get('/home/tg3/project/artifacts/native_pipeline_result.json',str(ROOT/'artifacts/native_pipeline_result.json'))
                finally:sftp.close()
            value=json.loads((ROOT/'artifacts/native_pipeline_result.json').read_text(encoding='utf-8'))
            if value['source_bundle_sha256']!=runtime['source_bundle_sha256']:
                raise RuntimeError('pipeline source snapshot mismatch')
            result['pipeline_passed']=value['passed']
    except Exception as exc:result.update(error=str(exc),traceback=traceback.format_exc())
    finally:
        client.close();result['finished']=datetime.now(timezone.utc).isoformat()
        (ROOT/'artifacts/native_ros_result.json').write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf-8')
    print(json.dumps(result,ensure_ascii=False,indent=2))
    return 0 if result['ros2_runtime_executed'] and not result.get('error') and (not args.pipeline or result['pipeline_passed']) else 2


if __name__=='__main__':raise SystemExit(main())
