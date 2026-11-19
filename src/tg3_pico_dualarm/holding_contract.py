"""Read installed holding/stance configuration without starting ROS or motion.

The report is evidence for review, never an actuator-enable permit.
"""
import json
import shlex


READ_HOLDING = r'''
import hashlib,json,os
from pathlib import Path
root=Path('/home/ubuntu/.local/lib/python3.12/site-packages/xmigcs/config').resolve()
result=dict(passive_only=True,robot_commands_sent=False,files=[],processes=[],errors=[])
names={'stop.yaml','standzero.yaml','hbwalk.yaml','dex_config.yaml','control_tool.yaml'}
if root.is_dir():
 for path in sorted(root.rglob('*.yaml')):
  if path.name.lower() not in names:continue
  actual=path.resolve()
  if not actual.is_relative_to(root):continue
  if len(result['files'])>=24:raise RuntimeError('too_many_holding_config_files')
  if actual.stat().st_size>262144:raise RuntimeError('holding_config_file_too_large')
  raw=actual.read_bytes()
  result['files'].append(dict(path=str(actual),sha256=hashlib.sha256(raw).hexdigest(),text=raw.decode('utf-8')))
else:result['errors'].append('installed_xmigcs_config_missing')
for folder in Path('/proc').iterdir():
 if not folder.name.isdigit():continue
 try:
  args=[v.decode('utf-8',errors='replace') for v in (folder/'cmdline').read_bytes().split(b'\0') if v]
  selected=next((a for a in args if any(s in a for s in ('robot_control_ros2_bridge','body_control','joystick_bridge_node_tg3','ros2_control_node'))),None)
  if selected and not any(a in ('-c','--command') for a in args):
   result['processes'].append(dict(pid=int(folder.name),executable=selected))
 except (OSError,ValueError):pass
print(json.dumps(result,ensure_ascii=True))
'''


def fetch_holding_contract(deployment,password):
    if not isinstance(password,str) or not password or any(c in password for c in '\r\n\0'):
        raise ValueError('current_session_password_required')
    command='sudo -k -S -p '+shlex.quote('')+' /usr/bin/python3 -s -c '+shlex.quote(READ_HOLDING)
    raw=deployment.command(command,timeout=15,stdin_data=(password+'\n').encode())
    report=json.loads(raw.strip().splitlines()[-1])
    if (report.get('passive_only') is not True or report.get('robot_commands_sent') is not False
            or not isinstance(report.get('files'),list) or len(report['files'])>24):
        raise ValueError('invalid_passive_holding_report')
    return report
