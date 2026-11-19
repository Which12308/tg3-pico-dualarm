"""Read bounded existing project fault logs; never start ROS or controllers."""
import json
import shlex

READ_SCRIPT = r'''
import json
from pathlib import Path
root=Path('/home/ubuntu/.local/share/tg3-teleop/diagnostics')
report={'scope':'read_existing_project_fault_logs_only','control_services_called':False,'files':{}}
for name in ('factory.log','remapped_bridge.log','restored_bridge.log'):
    path=root/name
    try:
        if path.is_symlink():raise ValueError('symlink_refused')
        with path.open('rb') as f:
            f.seek(0,2);size=f.tell();f.seek(max(0,size-65536));data=f.read(65536)
        report['files'][name]={'size_bytes':size,'tail_utf8':data.decode('utf-8','replace')}
    except (OSError,ValueError) as e:report['files'][name]={'error':str(e)}
print(json.dumps(report,ensure_ascii=False))
'''


def fetch_existing_logs(connection,password):
    if not isinstance(password,str) or not password or any(c in password for c in '\r\n\0'):
        raise ValueError('本次读取需要Ubuntu管理员密码，只经内存和SSH标准输入传递')
    command="sudo -k -S -p '' /usr/bin/python3 -c "+shlex.quote(READ_SCRIPT)
    text=connection.command(command,timeout=30,stdin_data=(password+'\n').encode())
    result=json.loads(text.strip())
    if result.get('scope')!='read_existing_project_fault_logs_only' or result.get('control_services_called') is not False:
        raise ValueError('unexpected_incident_report_scope')
    # Include current installed stance sources with the existing fault context.
    # This is a separate bounded read, not a ROS startup or a control request.
    from .holding_sources import READ_HOLDING_SOURCES
    command="sudo -k -S -p '' /usr/bin/python3 -s -c "+shlex.quote(READ_HOLDING_SOURCES)
    sources=connection.command(command,timeout=20,stdin_data=(password+'\n').encode())
    evidence=json.loads(sources.strip().splitlines()[-1])
    if evidence.get('passive_only') is not True or evidence.get('robot_commands_sent') is not False:
        raise ValueError('unexpected_holding_source_report_scope')
    result['holding_configuration']=evidence
    return result
