"""Bounded passive source inventory; no ROS or vendor import is executed."""
READ_HOLDING_SOURCES = r'''
import hashlib,json
from pathlib import Path
root=Path('/home/ubuntu/.local/lib/python3.12/site-packages/xmigcs').resolve()
result=dict(passive_only=True,robot_commands_sent=False,files=[],inventory=[],errors=[])
for extension in ('*.yaml','*.yml'):
 for path in sorted(root.rglob(extension)):
  actual=path.resolve()
  if not actual.is_relative_to(root):continue
  relative=actual.relative_to(root).as_posix()
  if len(result['inventory'])<250:result['inventory'].append(relative)
  lowered=relative.lower()
  if not any(word in lowered for word in ('stop','standzero','hbwalk')):continue
  if len(result['files'])>=24 or actual.stat().st_size>262144:
   result['errors'].append('holding_source_limit:'+relative);continue
  raw=actual.read_bytes()
  result['files'].append(dict(path=str(actual),sha256=hashlib.sha256(raw).hexdigest(),text=raw.decode('utf-8')))
print(json.dumps(result,ensure_ascii=True))
'''
