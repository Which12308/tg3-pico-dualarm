"""Run with an independently installed wheel directory as the first argument."""
import json
from pathlib import Path
import sys
import tempfile

ROOT=Path(__file__).resolve().parents[1]
target=Path(sys.argv[1]).resolve()
sys.path.insert(0,str(target))
import tg3_pico_dualarm
from tg3_pico_dualarm.cli import demo
from tg3_pico_dualarm.configuration import load
from tg3_pico_dualarm.model import DualArmModel,default_q
from tg3_pico_dualarm.wireless import InputGateway,WirelessServer,GatewayClient

assert target in Path(tg3_pico_dualarm.__file__).resolve().parents
config=load(ROOT/'config');assert config['vr']['translation_scale']==1.
model=DualArmModel();assert set(model.fk(default_q()))=={'left','right'}
with tempfile.TemporaryDirectory() as folder:
    result=demo(steps=80,preview=True,record=Path(folder))
    assert result['passed'] and not result['robot_commands_sent']
gateway=InputGateway();server=WirelessServer(gateway,'127.0.0.1',0).start()
client=GatewayClient(*server.address,timeout=1)
try:
    info=client.request('GET','/v1/info');assert info['version']==tg3_pico_dualarm.__version__
finally:client.close();server.close()
share=target/'share'
for name in ('ament_index/resource_index/packages/tg3_pico_dualarm',
             'tg3_pico_dualarm/package.xml','tg3_pico_dualarm/config/vr.yaml'):
    assert (share/name).is_file(),name
assets=target/'tg3_pico_dualarm/web_assets'
assert (assets/'index.html').is_file()
output=dict(passed=True,version=tg3_pico_dualarm.__version__,installed_from_wheel=True,
    imported_installed_package=True,model_assets=True,ros_share_metadata=True,
    actual_wireless_http=True,preview_demo=True,native_placo_executed=False,robot_commands_sent=False)
(ROOT/'artifacts/install_result.json').write_text(json.dumps(output,indent=2),encoding='utf-8')
print(json.dumps(output,indent=2))
