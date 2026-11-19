"""Actual hidden Tk/HTTP/recording smoke. Never connects to SSH or a robot."""
import json
from pathlib import Path
import sys
import tempfile

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'src'),str(ROOT/'tests'),str(ROOT/'desktop')]
from test_wireless import native
from tg3_pico_dualarm.wireless import GatewayClient
from start import Desk


def run():
    (ROOT/'artifacts').mkdir(exist_ok=True)
    dependencies={}
    if '--require-desktop-dependencies' in sys.argv:
        import paramiko
        import numpy
        import cryptography
        import mcap
        ssh=paramiko.SSHClient();ssh.close()  # Construct only, no connect.
        dependencies=dict(paramiko=paramiko.__version__,numpy=numpy.__version__,cryptography=cryptography.__version__)
    with tempfile.TemporaryDirectory() as folder:
        desk=Desk(visible=False,port=0,record_root=folder)
        client=GatewayClient('127.0.0.1',desk.server.address[1],timeout=1.)
        try:
            pin=desk.gateway.open_pairing()
            client.token=client.request('POST','/v1/pair',dict(protocol='tg3-native-1',pin=pin,device='test'))['token']
            for i in range(3):client.send(native(i))
            desk.poll();desk.window.update_idletasks()
            assert desk.gateway.tracking.status()['counters']['accepted']==3
            assert desk.recorder.steps==0 and desk.gateway.on_frame is None
            assert desk.manual_recording.process is None
            assert desk.deployment is None and desk.relay is None
            assert 'False' not in desk.state.get().split('已收帧')[0]
        finally:
            client.close();desk.close()
        assert not list(Path(folder).iterdir())
        result=dict(passed=True,actual_hidden_tk=True,actual_socket_frames=3,recorded_frames=0,automatic_recording=False,
                    ssh_started=False,robot_commands_sent=False,headset_tested=False,
                    isolated_desktop_environment=sys.prefix!=sys.base_prefix,desktop_dependencies=dependencies)
        (ROOT/'artifacts/desktop_result.json').write_text(json.dumps(result,indent=2),encoding='utf-8')
        print(json.dumps(result,indent=2))


if __name__=='__main__':run()
