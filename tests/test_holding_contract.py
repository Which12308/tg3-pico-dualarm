import contextlib
import io
import json
from pathlib import Path
import tempfile
from types import SimpleNamespace as NS
import unittest
from tg3_pico_dualarm.holding_contract import READ_HOLDING,fetch_holding_contract


class HoldingContractTests(unittest.TestCase):
    def test_read_only_collection_filters_config_paths_and_process_arguments(self):
        with tempfile.TemporaryDirectory() as folder:
            root=Path(folder);configs=root/'configs';configs.mkdir();proc=root/'proc';proc.mkdir()
            (configs/'stop.yaml').write_text('arm: [1, 2]',encoding='utf-8')
            (configs/'unrelated.yaml').write_text('private',encoding='utf-8')
            process=proc/'12';process.mkdir()
            (process/'cmdline').write_bytes(b'/bin/body_control\0--secret\0do-not-collect\0')
            script=READ_HOLDING.replace("'/home/ubuntu/.local/lib/python3.12/site-packages/xmigcs/config'",repr(str(configs)))
            script=script.replace("Path('/proc')",'Path('+repr(str(proc))+')')
            output=io.StringIO()
            with contextlib.redirect_stdout(output):exec(compile(script,'holding_readback','exec'),{})
            result=json.loads(output.getvalue())
            self.assertEqual(len(result['files']),1)
            self.assertEqual(result['files'][0]['text'],'arm: [1, 2]')
            self.assertNotIn('do-not-collect',output.getvalue())
            self.assertEqual(result['processes'],[dict(pid=12,executable='/bin/body_control')])
            self.assertFalse(result['robot_commands_sent'])

    def test_authentication_stays_in_stdin_and_no_ros_or_enable_command_is_run(self):
        seen=[]
        def command(value,**kwargs):
            seen.append((value,kwargs))
            return 'sudo diagnostic\n'+json.dumps(dict(passive_only=True,robot_commands_sent=False,files=[]))
        result=fetch_holding_contract(NS(command=command),'test-password')
        self.assertTrue(result['passive_only'])
        self.assertNotIn('test-password',seen[0][0]);self.assertEqual(seen[0][1]['stdin_data'],b'test-password\n')
        self.assertNotIn('ros2 ',seen[0][0]);self.assertNotIn('set_arm_mode',seen[0][0])

    def test_incompatible_report_is_rejected(self):
        with self.assertRaises(ValueError):
            fetch_holding_contract(NS(command=lambda *a,**kw:'{"passive_only":false,"files":[]}'),'password')
