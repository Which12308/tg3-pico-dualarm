from types import SimpleNamespace
from pathlib import Path
import unittest
from unittest.mock import Mock,patch
from tg3_pico_dualarm.cli import serve,main


class CliTests(unittest.TestCase):
    def test_native_start_failure_closes_input_socket_without_starting_threads(self):
        server=Mock();args=SimpleNamespace(port=0,preview=False,mode='pose',configuration=None)
        with patch('tg3_pico_dualarm.web_input.InputServer',return_value=server), \
             patch('tg3_pico_dualarm.cli.native_engine',side_effect=RuntimeError('native_unavailable')):
            with self.assertRaisesRegex(RuntimeError,'native_unavailable'):serve(args)
        server.httpd.server_close.assert_called_once()
        server.serve_forever.assert_not_called()

    def test_explicit_mode_and_scale_override_configured_defaults(self):
        config=Path(__file__).resolve().parents[1]/'config'
        with patch('tg3_pico_dualarm.cli.serve',return_value=0) as run:
            self.assertEqual(main(['serve','--config',str(config),'--mode','position','--scale','1']),0)
        args=run.call_args.args[0]
        self.assertEqual(args.mode,'position');self.assertEqual(args.scale,1.)
        self.assertEqual(args.port,8890)
