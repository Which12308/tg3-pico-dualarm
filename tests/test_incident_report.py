import json
import unittest
from tg3_pico_dualarm.incident_report import fetch_existing_logs,READ_SCRIPT


class IncidentReportTests(unittest.TestCase):
    def test_password_only_on_stdin_and_no_ros_or_hardware_start(self):
        calls=[]
        class Reader:
            def command(self,command,**kwargs):
                calls.append((command,kwargs))
                if len(calls)==2:return json.dumps(dict(passive_only=True,robot_commands_sent=False,files=[]))
                return json.dumps(dict(scope='read_existing_project_fault_logs_only',control_services_called=False,files={}))
        fetch_existing_logs(Reader(),'test-credential')
        command,options=calls[0]
        self.assertNotIn('test-credential',command)
        self.assertEqual(options['stdin_data'],b'test-credential\n')
        self.assertIn('/usr/bin/python3 -c',command)
        for forbidden in ('import rclpy','systemctl','Popen','set_arm_enable','station','os.kill'):
            self.assertNotIn(forbidden,READ_SCRIPT)
        self.assertIn('65536',READ_SCRIPT)

    def test_empty_or_multiline_credentials_refused_before_connection_command(self):
        for value in ('','a\nb',None):
            with self.assertRaises(ValueError):fetch_existing_logs(None,value)
