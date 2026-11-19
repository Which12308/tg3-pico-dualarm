"""Actual loopback tunnel and isolated SSH/SFTP fixtures; no robot/ADB calls."""
import json
from pathlib import Path
import socket
import sys
import tempfile
from types import SimpleNamespace as NS
import unittest
from unittest.mock import patch

from tg3_pico_dualarm.deployment import RobotDeployment,Tunnel,tcp_nodelay
from tg3_pico_dualarm.wireless import GatewayClient,InputGateway,WirelessServer


class TransportCaptureTests(unittest.TestCase):
    def test_station_failure_is_preserved_with_exit_code_after_split_stderr(self):
        logs=[];parts=[b'{"event":"station_failed","error":"watchdog_',
            b'expired_restart_required","hardware_motion_enabled":false}\n']
        class Channel:
            closed=False
            def recv_ready(self):return False
            def recv(self,_):return b''
            def recv_stderr_ready(self):return bool(parts)
            def recv_stderr(self,_):return parts.pop(0)
            def exit_status_ready(self):return not parts
            def recv_exit_status(self):return 2
        deployment=RobotDeployment('.',Path('unused'),lambda *a:False,logs.append)
        deployment.channel=Channel();deployment._drain()
        self.assertEqual(deployment.station_exit['exit_code'],2)
        self.assertEqual(deployment.station_exit['reason'],'watchdog_expired_restart_required')
        self.assertIsNone(deployment.station_exit['ssh_transport_active'])
        self.assertTrue(any('watchdog_expired_restart_required' in value for value in logs))

    def test_channel_loss_is_distinguished_from_a_confirmed_process_exit(self):
        class Channel:
            closed=False
            def recv_ready(self):raise EOFError('fixture transport lost')
            def recv(self,_):return b''
            def recv_stderr_ready(self):return False
            def recv_stderr(self,_):return b''
        deployment=RobotDeployment('.',Path('unused'),lambda *a:False,lambda *a:None)
        deployment.channel=Channel();deployment._drain()
        self.assertIsNone(deployment.station_exit['exit_code'])
        self.assertIn('station_channel_lost',deployment.station_exit['reason'])

    def test_closed_channel_still_reports_its_buffered_exit_status(self):
        channel=NS(closed=True,recv_ready=lambda:False,recv=lambda _:b'',
            recv_stderr_ready=lambda:False,recv_stderr=lambda _:b'',
            exit_status_ready=lambda:True,recv_exit_status=lambda:7)
        deployment=RobotDeployment('.',Path('unused'),lambda *a:False,lambda *a:None)
        deployment.channel=channel;deployment._drain()
        self.assertEqual(deployment.station_exit['exit_code'],7)
        self.assertEqual(deployment.station_exit['reason'],'station_exit:7')
        self.assertTrue(deployment.station_exit['channel_closed'])

    def test_missing_exit_status_keeps_carrier_failure_diagnostics(self):
        channel=NS(closed=True,eof_received=True,recv_ready=lambda:False,recv=lambda _:b'',
            recv_stderr_ready=lambda:False,recv_stderr=lambda _:b'',
            exit_status_ready=lambda:True,recv_exit_status=lambda:-1)
        transport=NS(is_active=lambda:False,get_exception=lambda:EOFError('fixture carrier lost'))
        deployment=RobotDeployment('.',Path('unused'),lambda *a:False,lambda *a:None)
        deployment.ssh=NS(get_transport=lambda:transport)
        deployment.channel=channel;deployment._drain()
        self.assertEqual(deployment.station_exit['exit_code'],-1)
        self.assertFalse(deployment.station_exit['ssh_transport_active'])
        self.assertEqual(deployment.station_exit['ssh_error'],'fixture carrier lost')
        self.assertTrue(deployment.station_exit['channel_eof_received'])

    def test_real_tunnel_accepted_tcp_socket_disables_nagle_and_passes_readonly_status(self):
        gateway=InputGateway(role='relay',key='x'*40)
        upstream=WirelessServer(gateway,'127.0.0.1',0).start();seen=[]
        class Transport:
            def open_channel(self,*args,**kwargs):
                sock=socket.create_connection(upstream.address,timeout=1)
                def forbidden_fileno():raise AssertionError('tunnel must not create a Paramiko readiness pipe')
                return NS(recv=sock.recv,sendall=sock.sendall,settimeout=sock.settimeout,
                    close=sock.close,fileno=forbidden_fileno)
        def checked(sock):
            result=tcp_nodelay(sock)
            seen.append((isinstance(sock,socket.socket),sock.getsockopt(socket.IPPROTO_TCP,socket.TCP_NODELAY)))
            return result
        tunnel=client=None
        try:
            with patch('tg3_pico_dualarm.deployment.tcp_nodelay',side_effect=checked):
                tunnel=Tunnel(Transport(),upstream.address[1]);client=GatewayClient('127.0.0.1',tunnel.port,'x'*40,timeout=1)
                self.assertEqual(client.request('GET','/v1/status')['role'],'relay')
                self.assertEqual(seen,[(True,1)])
        finally:
            if client:client.close()
            if tunnel:tunnel.close()
            upstream.close()

    def test_ssh_tunes_only_actual_carrier_socket_and_never_channel_like_objects(self):
        with socket.socket(socket.AF_INET,socket.SOCK_STREAM) as carrier,tempfile.TemporaryDirectory() as directory:
            fake=NS(connect=lambda *a,**k:None,set_missing_host_key_policy=lambda p:None,
                get_transport=lambda:NS(sock=carrier),close=lambda:None)
            deployment=RobotDeployment(directory,Path(directory)/'known_hosts',lambda *a:False,lambda *a:None)
            with patch.dict(sys.modules,{'paramiko':NS(MissingHostKeyPolicy=object,SSHClient=lambda:fake)}):
                deployment.connect('127.0.0.1','fixture','fixture-password-not-logged')
            self.assertEqual(carrier.getsockopt(socket.IPPROTO_TCP,socket.TCP_NODELAY),1)
            channel=NS(setsockopt=lambda *a:self.fail('not a real TCP socket'))
            self.assertFalse(tcp_nodelay(channel))

    def capture_fixture(self,root,value,command_error=None):
        script=root/'scripts/inspect_factory_readonly.py';script.parent.mkdir(parents=True);script.write_text('# fixture only')
        raw=json.dumps(value).encode();commands=[]
        class Sftp:
            def get_channel(self):return NS(settimeout=lambda _:None)
            def stat(self,name):return NS(st_size=len(raw))
            def get(self,remote,local):Path(local).write_bytes(raw)
            def close(self):pass
        deployment=RobotDeployment(root,root/'unused',lambda *a:False,lambda *a:None)
        deployment.ssh=NS(open_sftp=lambda:Sftp());deployment.remote='/home/ubuntu/project with spaces'
        def command(value,timeout):
            commands.append(value)
            if command_error:raise RuntimeError(command_error)
            return '{}'
        deployment.command=command
        return deployment,commands

    def test_readonly_capture_downloads_independent_json_and_reports_only_summary(self):
        value=dict(passive_only=True,hardware_motion_enabled=False,robot_commands_sent=False,
            motion_publishers_created=[],motion_services_called=[],graph=dict(topic_types={'/robot_state':['test/State']},
            arm_consumers={'/actual_consumer':{}}),parsed_feedback={'fixture':True})
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory).resolve();deployment,commands=self.capture_fixture(root,value)
            result=deployment.capture_interfaces(seconds=5)
            self.assertTrue(result['ok']);self.assertEqual(result['summary']['graph_topics'],1)
            self.assertEqual(result['summary']['arm_consumers'],['/actual_consumer'])
            self.assertEqual(json.loads(Path(result['path']).read_text()),value)
            self.assertEqual(Path(result['path']).parent,root/'.desktop/robot_reports')
            self.assertIn('xos/setup.bash',commands[0]);self.assertIn('.venv/bin/python',commands[0])
            self.assertIn('inspect_factory_readonly.py',commands[0]);self.assertNotIn('token',commands[0].lower())
            self.assertNotIn('password',commands[0].lower())

    def test_failed_collector_retains_its_passive_error_report_and_unverified_scope_is_rejected(self):
        value=dict(passive_only=True,hardware_motion_enabled=False,robot_commands_sent=False,
            motion_publishers_created=[],motion_services_called=[],error='fixture ROS unavailable')
        with tempfile.TemporaryDirectory() as directory:
            deployment,_=self.capture_fixture(Path(directory).resolve(),value,'robot_command_failed:2')
            result=deployment.capture_interfaces(seconds=5)
            self.assertFalse(result['ok']);self.assertTrue(Path(result['path']).exists())
        with tempfile.TemporaryDirectory() as directory:
            deployment,_=self.capture_fixture(Path(directory).resolve(),dict(value,hardware_motion_enabled=True))
            with self.assertRaisesRegex(ValueError,'safety_scope'):deployment.capture_interfaces(seconds=5)
            self.assertEqual(list((Path(directory)/'.desktop/robot_reports').iterdir()),[])


if __name__=='__main__':unittest.main()
