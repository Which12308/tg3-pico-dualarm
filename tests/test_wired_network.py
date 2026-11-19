"""Network fixtures and loopback only. No robot connection or OS changes."""
import copy
import json
import os
from pathlib import Path
import socket
import struct
import tempfile
from types import SimpleNamespace as NS
import unittest
from unittest.mock import patch
from tg3_pico_dualarm.wired_network import evaluate,WindowsWiredNetwork,target_ipv4,read_robot_interfaces,INVENTORY_SCRIPT
from tg3_pico_dualarm.deployment import RobotDeployment


def snapshot():
    return dict(adapters=[dict(ifIndex=5,Name='Ethernet',HardwareInterface=True,NdisPhysicalMedium=14,Status='Up'),
        dict(ifIndex=41,Name='Meta',HardwareInterface=False,NdisPhysicalMedium=0,Status='Up')],
        addresses=[dict(InterfaceIndex=5,IPAddress='10.12.0.2',PrefixLength=24,AddressState='Preferred')],
        route=dict(InterfaceIndex=41,DestinationPrefix='10.0.0.0/8',NextHop='198.18.0.2'),
        wired_routes=[dict(source=dict(InterfaceIndex=5,IPAddress='10.12.0.2'),
            route=dict(InterfaceIndex=5,DestinationPrefix='10.12.0.0/24',NextHop='0.0.0.0'))])


class SocketDouble:
    def __init__(self,*args):self.trace=[];self.local=None;self.closed=False
    def settimeout(self,v):self.trace.append(('timeout',v))
    def setsockopt(self,*args):self.trace.append(('option',args))
    def bind(self,local):self.local=local;self.trace.append(('bind',local))
    def connect(self,remote):self.trace.append(('connect',remote))
    def getsockname(self):return self.local
    def close(self):self.closed=True


class WiredNetworkTests(unittest.TestCase):
    @unittest.skipUnless(os.name=='nt','Real Windows address-state serialization check')
    def test_real_windows_inventory_serializes_address_state_names(self):
        # Find-NetRoute only reads local routing state; inspect opens no socket.
        report=WindowsWiredNetwork().inspect('192.0.2.9')
        for address in report['addresses']:
            self.assertIsInstance(address['AddressState'],str)
            self.assertIn(address['AddressState'],('Invalid','Tentative','Duplicate','Deprecated','Preferred'))

    def test_local_gateway_does_not_use_system_http_proxy_environment(self):
        # Real HTTP over loopback. This does not emulate a TUN/WFP driver.
        from tg3_pico_dualarm.wireless import InputGateway,WirelessServer,GatewayClient
        gateway=InputGateway(role='relay',key='p'*40)
        server=WirelessServer(gateway,'127.0.0.1',0).start()
        client=GatewayClient(*server.address,token='p'*40,timeout=1)
        try:
            with patch.dict('os.environ',HTTP_PROXY='http://127.0.0.1:1',HTTPS_PROXY='http://127.0.0.1:1',ALL_PROXY='socks5://127.0.0.1:1',NO_PROXY=''):
                result=client.request('GET','/v1/challenge')
            self.assertIn('nonce',result)
        finally:client.close();server.close()

    def test_virtual_default_route_can_stay_while_explicit_wired_route_passes(self):
        original=snapshot();before=copy.deepcopy(original)
        result=evaluate(original,'10.12.0.9')
        self.assertTrue(result['ready']);self.assertTrue(result['system_route_differs'])
        self.assertEqual(result['interface_index'],5);self.assertEqual(original,before)

    def test_missing_cable_wrong_subnet_virtual_source_and_gateway_refused(self):
        cases=[]
        data=snapshot();data['adapters'][0]['Status']='Disconnected';cases.append(data)
        data=snapshot();data['addresses'][0]['IPAddress']='169.254.1.2';cases.append(data)
        data=snapshot();data['wired_routes'][0]['source']['InterfaceIndex']=41;cases.append(data)
        data=snapshot();data['wired_routes'][0]['route']['NextHop']='10.12.0.1';cases.append(data)
        data=snapshot();data['addresses'][0]['AddressState']='Duplicate';cases.append(data)
        data=snapshot();data['adapters'][0]['HardwareInterface']=False;cases.append(data)
        for data in cases:self.assertFalse(evaluate(data,'10.12.0.9')['ready'])
        self.assertFalse(evaluate(snapshot(),'10.99.0.9')['ready'])
        self.assertFalse(evaluate(snapshot(),'10.12.0.2')['ready'])

    def test_ambiguous_physical_ports_require_explicit_choice(self):
        data=snapshot();adapter=copy.deepcopy(data['adapters'][0]);adapter['ifIndex']=6
        data['adapters'].append(adapter)
        data['addresses'].append(dict(InterfaceIndex=6,IPAddress='10.12.0.3',PrefixLength=24,AddressState='Preferred'))
        route=copy.deepcopy(data['wired_routes'][0]);route['route']['InterfaceIndex']=6
        route['source']=dict(InterfaceIndex=6,IPAddress='10.12.0.3');data['wired_routes'].append(route)
        self.assertFalse(evaluate(data,'10.12.0.9')['ready'])
        self.assertEqual(evaluate(data,'10.12.0.9',6)['source_ipv4'],'10.12.0.3')

    def test_source_and_windows_interface_are_bound_before_any_connect(self):
        created=[]
        def factory(*args):created.append(SocketDouble());return created[-1]
        network=WindowsWiredNetwork(socket_factory=factory)
        report=evaluate(snapshot(),'10.12.0.9');connection=network.connect('10.12.0.9',report=report)
        self.assertIn(('option',(socket.IPPROTO_IP,getattr(socket,'IP_UNICAST_IF',31),struct.pack('!I',5))),connection.trace)
        self.assertLess(next(i for i,v in enumerate(connection.trace) if v[0]=='bind'),next(i for i,v in enumerate(connection.trace) if v[0]=='connect'))
        self.assertEqual(connection.local,('10.12.0.2',0))

    def test_rejected_report_never_opens_a_socket_and_failure_closes_it(self):
        created=[]
        network=WindowsWiredNetwork(socket_factory=lambda *args:(created.append(SocketDouble()) or created[-1]))
        with self.assertRaises(RuntimeError):network.connect('10.12.0.9',report=dict(ready=False))
        self.assertFalse(created)
        class Failed(SocketDouble):
            def connect(self,remote):raise OSError('fixture disconnected')
        f=Failed();network=WindowsWiredNetwork(socket_factory=lambda *args:f)
        with self.assertRaises(OSError):network.connect('10.12.0.9',report=evaluate(snapshot(),'10.12.0.9'))
        self.assertTrue(f.closed)

    def test_input_injection_and_os_configuration_commands_are_not_allowed(self):
        for target in ('127.0.0.1','0.0.0.0','224.0.0.1','255.255.255.255','10.0.0.1; exit','robot.local'):
            with self.assertRaises(ValueError):target_ipv4(target)
        for forbidden in ('Set-Net','New-Net','Remove-Net','Disable-Net','Set-ItemProperty','netsh'):
            self.assertNotIn(forbidden,INVENTORY_SCRIPT)

    def test_robot_interface_read_uses_only_ip_and_sysfs_no_control_calls(self):
        calls=[]
        class Reader:
            def command(self,cmd,**kwargs):
                calls.append(cmd);return json.dumps(dict(scope='network_observation_only',interfaces=[],robot_commands_sent=False))
        self.assertEqual(read_robot_interfaces(Reader())['interfaces'],[])
        for forbidden in ('rclpy','systemctl','sudo','set_arm','ip addr add'):
            self.assertNotIn(forbidden,calls[0])

    def test_deployment_uses_pinned_socket_and_never_calls_proxy_configuration(self):
        connection=SocketDouble();connection.bind(('10.12.0.2',0))
        class Network:
            def inspect(self,target):return evaluate(snapshot(),target)
            def connect(self,*args,**kwargs):return connection
        client=NS(load_host_keys=lambda _:None,set_missing_host_key_policy=lambda _:None,
            connect=lambda *args,**kwargs:setattr(client,'options',kwargs),close=lambda:None,
            get_transport=lambda:NS(sock=None))
        with tempfile.TemporaryDirectory() as temp,patch('paramiko.SSHClient',return_value=client):
            d=RobotDeployment(Path(temp),Path(temp)/'known',lambda *args:True,lambda _:None,network_mode='wired',network_provider=Network())
            d.connect('10.12.0.9','ubuntu','fixture-password')
        self.assertIs(client.options['sock'],connection)
        self.assertTrue(d.network_report['ssh_bound'])

    def test_disconnected_route_stops_before_ssh_or_deployment(self):
        network=NS(inspect=lambda target:dict(ready=False,reason='cable disconnected'))
        with tempfile.TemporaryDirectory() as temp,patch('paramiko.SSHClient') as client:
            d=RobotDeployment(temp,Path(temp)/'known',lambda *args:True,lambda _:None,network_mode='wired',network_provider=network)
            with self.assertRaisesRegex(RuntimeError,'disconnected'):d.connect('10.12.0.9','ubuntu','fixture-password')
        client.assert_not_called()
