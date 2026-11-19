"""Read-only Windows route checks and interface-bound SSH sockets.

No address/route/VPN changes, robot discovery scans or ROS operations.
IP_UNICAST_IF uses a network-order DWORD (Microsoft Winsock documentation).
"""
import ipaddress
import json
import os
import socket
import struct
import subprocess
import time


def target_ipv4(value):
    try:address=ipaddress.IPv4Address(value)
    except (ValueError,TypeError):raise ValueError('有线模式请填写机器人的实际IPv4地址') from None
    if address.is_loopback or address.is_unspecified or address.is_multicast or str(address)=='255.255.255.255':
        raise ValueError('机器人地址不能是本机、广播或组播地址')
    return str(address)


def evaluate(snapshot,target,preferred_interface_index=None):
    """Evaluate recorded OS facts; never infer Ethernet from adapter name."""
    target=target_ipv4(target)
    adapters=snapshot.get('adapters') or [];addresses=snapshot.get('addresses') or []
    system_route=snapshot.get('route') or {}
    if isinstance(adapters,dict):adapters=[adapters]
    if isinstance(addresses,dict):addresses=[addresses]
    wired=[a for a in adapters if a.get('HardwareInterface') is True and a.get('NdisPhysicalMedium')==14]
    result=dict(ready=False,target_ipv4=target,adapters=adapters,addresses=addresses,
        wired_adapters=wired,system_route=system_route,checked_at=time.monotonic())
    def fail(code,reason):return dict(result,code=code,reason=reason)
    if not wired:return fail('no_physical_ethernet','未发现物理以太网卡；请确认网口或USB转网口')
    active=[a for a in wired if a.get('Status')=='Up']
    if not active:return fail('ethernet_disconnected','有线网卡未连接；请检查网线和机器人LAN口')
    eligible=[]
    candidates=snapshot.get('wired_routes') or []
    if isinstance(candidates,dict):candidates=[candidates]
    for candidate in candidates:
        route=candidate.get('route') or {};source=candidate.get('source') or {}
        index=route.get('InterfaceIndex')
        adapter=next((a for a in active if a.get('ifIndex')==index),None)
        if adapter is None or source.get('InterfaceIndex')!=index:continue
        if preferred_interface_index is not None and index!=preferred_interface_index:continue
        local=source.get('IPAddress')
        assigned=next((a for a in addresses if a.get('InterfaceIndex')==index and a.get('IPAddress')==local
            and a.get('AddressState')=='Preferred'),None)
        if assigned is None:continue
        try:
            address=ipaddress.IPv4Address(local)
            network=ipaddress.IPv4Network((local,assigned['PrefixLength']),strict=False)
            destination=ipaddress.IPv4Network(route['DestinationPrefix'],strict=False)
        except (ValueError,TypeError,KeyError):continue
        if address.is_link_local or address.is_unspecified or address.is_loopback or local==target:continue
        # Robot cable is an on-link connection. A VPN/default route may remain
        # selected globally; use this explicit physical-interface route only.
        if route.get('NextHop')!='0.0.0.0' or ipaddress.IPv4Address(target) not in network or ipaddress.IPv4Address(target) not in destination:continue
        eligible.append((adapter,source,route))
    if not eligible:return fail('no_onlink_ethernet_route','有线网口已连接，但没有到机器人IPv4的同网段直连路径；请核对双方地址，不需关闭全局代理')
    if len(eligible)>1:return fail('multiple_ethernet_routes','多个有线口可达机器人，请在网卡选择中指定一个')
    adapter,source,route=eligible[0];index=route['InterfaceIndex'];local=source['IPAddress']
    return dict(result,ready=True,code='wired_route_verified',reason='有线路由及源地址已核对，尚未建立SSH连接',
        route=route,source=source,system_route_differs=system_route.get('InterfaceIndex')!=index,
        interface_index=index,interface_name=adapter.get('Name'),source_ipv4=local,
        destination_prefix=route.get('DestinationPrefix'),next_hop=route.get('NextHop'))


INVENTORY_SCRIPT=r'''
$ErrorActionPreference='Stop'
[Console]::OutputEncoding=[System.Text.UTF8Encoding]::new($false)
$target='TARGET'
$adapters=@(Get-NetAdapter -IncludeHidden | Select-Object ifIndex,Name,InterfaceDescription,Status,NdisPhysicalMedium,HardwareInterface,LinkSpeed)
$addresses=@(Get-NetIPAddress -AddressFamily IPv4 | Select-Object InterfaceIndex,IPAddress,PrefixLength,@{Name='AddressState';Expression={$_.AddressState.ToString()}})
$route=@();$routeError=$null
try {$route=@(Find-NetRoute -RemoteIPAddress $target -ErrorAction Stop)} catch {$routeError=$_.Exception.Message}
$source=$route | Where-Object {$null -ne $_.IPAddress} | Select-Object -First 1 IPAddress,InterfaceIndex
$best=$route | Where-Object {$null -ne $_.DestinationPrefix} | Select-Object -First 1 InterfaceIndex,InterfaceAlias,DestinationPrefix,NextHop
$wired=@()
foreach ($a in $adapters) {
 if ($a.HardwareInterface -and $a.NdisPhysicalMedium -eq 14 -and $a.Status -eq 'Up') {
  try {
   $r=@(Find-NetRoute -RemoteIPAddress $target -InterfaceIndex $a.ifIndex -ErrorAction Stop)
   $s=$r | Where-Object {$null -ne $_.IPAddress} | Select-Object -First 1 IPAddress,InterfaceIndex
   $b=$r | Where-Object {$null -ne $_.DestinationPrefix} | Select-Object -First 1 InterfaceIndex,InterfaceAlias,DestinationPrefix,NextHop
   $wired+=@{source=$s;route=$b}
  } catch {}
 }
}
@{adapters=$adapters;addresses=$addresses;source=$source;route=$best;route_error=$routeError;wired_routes=$wired} | ConvertTo-Json -Depth 6 -Compress
'''


class WindowsWiredNetwork:
    def __init__(self,runner=None,socket_factory=None,preferred_interface_index=None):
        self.runner=runner or subprocess.run;self.socket_factory=socket_factory or socket.socket
        self.preferred_interface_index=preferred_interface_index

    def inspect(self,target):
        target=target_ipv4(target)
        if os.name!='nt':raise RuntimeError('当前有线检查器针对Windows操作台；本机平台不支持此接口绑定')
        result=self.runner(['powershell.exe','-NoProfile','-NonInteractive','-Command',
            INVENTORY_SCRIPT.replace('TARGET',target)],capture_output=True,timeout=12,
            creationflags=getattr(subprocess,'CREATE_NO_WINDOW',0))
        if result.returncode:raise RuntimeError('Windows网卡检查失败：'+result.stderr.decode('utf-8',errors='replace')[-500:])
        try:snapshot=json.loads(result.stdout.decode('utf-8-sig'))
        except (ValueError,UnicodeError):raise RuntimeError('Windows网卡检查未返回可读结果') from None
        return evaluate(snapshot,target,self.preferred_interface_index)

    def connect(self,target,port=22,timeout=8.,report=None):
        target=target_ipv4(target);report=report or self.inspect(target)
        if not report.get('ready') or report.get('target_ipv4')!=target:raise RuntimeError(report.get('reason','有线路由未通过'))
        connection=self.socket_factory(socket.AF_INET,socket.SOCK_STREAM)
        try:
            connection.settimeout(timeout)
            # Winsock IP_UNICAST_IF=31; value is an interface index in network
            # byte order, unlike the IPv6 option. Do not use a virtual adapter.
            connection.setsockopt(socket.IPPROTO_IP,getattr(socket,'IP_UNICAST_IF',31),
                struct.pack('!I',report['interface_index']))
            connection.bind((report['source_ipv4'],0))
            connection.connect((target,port))
            if connection.getsockname()[0]!=report['source_ipv4']:raise RuntimeError('SSH实际源地址与核对的有线IPv4不一致')
            return connection
        except BaseException:
            connection.close();raise

    def probe(self,target,count=3):
        """Explicit operator check only. TCP handshake time is not pure ping."""
        if type(count) is not int or not 1<=count<=5:raise ValueError('invalid_network_probe_count')
        report=self.inspect(target)
        if not report['ready']:return report
        samples=[]
        for _ in range(count):
            started=time.monotonic();connection=self.connect(target,timeout=3.,report=report)
            samples.append((time.monotonic()-started)*1000);connection.close()
        after=self.inspect(target)
        if not after['ready'] or (after['interface_index'],after['source_ipv4'])!=(report['interface_index'],report['source_ipv4']):
            raise RuntimeError('测量期间有线路由变化；本次结果作废')
        report['tcp_connect_ms']=dict(samples=count,min=min(samples),median=sorted(samples)[len(samples)//2],max=max(samples))
        report['reason']='有线路由与绑定TCP连接通过；这不是头显至机械臂的总响应时间'
        return report


ROBOT_NETWORK_SCRIPT=r'''
import json,subprocess
from pathlib import Path
data=json.loads(subprocess.check_output(['ip','-j','-4','addr','show'],timeout=5))
interfaces=[]
for item in data:
    name=item['ifname']
    if name=='lo' or not name or '/' in name:continue
    root=Path('/sys/class/net')/name
    try:carrier=(root/'carrier').read_text().strip()=='1'
    except OSError:carrier=None
    interfaces.append(dict(name=name,state=item.get('operstate'),carrier=carrier,
        wireless=(root/'wireless').exists(),ipv4=[dict(address=a['local'],prefix_length=a['prefixlen'])
        for a in item.get('addr_info',[]) if a.get('family')=='inet']))
print(json.dumps(dict(scope='network_observation_only',interfaces=interfaces,robot_commands_sent=False)))
'''


def read_robot_interfaces(connection):
    import shlex
    value=json.loads(connection.command('/usr/bin/python3 -c '+shlex.quote(ROBOT_NETWORK_SCRIPT),timeout=15))
    if value.get('scope')!='network_observation_only' or value.get('robot_commands_sent') is not False:
        raise RuntimeError('机器人网口只读报告范围不符')
    return value
