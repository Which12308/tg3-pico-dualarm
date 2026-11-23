"""Windows wireless desk: model validation or explicitly admitted factory output."""
import hashlib
import json
import os
from pathlib import Path
import queue
import re
import secrets
import socket
import sys
import threading
import time
import tkinter as tk
from tkinter import ttk,messagebox,filedialog

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src'))
from tg3_pico_dualarm import __version__
from tg3_pico_dualarm.deployment import RobotDeployment,reviewed_factory_overrides,reviewed_brainco_overrides
from tg3_pico_dualarm.manufacturer_session import load_factory_bundle
from tg3_pico_dualarm.diagnostics import DiagnosticsSink
from tg3_pico_dualarm.manual_recording import ManualRecording,discover
from tg3_pico_dualarm.recording_writer import write_outcome
from tg3_pico_dualarm.wireless import InputGateway,WirelessServer,GatewayClient,LatestRelay
from tg3_pico_dualarm.input_policy import NORMAL,policy_for
from tg3_pico_dualarm.tracking import LatestTracking
from tg3_pico_dualarm.wired_network import WindowsWiredNetwork,read_robot_interfaces
from tg3_pico_dualarm.operator_status import motion_view,timing_summary,current_failure
from install_headset import artifact_info,install,prepare_usb
from preview import ArmPreview


class Desk:
    def __init__(self,visible=True,port=8877,record_root=None):
        self.window=tk.Tk();self.window.title('天工 PICO 双臂遥操作 '+__version__)
        if not visible:self.window.withdraw()
        self.window.geometry('1100x850');self.window.minsize(900,700)
        self.events=queue.Queue();self.closed=False;self.busy=False
        self._poll_after_id=None
        self._adb_lock=threading.Lock();self._usb_busy=False
        self.deployment=self.relay=self.remote_client=self.remote_endpoint=None
        self.pending_deployment=None;self.remote_tracking={}
        self._runtime_mode_running=None;self._generation=0
        self._connection_mode_running=None
        self._rpc_lock=threading.RLock();self._stop_lock=threading.RLock()
        self._control_pending=False;self._link_disable_attempted=False
        self._enable_queued=False;self._enable_baseline=0;self._remote_status_at=None
        self._enable_request_number=None
        self.network_report=None;self._wired_interface_map={'自动':None}
        self.manual_recording=ManualRecording();self._session_credentials=None;self._record_pending=False
        self._record_epoch=0
        self.record_destination=None;self.record_topics=None;self.record_rate=1024*1024
        self.factory_path=None;self.factory_digest=None;self.factory_bundle=None
        self.hand_config_path=None;self.hand_config_digest=None
        # Python 3.12 on Windows uses 15.6ms GetTickCount64 for monotonic().
        # Keep receiver and relay on the same high-resolution monotonic clock;
        # no cross-clock subtraction and no renewal of a frame's receipt time.
        self.gateway=InputGateway(clock=time.perf_counter)
        self.recorder=DiagnosticsSink(ROOT/'.desktop/diagnostics',budget=8*1024*1024)
        self.server=WirelessServer(self.gateway,port=port).start()
        self.tool_config=json.loads((ROOT/'config/calibration.yaml').read_text(encoding='utf-8'))
        saved=ROOT/'.desktop/tool_calibration.json'
        if saved.exists():self.tool_config=json.loads(saved.read_text(encoding='utf-8'))
        try:
            self._apk,self._apk_receipt=artifact_info(ROOT)
            apk_status='头显 App '+__version__+' 已构建并校验；实际头显运行待现场验收'
        except (OSError,ValueError,RuntimeError) as exc:
            self._apk=self._apk_receipt=None;apk_status=str(exc)
        panel=ttk.Frame(self.window,padding=12);panel.pack(fill='both',expand=True)
        ttk.Label(panel,text='PICO 双臂操作台（USB 优先）',font=('Microsoft YaHei UI',20)).pack(anchor='w')
        self.banner=tk.StringVar(value='模型验证：仅计算与显示，录制关闭，硬件输出关闭')
        self.banner_label=ttk.Label(panel,textvariable=self.banner,foreground='#9c3a00',wraplength=1060)
        self.banner_label.pack(anchor='w',pady=6)
        ttk.Label(panel,text=apk_status,wraplength=1060).pack(anchor='w')
        network=ttk.Frame(panel);network.pack(fill='x',pady=8)
        ttk.Label(network,text='机器人地址').grid(row=0,column=0)
        self.host=tk.StringVar(value='');ttk.Entry(network,textvariable=self.host,width=20).grid(row=0,column=1,padx=5)
        ttk.Label(network,text='用户名').grid(row=0,column=2)
        self.user=tk.StringVar(value='ubuntu');ttk.Entry(network,textvariable=self.user,width=12).grid(row=0,column=3,padx=5)
        ttk.Label(network,text='SSH 密码（仅本次）').grid(row=0,column=4)
        self.password=tk.StringVar();ttk.Entry(network,textvariable=self.password,show='*',width=20).grid(row=0,column=5,padx=5)
        wired=ttk.Frame(panel);wired.pack(fill='x',pady=3)
        ttk.Label(wired,text='机器人链路').pack(side='left')
        self.robot_link_mode=tk.StringVar(value='有线（保留代理）')
        self.robot_link_selector=ttk.Combobox(wired,textvariable=self.robot_link_mode,
            values=('有线（保留代理）','现有网络（不限定）'),state='readonly',width=20)
        self.robot_link_selector.pack(side='left',padx=4)
        self.wired_interface=tk.StringVar(value='自动')
        self.wired_selector=ttk.Combobox(wired,textvariable=self.wired_interface,values=('自动',),state='readonly',width=18)
        self.wired_selector.pack(side='left',padx=4)
        ttk.Button(wired,text='检查电脑有线',command=self.check_wired_network).pack(side='left',padx=2)
        ttk.Button(wired,text='读取机器人网口（只读）',command=self.read_robot_network).pack(side='left',padx=2)
        self.network_state=tk.StringVar(value='有线尚未检查；保留系统代理/虚拟网卡，不自动修改IP或路由')
        ttk.Label(panel,textvariable=self.network_state,wraplength=1060).pack(anchor='w')
        address='请查看电脑当前 Wi-Fi IPv4'
        try:
            with socket.socket(socket.AF_INET,socket.SOCK_DGRAM) as sock:
                sock.connect((self.host.get(),9));address=sock.getsockname()[0]
        except OSError:pass
        self._wifi_address=address;self._pair_pin=''
        connection=ttk.Frame(panel);connection.pack(fill='x',pady=4)
        ttk.Label(connection,text='头显连接').pack(side='left')
        self.connection_mode=tk.StringVar(value='USB')
        connection_selector=ttk.Combobox(connection,textvariable=self.connection_mode,values=('USB','USB（机器人直连）','Wi-Fi'),state='readonly',width=18)
        connection_selector.pack(side='left',padx=4)
        connection_selector.bind('<<ComboboxSelected>>',lambda _event:self._connection_changed())
        ttk.Button(connection,text='准备 USB 通道',command=self.prepare_usb).pack(side='left',padx=4)
        self.usb_status=tk.StringVar(value='USB：等待准备')
        ttk.Label(connection,textvariable=self.usb_status,wraplength=790).pack(side='left',padx=4)
        self.pairing=tk.StringVar()
        ttk.Label(panel,textvariable=self.pairing,font=('Microsoft YaHei UI',11),wraplength=1060).pack(anchor='w',pady=4)
        actions=ttk.Frame(panel);actions.pack(fill='x',pady=4)
        ttk.Button(actions,text='重新配对头显',command=self.pair).pack(side='left',padx=2)
        ttk.Button(actions,text='首次安装头显',command=self.install_headset).pack(side='left',padx=2)
        self.start_button=ttk.Button(actions,text='部署并启动检查',command=self.start);self.start_button.pack(side='left',padx=2)
        ttk.Button(actions,text='停用并停止本项目',command=self.stop_async).pack(side='left',padx=2)
        ttk.Button(actions,text='工具标定',command=self.tool_dialog).pack(side='left',padx=2)
        self.enable_button=ttk.Button(actions,text='启用真机运动',command=self.enable);self.enable_button.pack(side='left',padx=2)
        self.home_button=ttk.Button(actions,text='双臂回初始姿态',command=self.return_home,state='disabled')
        self.home_button.pack(side='left',padx=2)
        ttk.Button(actions,text='核对接管配置（不发动作）',command=self.read_holding_contract).pack(side='left',padx=2)
        options=ttk.Frame(panel);options.pack(fill='x',pady=4)
        ttk.Label(options,text='运行模式').pack(side='left')
        self.runtime_mode=tk.StringVar(value='dry_run')
        self.runtime_selector=ttk.Combobox(options,textvariable=self.runtime_mode,values=('dry_run','hardware'),state='readonly',width=12)
        self.runtime_selector.pack(side='left',padx=4)
        self.runtime_selector.bind('<<ComboboxSelected>>',lambda _event:self._mode_changed())
        ttk.Label(options,text='机械臂接口').pack(side='left',padx=(8,0))
        self.hardware_backend=tk.StringVar(value='xarm')
        self.backend_selector=ttk.Combobox(options,textvariable=self.hardware_backend,
            values=('xarm','manufacturer'),state='readonly',width=13)
        self.backend_selector.pack(side='left',padx=4)
        ttk.Label(options,text='手部目标').pack(side='left',padx=(12,0))
        self.mode=tk.StringVar(value='pose')
        self.pose_selector=ttk.Combobox(options,textvariable=self.mode,values=('pose','position'),state='readonly',width=11)
        self.pose_selector.pack(side='left',padx=4)
        profile_row=ttk.Frame(panel);profile_row.pack(fill='x',pady=3)
        ttk.Label(profile_row,text='输入时效档').pack(side='left')
        self.input_profile=tk.StringVar(value='normal')
        self.profile_selector=ttk.Combobox(profile_row,textvariable=self.input_profile,
            values=('normal','wifi_trial_200ms'),state='readonly',width=22)
        self.profile_selector.pack(side='left',padx=4)
        self.profile_selector.bind('<<ComboboxSelected>>',lambda _event:self._profile_changed())
        self.profile_status=tk.StringVar(value='正常档150ms；临时试验档200ms，关停后恢复正常；运动限速不变')
        ttk.Label(profile_row,textvariable=self.profile_status,wraplength=730).pack(side='left')
        ttk.Button(profile_row,text='读取上次故障日志（不发动作）',command=self.read_incident_logs).pack(side='left',padx=4)
        adapters=ttk.Frame(panel);adapters.pack(fill='x',pady=4)
        ttk.Button(adapters,text='加载厂家扩展（可选）',command=self.load_factory).pack(side='left',padx=2)
        self.factory_status=tk.StringVar(value='XARM：使用厂家双臂控制器，启动时核对实际安装')
        ttk.Label(adapters,textvariable=self.factory_status,wraplength=500).pack(side='left',padx=4)
        hands=ttk.Frame(panel);hands.pack(fill='x',pady=2)
        ttk.Button(hands,text='加载灵巧手标定',command=self.load_hands).pack(side='left',padx=2)
        self.hand_config_status=tk.StringVar(value='未加载：XARM 仅控制手臂；抓放需要本机实测标定')
        ttk.Label(hands,textvariable=self.hand_config_status,wraplength=880).pack(side='left',padx=4)
        recording=ttk.Frame(panel);recording.pack(fill='x',pady=4)
        ttk.Button(recording,text='选择保存目录',command=self.choose_record_directory).pack(side='left',padx=2)
        ttk.Button(recording,text='选择数据／相机',command=self.configure_recording).pack(side='left',padx=2)
        self.record_start=ttk.Button(recording,text='开始录制（未就绪）',command=self.start_recording,state='disabled')
        self.record_start.pack(side='left',padx=2)
        self.record_stop=ttk.Button(recording,text='停止录制',command=self.stop_recording,state='disabled')
        self.record_stop.pack(side='left',padx=2)
        self.record_outcome=tk.StringVar(value='未标记')
        ttk.Combobox(recording,textvariable=self.record_outcome,values=('未标记','成功','失败'),
                     state='readonly',width=7).pack(side='left',padx=4)
        self.record_label=ttk.Button(recording,text='保存示范结果',command=self.save_record_outcome,state='disabled')
        self.record_label.pack(side='left',padx=2)
        self.record_state=tk.StringVar(value='录制关闭；尚未选择保存目录和数据；不影响操控')
        ttk.Label(panel,textvariable=self.record_state,wraplength=1060).pack(anchor='w')
        self.state=tk.StringVar(value='等待头显连接')
        ttk.Label(panel,textvariable=self.state,wraplength=1060).pack(anchor='w',pady=6)
        self.timing_state=tk.StringVar(value='时效尚未测量；网络检查与机械响应分别显示')
        ttk.Label(panel,textvariable=self.timing_state,wraplength=1060).pack(anchor='w')
        self.notebook=ttk.Notebook(panel);self.notebook.pack(fill='both',expand=True,pady=5)
        self.preview=ArmPreview(self.notebook);self.notebook.add(self.preview,text='URDF 模型预览')
        issues_page=ttk.Frame(self.notebook);self.notebook.add(issues_page,text='就绪条件与缺项')
        logs_page=ttk.Frame(self.notebook);self.notebook.add(logs_page,text='运行日志')
        self.issuebox=self._scroll_text(issues_page);self.logbox=self._scroll_text(logs_page)
        self.issues=tk.StringVar();self._last_issues=''
        ttk.Label(panel,text='握持键：跟随／松开保持；扳机：渐闭并保留；左 Y、右 B：张手。走动观察前松开握持键。\n数据录制默认关闭；操控不依赖录制。',wraplength=1060).pack(anchor='w',pady=5)
        self.window.protocol('WM_DELETE_WINDOW',self.close)
        self.pair();self.poll()
        # Hidden test windows must never discover or modify physical ADB devices.
        if visible:self.prepare_usb()

    def _scroll_text(self,parent):
        scroll=ttk.Scrollbar(parent);scroll.pack(side='right',fill='y')
        box=tk.Text(parent,wrap='word',state='disabled',yscrollcommand=scroll.set,height=12)
        box.pack(side='left',fill='both',expand=True);scroll.configure(command=box.yview)
        return box

    def _network_mode(self):
        return 'wired' if self.robot_link_mode.get()=='有线（保留代理）' else 'auto'

    def _network_provider(self):
        if self.wired_interface.get() not in self._wired_interface_map:raise ValueError('请重新检查并选择物理有线网卡')
        return WindowsWiredNetwork(preferred_interface_index=self._wired_interface_map[self.wired_interface.get()])

    def check_wired_network(self):
        if self.busy or self.deployment:
            messagebox.showinfo('先停止本项目','网络检查在运动关闭且项目未部署时运行。');return
        try:host=self.host.get();provider=self._network_provider()
        except ValueError as exc:messagebox.showerror('有线检查未开始',str(exc));return
        self.network_state.set('正在检查物理网卡、同网段路线与绑定TCP连接；不修改代理')
        self.busy=True;self.start_button.configure(state='disabled')
        def work():
            try:self.events.put(('network_report',provider.probe(host)))
            except Exception as exc:self.events.put(('network_error',str(exc)))
            finally:self.events.put(('idle',None))
        threading.Thread(target=work,daemon=True).start()

    def read_robot_network(self):
        if self.busy or self.deployment:
            messagebox.showinfo('先停止本项目','网口读取在未部署状态运行，不调用机器人控制服务。');return
        host,user,password=self.host.get(),self.user.get(),self.password.get();self.password.set('')
        self.busy=True;self.start_button.configure(state='disabled')
        def work():
            nonlocal password
            reader=None
            try:
                folder=Path(os.environ.get('LOCALAPPDATA',str(ROOT/'.desktop')))/'TG3Wireless'
                reader=RobotDeployment(ROOT,folder/'known_hosts',self.confirm_host,lambda value:None)
                reader.connect(host,user,password)
                self.events.put(('robot_network_report',read_robot_interfaces(reader)))
            except Exception as exc:self.events.put(('network_error','读取网口失败：'+str(exc)))
            finally:
                password=None
                if reader:reader.close()
                self.events.put(('idle',None))
        threading.Thread(target=work,daemon=True).start()

    def _robot_network_dialog(self,report):
        dialog=tk.Toplevel(self.window);dialog.title('机器人网口（只读，不改地址）')
        ttk.Label(dialog,text='可先通过已知管理地址读取。选择接线对应的网口IP，填入主窗口后重新做有线检查。\n不根据LAN-1/LAN-2名称猜IP，不关闭代理。',wraplength=700).pack(padx=10,pady=8)
        choices=[]
        for item in report.get('interfaces',[]):
            for address in item.get('ipv4',[]):
                text='%s：%s/%s，链路%s，%s'%(item['name'],address['address'],address['prefix_length'],
                    '已连接' if item.get('carrier') else '未连接/未知','无线' if item.get('wireless') else '非无线接口')
                ttk.Label(dialog,text=text).pack(anchor='w',padx=10)
                if not item.get('wireless'):choices.append((text,address['address']))
        selected=tk.StringVar(value=choices[0][0] if choices else '')
        ttk.Combobox(dialog,textvariable=selected,values=tuple(t for t,_ in choices),state='readonly',width=82).pack(padx=10,pady=8)
        def use():
            address=dict(choices).get(selected.get())
            if address:self.host.set(address);self.network_report=None
            dialog.destroy()
        ttk.Button(dialog,text='将所选IP填入机器人地址',command=use,state='normal' if choices else 'disabled').pack(pady=8)
        ttk.Label(dialog,text='电脑有线IPv4若不在同一网段，需在Windows以太网设置中配置空闲地址；本程序不会自动改IP。',wraplength=700).pack(padx=10,pady=8)

    def log(self,value):
        self.events.put(('log',str(value)))
        self.recorder.event('desktop_event',__import__('time').monotonic(),{'reason':str(value)[:1500]})

    def confirm_host(self,host,fingerprint):
        done=threading.Event();answer=[]
        self.events.put(('host',(host,fingerprint,done,answer)));done.wait(120)
        return bool(answer and answer[0])

    def pair(self):
        self.stop_recording()
        if self.connection_mode.get()=='USB（机器人直连）':
            if self.remote_endpoint is None:
                self._pair_pin='';self._refresh_pairing();return
            endpoint=self.remote_endpoint;generation=self._generation
            def work():
                client=GatewayClient(*endpoint)
                try:self.events.put(('robot_headset_pairing',(generation,client.reopen_headset_pairing()['pin'])))
                except Exception as exc:self.log('机器人头显配对未打开：'+str(exc))
                finally:client.close()
            threading.Thread(target=work,daemon=True).start();return
        self._pair_pin=self.gateway.open_pairing()
        self._refresh_pairing()

    def _refresh_pairing(self):
        if self.connection_mode.get()=='USB（机器人直连）':
            self.pairing.set('USB直连机器人：头显选择USB，地址127.0.0.1；'+(
                '配对码 '+self._pair_pin+'（120秒）' if self._pair_pin else '部署后显示机器人端配对码'))
            return
        transport=('USB：头显默认地址 127.0.0.1，无需填写电脑 IP；请保持数据线连接'
            if self.connection_mode.get()=='USB' else 'Wi-Fi：电脑地址 '+self._wifi_address+'；头显选择 Wi-Fi 并填写此地址')
        self.pairing.set(transport+'；配对码 '+self._pair_pin+'（120 秒）')

    def _connection_changed(self):
        if self.deployment or self.busy:
            self.connection_mode.set(self._connection_mode_running or 'USB')
            self.log('请先停止本项目，再更改头显接线方式。');return
        self.pair()  # Revoke previous credentials before changing connection source.
        if self.connection_mode.get() in ('USB','USB（机器人直连）'):self.prepare_usb()

    def prepare_usb(self):
        if self.connection_mode.get()=='USB（机器人直连）':
            self.usb_status.set('USB直连机器人：部署时检查头显连接与授权');return
        if self.closed or self._usb_busy:return
        self._usb_busy=True;self.usb_status.set('USB：正在准备，请在头显允许调试')
        def work():
            try:
                from adb_tools import ensure_adb
                with self._adb_lock:
                    adb=ensure_adb(ROOT,self.log)
                    result=prepare_usb(adb)
                self.events.put(('usb_result',(True,result)))
            except Exception as exc:self.events.put(('usb_result',(False,str(exc))))
        threading.Thread(target=work,daemon=True).start()

    def install_headset(self):
        try:artifact_info(ROOT)
        except (OSError,ValueError,RuntimeError) as exc:
            messagebox.showerror('安装包检查未通过',str(exc));return
        def work():
            try:
                from adb_tools import ensure_adb
                with self._adb_lock:
                    adb=ensure_adb(ROOT,self.log)
                    self.log(install(ROOT,adb))
            except Exception as exc:self.log('头显安装未完成：'+str(exc))
        threading.Thread(target=work,daemon=True).start()

    def _mode_changed(self):
        if self.deployment or self.busy:return
        self.banner.set('模型验证：仅计算与显示，录制关闭，硬件输出关闭' if self.runtime_mode.get()=='dry_run'
            else '真机接入：检查本机厂家控制器和实时反馈后，等待显式启用；当前输出关闭')

    def load_factory(self):
        if self.busy or self.deployment:
            messagebox.showinfo('先停止本项目','厂家配置修改在下次部署生效，请先停止本项目。');return
        selected=filedialog.askopenfilename(title='选择已审查的厂家 bundle JSON',filetypes=[('JSON','*.json')])
        if not selected:return
        try:
            path=Path(selected).resolve();raw=path.read_bytes();digest=hashlib.sha256(raw).hexdigest()
            bundle=load_factory_bundle(path)
            if path.read_bytes()!=raw:raise ValueError('厂家配置在校验期间发生变化，请重新加载。')
            self.factory_path,self.factory_digest,self.factory_bundle=path,digest,bundle
            self.factory_status.set(bundle.evidence['controller']+' / '+bundle.evidence['version']+'：文件已校验，控制权待实时确认')
            self.log('厂家 JSON 与单个驱动源码 SHA 已校验；未执行驱动、未授权运动。')
        except Exception as exc:messagebox.showerror('厂家配置未加载',str(exc))

    def _profile_changed(self):
        if self.busy or self.deployment:
            self.input_profile.set(self.gateway.tracking.policy.name)
            return
        policy=policy_for(self.input_profile.get())
        with self.gateway.lock:
            self.gateway.tracking=LatestTracking(clock=self.gateway.clock,policy=policy)
            self.gateway.open_pairing()
        self.pair()
        self.log('本次输入档：'+policy.name+'；累计上限 '+str(round(policy.total_age_s*1000))+'ms；运动未启用，录制关闭。')

    def read_holding_contract(self):
        if self.busy or self.deployment:
            messagebox.showinfo('请先停止本项目','接管配置核对仅在未部署时读取，不启动ROS或控制器。');return
        host,user,password=self.host.get(),self.user.get(),self.password.get()
        if not password:
            self.log('请在本窗口填写本次Ubuntu密码，再点核对接管配置。');return
        network_mode='wired' if self.robot_link_mode.get()=='有线（保留代理）' else 'auto'
        try:provider=self._network_provider() if network_mode=='wired' else None
        except ValueError as exc:
            self.log('接管配置核对未开始：'+str(exc));return
        self.busy=True;self.start_button.configure(state='disabled')
        def work():
            nonlocal password
            reader=None
            try:
                from tg3_pico_dualarm.holding_contract import fetch_holding_contract
                folder=Path(os.environ.get('LOCALAPPDATA',str(ROOT/'.desktop')))/'TG3Wireless'
                reader=RobotDeployment(ROOT,folder/'known_hosts',self.confirm_host,lambda value:None,
                    network_mode=network_mode,network_provider=provider)
                reader.connect(host,user,password)
                report=fetch_holding_contract(reader,password)
                destination=ROOT/'.desktop/robot_reports'/('holding_contract_'+secrets.token_hex(8)+'.json')
                destination.parent.mkdir(parents=True,exist_ok=True)
                destination.write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
                self.log('接管配置已读取：'+str(destination)+'；未部署、未切换模式、未发送动作。')
            except Exception as exc:self.log('读取接管配置失败：'+str(exc))
            finally:
                password=None
                if reader:reader.close()
                self.events.put(('idle',None))
        threading.Thread(target=work,daemon=True).start()

    def read_incident_logs(self):
        if self.busy or self.deployment:
            messagebox.showinfo('请先停止本项目','故障读取只在未部署的状态运行，不启动厂家或ROS进程。');return
        host,user,password=self.host.get(),self.user.get(),self.password.get();self.password.set('')
        if not password:
            self.log('未填写本次Ubuntu密码，未建立诊断连接；若已有“故障日志已读取”记录，无需重复读取。');return
        self.busy=True;self.start_button.configure(state='disabled')
        def work():
            nonlocal password
            reader=None
            try:
                from tg3_pico_dualarm.incident_report import fetch_existing_logs
                folder=Path(os.environ.get('LOCALAPPDATA',str(ROOT/'.desktop')))/'TG3Wireless'
                reader=RobotDeployment(ROOT,folder/'known_hosts',self.confirm_host,lambda value:None)
                reader.connect(host,user,password)
                report=fetch_existing_logs(reader,password)
                destination=ROOT/'.desktop/robot_reports'/('factory_incident_'+secrets.token_hex(8)+'.json')
                destination.parent.mkdir(parents=True,exist_ok=True)
                destination.write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
                self.log('已有故障日志已读取：'+str(destination)+'；未部署、未启用、未调用控制服务。')
            except Exception as exc:self.log('读取故障日志失败：'+str(exc))
            finally:
                password=None
                if reader:reader.close()
                self.events.put(('idle',None))
        threading.Thread(target=work,daemon=True).start()

    def _startup_overrides(self,runtime_mode):
        vr=json.loads((ROOT/'config/vr.yaml').read_text(encoding='utf-8'));vr['mode']=self.mode.get()
        policy=policy_for(self.input_profile.get())
        if policy != self.gateway.tracking.policy:raise ValueError('输入档尚未应用，请重新选择输入时效档。')
        vr.update(input_profile=policy.name,tracking_timeout_s=policy.total_age_s,source_age_limit_s=policy.source_age_s)
        robot=json.loads((ROOT/'config/robot.yaml').read_text(encoding='utf-8'))
        robot['mode']=runtime_mode;robot['hardware_motion_enabled']=False
        overrides={'config/calibration.yaml':json.dumps(self.tool_config,allow_nan=False).encode(),
            'config/vr.yaml':json.dumps(vr,allow_nan=False).encode(),
            'config/robot.yaml':json.dumps(robot,allow_nan=False).encode()}
        if runtime_mode=='hardware' and self.hardware_backend.get()=='xarm':
            from tg3_pico_dualarm.xarm_config import load_xarm_settings
            load_xarm_settings(ROOT/'config/xarm.yaml')
            if self.hand_config_path is not None:
                overrides.update(reviewed_brainco_overrides(self.hand_config_path,self.hand_config_digest))
        elif runtime_mode=='hardware':
            if self.factory_path is None:
                raise RuntimeError('真机模式缺少实际厂家驱动配置。请加载已审查的 JSON 和对应本机源码；模型验证无需此项。')
            overrides.update(reviewed_factory_overrides(self.factory_path,self.factory_digest))
        return overrides

    def load_hands(self):
        if self.busy or self.deployment:
            messagebox.showinfo('先停止本项目','灵巧手标定在下次部署生效，请先停止本项目。');return
        selected=filedialog.askopenfilename(title='选择本机灵巧手实测标定',filetypes=[('JSON','*.json')])
        if not selected:return
        try:
            path=Path(selected).resolve();digest=hashlib.sha256(path.read_bytes()).hexdigest()
            checked=reviewed_brainco_overrides(path,digest)
            hand_config=json.loads(checked['config/brainco.json']);sides=list(hand_config['hands'])
            self.hand_config_path,self.hand_config_digest=path,digest
            self.hand_config_status.set('已加载 '+', '.join(sides)+
                ('；右手触觉抓握' if 'right_tactile' in hand_config else '；位置抓握'))
            self.log('灵巧手实测配置已校验；尚未启用输出。')
        except Exception as exc:messagebox.showerror('灵巧手标定未加载',str(exc))

    def start(self):
        if self.busy:return
        if self.deployment:
            messagebox.showinfo('已启动','请先停止本项目，再更改配置或重新部署。');return
        runtime_mode=self.runtime_mode.get()
        hardware_backend=self.hardware_backend.get()
        network_mode=self._network_mode()
        connection_mode=self.connection_mode.get()
        headset_transport='robot_usb' if connection_mode=='USB（机器人直连）' else 'workstation'
        try:
            network_provider=self._network_provider() if network_mode=='wired' else None
            overrides=self._startup_overrides(runtime_mode)
        except Exception as exc:messagebox.showerror('启动条件未满足',str(exc));return
        self.busy=True;self.start_button.configure(state='disabled')
        self._connection_mode_running=connection_mode
        self.runtime_selector.configure(state='disabled');self.pose_selector.configure(state='disabled')
        self.backend_selector.configure(state='disabled')
        self.profile_selector.configure(state='disabled')
        self.robot_link_selector.configure(state='disabled');self.wired_selector.configure(state='disabled')
        self._enable_queued=False;self._remote_status_at=None
        self._generation+=1;generation=self._generation
        host,user,password=self.host.get(),self.user.get(),self.password.get();self.password.set('')
        def work():
            nonlocal password
            deployment=None
            try:
                self.log('正在部署 '+runtime_mode+'；首次安装项目独立依赖，硬件输出默认关闭。')
                state_dir=Path(os.environ.get('LOCALAPPDATA',str(ROOT/'.desktop')))/'TG3Wireless'
                deployment=RobotDeployment(ROOT,state_dir/'known_hosts',self.confirm_host,self.log,
                    overrides=overrides,runtime_mode=runtime_mode,hardware_backend=hardware_backend,headset_transport=headset_transport,
                    network_mode=network_mode,network_provider=network_provider)
                self.pending_deployment=deployment
                deployment.connect(host,user,password)
                self.events.put(('network_live',(generation,getattr(deployment,'network_report',None))))
                deployment.deploy()
                if headset_transport=='robot_usb':deployment.prepare_robot_usb(administrator_password=password)
                try:
                    evidence=deployment.capture_interfaces()
                    self.log('厂家接口只读报告：'+evidence['path'])
                    self.log('只读采集摘要：'+json.dumps(evidence['summary'],ensure_ascii=False))
                    if not evidence['ok'] and runtime_mode!='dry_run':
                        raise RuntimeError('只读厂家接口采集未通过，硬件会话未启动。')
                except Exception as exc:
                    self.log('厂家接口只读采集未完成：'+str(exc))
                    if runtime_mode!='dry_run':raise
                    self.log('继续 dry_run 模型验证；报告失败不视为厂家接口已验证。')
                key=secrets.token_urlsafe(32)
                port=deployment.start(key,administrator_password=password if runtime_mode=='hardware' and hardware_backend=='xarm' else None)
                if self.closed or generation!=self._generation or deployment.closing.is_set():
                    deployment.close();return
                self._session_credentials=dict(host=host,user=user,password=password,
                    known_hosts=str(deployment.known_hosts),remote=deployment.remote)
                password=None
                self.deployment=deployment;self._runtime_mode_running=runtime_mode
                self.remote_endpoint=('127.0.0.1',port,key)
                # Status is read-only and can exceed the input/control budget.
                # Relay/control retain GatewayClient's strict 120 ms default.
                self.remote_client=GatewayClient(*self.remote_endpoint,timeout=1.0)
                if headset_transport=='workstation':
                    self.relay=LatestRelay(self.gateway.tracking,GatewayClient(*self.remote_endpoint,
                        cache_challenge=network_mode!='wired'),clock=self.gateway.clock).start()
                else:self.events.put(('robot_headset_pairing',(generation,deployment.headset_pairing_code)))
                self.log('模型验证已启动；不会驱动机器人。' if runtime_mode=='dry_run'
                    else '真机会话已启动但未使能；需满足实时条件后点击“启用真机运动”。')
                threading.Thread(target=self.remote_status,daemon=True).start()
            except Exception as exc:
                if deployment:
                    try:deployment.close()
                    except Exception as cleanup:self.log('部署清理异常：'+str(cleanup))
                self.log('启动未完成：'+str(exc))
                self._session_credentials=None
            finally:
                password=None
                self.pending_deployment=None;self.events.put(('idle',None))
        threading.Thread(target=work,daemon=True).start()

    def remote_status(self):
        client=self.remote_client;generation=self._generation
        while not self.closed and self.remote_client is client:
            exit_status=getattr(self.deployment,'station_exit',None)
            if exit_status is not None:
                self.events.put(('station_exit',(client,generation,exit_status)));client.close();return
            try:self.events.put(('remote',(client,generation,client.request('GET','/v1/status'))))
            except Exception as exc:
                self.events.put(('remote_error',(client,generation,str(exc))));client.close()
            threading.Event().wait(.5)

    def _rpc_control(self,operation,session,endpoint,generation=None):
        with self._rpc_lock:
            if operation in ('enable','return_home') and (self.closed or generation!=self._generation or endpoint!=self.remote_endpoint):
                raise RuntimeError('connection_changed_'+operation+'_request_cancelled')
            if operation in ('enable','return_home') and self.deployment and getattr(self.deployment,'network_mode',None)=='wired':
                self.deployment.verify_network()
            client=GatewayClient(*endpoint)
            try:return client.control(operation,session)
            finally:client.close()

    def _control_worker(self,operation,session=None):
        endpoint=self.remote_endpoint;generation=self._generation
        if endpoint is None:
            self.log('尚未建立本项目的机器人连接。');return
        def work():
            try:
                result=self._rpc_control(operation,session,endpoint,generation)
                self.events.put(('control_result',(operation,result,generation)))
            except Exception as exc:
                self.log({'enable':'使能','disable':'停用','return_home':'回初始姿态'}[operation]+'请求未确认：'+str(exc))
                self.events.put(('control_error',(operation,generation)))
                if operation=='enable':
                    try:self._rpc_control('disable',None,endpoint)
                    except Exception:pass
                    self.log('已尝试发送停用；实际输出状态等待机器人确认。')
            finally:self.events.put(('control_idle',generation))
        self._control_pending=True;threading.Thread(target=work,daemon=True).start()

    def enable(self):
        if self._control_pending or self._enable_queued:return
        if self.remote_tracking.get('input_profile')!=self.gateway.tracking.policy.name:
            messagebox.showinfo('输入档尚未确认','等待机器人确认相同输入档；请勿启用运动。');return
        if self._runtime_mode_running!='hardware':
            messagebox.showinfo('当前是模型验证','当前模式只显示模型和录制输入。真机接入需先选择 hardware 并加载实际厂家驱动，再部署启动。');return
        ready=self.gateway.runtime.get('readiness',{})
        if not ready.get('ready_to_request_motion'):
            messagebox.showinfo('真机条件未满足','\n'.join(ready.get('blockers',[])) or '等待机器人返回实时检查结果。');return
        session=self.remote_tracking.get('session')
        if not isinstance(session,str) or not session:
            messagebox.showinfo('等待无线输入','等待机器人确认当前头显输入会话，先松开全部按键。');return
        self._enable_baseline=self.gateway.runtime.get('motion',{}).get('attempt_id',0)
        self._enable_request_number=None
        self._enable_queued=True
        self._control_worker('enable',session)

    def return_home(self):
        if self._control_pending or self.home_button.instate(('disabled',)):return
        self._control_worker('return_home',self.remote_tracking.get('session'))

    def tool_dialog(self):
        if self.busy or self.deployment:
            messagebox.showinfo('先停止本项目','标定修改仅在下次部署启动时生效，请先停止本项目。');return
        from tg3_pico_dualarm.tool_setup import measured_tools
        import math
        dialog=tk.Toplevel(self.window);dialog.title('末端参考：模型TCP或实测抓握点')
        ttk.Label(dialog,text='默认使用OpenXR握持点→模型TCP相对控制，可直接进行末端跟随测试。\n需要精确掌心操作点时，再填写实测偏移：XYZ毫米，Roll/Pitch/Yaw度；旋转顺序Rz Ry Rx。',wraplength=800).grid(row=0,column=0,columnspan=8,padx=15,pady=12)
        fields={};row=1
        for name,label in (('controller_to_hand','控制器握持坐标 → 人体操作点'),('tcp_to_hand','机器人 URDF TCP → 实际掌心')):
            fields[name]={}
            for side,side_label in (('left','左'),('right','右')):
                ttk.Label(dialog,text=side_label+' '+label).grid(row=row,column=0,padx=5)
                fields[name][side]=[]
                matrix=self.tool_config[name][side]
                pitch=math.asin(max(-1.,min(1.,-matrix[2][0])))
                if abs(math.cos(pitch))>1e-8:
                    roll=math.atan2(matrix[2][1],matrix[2][2]);yaw=math.atan2(matrix[1][0],matrix[0][0])
                else:roll=0.;yaw=math.atan2(-matrix[0][1],matrix[1][1])
                current=[matrix[i][3]*1000 for i in range(3)]+[math.degrees(v) for v in (roll,pitch,yaw)]
                for column in range(6):
                    var=tk.StringVar(value=f'{current[column]:.6g}');fields[name][side].append(var)
                    ttk.Entry(dialog,textvariable=var,width=8).grid(row=row,column=column+1,padx=3,pady=4)
                row+=1
        ttk.Label(dialog,text='测量记录／依据').grid(row=row,column=0)
        evidence=tk.StringVar(value=self.tool_config.get('measurement_evidence',''));ttk.Entry(dialog,textvariable=evidence,width=70).grid(row=row,column=1,columnspan=6,sticky='ew')
        confirmed=tk.BooleanVar(value=False)
        ttk.Checkbutton(dialog,text='以上为本设备左右分别实测的偏移',variable=confirmed).grid(row=row+1,column=0,columnspan=7,pady=8)
        def save():
            try:
                value=measured_tools(self.tool_config,{name:{side:[v.get() for v in values] for side,values in sides.items()}
                    for name,sides in fields.items()},evidence.get(),confirmed.get())
                destination=ROOT/'.desktop/tool_calibration.json';destination.parent.mkdir(exist_ok=True)
                destination.write_text(json.dumps(value,ensure_ascii=False,indent=2),encoding='utf-8')
                self.tool_config=value;self.log('工具标定已保存；下次部署启动生效。');dialog.destroy()
            except (ValueError,OSError) as exc:messagebox.showerror('未保存',str(exc),parent=dialog)
        ttk.Button(dialog,text='保存实测标定',command=save).grid(row=row+2,column=1,columnspan=3,pady=12)
        def use_tcp():
            from tg3_pico_dualarm.tool_setup import tcp_reference_tools
            value=tcp_reference_tools(self.tool_config)
            destination=ROOT/'.desktop/tool_calibration.json';destination.parent.mkdir(exist_ok=True)
            destination.write_text(json.dumps(value,ensure_ascii=False,indent=2),encoding='utf-8')
            self.tool_config=value;self.log('已选择模型TCP参考；未声明掌心已实测，下次部署生效。');dialog.destroy()
        ttk.Button(dialog,text='使用模型TCP参考',command=use_tcp).grid(row=row+2,column=4,columnspan=3,pady=12)

    def choose_record_directory(self):
        if self.manual_recording.active or self._record_pending:return
        selected=filedialog.askdirectory(title='选择U盘上的保存目录（只写入所选位置，不自动换盘）')
        if selected:self.record_destination=str(Path(selected).resolve())

    def configure_recording(self):
        if self.manual_recording.active or self._record_pending:return
        if not self._session_credentials:
            messagebox.showinfo('录制未就绪','请先部署并启动检查；配置录制不要求启用运动。');return
        config=dict(self._session_credentials);generation=self._generation
        def work():
            try:self.events.put(('record_catalogue',(generation,discover(config))))
            except Exception as exc:self.events.put(('recording_error',str(exc)))
        threading.Thread(target=work,daemon=True).start()

    def _record_dialog(self,catalogue):
        dialog=tk.Toplevel(self.window);dialog.title('选择录制数据（相机默认不选）')
        ttk.Label(dialog,text='位姿、按键和操作状态：JSONL；勾选的ROS消息：MCAP。\n相机由你明确勾选；不启动相机驱动。未验收U盘持续写入与现场操控隔离。',wraplength=750).pack(padx=12,pady=8)
        box=ttk.Frame(dialog);box.pack(fill='both',expand=True,padx=12)
        variables=[]
        for item in catalogue:
            selected=tk.BooleanVar(value=(item['topic'] in self.record_topics if self.record_topics is not None else not item['camera']))
            ttk.Checkbutton(box,text=item['topic']+'  '+item['type'],variable=selected).pack(anchor='w')
            variables.append((item,selected))
        rate=tk.StringVar(value=str(self.record_rate//1024))
        row=ttk.Frame(dialog);row.pack(pady=6)
        ttk.Label(row,text='独立录制通道限速 KiB/s').pack(side='left')
        ttk.Combobox(row,textvariable=rate,values=('256','512','1024','2048','4096'),state='readonly',width=8).pack(side='left')
        ttk.Label(dialog,text='原始图像可能超过限速：若积压超过1秒或64MiB，将结束录制并标记不完整。\n相机需同时选择对应CameraInfo；控制的实时保护不变。',wraplength=750).pack(padx=12,pady=6)
        def save():
            self.record_topics={item['topic']:item['type'] for item,var in variables if var.get()}
            self.record_rate=int(rate.get())*1024;dialog.destroy()
        ttk.Button(dialog,text='确认所选数据',command=save).pack(pady=10)

    def start_recording(self):
        if self._record_pending or self.manual_recording.active:return
        if self._connection_mode_running=='USB（机器人直连）':
            messagebox.showinfo('直连录制未验收','本轮先验证无录制VR控制；直连输入镜像录制仍待专项验收。');return
        if not (self._session_credentials and self.record_destination and self.record_topics is not None and self.remote_endpoint):
            messagebox.showinfo('录制未就绪','先连接机器人，再选择保存目录和需要录制的数据。');return
        config=dict(self._session_credentials,destination=self.record_destination,
            topics=dict(self.record_topics),rate=self.record_rate,software=__version__,
            calibration={'hand_sha256':self.hand_config_digest,
                         'tool_sha256':hashlib.sha256(json.dumps(
                             self.tool_config,sort_keys=True).encode('utf-8')).hexdigest()})
        self.record_outcome.set('未标记')
        endpoint=self.remote_endpoint;generation=self._generation;epoch=self._record_epoch;self._record_pending=True
        def work():
            client=GatewayClient(*endpoint)
            try:
                config['token']=client.recording_access('start')['token']
                with self._stop_lock:
                    if self.closed or generation!=self._generation or epoch!=self._record_epoch:raise RuntimeError('录制启动已取消')
                    self.manual_recording.start(config)
                self.events.put(('record_started',None))
            except Exception as exc:self.events.put(('recording_error',str(exc)))
            finally:client.close()
        threading.Thread(target=work,daemon=True).start()

    def stop_recording(self):
        self._record_epoch+=1
        self.manual_recording.stop()
        # The worker drains and finalizes through its own SSH channel. Closing
        # or failing that recording connection has no control authority.

    def save_record_outcome(self):
        status=self.manual_recording.status
        outcome={'成功':'success','失败':'failure','未标记':'unlabeled'}[self.record_outcome.get()]
        try:
            write_outcome(status['path'],outcome)
            self._log_ui('已保存示范结果：'+self.record_outcome.get())
        except (OSError,ValueError) as exc:
            self._log_ui('示范结果保存失败：'+str(exc))

    def _recording_ui(self):
        previous=self.manual_recording.status.get('state');status=self.manual_recording.poll()
        active=self.manual_recording.active or self._record_pending
        ready=bool(self._session_credentials and self.remote_endpoint and not self.gateway.runtime.get('connection_error')
            and self.record_destination and self.record_topics is not None and self._connection_mode_running!='USB（机器人直连）')
        self.record_start.configure(text='开始录制（待现场验收）' if ready else '开始录制（未就绪）',state='normal' if ready and not active else 'disabled')
        self.record_stop.configure(state='normal' if active else 'disabled')
        self.record_label.configure(state='normal' if status.get('complete') and not active else 'disabled')
        labels={'off':'关闭','checking':'检查磁盘和带宽','recording':'正在录制','stopping':'正在停止并封存','error':'失败，该段数据不完整'}
        self.record_state.set('录制：'+labels.get(status['state'],status['state'])+'；位置：'+str(status.get('path',self.record_destination or '未选择'))+
            f"；时长 {status.get('duration_s',0):.0f}秒；数据 {status.get('bytes',0)/1024**2:.1f}MiB"+
            (f"；剩余 {status['free_bytes']/1024**3:.1f}GiB" if 'free_bytes' in status else '')+
            ('；'+status['reason'] if status.get('reason') else ''))
        if status['state']=='error' and previous!='error':
            self._log_ui('录制停止且不完整：'+str(status.get('reason'))+'；不因此停用VR操控。')

    def stop_async(self):
        self._generation+=1
        self.gateway.tracking.invalidate('operator_stopped_or_cancelled')
        self.gateway.runtime={};self.remote_tracking={}
        self._enable_queued=False;self._remote_status_at=None
        if self.busy and self.pending_deployment:
            self.pending_deployment.closing.set();self.log('正在取消本项目部署。');return
        if self.busy:return
        self.busy=True;self.start_button.configure(state='disabled')
        def work():
            try:self.stop()
            finally:self.events.put(('idle',None))
        threading.Thread(target=work,daemon=True).start()

    def stop(self):
        with self._stop_lock:
            self._enable_queued=False;self._remote_status_at=None
            self.stop_recording();self._session_credentials=None;self.record_topics=None
            self._generation+=1
            endpoint=self.remote_endpoint
            if endpoint is not None and self._runtime_mode_running=='hardware':
                try:
                    self._rpc_control('disable',None,endpoint)
                    self.log('已发送停用请求；等待厂家停止／释放确认，未据此宣称实物已停止。')
                except Exception as exc:self.log('停用请求未确认：'+str(exc))
            self.remote_endpoint=None
            if self.pending_deployment:
                try:self.pending_deployment.close()
                except Exception as exc:self.log('部署连接清理异常：'+str(exc))
            if self.relay:
                self.relay.close();self.relay=None
            client,self.remote_client=self.remote_client,None
            if client:client.close()
            deployment,self.deployment=self.deployment,None
            if deployment:
                try:deployment.close()
                except Exception as exc:self.log('机器人会话关闭异常：'+str(exc))
            self.remote_tracking={};self._runtime_mode_running=None
            self._connection_mode_running=None
            self._control_pending=False
            self.gateway.tracking.invalidate('operator_stopped')
            self.gateway.runtime={}
            self.log('本项目连接已关闭；真机停用与恢复以厂家确认日志为准。')
            self.events.put(('reset_input_profile',None))

    def _log_ui(self,value):
        self.logbox.configure(state='normal');self.logbox.insert('end',value+'\n');self.logbox.see('end')
        if int(self.logbox.index('end-1c').split('.')[0])>500:self.logbox.delete('1.0','150.0')
        self.logbox.configure(state='disabled')

    def poll(self):
        if self.closed:return
        if self._poll_after_id is not None:
            try:self.window.after_cancel(self._poll_after_id)
            except tk.TclError:pass
            self._poll_after_id=None
        while not self.events.empty():
            kind,value=self.events.get()
            if kind=='log':self._log_ui(value)
            elif kind=='usb_result':
                self._usb_busy=False;ok,detail=value
                if self.connection_mode.get()=='USB（机器人直连）':continue
                self.usb_status.set('USB：通道已设置，等待头显配对' if ok else 'USB：未就绪，请检查数据线和调试授权')
                self._log_ui(detail if ok else 'USB 通道未准备：'+detail)
            elif kind=='robot_headset_pairing':
                generation,pin=value
                if generation!=self._generation:continue
                if not isinstance(pin,str) or not re.fullmatch('[0-9]{8}',pin):
                    self._log_ui('机器人端配对码格式异常，未显示。');continue
                self._pair_pin=pin;self._refresh_pairing()
                self.usb_status.set('USB直连机器人：通道已设置，等待头显配对')
            elif kind=='host':
                host,fingerprint,done,answer=value
                answer.append(messagebox.askyesno('首次连接机器人',host+'\nSSH 主机指纹：\n'+fingerprint+'\n请与机器人终端核对，是否信任？'));done.set()
            elif kind=='idle':
                self.busy=False;self.start_button.configure(state='normal')
                state='disabled' if self.deployment else 'readonly'
                self.runtime_selector.configure(state=state);self.pose_selector.configure(state=state)
                self.backend_selector.configure(state=state)
                self.profile_selector.configure(state=state)
                self.robot_link_selector.configure(state=state);self.wired_selector.configure(state=state)
            elif kind=='network_report':
                self.network_report=value
                self._wired_interface_map={'自动':None,**{str(a.get('Name'))+' ['+str(a['ifIndex'])+']':a['ifIndex'] for a in value.get('wired_adapters',[])}}
                self.wired_selector.configure(values=tuple(self._wired_interface_map))
                if self.wired_interface.get() not in self._wired_interface_map:self.wired_interface.set('自动')
                self.network_state.set(str(value.get('reason','检查未完成')))
                self._log_ui('有线检查：'+json.dumps(value,ensure_ascii=False))
            elif kind=='network_live':
                generation,report=value
                if generation!=self._generation:continue
                self.network_report=report
                self.network_state.set(('SSH已绑定物理有线：'+str(report['interface_name'])+' / '+report['source_ipv4']+'；系统代理及虚拟网卡保持原样') if report else '机器人连接未限定接口；不作为有线验收结果')
            elif kind=='network_error':
                self.network_state.set(str(value));self._log_ui(str(value))
            elif kind=='robot_network_report':
                self._robot_network_dialog(value)
            elif kind=='reset_input_profile':
                self.input_profile.set('normal')
                with self.gateway.lock:
                    self.gateway.tracking=LatestTracking(clock=self.gateway.clock,policy=NORMAL)
                if not self.closed:self.pair()
            elif kind=='remote':
                client,generation,data=value
                if client is not self.remote_client or generation!=self._generation:continue
                self.remote_tracking=data.get('tracking',{});self.gateway.runtime=data.get('runtime',{})
                self._remote_status_at=time.monotonic()
                motion=data.get('runtime',{}).get('motion',{})
                if (motion.get('attempt_id',0)>self._enable_baseline or motion.get('state') in ('preparing','acquiring','settling','armed','holding')):
                    self._enable_queued=False
                processed=data.get('runtime',{}).get('motion_request_result',{})
                if self._enable_request_number is not None and processed.get('request_number')==self._enable_request_number and not processed.get('pending'):
                    self._enable_queued=False
                self._link_disable_attempted=False
            elif kind=='remote_error':
                client,generation,error=value
                if client is not self.remote_client or generation!=self._generation:continue
                if self.gateway.runtime.get('connection_error')!=error:self._log_ui('机器人链路异常：'+error)
                self.gateway.runtime={'connection_error':error}
                self.stop_recording()
                if self._runtime_mode_running=='hardware' and not self._link_disable_attempted:
                    self._link_disable_attempted=True;self._control_worker('disable',None)
            elif kind=='station_exit':
                client,generation,detail=value
                if client is not self.remote_client or generation!=self._generation:continue
                reason=str(detail.get('reason','station_exited'))
                self.gateway.runtime={'connection_error':reason,'station_exit':detail}
                self.stop_recording()
                if self.relay:self.relay.stop.set()
                self._log_ui('机器人接收程序停止：'+reason+'。请停止本项目后重新启动；运动不会自动恢复。')
            elif kind=='control_result':
                operation,result,generation=value
                if generation!=self._generation:continue
                self._log_ui({'enable':'使能','disable':'停用','return_home':'回初始姿态'}[operation]+
                             '响应：'+json.dumps(result,ensure_ascii=False))
                if operation=='enable' and result.get('accepted') and result.get('queued'):
                    self._enable_request_number=result.get('request_number')
                    self._log_ui('请求仅已入队；以随后实时“armed／holding”及厂家确认判断输出状态。')
                elif operation=='enable':self._enable_queued=False
            elif kind=='control_idle':
                if value==self._generation:self._control_pending=False
            elif kind=='control_error':
                operation,generation=value
                if generation==self._generation and operation=='enable':self._enable_queued=False
            elif kind=='recording_error':
                self._record_pending=False
                self._log_ui('录制未开始或已停止：'+str(value)+'；操控保持独立。')
                messagebox.showerror('录制未完成',str(value)+'\n只停止录制，不停用操控。')
            elif kind=='record_catalogue':
                generation,catalogue=value
                if generation==self._generation:self._record_dialog(catalogue)
            elif kind=='record_started':
                self._record_pending=False
        direct_usb=bool(self.deployment and getattr(self.deployment,'headset_transport',None)=='robot_usb')
        tracking=(self.remote_tracking or self.gateway.tracking.status()) if direct_usb else self.gateway.tracking.status()
        runtime=self.gateway.runtime;bag=runtime.get('rosbag',{})
        if direct_usb:
            usb=runtime.get('usb',{});age=tracking.get('sample_age_s')
            fresh=bool(tracking.get('focused') and tracking.get('head_tracked') and tracking.get('left_tracked')
                and tracking.get('right_tracked') and isinstance(age,(float,int)) and 0<=age<=self.gateway.tracking.policy.total_age_s)
            self.usb_status.set('机器人USB：'+('通道已建立' if usb.get('reverse_ready') else '等待检查')+
                '；头显跟踪：'+('正常' if tracking.get('head_tracked') else '未就绪')+
                '；数据新鲜：'+('是' if fresh else '否')+
                ('；输入年龄 %.1fms'%(age*1000) if isinstance(age,(float,int)) else ''))
        camera=any(typ in ('sensor_msgs/msg/Image','sensor_msgs/msg/CompressedImage') for typ in bag.get('topics',{}).values())
        policy=self.gateway.tracking.policy
        remote_age=self.remote_tracking.get('sample_age_s')
        self.profile_status.set(('临时低速试验' if policy.name!='normal' else '正常档')+
            '；累计上限 %.0fms；机器人输入年龄 '%(policy.total_age_s*1000)+
            ('%.1fms'%(remote_age*1000) if isinstance(remote_age,(int,float)) else '待确认')+
            '；机器人档：'+str(self.remote_tracking.get('input_profile','待确认')))
        motion=runtime.get('motion',{});ready=runtime.get('readiness',{})
        mode=self._runtime_mode_running or self.runtime_mode.get()
        stale=bool(self.deployment and self._remote_status_at is not None and time.monotonic()-self._remote_status_at>2.)
        processed=runtime.get('motion_request_result',{})
        request_error=(processed.get('reason') if self._enable_request_number is not None and processed.get('request_number')==self._enable_request_number
            and processed.get('accepted') is False and not processed.get('pending') else None)
        if request_error:self._enable_queued=False
        phase,text=motion_view(runtime,mode,queued=self._enable_queued or self._control_pending,stale=stale,request_error=request_error)
        self.banner.set(text)
        self.banner_label.configure(foreground='#006b34' if phase in ('enabled','following') else '#b00020' if phase in ('failed','fault','unknown') else '#9c3a00')
        can_enable=bool(mode=='hardware' and ready.get('ready_to_request_motion') and not stale and phase in ('disabled','failed')
            and not self._enable_queued and not self._control_pending)
        self.enable_button.configure(state='normal' if can_enable else 'disabled')
        current=runtime.get('control',{})
        home_state=current.get('return_home')
        can_return=bool(mode=='hardware' and phase in ('enabled','following') and not stale
            and current.get('home_available') and home_state not in ('returning','waiting_for_release')
            and self.remote_tracking.get('session') and not self._control_pending)
        self.home_button.configure(state='normal' if can_return else 'disabled')
        self.timing_state.set(timing_summary(self.remote_tracking,runtime,self.relay.status if self.relay else None))
        right_hand=((runtime.get('control',{}).get('hands') or {}).get('motor_diagnostics') or {}).get('right',{})
        grasp_phase={'opening_fingers':'张指','opening_thumb':'拇指归位','opposing_thumb':'拇指对掌',
            'closing':'包握','holding':'触觉持握','hold_position':'位置保持','waiting_for_tactile':'等待右手触觉数据'}
        home_text={'returning':'双臂正在回初始姿态','arrived':'双臂已到初始姿态',
                   'waiting_for_release':'回位已中止，请松开手柄按键','cancelled':'回位已取消'}
        home_result=processed.get('reason') if processed.get('operation')=='return_home' and processed.get('accepted') is False else None
        self.state.set('头部：'+str(tracking['head_tracked'])+'   左手：'+str(tracking['left_tracked'])+
            '   右手：'+str(tracking['right_tracked'])+'   已收帧：'+str(tracking.get('counters',{}).get('accepted',0))+
            '   输入转发：'+('USB直达机器人' if direct_usb else self.relay.status['state'] if self.relay else '尚未启动')+
            '\n软件：'+str(runtime.get('control',{}).get('state','等待原生求解'))+
            '   厂家接管：'+str(motion.get('state','未请求'))+'   尝试编号：'+str(motion.get('attempt_id','待确认'))+
            '   本次实际输出帧：'+str(motion.get('actual_output_count','待确认'))+
            '   数据录制：'+str(self.manual_recording.status['state'])+
            '\n握持值：左 '+str(tracking.get('left_grip','待测'))+'，右 '+str(tracking.get('right_grip','待测'))+
            '   跟随原因：'+str(runtime.get('control',{}).get('reasons',{}))+
            ('\n'+home_text[home_state] if home_state in home_text else '')+
            ('\n回位请求未执行：'+home_result if home_result else '')+
            ('\n右手：'+grasp_phase.get(right_hand['grasp_phase'],right_hand['grasp_phase'])
             if 'grasp_phase' in right_hand else ''))
        self._recording_ui()
        blockers=ready.get('blockers')
        if blockers is None:
            blockers=['等待机器人端原生环境及实时状态。']
            if mode=='hardware':
                if self.hardware_backend.get()=='xarm':blockers.append('等待核对本机 XARM、双臂控制器及物理急停反馈。')
                elif self.factory_path is None:blockers.append('缺少实际厂家驱动 JSON 及已审查源码。')
            blockers.append('无线、透视与实际抓取效果等待现场验收。')
        description='\n'.join(str(item) for item in blockers)
        description+='\n\n当前控制状态：'+str(motion.get('state','未请求'))+'\n原因：'+str(motion.get('reason','等待检查'))
        if motion.get('last_failure'):
            description+=('\n本次接管失败：' if current_failure(motion) else '\n历史失败（不代表本次）：')+json.dumps(motion['last_failure'],ensure_ascii=False)
        if runtime.get('timing'):description+='\n时效统计（毫秒；保守输入年龄，不是实测完整动作延迟）：'+json.dumps(runtime['timing'],ensure_ascii=False)
        description+='\n真机整体验收：'+('已记录通过' if ready.get('hardware_ready') else '尚未完成')
        reference=ready.get('end_effector_reference') or {}
        if reference:description+='\n末端参考：'+str(reference.get('description',reference.get('kind')))
        xarm=runtime.get('manufacturer',{}).get('xarm_status',{})
        if xarm:description+='\n灵巧手：'+json.dumps(xarm.get('hands',{}),ensure_ascii=False)
        request_result=runtime.get('motion_request_result')
        if request_result:description+='\n最近请求处理结果：'+json.dumps(request_result,ensure_ascii=False)
        self.issues.set(description)
        if description!=self._last_issues:
            self.issuebox.configure(state='normal');self.issuebox.delete('1.0','end');self.issuebox.insert('end',description)
            self.issuebox.configure(state='disabled');self._last_issues=description
        self.preview.update_control(runtime.get('control',{}))
        self._poll_after_id=self.window.after(100,self.poll)

    def close(self):
        if self.closed:return
        if self._poll_after_id is not None:
            try:self.window.after_cancel(self._poll_after_id)
            except tk.TclError:pass
            self._poll_after_id=None
        self.closed=True;self.gateway.tracking.invalidate('desktop_closed')
        try:self.stop()
        finally:
            self.server.close()
            self.manual_recording.close()
            try:self.recorder.close()
            finally:self.window.destroy()


if __name__=='__main__':
    import multiprocessing
    multiprocessing.freeze_support()
    try:desk=Desk();desk.window.mainloop()
    except Exception as exc:
        messagebox.showerror('启动失败',str(exc));raise SystemExit(2)
