"""Owned factory startup and reversible bridge handoff for the VR station.

Run by the reviewed launcher as root. Secrets arrive only on stdin. Startup
loads inactive controllers; only the station's explicit operator admission
may request arm enable. Factory and Placo use different native environments.
"""
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import threading
import time
import uuid

RECOVERY_ROOT=Path('/home/ubuntu/.local/share/tg3-teleop/recovery')


def event(name,**fields):
    print(json.dumps(dict(event=name,**fields),ensure_ascii=False),flush=True)


class FactoryObserver:
    def __init__(self):
        import rclpy
        from .manufacturer_feedback import ManufacturerFeedback
        from sensor_msgs.msg import Joy
        from rclpy.qos import qos_profile_sensor_data
        self.rclpy=rclpy;rclpy.init(args=[])
        self.node=rclpy.create_node('tg3_factory_supervisor_'+str(os.getpid()),enable_rosout=False,start_parameter_services=False)
        self.feedback=ManufacturerFeedback().subscribe(self.node);self.remote=None
        self.subscription=self.node.create_subscription(Joy,'/sbus_data',self._joy,qos_profile_sensor_data)

    def _joy(self,message):self.remote=(tuple(message.axes),time.monotonic())

    def spin(self,seconds=.02):self.rclpy.spin_once(self.node,timeout_sec=seconds)

    def guard(self):
        from .xarm_preparation import XArmPreparation
        XArmPreparation.physical_guard(self)
        snapshot=self.feedback.snapshot(require_hands=False)
        if not snapshot['ready']:raise RuntimeError('factory_feedback_not_ready:'+','.join(snapshot['issues']))
        if snapshot['mode']['current_state'] not in ('SWR','HBWALK'):raise RuntimeError('settled_SWR_or_HBWALK_required')
        return snapshot

    clock=staticmethod(time.monotonic)

    def wait_ready(self,timeout=10.,require_hands=False):
        # Startup DDS discovery is separate from the unchanged .15/.25/.5 s
        # live freshness limits. No process handoff occurs before this gate.
        from .feedback_startup import FeedbackStartupGate
        gate=FeedbackStartupGate(self.feedback,require_hands=require_hands,guard=self.guard,timeout=timeout)
        while True:
            self.spin()
            try:
                if gate.poll():return gate.status()
            finally:self.last_startup_diagnostics=gate.status()

    def observe(self,*,require_neutral=True,seconds=2.):
        end=time.monotonic()+seconds;error=None
        while time.monotonic()<end:
            self.spin()
            try:self.guard();error=None
            except RuntimeError as exc:error=exc
        if error:raise error
        return self.guard()

    def call(self,typ,name,configure=lambda request:None):
        client=self.node.create_client(typ,name)
        try:
            if not client.wait_for_service(timeout_sec=.5):raise RuntimeError('factory_cleanup_service_missing:'+name)
            request=typ.Request();configure(request);future=client.call_async(request)
            deadline=time.monotonic()+.15
            while not future.done() and time.monotonic()<deadline:self.spin(.002)
            if not future.done() or future.result() is None:raise RuntimeError('factory_cleanup_ack_timeout:'+name)
            return future.result()
        finally:self.node.destroy_client(client)

    def no_existing_factory(self):
        graph=dict(self.node.get_service_names_and_types())
        if any(n.endswith('/list_controllers') or n.startswith('/EAIHardware/') for n in graph):
            raise RuntimeError('existing_factory_controller_do_not_start_duplicate')

    def wait_disabled(self,factory,stop):
        from controller_manager_msgs.srv import ListControllers
        deadline=time.monotonic()+35.
        client=self.node.create_client(ListControllers,'/controller_manager/list_controllers');pending=None
        try:
            while time.monotonic()<deadline and not stop.is_set():
                self.spin()
                # Loading the native plugin/ldd/xacro can block this observer
                # longer than the radio freshness window. Output is still
                # isolated and disabled: reacquire fresh data before ready,
                # rather than treating startup discovery as motion failure.
                if self.feedback.snapshot(require_hands=False)['physical_estop'] is True:
                    raise RuntimeError('physical_estop_active_during_disabled_startup')
                if factory.process.poll() is not None:raise RuntimeError('disabled_factory_exited_see_factory_log')
                if client.service_is_ready() and pending is None:
                    pending=client.call_async(ListControllers.Request())
                if pending is not None and pending.done():
                    response=pending.result();pending=None
                    if response and all(any(c.name==name and c.state=='inactive' for c in response.controller) for name in
                        ('empty_L_controller','empty_R_controller','jointspace_ruckig_qp_controller')):
                        if any(c.state=='active' and c.claimed_interfaces for c in response.controller):
                            raise RuntimeError('unexpected_active_factory_resource_controller')
                        raw=self.node.get_publishers_info_by_topic('/tg3/xarm/arm_raw')
                        if len(raw)==1 and raw[0].node_name=='EAIHardware' and raw[0].topic_type=='ros2_bridge_msgs/msg/ArmCtrl':
                            if self.node.get_publishers_info_by_topic('/arm/cmd'):raise RuntimeError('unisolated_real_arm_writer')
                            try:self.guard()
                            except RuntimeError:continue
                            return self.configure_holding_parameters(factory.holding_profile)
            raise RuntimeError('inactive_factory_startup_timeout_or_cancelled')
        finally:self.node.destroy_client(client)

    def configure_holding_parameters(self,profile,*,types=None):
        """Configure only this inactive, isolated owned factory; never enable."""
        if types is None:
            from rcl_interfaces.srv import GetParameters,SetParameters
            from rcl_interfaces.msg import Parameter,ParameterValue
            from controller_manager_msgs.srv import ListControllers
            types=dict(get=GetParameters,set=SetParameters,parameter=Parameter,value=ParameterValue,list=ListControllers)
        from .xarm_adapter import _controllers,_arm_controller
        def inactive():
            controllers=_controllers(self.call(types['list'],'/controller_manager/list_controllers'))
            if any(item['state']=='active' and _arm_controller(name,item) for name,item in controllers.items()):
                raise RuntimeError('holding_gain_change_requires_inactive_arms')
            if self.node.get_publishers_info_by_topic('/arm/cmd'):
                raise RuntimeError('holding_gain_change_requires_no_actuator_publisher')
        def read():
            response=self.call(types['get'],'/EAIHardware/get_parameters',lambda r:setattr(r,'names',['kp','kd']))
            return profile.parameter_values(response)
        inactive();before=read()
        if not profile.matches(before):
            def configure(request):
                request.parameters=[]
                for name in ('kp','kd'):
                    parameter=types['parameter']();parameter.name=name;parameter.value=types['value']()
                    parameter.value.type=8;parameter.value.double_array_value=list(getattr(profile,name))
                    request.parameters.append(parameter)
            result=self.call(types['set'],'/EAIHardware/set_parameters',configure)
            if len(result.results)!=2 or any(item.successful is not True for item in result.results):
                raise RuntimeError('factory_holding_gain_update_rejected')
        after=read();inactive()
        if not profile.matches(after):raise RuntimeError('factory_holding_gain_readback_mismatch')
        return dict(before={k:list(v) for k,v in before.items()},after={k:list(v) for k,v in after.items()},
            hardware_motion_enabled=False,arm_controllers_inactive=True,actuator_publishers=0)

    def fallback_release(self,directory):
        """Recover only a journaled admission; never enable or send arm targets."""
        path=Path(directory)/'preparation.json'
        if not path.exists():return dict(required=False,reason='no_operator_admission_journal')
        journal=json.loads(path.read_text(encoding='utf-8'))
        if journal.get('restored') is True:return dict(required=False,reason='station_restoration_confirmed')
        from std_srvs.srv import SetBool
        from eai_manipulator_msgs.srv import Mode
        from controller_manager_msgs.srv import ListControllers,SwitchController
        from .xarm_adapter import _controllers,_arm_controller,_resources,JOINT_NAMES
        response=self.call(SetBool,'/EAIHardware/set_arm_enable',lambda r:setattr(r,'data',False))
        if response.success is not True:raise RuntimeError('fallback_arm_disable_not_acknowledged')
        controllers=_controllers(self.call(ListControllers,'/controller_manager/list_controllers'));active=[]
        for name,item in controllers.items():
            if item['state']=='active' and _arm_controller(name,item):
                if not item['claimed'] or not _resources(item['claimed'])<=set(JOINT_NAMES):
                    raise RuntimeError('fallback_unknown_or_nonarm_controller_resources')
                active.append(name)
        if active:
            def deactivate(r):
                if hasattr(r,'deactivate_controllers'):r.deactivate_controllers=active;r.activate_controllers=[]
                else:r.stop_controllers=active;r.start_controllers=[]
                r.strictness=2;r.timeout.sec=0;r.timeout.nanosec=150000000
            if self.call(SwitchController,'/controller_manager/switch_controller',deactivate).ok is not True:
                raise RuntimeError('fallback_controller_deactivation_rejected')
            controllers=_controllers(self.call(ListControllers,'/controller_manager/list_controllers'))
            if any(item['state']=='active' and _arm_controller(name,item) for name,item in controllers.items()):
                raise RuntimeError('fallback_controller_deactivation_not_observed')
        if journal.get('mode_change_requested'):
            from .xarm_preparation import XArmPreparation
            original=journal['original_arm_mode']
            if original not in (0,3):raise RuntimeError('fallback_original_arm_mode_unknown')
            if self.call(Mode,'/EAIHardware/set_arm_mode',lambda r:setattr(r,'mode',original)).result is not True:
                raise RuntimeError('fallback_original_arm_mode_rejected')
            if XArmPreparation._mode(self.call(Mode,'/EAIHardware/get_arm_mode'))!=original:
                raise RuntimeError('fallback_original_arm_mode_unconfirmed')
        body_restored=False
        if journal.get('original_body')=='SWR':
            from std_msgs.msg import String
            publisher=self.node.create_publisher(String,'/hric/robot/fsm_state_cmd',1)
            try:
                deadline=time.monotonic()+4.;frames=0;next_frame=0.
                while time.monotonic()<deadline:
                    self.spin()
                    from .xarm_preparation import XArmPreparation
                    XArmPreparation.physical_guard(self)
                    snap=self.feedback.snapshot(require_hands=False);state=self.feedback.read_state()
                    if state.estop is not False or state.quality!='measured' or state.mode=='UNKNOWN':
                        raise RuntimeError('fallback_body_feedback_or_estop_not_safe')
                    mode=state.mode
                    if mode=='SWR' and snap['ready']:body_restored=True;break
                    if mode=='SWR':continue
                    if mode!='HBWALK':break
                    if frames<3 and time.monotonic()>=next_frame:
                        message=String();message.data='gotoSTART';publisher.publish(message)
                        frames+=1;next_frame=time.monotonic()+.04
            finally:self.node.destroy_publisher(publisher)
            if not body_restored:raise RuntimeError('fallback_SWR_restoration_unconfirmed')
        return dict(required=True,arm_disable_acknowledged=True,body_restored=body_restored)

    def close(self):
        self.feedback.close();self.node.destroy_node();self.rclpy.shutdown()


def station_environment(app,directory):
    env=os.environ.copy();env['PYTHONNOUSERSITE']='1'
    env['PYTHONPATH']=str(app/'src')+(':'+env['PYTHONPATH'] if env.get('PYTHONPATH') else '')
    env['LD_LIBRARY_PATH']=str(app/'.venv/lib/python3.12/site-packages/cmeel.prefix/lib')+':'+env.get('LD_LIBRARY_PATH','')
    env.update(OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1',MKL_NUM_THREADS='1',TG3_SUPERVISED_SESSION=str(directory))
    return env


def run(argv=None):
    from .factory_runtime import FactoryBridgeHandoff,DisabledFactoryProcess,stop_owned_group
    if sys.platform!='linux' or os.geteuid()!=0:raise RuntimeError('managed_XARM_requires_robot_administrator_launcher')
    argv=list(sys.argv[1:] if argv is None else argv)
    if '--hardware' not in argv or '--xarm' not in argv:raise RuntimeError('managed_factory_requires_explicit_hardware_xarm')
    if '--headset-usb-local' in argv:
        from .robot_usb import RobotHeadsetUSB
        RobotHeadsetUSB().check_available()  # Same root identity as station; fail before factory handoff.
    key=sys.stdin.readline().strip()
    if len(key)<32:raise RuntimeError('private_relay_key_missing')
    app=Path(__file__).resolve().parents[2]
    # These are recovery/configuration receipts, never trajectory recordings.
    directory=RECOVERY_ROOT/('factory_'+time.strftime('%Y%m%d_%H%M%S')+'_'+uuid.uuid4().hex[:8])
    directory.mkdir(parents=True);directory.chmod(0o700)
    stop=threading.Event();child=None;observer=None;bridge=None;factory=None;errors=[]
    for sig in (signal.SIGINT,signal.SIGTERM,signal.SIGHUP):signal.signal(sig,lambda *_:stop.set())
    def parent_watch():sys.stdin.read();stop.set()
    threading.Thread(target=parent_watch,daemon=True).start()
    try:
        from .holding_profile import load_holding_profile
        holding_profile=load_holding_profile()  # Before bridge/body/controller mutations.
        event('verified_holding_configuration',**holding_profile.status())
        observer=FactoryObserver();observer.wait_ready();observer.no_existing_factory()
        bridge=FactoryBridgeHandoff(directory,observe=observer.observe,log=lambda msg:event('factory_handoff',message=msg))
        bridge.start()
        factory=DisabledFactoryProcess(directory,holding_profile=holding_profile);factory.start()
        holding_parameters=observer.wait_disabled(factory,stop)
        event('factory_holding_parameters_confirmed',parameters=holding_parameters)
        if stop.is_set():raise RuntimeError('startup_cancelled')
        event('factory_ready_disabled',model_sha256=factory.model_sha,session=str(directory),hardware_motion_enabled=False)
        command=[str(app/'.venv/bin/python'),'-s','-m','tg3_pico_dualarm.station','--key-stdin',*argv]
        child=subprocess.Popen(command,stdin=subprocess.PIPE,env=station_environment(app,directory),start_new_session=True)
        child.stdin.write((key+'\n').encode());child.stdin.flush();key=None
        while not stop.wait(.02):
            observer.spin(.005)
            if child.poll() is not None:return child.returncode
            if factory.process.poll() is not None:raise RuntimeError('owned_factory_exited_station_stopping')
        return 0
    finally:
        if child:
            if child.stdin and not child.stdin.closed:child.stdin.close()
            try:child.wait(timeout=40.)
            except subprocess.TimeoutExpired:stop_owned_group(child)
        if observer and factory and factory.process and factory.process.poll() is None:
            try:
                result=observer.fallback_release(directory)
                event('factory_release_checked',**result)
            except Exception as exc:errors.append('factory_release:'+str(exc))
        if factory:
            try:factory.close()
            except Exception as exc:errors.append('factory_process_stop:'+str(exc))
        if bridge:
            try:bridge.close()
            except Exception as exc:errors.append('factory_bridge_restore:'+str(exc))
        if observer:
            try:observer.close()
            except Exception as exc:errors.append('observer_close:'+str(exc))
        receipt=dict(event='factory_session_closed',errors=errors,original_bridge_restored=bool(bridge and bridge.restored))
        (directory/'supervisor_cleanup.json').write_text(json.dumps(receipt,indent=2),encoding='utf-8')
        event('factory_session_closed',errors=errors,original_bridge_restored=receipt['original_bridge_restored'])
        if errors:raise RuntimeError('factory_cleanup_unconfirmed:'+';'.join(errors))


def main():
    try:return run()
    except Exception as exc:
        event('station_failed',error=str(exc),hardware_motion_enabled=False);return 2


if __name__=='__main__':raise SystemExit(main())
