"""Native wireless control station: dry-run by default, explicit verified hardware."""
from . import safety as motion_policy
import argparse
from contextlib import ExitStack
import json
from pathlib import Path
import sys
import threading
import time
from . import __version__
from .configuration import load
from .controller import Teleoperation
from .model import DualArmModel, default_q, MODEL_SHA256
from .readiness import readiness
from .recording import serializable
from .diagnostics import DiagnosticsSink
from .robot import DryRunRobot
from .wireless import InputGateway, WirelessServer
from .watchdog import CommandWatchdog


def run(args, stop=None):
    stop=stop or threading.Event()
    if not args.key_stdin:raise ValueError('relay_key_must_arrive_on_private_stdin')
    key=sys.stdin.readline().strip()
    if len(key)<32:raise ValueError('missing_relay_key_on_stdin')
    def parent_watch():
        sys.stdin.read()
        stop.set()
    threading.Thread(target=parent_watch,daemon=True).start()
    hardware=bool(getattr(args,'hardware',False))
    robot_usb=bool(getattr(args,'headset_usb_local',False))
    xarm=bool(getattr(args,'xarm',False))
    if xarm and not hardware:raise ValueError('xarm_requires_explicit_hardware_mode')
    config=load(args.config,allow_hardware=hardware)
    from .input_policy import from_config
    input_policy=from_config(config['vr'])
    if (config['robot']['mode']=='hardware')!=hardware:
        raise ValueError('hardware_mode_requires_both_explicit_start_and_matching_configuration')
    bundle=None
    xarm_settings=None
    if xarm:
        from .xarm_config import load_xarm_settings
        xarm_settings=load_xarm_settings(args.config/'xarm.yaml')
    elif hardware:
        if not getattr(args,'factory_config',None):
            raise ValueError('reviewed_manufacturer_driver_bundle_required_no_motion_publishers_created')
        from .manufacturer_session import load_factory_bundle
        bundle=load_factory_bundle(args.factory_config)
    observed={}
    bag_status={'state':'off','ready':False,'reason':'manual_recording_only','topics':{}}
    warnings=[]
    with ExitStack() as cleanup:
        model=DualArmModel()
        safety=config['safety']
        joint_speed=safety['joint_speed_rad_s']
        feedback_lead=safety['feedback_lead_rad']
        bounds=None
        if bundle:
            bounds=bundle.limits.lower,bundle.limits.upper
            model.restrict_joint_limits(*bounds)
            joint_speed=min(joint_speed,min(bundle.limits.velocity))
            feedback_lead=min(feedback_lead,min(bundle.limits.feedback_lead))
        if xarm_settings:
            joint_speed=min(joint_speed,min(xarm_settings.velocity_cap))
            feedback_lead=min(feedback_lead,min(xarm_settings.feedback_lead))
        from .kinematics_worker import ProcessKinematics
        calibration=config['calibration']
        from .tool_setup import reference_status
        end_effector=reference_status(calibration)
        engine=ProcessKinematics(mode=config['vr']['mode'],joint_limits=bounds,velocity_limit=joint_speed,warmup_iterations=12,
            position_weight=calibration['position_weight'],orientation_weight=calibration['orientation_weight'],
            posture_weight=calibration['posture_weight'],torso_waist_locked=config['robot']['torso_waist_locked'])
        cleanup.callback(engine.close)
        ros_session=factory_session=factory_observer=None
        robot=DryRunRobot(default_q())
        admission=None
        if hardware:
            from .motion_control import MotionAdmission
            if xarm:
                from .xarm_adapter import XArmRosSession
                factory_session=XArmRosSession.create(settings=xarm_settings,wait_for_feedback_s=5.,
                    hands_config=getattr(args,'hands_config',None),managed=True)
            else:factory_session=bundle.create_session(wait_for_feedback_s=5.)
            cleanup.callback(factory_session.close)
            robot=factory_session.robot
            factory_observer=factory_session.feedback
            admission=MotionAdmission(robot,calibration_config=calibration,policy=input_policy)
        elif args.ros:
            from .ros_bridge import RosDryRunSession
            try:
                ros_session=RosDryRunSession(observe_robot=True)
            except RuntimeError as exc:
                # Still real ROS telemetry, with missing manufacturer observation
                # reported. This never substitutes fake IK or fake robot state.
                warnings.append('manufacturer_observation_unavailable:'+str(exc))
                ros_session=RosDryRunSession(observe_robot=False)
            cleanup.callback(ros_session.close)
            robot=ros_session.robot
            try:
                from .manufacturer_feedback import ManufacturerFeedback
                factory_observer=ManufacturerFeedback().subscribe(ros_session.node)
                cleanup.callback(factory_observer.close)
            except (RuntimeError,ImportError,ValueError) as exc:
                warnings.append('manufacturer_hand_observation_unavailable:'+str(exc))
                factory_observer=None
        diagnostics=Path('/home/ubuntu/.local/share/tg3-teleop/diagnostics')
        recorder=DiagnosticsSink(diagnostics)
        cleanup.callback(recorder.close)
        if args.ros or hardware:
            probe_finished=threading.Event()
            def probe():
                try:
                    from .preflight import run as observe
                    observed.update(observe(5.))
                except (RuntimeError,ValueError,ImportError,OSError) as exc:
                    observed.update(observation_ready=False,error=str(exc))
                try:
                    recorder.report('robot_preflight',observed)
                finally:probe_finished.set()
            threading.Thread(target=probe,daemon=True).start()
            # Read-only DDS discovery can consume a whole
            # tick on a loaded host. Finish startup before admitting input or
            # starting the command watchdog; do not relax the runtime budget.
            if not probe_finished.wait(10.):raise RuntimeError('startup_readonly_probe_timeout')
        controller=Teleoperation(robot,model,engine,mode=config['vr']['mode'],
            translation_scale=config['vr']['translation_scale'],orientation_scale=config['vr']['orientation_scale'],
            recorder=recorder,allow_hardware=hardware,policy=input_policy,
            watchdog_timeout_s=safety['watchdog_timeout_s'],
            calibration_options={k:config['calibration'][k] for k in
                ('controller_to_hand','tcp_to_hand','tool_offsets_calibrated','tool_reference') if k in config['calibration']})
        from .safety import JointSafetyFilter,SafetyLimits
        controller.filter=JointSafetyFilter(model,SafetyLimits(
            joint_speed=joint_speed,joint_acceleration=safety['joint_acceleration_rad_s2'],
            feedback_lead=feedback_lead,translation_speed=safety['translation_speed_m_s'],
            rotation_speed=safety['rotation_speed_rad_s'],feedback_timeout=safety['feedback_timeout_s']),
            boundary_hold=bool(xarm and getattr(getattr(robot,'transport',None),'holding_profile',None)))
        controller.workspace_extent=safety['workspace_extent_m']
        gateway=InputGateway(role='relay',key=key,policy=input_policy)
        if admission:admission.frame_provider=gateway.tracking.sample
        from .timing import TimingStats
        timing=TimingStats()
        usb_status={'selected':robot_usb,'reverse_ready':False}
        from .motion_requests import MotionRequests
        requests=MotionRequests(lambda:robot.hardware_writes_enabled)
        request_result={}
        if admission:
            def request_control(value):
                # Manual stop must close output even while native IK is waiting.
                if motion_policy.LIMITS_MANAGED_EXTERNALLY and value['operation']=='disable':
                    controller.disarm('explicit_operator_disabled')
                return requests.enqueue(value)
            gateway.control_callback=request_control
        server=WirelessServer(gateway,host='127.0.0.1',port=args.port).start()
        cleanup.callback(server.close)
        headset_code=None
        if robot_usb:
            from .robot_usb import RobotHeadsetUSB
            peer=InputGateway(role='headset',policy=input_policy)
            peer.tracking=gateway.tracking
            gateway.headset_gateway=peer;gateway.accept_frames=False
            gateway.headset_pairing_allowed=lambda:bool(not hardware or (
                robot.state=='disabled' and not admission.preparing and (
                    admission.preparation is None or admission.preparation.restored)))
            head_server=WirelessServer(peer,host='127.0.0.1',port=0).start()
            cleanup.callback(head_server.close)
            usb=RobotHeadsetUSB();cleanup.callback(usb.close)
            usb_status.update(usb.prepare(head_server.address[1]),reverse_ready=True)
            headset_code=peer.open_pairing()
        watchdog=CommandWatchdog(robot.disarm,timeout=safety['watchdog_timeout_s'],active=not hardware)
        watchdog.start()
        cleanup.callback(watchdog.close)
        started=dict(event='robot_station_started',protocol='tg3-native-1',version=__version__,
            input_profile=input_policy.name,runtime_mode='hardware' if hardware else 'dry_run',listen_port=server.address[1],native_placo=True,
            hardware_motion_enabled=False,recording=None,
            headset_transport='robot_usb' if robot_usb else 'workstation')
        if headset_code:started['headset_pairing_code']=headset_code
        print(json.dumps(started),flush=True)
        previous=time.monotonic()-.02
        last_admission_failure=None
        try:
            while not stop.is_set():
                now=time.monotonic()
                if watchdog.check():
                    controller.disarm('command_watchdog_expired')
                    raise RuntimeError('watchdog_expired_restart_required')
                frame=gateway.tracking.sample(now)
                admission_started=time.monotonic()
                if admission:
                    admission.tick(frame)
                    if admission.consume_enable_ack():controller.reseed_after_hardware_enable()
                    action=requests.pop()
                    if action:
                        # No actuator lease exists while awaiting the first
                        # explicit enable. Protect the complete acquisition
                        # before requesting any factory mutation.
                        if action['operation']=='enable':watchdog.activate()
                        if action['operation']=='return_home':
                            if admission.initial_hold_pending or not 0<=time.monotonic()-action['requested_at']<=.12:
                                request_result=dict(accepted=False,reason='return_home_request_unavailable')
                            else:
                                request_result=controller.request_home(gateway.tracking.sample(),action['session'])
                            recorder.event('operator_return_home:'+request_result['reason'],time.monotonic())
                        elif not motion_policy.LIMITS_MANAGED_EXTERNALLY and action['operation']=='enable' and not 0<=time.monotonic()-action['requested_at']<=.12:
                            request_result=dict(accepted=False,reason='motion_request_expired')
                        else:
                            request_result=admission.request(action['operation'],frame,expected_session=action['session'])
                            recorder.event('operator_'+action['operation']+':'+request_result['reason'],time.monotonic())
                        request_result=dict(request_result,operation=action['operation'],
                            request_number=action['request_number'],attempt_id=admission.attempt_id)
                    if admission.consume_enable_ack():controller.reseed_after_hardware_enable()
                    if admission.validated_frame is not None:frame=admission.validated_frame
                timing.add('admission_processing',time.monotonic()-admission_started)
                if admission and admission.last_failure:
                    failure=admission.last_failure
                    if failure['received_at']!=last_admission_failure:
                        last_admission_failure=failure['received_at']
                        request_result=dict(request_result,accepted=False,pending=False,reason=failure['reason'],attempt_id=admission.attempt_id)
                        recorder.event('motion_admission_failed:'+failure['reason'],now,details=failure)
                        print(json.dumps(dict(event='motion_admission_failed',**failure)),file=sys.stderr,flush=True)
                elapsed=now-previous
                timing.add('cycle_interval',elapsed)
                previous=now
                if hardware and (not robot.hardware_writes_enabled or admission.initial_hold_pending):
                    status=dict(state='waiting_hardware_enable',factory_state=robot.state,
                        reason=robot.reason,robot_commands_sent=False)
                    if controller.last_status.get('state')=='disarmed':
                        status['last_control_failure']=controller.last_status.copy()
                else:
                    status=controller.tick(frame,elapsed,record_task_space=gateway.recording_capture_active)
                if engine.faulted and not motion_policy.LIMITS_MANAGED_EXTERNALLY:
                    raise RuntimeError('native_ik_worker_fault:'+str(engine.fault_reason))
                actual_feedback={}
                manufacturer_snapshot={}
                if factory_observer:
                    manufacturer_snapshot=factory_observer.snapshot(
                        require_hands=robot.profiles if hardware else False)
                    try:actual_feedback=serializable(factory_observer.read_state(
                        robot.read_state().estop if hardware else None))
                    except RuntimeError as exc:actual_feedback={'error':str(exc)}
                elif ros_session:
                    try:actual_feedback=serializable(ros_session.node.observed_robot.read_state())
                    except RuntimeError as exc:actual_feedback={'error':str(exc)}
                factory_status=None
                if admission:
                    factory_status=dict(available=True,state=robot.state,reason=robot.reason,
                        hardware_motion_enabled=robot.hardware_writes_enabled,
                        feedback_ready=manufacturer_snapshot.get('ready',False),
                        physical_estop=actual_feedback.get('estop'),
                        hand_sides=list(robot.profiles),driver_evidence=(dict(
                            controller=xarm_settings.controller,interface_reference='XARM algorithm manual sections 2.6, 4, 7')
                            if xarm else bundle.evidence))
                    if xarm:
                        factory_status['stop_confirmed']=robot.stop_confirmed
                        factory_status['xarm_status']=robot.status()
                tracking=gateway.tracking.status()
                if frame is not None:
                    receive_age=time.monotonic()-frame.received_at
                    timing.add('input_receive_wait',receive_age)
                    timing.add('input_source_budget',frame.source_age_s)
                    timing.add('input_total_age',receive_age+frame.source_age_s)
                timing.add('cycle_processing',time.monotonic()-now)
                gateway.runtime=dict(input_profile=input_policy.name,control=serializable(status),recording=None,
                    usb=usb_status,timing=timing.snapshot(),
                    headset_transport='robot_usb' if robot_usb else 'workstation',
                    runtime_mode='hardware' if hardware else 'dry_run',
                    ros_observation=observed.copy(),rosbag=bag_status.copy(),warnings=list(warnings),
                    actual_feedback=actual_feedback,manufacturer_feedback=serializable(manufacturer_snapshot),
                    manufacturer=factory_status,
                    motion=admission.status() if admission else dict(state='dry_run',hardware_motion_enabled=False),
                    motion_request_result=request_result,
                    readiness=readiness(tracking=tracking,native_ik=True,
                        ros_feedback=actual_feedback.get('quality')=='measured' and actual_feedback.get('mode')!='UNKNOWN',
                        headset_build=tracking.get('input_source') in ('xrobotoolkit','openxr'),
                        tools_calibrated=config['calibration'].get('tool_offsets_calibrated',False),
                        end_effector=end_effector,
                        manufacturer=factory_status))
                if hardware and robot.state=='disabled' and not admission.preparing and (
                        admission.preparation is None or admission.preparation.restored):
                    if not watchdog.deactivate():raise RuntimeError('watchdog_expired_restart_required')
                watchdog.beat()
                stop.wait(max(0.,.02-(time.monotonic()-now)))
        except (RuntimeError,ValueError,OSError) as exc:
            from .station_fault import latch_fault
            # Stop control but retain the authenticated status server so the
            # operator sees the initiating failure instead of endless retries.
            fault=latch_fault(gateway,controller,str(exc),details=getattr(engine,'last_diagnostics',{}))
            if admission:admission.request('disable',None)
            print(json.dumps(dict(event='station_control_fault',error=str(exc),
                restart_required=True,hardware_motion_enabled=False)),file=sys.stderr,flush=True)
            try:
                recorder.report('station_fault',fault)
            except (OSError,ValueError,TypeError):pass
            while not stop.wait(.02):
                # A fault revokes input immediately, but asynchronous OEM
                # release still needs progress while diagnostics stay alive.
                if xarm and robot.state=='releasing':
                    robot.tick_transition()
                    gateway.runtime['motion']=admission.status()
                    gateway.runtime['manufacturer_stop_confirmed']=robot.stop_confirmed
                if admission and admission.preparation:admission.tick(None)
        finally:
            controller.disarm('station_stopped')
            robot.disarm()
            # Finish hardware release before closing the remaining contexts in the
            # ExitStack. EOF must not strand an asynchronous OEM transition.
            if admission and admission.preparation:
                admission.request('disable',None)
                deadline=time.monotonic()+7.
                while time.monotonic()<deadline:
                    admission.tick(None)
                    if robot.state=='disabled' and admission.preparation.restored:break
                    time.sleep(.01)
    return 0


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--key-stdin',action='store_true')
    parser.add_argument('--port',type=int,default=8878)
    parser.add_argument('--config',type=Path,required=True)
    parser.add_argument('--record',type=Path,required=True)
    parser.add_argument('--ros',action='store_true')
    parser.add_argument('--headset-usb-local',action='store_true',help='PICO USB connects to robot Ubuntu; Windows is control/status only')
    parser.add_argument('--hardware',action='store_true',help='Requires a reviewed actual factory driver; starts disarmed')
    parser.add_argument('--xarm',action='store_true',help='Use installed documented XARM controllers; requires --hardware, starts disarmed')
    parser.add_argument('--hands-config',type=Path,help='Measured BrainCo calibration and raw limits for XARM; omitted means arms only')
    parser.add_argument('--factory-config',type=Path,help='Verified manufacturer bundle; not an enable/permission file')
    args=parser.parse_args(argv)
    try:return run(args)
    except (RuntimeError,ValueError,ImportError,OSError) as exc:
        print(json.dumps(dict(event='station_failed',error=str(exc),hardware_motion_enabled=False),
            ensure_ascii=False),file=sys.stderr,flush=True)
        return 2
    except KeyboardInterrupt:return 0


if __name__=='__main__':raise SystemExit(main())
