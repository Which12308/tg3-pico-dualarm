"""Preview, real-native offline IK, recording, and read-only robot inspection."""
import argparse
from contextlib import ExitStack
from dataclasses import replace
import importlib.util
import json
from pathlib import Path
import subprocess
import sys
import threading
import time
import numpy as np
from . import __version__
from .controller import Teleoperation
from .mapping import Calibration
from .model import DualArmModel,MODEL_SHA256,default_q
from .recording import SessionRecorder,serializable
from .robot import DryRunRobot
from .tracking import ControllerState,Pose,TrackingFrame,inspect_pico
from .safety import JointSafetyFilter,SafetyLimits
from .watchdog import CommandWatchdog


def native_engine(mode,configuration=None,*,joint_limits=None,velocity_limit=.24):
    from .kinematics import Kinematics
    options={}
    if configuration:
        calibration=configuration['calibration']
        options=dict(position_weight=calibration['position_weight'],orientation_weight=calibration['orientation_weight'],
                     posture_weight=calibration['posture_weight'],torso_waist_locked=configuration['robot']['torso_waist_locked'])
    return Kinematics(mode=mode,joint_limits=joint_limits,velocity_limit=velocity_limit,**options)


def demo(steps=240,mode='pose',record=None,preview=False):
    model=DualArmModel();clock=[1.]
    robot=DryRunRobot(default_q(),monotonic=lambda:clock[0])
    engine=None if preview else native_engine(mode)
    recorder=SessionRecorder(record,dict(version=__version__,mode=mode,source='synthetic_demo',
        robot_adapter='dry_run',model_sha256=MODEL_SHA256,physical_robot_commands_sent=False,
        native_placo_used=not preview,cartesian_preview_only=preview)) if record else None
    controller=Teleoperation(robot,model,engine,mode=mode,recorder=recorder,monotonic=lambda:clock[0])
    head=Pose((0.,1.6,0.),(0.,0.,0.,1.))
    result={};active_seen=dict(left=False,right=False)
    try:
        for i in range(steps):
            clock[0]+=.02
            # Small, reachable controller offsets. Both hands are first
            # released; only right begins, then both, then both release.
            left_grip=1. if 25<=i<steps-20 else 0.
            right_grip=1. if 5<=i<steps-20 else 0.
            forward=.012*min(max(i-30,0)/60,1)
            def hand(side,grip):
                return ControllerState(Pose((-.25 if side=='left' else .25,1.1,-.3-forward),
                    (0.,0.,0.,1.)),True,True,grip,0.,(),False)
            frame=TrackingFrame('offline_demo',i,clock[0],clock[0],head,hand('left',left_grip),hand('right',right_grip))
            result=controller.tick(frame,.02)
            if result.get('state')=='disarmed':
                raise RuntimeError('offline controller disarmed: '+result.get('reason','unknown'))
            for side,value in result.get('active',{}).items():active_seen[side]|=value
        if not all(active_seen.values()):raise RuntimeError('demo did not activate both arms')
        before=np.asarray(robot.read_state().joint_positions)
        invalid=replace(frame,sequence=steps,source_time_s=clock[0]+.02,received_at=clock[0]+.02,focused=False)
        clock[0]+=.02;controller.tick(invalid,.02)
        if not np.allclose(before,robot.read_state().joint_positions):raise RuntimeError('tracking loss moved dry-run joints')
        return dict(passed=True,native_placo_used=not preview,cartesian_preview_only=preview,
                    robot_commands_sent=False,simulated_commands=len(robot.commands),
                    both_arms_activated=active_seen,tracking_loss_hold_verified=True,
                    recording=str(recorder.directory) if recorder else None,last_status=result)
    finally:
        if recorder:recorder.close()


def serve(args):
    with ExitStack() as cleanup:
        from .web_input import InputServer
        server=InputServer(port=args.port)
        cleanup.callback(server.httpd.server_close)
        engine=None if args.preview else native_engine(args.mode,args.configuration)
        model=DualArmModel();ros_session=None
        if args.ros:
            from .ros_bridge import RosDryRunSession
            ros_session=RosDryRunSession()
            cleanup.callback(ros_session.close)
            robot=ros_session.robot
        else:robot=DryRunRobot(default_q())
        recorder=SessionRecorder(args.record,dict(version=__version__,mode=args.mode,source='live_webxr',
            robot_adapter='dry_run',model_sha256=MODEL_SHA256,physical_robot_commands_sent=False,
            native_placo_used=not args.preview,cartesian_preview_only=args.preview)) if args.record else None
        if recorder:cleanup.callback(recorder.close)
        controller=Teleoperation(robot,model,engine,mode=args.mode,translation_scale=args.scale,
                                 orientation_scale=args.orientation_scale,recorder=recorder,
                                 calibration_options={k:args.configuration['calibration'][k]
                                    for k in ('controller_to_hand','tcp_to_hand','tool_offsets_calibrated','tool_reference')
                                    if args.configuration and k in args.configuration['calibration']})
        watchdog_timeout=.15
        if args.configuration:
            safety=args.configuration['safety']
            controller.filter=JointSafetyFilter(model,SafetyLimits(
                joint_speed=safety['joint_speed_rad_s'],joint_acceleration=safety['joint_acceleration_rad_s2'],
                feedback_lead=safety['feedback_lead_rad'],translation_speed=safety['translation_speed_m_s'],
                rotation_speed=safety['rotation_speed_rad_s'],feedback_timeout=safety['feedback_timeout_s']))
            controller.workspace_extent=safety['workspace_extent_m']
            watchdog_timeout=safety['watchdog_timeout_s']
        watchdog=CommandWatchdog(robot.disarm,timeout=watchdog_timeout)
        thread=threading.Thread(target=server.serve_forever,daemon=True);thread.start()
        cleanup.callback(thread.join,timeout=3)
        cleanup.callback(server.shutdown)
        address=f'http://localhost:{server.server_address[1]}'
        print(json.dumps(dict(url=address,mode='cartesian_preview' if args.preview else 'native_placo_dry_run',
            robot_commands_sent=False,hardware_writes_enabled=False,
            headset_connection=f'adb reverse tcp:{args.port} tcp:{args.port}',
            recording=str(recorder.directory) if recorder else None),ensure_ascii=False),flush=True)
        previous=None;deadline=time.monotonic();last_tick=deadline-.02
        watchdog.start()
        cleanup.callback(watchdog.close)
        try:
            while True:
                if watchdog.check():
                    controller.disarm('command_watchdog_expired')
                    raise RuntimeError('command_watchdog_expired_restart_required')
                now=time.monotonic();dt=now-last_tick;last_tick=now
                status=controller.tick(server.tracking.sample(now),dt)
                if hasattr(server,'set_runtime_status'):server.set_runtime_status(serializable(status))
                watchdog.beat()
                current=(status.get('state'),str(status.get('active')),str(status.get('reason',status.get('reasons'))))
                if current!=previous:
                    print(json.dumps(serializable(status),ensure_ascii=False),flush=True);previous=current
                deadline+=.02
                if deadline<time.monotonic():deadline=time.monotonic()
                time.sleep(max(0,deadline-time.monotonic()))
        except KeyboardInterrupt:
            controller.disarm('operator_interrupt')
        return 0


def main(argv=None):
    actual_argv=list(sys.argv[1:] if argv is None else argv)
    if actual_argv[:1]==['record-ros']:
        from .rosbag_record import main as record_main
        return record_main(actual_argv[1:])
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--version',action='version',version=__version__)
    commands=p.add_subparsers(dest='command',required=True)
    live=commands.add_parser('serve',help='Live PICO input, preview or native IK dry-run; no hardware commands')
    live.add_argument('--port',type=int)
    live.add_argument('--mode',choices=('position','pose'))
    live.add_argument('--scale',type=float)
    live.add_argument('--orientation-scale',type=float)
    live.add_argument('--preview',action='store_true',help='No IK; show calibrated Cartesian targets only')
    live.add_argument('--ros',action='store_true',help='Publish this same dry-run control state through ROS diagnostics')
    live.add_argument('--record',type=Path)
    live.add_argument('--config',type=Path,help='Directory with vr/robot/calibration/safety YAML-subset JSON files')
    off=commands.add_parser('demo',help='Dual-arm synthetic input through real native IK or explicit preview')
    off.add_argument('--steps',type=int,default=240)
    off.add_argument('--mode',choices=('position','pose'),default='pose')
    off.add_argument('--preview',action='store_true')
    off.add_argument('--record',type=Path)
    commands.add_parser('selftest',help='Require native Placo, then run both-arm offline control')
    probe=commands.add_parser('probe-robot',help='Read-only ROS discovery and interface inspection')
    probe.add_argument('--seconds',type=float,default=10.)
    device=commands.add_parser('probe-pico',help='Read-only ADB model and software inspection')
    device.add_argument('--adb',default='adb');device.add_argument('--serial')
    bridge=commands.add_parser('bridge',help='ROS dry-run bridge; hardware mode refuses writes')
    bridge.add_argument('--mode',choices=('dry_run','hardware'),default='dry_run')
    ros_record=commands.add_parser('record-ros',help='Supervised rosbag2 recording of observed topics only')
    ros_record.add_argument('recorder_args',nargs=argparse.REMAINDER)
    args=p.parse_args(actual_argv)
    try:
        if args.command=='serve':
            args.configuration=None
            if args.config:
                from .configuration import load
                args.configuration=load(args.config)
            defaults=dict(port=8890,translation_scale=1.,orientation_scale=1.,mode='pose')
            if args.configuration:defaults.update(args.configuration['vr'])
            for target,key in (('port','port'),('scale','translation_scale'),('orientation_scale','orientation_scale'),('mode','mode')):
                if getattr(args,target) is None:setattr(args,target,defaults[key])
            if not .01<=args.scale<=1 or not 0<=args.orientation_scale<=1:
                raise ValueError('invalid mapping scales')
            return serve(args)
        if args.command in ('demo','selftest'):
            if args.command=='demo' and args.steps<80:p.error('--steps must be at least 80')
            result=demo(args.steps,args.mode,args.record,args.preview) if args.command=='demo' else demo()
            print(json.dumps(serializable(result),ensure_ascii=False,indent=2));return 0
        if args.command=='probe-pico':
            print(json.dumps(inspect_pico(args.adb,args.serial),ensure_ascii=False,indent=2));return 0
        if args.command=='probe-robot':
            from .preflight import main as probe_main
            return probe_main(['--seconds',str(args.seconds)])
        if args.command=='bridge':
            from .ros_bridge import main as bridge_main
            return bridge_main(['--mode',args.mode])
        if args.command=='record-ros':
            from .rosbag_record import main as record_main
            return record_main(args.recorder_args)
    except (RuntimeError,ValueError,OSError) as exc:
        print(json.dumps(dict(error=str(exc),robot_commands_sent=False),ensure_ascii=False),file=sys.stderr)
        return 2


if __name__=='__main__':raise SystemExit(main())
