"""One dual-arm control loop; the robot adapter is the sole output boundary."""
from . import safety as motion_policy
from dataclasses import asdict,replace
from copy import deepcopy
import math
import time
import numpy as np
from .mapping import Calibration,RelativeMapper,SIDES
from .robot import JointCommand
from .safety import JointSafetyFilter,SafetyError
from .hands import ContinuousHands
from .input_policy import NORMAL
from .input_validation import validate_frame


RETURN_SPEED_RAD_S = .12
RETURN_POSITION_TOLERANCE_RAD = .02


class Teleoperation:
    def __init__(self,robot,model,kinematics,*,mode='pose',calibration=None,
                 translation_scale=1.,orientation_scale=1.,calibration_options=None,
                 recorder=None,monotonic=time.monotonic,allow_hardware=False,policy=NORMAL,
                 watchdog_timeout_s=.15):
        self.robot,self.model,self.kinematics=robot,model,kinematics
        self.policy=policy
        if not math.isfinite(watchdog_timeout_s) or watchdog_timeout_s<=0 or (not motion_policy.LIMITS_MANAGED_EXTERNALLY and watchdog_timeout_s>.15):
            raise ValueError('invalid_controller_watchdog_timeout')
        self.watchdog_timeout_s=watchdog_timeout_s
        self.cycle_timing={}
        self._manual_stop=False
        self._output_generation=0
        self.home_joints=None
        self._home_move=None
        self._home_needs_release=False
        self.home_phase=None
        self._hardware=False
        if allow_hardware:
            from .manufacturer import ManufacturerAdapter
            from .xarm_adapter import XArmAdapter
            if not isinstance(robot,(ManufacturerAdapter,XArmAdapter)):raise ValueError('verified_manufacturer_adapter_required')
            self._hardware=True
        self.mode=mode;self.clock=monotonic;self.recorder=recorder
        self.workspace_extent=.20
        self.scale,self.rotation_scale=translation_scale,orientation_scale
        self.calibration_options=dict(calibration_options or {})
        if calibration is not None:
            self.scale,self.rotation_scale=calibration.translation_scale,calibration.orientation_scale
            self.calibration_options.update(controller_to_hand=calibration.controller_to_hand,
                tcp_to_hand=calibration.tcp_to_hand,tool_offsets_calibrated=calibration.tool_offsets_calibrated,
                tool_reference=calibration.tool_reference)
        self.mapper=RelativeMapper(calibration,mode,policy=policy) if calibration else None
        self._calibration_session=None
        self.filter=JointSafetyFilter(model)
        state=robot.read_state()
        self.target=np.array(state.joint_positions,dtype=float)
        self.velocity=np.zeros(14);self._active=dict.fromkeys(SIDES,False)
        self.last_status=dict(state='waiting_for_calibration',robot_commands_sent=False)
        self._last_disarm_reason=None
        self.hands=ContinuousHands(policy=policy)
        self.manufacturer_grasp=None
        if self._hardware and robot.profiles:
            from .manufacturer_grasp import ManufacturerGrasp
            self.manufacturer_grasp=ManufacturerGrasp(robot,clock=self.clock,policy=policy)

    def disarm(self,reason,details=None):
        self._manual_stop=True
        self._output_generation+=1
        self.home_joints=None
        self._home_move=None
        self._home_needs_release=False
        self.home_phase=None
        stop_result=self.robot.disarm(str(reason)) if self._hardware else None
        if motion_policy.LIMITS_MANAGED_EXTERNALLY and not self._hardware:self.robot.disarm()
        self.hands.invalidate()
        if self.manufacturer_grasp:self.manufacturer_grasp.invalidate()
        if self.mapper:self.mapper.invalidate()
        self._active=dict.fromkeys(SIDES,False)
        self.velocity[:]=0
        try:self.target=np.asarray(self.robot.read_state().joint_positions,dtype=float).copy()
        except RuntimeError:pass
        if self.kinematics:self.kinematics.reset()
        self.last_status=dict(state='disarmed',reason=str(reason),active=self._active.copy(),
                              robot_commands_sent=False)
        if details is not None:self.last_status['failure_details']=details
        if stop_result is not None:
            self.last_status['manufacturer_stop_acknowledged']=stop_result.accepted
            self.last_status['manufacturer_stop_reason']=stop_result.reason
        if self.recorder and self._last_disarm_reason!=str(reason) and not self.recorder.recording_failed:
            try:self.recorder.event(reason,self.clock(),details=details)
            except (OSError,ValueError,TypeError):self.recorder.recording_failed=True
        self._last_disarm_reason=str(reason)
        return self.last_status

    def reseed_after_hardware_enable(self):
        if not self._hardware or not self.robot.hardware_writes_enabled:
            raise RuntimeError('hardware_enable_ack_required_before_reseed')
        # ACK establishes an accepted target. Fresh feedback still supplies the
        # mapping and latched IK posture, without jumping this command history.
        self._manual_stop=False
        self._output_generation+=1
        target=self.robot.commanded_joint_positions
        if target is None:raise RuntimeError('accepted_manufacturer_joint_target_required')
        self.target=np.asarray(target,dtype=float).copy()
        state=self.robot.read_state()
        self.home_joints=np.asarray(state.joint_positions,dtype=float) if state.quality=='measured' else None
        self._home_move=None
        self._home_needs_release=False
        self.home_phase=None
        self.velocity[:]=0.;self._active=dict.fromkeys(SIDES,False)
        self.mapper=None;self._calibration_session=None
        self.kinematics.reset()
        verified=getattr(self.robot,'verified_velocity',None)
        if verified:
            self.filter=JointSafetyFilter(self.model,replace(self.filter.config,
                joint_speed=min(self.filter.config.joint_speed,min(verified))),
                boundary_hold=self.filter.boundary_hold)
        if self.manufacturer_grasp:self.manufacturer_grasp.initialize()

    def _home_input_ready(self,frame,session,now):
        try:validate_frame(frame,now,self.policy)
        except ValueError:return False
        return (frame.session==session and frame.focused and frame.head is not None
                and 0<=now-frame.received_at+frame.source_age_s<=self.policy.total_age_s
                and all(item.connected and item.tracked and item.neutral and not item.release_required
                        for item in (frame.left,frame.right)))

    def request_home(self,frame,session):
        if self._manual_stop or self.home_joints is None or not self.robot.hardware_writes_enabled:
            return dict(accepted=False,reason='home_requires_enabled_arm_and_measured_start')
        if self._home_move is not None:
            return dict(accepted=False,reason='already_returning_home')
        if not self._home_input_ready(frame,session,self.clock()):
            return dict(accepted=False,reason='home_requires_fresh_neutral_controllers')
        try:feedback=self.robot.read_state()
        except RuntimeError:return dict(accepted=False,reason='home_requires_measured_feedback')
        if feedback.quality!='measured':
            return dict(accepted=False,reason='home_requires_measured_feedback')
        start=np.asarray(self.robot.commanded_joint_positions,dtype=float).copy()
        peak_delta=float(np.max(np.abs(self.home_joints-start)))
        self._home_move=dict(start=start,duration=max(.5,1.875*peak_delta/RETURN_SPEED_RAD_S),
                             elapsed=0.,session=session)
        self.home_phase='returning'
        self._active=dict.fromkeys(SIDES,False)
        if self.mapper:self.mapper.invalidate()
        self.kinematics.reset()
        return dict(accepted=True,reason='return_home_started')

    def _tick_home(self,frame,state,now,dt,generation):
        move=self._home_move
        if move is None or state.quality!='measured' or not self._home_input_ready(frame,move['session'],now):
            if move is not None:
                self._home_move=None
                self._home_needs_release=True
            else:
                self._home_needs_release=not (frame and self._home_input_ready(frame,frame.session,now))
            if self._manual_stop or generation!=self._output_generation:return self.last_status
            result=self.robot.hold()
            if not result.accepted:return self._failure('return_home_hold_failed:'+result.reason)
            self.velocity[:]=0.
            self.home_phase='waiting_for_release' if self._home_needs_release else 'cancelled'
            self.last_status=dict(state='hardware',robot_commands_sent=result.accepted,
                active=self._active.copy(),home_available=self.home_joints is not None,
                return_home=self.home_phase,joint_positions=self.target.tolist(),
                reason='return_home_waiting_for_neutral')
            return self.last_status
        move['elapsed']+=min(dt,.02)
        fraction=min(1.,move['elapsed']/move['duration'])
        blend=fraction**3*(10+fraction*(-15+6*fraction))
        proposed=move['start']+blend*(self.home_joints-move['start'])
        try:
            interval=getattr(self.robot,'output_interval_s',None)
            output_dt=min(dt,interval) if interval is not None else dt
            target,velocity=self.filter.step(proposed,state.joint_positions,self.target,self.velocity,output_dt)
            if self._manual_stop or generation!=self._output_generation:return self.last_status
            command=JointCommand(target,velocity,now)
            result=self.robot.submit(command)
            if not result.accepted:raise RuntimeError(result.reason)
        except (RuntimeError,ValueError,SafetyError) as exc:
            self._home_move=None
            self._home_needs_release=True
            self.home_phase='failed'
            return self._failure('return_home_failed:'+str(exc))
        self.target,self.velocity=target,velocity
        arrived=(fraction==1. and np.max(np.abs(target-self.home_joints))<1e-5 and state.quality=='measured'
                 and np.max(np.abs(np.asarray(state.joint_positions)-self.home_joints))<=RETURN_POSITION_TOLERANCE_RAD)
        if arrived:
            self._home_move=None
            self.home_phase='arrived'
        self.last_status=dict(state='hardware',robot_commands_sent=True,active=self._active.copy(),
            joint_positions=target.tolist(),joint_velocities=velocity.tolist(),home_available=True,
            return_home=self.home_phase,return_fraction=fraction,
            cycle_timing=self.cycle_timing.copy())
        return self.last_status

    def _failure(self,reason,details=None):
        if not motion_policy.LIMITS_MANAGED_EXTERNALLY:return self.disarm(reason,details)
        self.last_status=dict(state='control_error',reason=str(reason),robot_commands_sent=False,limits_owner='external')
        if details is not None:self.last_status['failure_details']=details
        return self.last_status

    def tick(self,frame,dt=.02,*,record_task_space=False):
        if motion_policy.LIMITS_MANAGED_EXTERNALLY and self._manual_stop:return self.last_status
        generation=self._output_generation
        tick_started=time.perf_counter()
        now=self.clock()
        if type(dt) not in (float,int) or not math.isfinite(dt) or dt<=0 or (not motion_policy.LIMITS_MANAGED_EXTERNALLY and dt>self.watchdog_timeout_s):
            return self._failure('control_tick_overrun',details=dict(tick_interval_s=dt if isinstance(dt,(float,int)) and math.isfinite(dt) else None))
        # OS/audit scheduling jitter is not a trajectory integration interval.
        # Never catch up a delayed cycle with a larger robot increment. The
        # independent command watchdog still stops at its original deadline.
        observed_dt=dt
        dt=dt if motion_policy.LIMITS_MANAGED_EXTERNALLY else min(dt,.02)
        self.cycle_timing=dict(observed_interval_s=observed_dt,integration_interval_s=dt)
        try:state=self.robot.read_state()
        except RuntimeError as exc:
            if not motion_policy.LIMITS_MANAGED_EXTERNALLY:return self._failure('missing_joint_feedback')
            self.last_status=dict(state='input_error',reason=str(exc),robot_commands_sent=False)
            return self.last_status
        now=self.clock()
        # No unverified state can cross this boundary. The current live ROS
        # adapter is read-only and reports why actual dual-arm writes are gated.
        if self._hardware:
            if not self.robot.hardware_writes_enabled:return self._failure('explicit_hardware_enable_or_factory_ack_required')
            if (state.quality not in (('measured','stale') if motion_policy.LIMITS_MANAGED_EXTERNALLY else ('measured',))
                    or (not motion_policy.LIMITS_MANAGED_EXTERNALLY and state.estop is not False)):return self._failure('measured_feedback_and_known_physical_estop_required')
            if state.estop is True:return self.disarm('physical_estop')
            if self._home_move is not None or self._home_needs_release:
                return self._tick_home(frame,state,now,dt,generation)
            try:validate_frame(frame,now,self.policy)
            except ValueError as exc:
                if not motion_policy.LIMITS_MANAGED_EXTERNALLY:return self._failure('invalid_hardware_tracking_frame')
                self.last_status=dict(state='input_error',reason=str(exc),robot_commands_sent=False)
                return self.last_status
            if not motion_policy.LIMITS_MANAGED_EXTERNALLY and (not frame.focused or frame.head is None):return self._failure('hardware_tracking_lost_explicit_reenable_required')
        elif state.quality!='simulated':
            return self._failure('verified_dual_arm_hardware_adapter_required')
        measured=np.asarray(state.joint_positions,dtype=float)
        if state.estop is True:return self.disarm('physical_estop')
        if not motion_policy.LIMITS_MANAGED_EXTERNALLY and not 0<=now-state.timestamp<=self.filter.config.feedback_timeout:
            return self._failure('stale_joint_feedback')
        fresh_frame=frame is not None and (motion_policy.LIMITS_MANAGED_EXTERNALLY or (frame.focused and 0<=now-frame.received_at+frame.source_age_s<=self.policy.total_age_s))
        if fresh_frame:
            if self._calibration_session is not None and frame.session!=self._calibration_session:
                if not motion_policy.LIMITS_MANAGED_EXTERNALLY:self.disarm('tracking_reference_changed')
                self.mapper=None
                if self._hardware and not motion_policy.LIMITS_MANAGED_EXTERNALLY:return self.last_status
            self._calibration_session=frame.session
        if self.mapper is None:
            if frame is None or frame.head is None or (not motion_policy.LIMITS_MANAGED_EXTERNALLY and (not frame.focused or not 0<=now-frame.received_at+frame.source_age_s<=self.policy.total_age_s)):
                if motion_policy.LIMITS_MANAGED_EXTERNALLY:
                    self.last_status=dict(state='waiting_for_head_pose',robot_commands_sent=False)
                    return self.last_status
                return self._failure('waiting_for_fresh_head_pose')
            controllers=[getattr(frame,s) for s in SIDES]
            if not motion_policy.LIMITS_MANAGED_EXTERNALLY and any(not c.neutral for c in controllers):
                return self._failure('release_inputs_for_heading_calibration')
            try:
                calibration=Calibration.from_head(frame.head,translation_scale=self.scale,
                                                  orientation_scale=self.rotation_scale,**self.calibration_options)
            except ValueError as exc:
                if not motion_policy.LIMITS_MANAGED_EXTERNALLY:return self._failure(str(exc))
                self.last_status=dict(state='input_error',reason=str(exc),robot_commands_sent=False)
                return self.last_status
            self.mapper=RelativeMapper(calibration,self.mode,max_displacement=self.workspace_extent,policy=self.policy)
        current=self.model.fk(measured)
        mapping=self.mapper.update(frame,current,now)
        hand_status=(self.manufacturer_grasp.step(frame) if self.manufacturer_grasp else
                     dict(hardware_hand_commands_sent=False,reason='hand_output_not_configured',closure={}) if self._hardware
                     else self.hands.update(frame,now,dt))
        if hand_status.get('fault'):return self._failure('hand_rejected:'+hand_status['fault'])
        self._last_disarm_reason=None
        if self.kinematics is None:
            self.last_status=dict(state='cartesian_preview_only',active=mapping.active,
                                  reasons=mapping.reasons,targets={s:t.tolist() for s,t in mapping.targets.items()},
                                  robot_commands_sent=False,joint_commands_computed=False,
                                  calibration=self.mapper.calibration.as_dict())
            self.last_status['model_links']={side:{name:point.tolist() for name,point in self.model.arms[side].links(measured[i*7:(i+1)*7]).items()} for i,side in enumerate(SIDES)}
            self.last_status['hands']=hand_status
            if record_task_space:self.last_status['task_space']=self._task_space(current,mapping)
            self._record(now,frame,state,mapping,None,None)
            return self.last_status
        for side,offset in (('left',0),('right',7)):
            sl=slice(offset,offset+7)
            if mapping.relatch[side]:
                if not self._hardware:self.target[sl]=measured[sl]
                self.velocity[sl]=0
                self.kinematics.latch_posture(side,measured[sl])
            if not mapping.active[side] and self._active[side]:
                if not self._hardware:self.target[sl]=measured[sl]
                self.velocity[sl]=0
                self.kinematics.reset(side)
        try:
            if any(mapping.active.values()):
                proposed=self.kinematics.solve(self.target,mapping.targets,mapping.active,dt)
                if motion_policy.LIMITS_MANAGED_EXTERNALLY and (self._manual_stop or generation!=self._output_generation):return self.last_status
                output_interval=getattr(self.robot,'output_interval_s',None) if self._hardware else None
                filter_dt=min(dt,output_interval) if output_interval is not None else dt
                target,velocity=self.filter.step(proposed,measured,self.target,self.velocity,filter_dt)
                for side,pose in self.model.fk(target).items():
                    if mapping.active[side] and not self.mapper.within_workspace(side,pose):
                        raise SafetyError('braking_target_outside_clutch_workspace:'+side)
            else:target,velocity=self.target.copy(),np.zeros(14)
            command=JointCommand(tuple(map(float,target)),tuple(map(float,velocity)),now)
            if self._hardware and not any(mapping.active.values()):
                accepted=self.robot.hold()
                if accepted.accepted:
                    target=np.asarray(self.robot.commanded_joint_positions,dtype=float).copy()
                velocity=np.zeros(14)
                # Record the actual OEM hold target, not the pre-hold seed.
                command=JointCommand(tuple(map(float,target)),tuple(map(float,velocity)),now)
            else:accepted=self.robot.submit(command)
            if not accepted.accepted:return self._failure('robot_rejected:'+accepted.reason)
            self.target,self.velocity=target,velocity
            self._active=mapping.active.copy()
            if any(mapping.active.values()):self.home_phase=None
            self.last_status=dict(state='hardware' if self._hardware else 'dry_run',active=mapping.active,reasons=mapping.reasons,
                                  joint_positions=target.tolist(),joint_velocities=velocity.tolist(),
                                  robot_commands_sent=self._hardware,simulated_command_accepted=not self._hardware,
                                  home_available=self.home_joints is not None,return_home=self.home_phase,
                                  calibration=self.mapper.calibration.as_dict(),
                                  ik=getattr(self.kinematics,'last_diagnostics',{}))
            self.last_status['cycle_timing']=self.cycle_timing.copy()
            self.last_status['safety']=self.filter.last_diagnostics.copy()
            self.last_status['model_links']={side:{name:point.tolist() for name,point in self.model.arms[side].links(target[i*7:(i+1)*7]).items()} for i,side in enumerate(SIDES)}
            self.last_status['hands']=hand_status
            if record_task_space:self.last_status['task_space']=self._task_space(current,mapping)
            self._record(now,frame,state,mapping,command,accepted)
        except (RuntimeError,ValueError,SafetyError) as exc:
            if motion_policy.LIMITS_MANAGED_EXTERNALLY:
                self.last_status=dict(state='control_error',reason=str(exc),robot_commands_sent=False,
                    limits_owner='external',last_ik=deepcopy(getattr(self.kinematics,'last_diagnostics',{})))
                return self.last_status
            # Preserve native solver timing before reset/cleanup. This is
            # diagnostic evidence only; failed proposals are never submitted.
            return self._failure('control_failed:'+str(exc),details=dict(
                tick_wall_s=time.perf_counter()-tick_started,
                last_ik=deepcopy(getattr(self.kinematics,'last_diagnostics',{}))))
        return self.last_status

    def _task_space(self, measured_poses, mapping):
        offsets=self.mapper.calibration.tcp_to_hand
        return dict(frame=self.model.base_frame,
                    measured_tcp={side:pose.tolist() for side,pose in measured_poses.items()},
                    target_tcp={side:pose.tolist() for side,pose in mapping.targets.items()},
                    measured_grasp={side:(pose@offsets[side]).tolist() for side,pose in measured_poses.items()},
                    target_grasp={side:(pose@offsets[side]).tolist() for side,pose in mapping.targets.items()})

    def _record(self,now,frame,state,mapping,command,result):
        if self.recorder and getattr(self.recorder,'samples_enabled',True):
            try:
                self.recorder.append(dict(timestamp=now,tracking=frame,robot_state=state,
                    mapping=mapping,command=command,result=result,status=self.last_status,
                    calibration=self.mapper.calibration.as_dict()))
            except (OSError,ValueError,TypeError) as exc:
                self.recorder.recording_failed=True
                self.last_status['recording_error']=str(exc)
