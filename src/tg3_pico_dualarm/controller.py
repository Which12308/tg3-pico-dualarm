"""One dual-arm control loop; the robot adapter is the sole output boundary."""
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


class Teleoperation:
    def __init__(self,robot,model,kinematics,*,mode='pose',calibration=None,
                 translation_scale=1.,orientation_scale=1.,calibration_options=None,
                 recorder=None,monotonic=time.monotonic,allow_hardware=False,policy=NORMAL,
                 watchdog_timeout_s=.15):
        self.robot,self.model,self.kinematics=robot,model,kinematics
        self.policy=policy
        if not math.isfinite(watchdog_timeout_s) or not 0<watchdog_timeout_s<=.15:
            raise ValueError('invalid_controller_watchdog_timeout')
        self.watchdog_timeout_s=watchdog_timeout_s
        self.cycle_timing={}
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
        stop_result=self.robot.disarm(str(reason)) if self._hardware else None
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
        target=self.robot.commanded_joint_positions
        if target is None:raise RuntimeError('accepted_manufacturer_joint_target_required')
        self.target=np.asarray(target,dtype=float).copy()
        self.velocity[:]=0.;self._active=dict.fromkeys(SIDES,False)
        self.mapper=None;self._calibration_session=None
        self.kinematics.reset()
        verified=getattr(self.robot,'verified_velocity',None)
        if verified:
            self.filter=JointSafetyFilter(self.model,replace(self.filter.config,
                joint_speed=min(self.filter.config.joint_speed,min(verified))),
                boundary_hold=self.filter.boundary_hold)
        if self.manufacturer_grasp:self.manufacturer_grasp.initialize()

    def tick(self,frame,dt=.02):
        tick_started=time.perf_counter()
        now=self.clock()
        if type(dt) not in (float,int) or not math.isfinite(dt) or not 0<dt<=self.watchdog_timeout_s:
            return self.disarm('control_tick_overrun',details=dict(tick_interval_s=dt if isinstance(dt,(float,int)) and math.isfinite(dt) else None))
        # OS/audit scheduling jitter is not a trajectory integration interval.
        # Never catch up a delayed cycle with a larger robot increment. The
        # independent command watchdog still stops at its original deadline.
        observed_dt=dt
        dt=min(dt,.02)
        self.cycle_timing=dict(observed_interval_s=observed_dt,integration_interval_s=dt)
        try:state=self.robot.read_state()
        except RuntimeError:return self.disarm('missing_joint_feedback')
        now=self.clock()
        # No unverified state can cross this boundary. The current live ROS
        # adapter is read-only and reports why actual dual-arm writes are gated.
        if self._hardware:
            if not self.robot.hardware_writes_enabled:return self.disarm('explicit_hardware_enable_or_factory_ack_required')
            if state.quality!='measured' or state.estop is not False:return self.disarm('measured_feedback_and_known_physical_estop_required')
            from .input_validation import validate_frame
            try:validate_frame(frame,now,self.policy)
            except ValueError:return self.disarm('invalid_hardware_tracking_frame')
            if not frame.focused or frame.head is None:return self.disarm('hardware_tracking_lost_explicit_reenable_required')
        elif state.quality!='simulated':
            return self.disarm('verified_dual_arm_hardware_adapter_required')
        measured=np.asarray(state.joint_positions,dtype=float)
        if state.estop is True:return self.disarm('physical_estop')
        if not 0<=now-state.timestamp<=self.filter.config.feedback_timeout:
            return self.disarm('stale_joint_feedback')
        fresh_frame=frame is not None and frame.focused and 0<=now-frame.received_at+frame.source_age_s<=self.policy.total_age_s
        if fresh_frame:
            if self._calibration_session is not None and frame.session!=self._calibration_session:
                self.disarm('tracking_reference_changed')
                self.mapper=None
                if self._hardware:return self.last_status
            self._calibration_session=frame.session
        if self.mapper is None:
            if frame is None or not frame.focused or frame.head is None or not 0<=now-frame.received_at+frame.source_age_s<=self.policy.total_age_s:
                return self.disarm('waiting_for_fresh_head_pose')
            controllers=[getattr(frame,s) for s in SIDES]
            if any(not c.neutral for c in controllers):
                return self.disarm('release_inputs_for_heading_calibration')
            try:
                calibration=Calibration.from_head(frame.head,translation_scale=self.scale,
                                                  orientation_scale=self.rotation_scale,**self.calibration_options)
            except ValueError as exc:return self.disarm(str(exc))
            self.mapper=RelativeMapper(calibration,self.mode,max_displacement=self.workspace_extent,policy=self.policy)
        current=self.model.fk(measured)
        mapping=self.mapper.update(frame,current,now)
        hand_status=(self.manufacturer_grasp.step(frame) if self.manufacturer_grasp else
                     dict(hardware_hand_commands_sent=False,reason='hand_output_not_configured',closure={}) if self._hardware
                     else self.hands.update(frame,now,dt))
        if hand_status.get('fault'):return self.disarm('hand_rejected:'+hand_status['fault'])
        self._last_disarm_reason=None
        if self.kinematics is None:
            self.last_status=dict(state='cartesian_preview_only',active=mapping.active,
                                  reasons=mapping.reasons,targets={s:t.tolist() for s,t in mapping.targets.items()},
                                  robot_commands_sent=False,joint_commands_computed=False,
                                  calibration=self.mapper.calibration.as_dict())
            self.last_status['model_links']={side:{name:point.tolist() for name,point in self.model.arms[side].links(measured[i*7:(i+1)*7]).items()} for i,side in enumerate(SIDES)}
            self.last_status['hands']=hand_status
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
            if not accepted.accepted:return self.disarm('robot_rejected:'+accepted.reason)
            self.target,self.velocity=target,velocity
            self._active=mapping.active.copy()
            self.last_status=dict(state='hardware' if self._hardware else 'dry_run',active=mapping.active,reasons=mapping.reasons,
                                  joint_positions=target.tolist(),joint_velocities=velocity.tolist(),
                                  robot_commands_sent=self._hardware,simulated_command_accepted=not self._hardware,
                                  calibration=self.mapper.calibration.as_dict(),
                                  ik=getattr(self.kinematics,'last_diagnostics',{}))
            self.last_status['cycle_timing']=self.cycle_timing.copy()
            self.last_status['safety']=self.filter.last_diagnostics.copy()
            self.last_status['model_links']={side:{name:point.tolist() for name,point in self.model.arms[side].links(target[i*7:(i+1)*7]).items()} for i,side in enumerate(SIDES)}
            self.last_status['hands']=hand_status
            self._record(now,frame,state,mapping,command,accepted)
        except (RuntimeError,ValueError,SafetyError) as exc:
            # Preserve native solver timing before reset/cleanup. This is
            # diagnostic evidence only; failed proposals are never submitted.
            return self.disarm('control_failed:'+str(exc),details=dict(
                tick_wall_s=time.perf_counter()-tick_started,
                last_ik=deepcopy(getattr(self.kinematics,'last_diagnostics',{}))))
        return self.last_status

    def _record(self,now,frame,state,mapping,command,result):
        if self.recorder and getattr(self.recorder,'samples_enabled',True):
            try:
                self.recorder.append(dict(timestamp=now,tracking=frame,robot_state=state,
                    mapping=mapping,command=command,result=result,status=self.last_status,
                    calibration=self.mapper.calibration.as_dict()))
            except (OSError,ValueError,TypeError) as exc:
                self.recorder.recording_failed=True
                self.last_status['recording_error']=str(exc)
