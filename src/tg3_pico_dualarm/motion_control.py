"""Explicit hardware admission, separate from tracking/clutch and configuration."""
import time
import math
from .input_validation import validate_frame
from .input_policy import NORMAL
from .manufacturer import ManufacturerAdapter


class MotionAdmission:
    def __init__(self,robot,*,tools_calibrated=False,calibration_config=None,clock=time.monotonic,frame_provider=None,policy=NORMAL):
        from .xarm_adapter import XArmAdapter
        if not isinstance(robot,(ManufacturerAdapter,XArmAdapter)):raise TypeError('manufacturer_adapter_required')
        self.robot,self.clock=robot,clock
        self.policy=policy
        self.tools_calibrated=tools_calibrated is True
        from .tool_setup import reference_status
        self.reference=(reference_status(calibration_config) if calibration_config is not None else None)
        self.session=None
        self.reason='explicit_operator_enable_required'
        self.just_enabled=False
        self.preparation=getattr(robot,'preparation',None)
        self.preparing=False;self.initial_hold_pending=False
        self.hold_started=None;self.stable_since=None
        self.last_failure=None
        self.input_timing=None
        self.frame_provider=frame_provider
        self.validated_frame=None;self.check_phase='idle'
        self.hold_diagnostics=None
        self.hold_target=None
        self.attempt_id=0;self.attempt_started_at=None;self.output_baseline=None

    def current_alignment(self):
        transport=getattr(self.robot,'transport',None)
        alignment=transport.alignment_status() if hasattr(transport,'alignment_status') else None
        if not alignment or self.attempt_started_at is None:return None
        started=alignment.get('started_at')
        return alignment if type(started) in (int,float) and started>=self.attempt_started_at else None

    def attempt_output_count(self):
        count=getattr(getattr(self.robot,'transport',None),'actual_count',None)
        if type(count) is int and type(self.output_baseline) is int:return max(0,count-self.output_baseline)
        return None

    def _remember_failure(self,reason):
        self.last_failure=dict(reason=str(reason),received_at=self.clock(),
            preparation=self.preparation.status() if self.preparation else None,
            robot_state=self.robot.state,input_timing=self.input_timing,stage=self.check_phase,
            attempt_id=self.attempt_id,actual_output_count=self.attempt_output_count())
        if str(reason).startswith('initial_hold'):
            self.last_failure['hold_diagnostics']=self.hold_diagnostics
        transport=getattr(self.robot,'transport',None)
        if hasattr(transport,'alignment_status'):
            self.last_failure['initial_output_alignment']=self.current_alignment()

    def _input(self,frame,neutral=False):
        # Fetch at the actual validation boundary, after transition processing.
        # Never renew receipt timestamps, fall back to an older valid pose,
        # or bypass the normal freshness/session/neutral checks.
        if self.frame_provider is not None:frame=self.frame_provider()
        now=self.clock();received=getattr(frame,'received_at',None);age=getattr(frame,'source_age_s',None)
        self.input_timing=None
        if all(type(v) in (int,float) and math.isfinite(v) for v in (now,received,age)):
            self.input_timing=dict(now_s=now,received_at=received,received_age_s=now-received,
                source_age_s=age,total_age_s=now-received+age)
        validate_frame(frame,now,self.policy)
        if not frame.focused or frame.head is None:raise ValueError('fresh_focused_head_tracking_required')
        if frame.scene_visible is not True:raise ValueError('passthrough_observation_required_for_motion')
        if neutral and any(not c.connected or not c.tracked or c.release_required or not c.neutral
                           for c in (frame.left,frame.right)):
            raise ValueError('both_controllers_tracked_and_released_required')
        self.validated_frame=frame
        return frame

    def request(self,operation,frame,*,expected_session=None):
        if operation=='disable':
            self.just_enabled=False
            self.preparing=False;self.initial_hold_pending=False
            if self.preparation:self.preparation.cancel()
            result=self.robot.disarm('explicit_operator_disabled');self.session=None;self.reason=result.reason
            return dict(accepted=result.accepted,reason=result.reason)
        if operation!='enable':raise ValueError('unknown_motion_operation')
        # A duplicate click must neither erase an unconsumed factory ACK nor
        # replace the admitted tracking session while control is already held.
        if self.robot.state!='disabled' or self.preparing or (self.preparation and not self.preparation.restored):
            return dict(accepted=False,pending=self.robot.state=='acquiring',reason='disable_before_rearming')
        self.attempt_id+=1;self.attempt_started_at=self.clock()
        self.output_baseline=getattr(getattr(self.robot,'transport',None),'actual_count',None)
        self.hold_diagnostics=None
        self.last_failure=None
        self.validated_frame=None;self.check_phase='enable_input'
        try:
            fault=getattr(getattr(self.robot,'transport',None),'output_fault',None)
            if fault:
                self.check_phase='enable_factory_output_latch'
                raise ValueError('factory_output_fault_latched_restart_required:'+str(fault))
            if not (self.reference['ready'] if self.reference is not None else self.tools_calibrated):
                raise ValueError('valid_end_effector_reference_required')
            frame=self._input(frame,neutral=True)
            if frame.session!=expected_session:raise ValueError('input_session_changed_before_enable')
            if self.reference is not None and self.robot.profiles:
                snapshot=self.robot.feedback.snapshot(require_hands=tuple(self.robot.profiles))
                if not snapshot['ready']:raise ValueError('required_feedback_not_ready:'+','.join(snapshot['issues']))
            measured=self.robot.read_state().joint_positions
            if any(not low<=value<=high for value,low,high in zip(measured,self.robot.limits.lower,self.robot.limits.upper)):
                raise ValueError('measured_arm_outside_verified_manufacturer_limits')
            self.session=frame.session
            if self.preparation:
                self.preparation.begin();self.preparing=True;self.reason='factory_preparation_pending'
                return dict(accepted=False,pending=True,reason=self.reason)
            result=self.robot.request_enable(True)
            if result.accepted:self.just_enabled=True
            self.reason=result.reason
            return dict(accepted=result.accepted,pending=self.robot.state=='acquiring',reason=result.reason)
        except (RuntimeError,ValueError) as exc:
            self._remember_failure(exc)
            self.session=None;self.reason=str(exc)
            return dict(accepted=False,reason=self.reason)

    def tick(self,frame):
        from .xarm_adapter import XArmAdapter
        self.validated_frame=None
        # Acquisition completion must pass poll_enable below, so its ACK is
        # consumed exactly once and reseeds the robot target before any output.
        if isinstance(self.robot,XArmAdapter) and self.robot.state!='acquiring':self.robot.tick_transition()
        if self.preparing:
            try:
                self.check_phase='preparation_input'
                frame=self._input(frame,neutral=True)
                if frame.session!=self.session:raise ValueError('tracking_session_changed_during_preparation')
                if self.preparation.step():
                    self.preparing=False
                    result=self.robot.request_enable(True);self.reason=result.reason
                    if result.accepted:self._begin_hold()
            except (RuntimeError,ValueError) as exc:
                self._remember_failure(exc)
                self.request('disable',None);self.reason=str(exc)
            return self.status()
        if self.preparation and self.robot.state not in ('acquiring','armed','holding'):
            self.preparation.cancel()
            try:
                safe=self.robot.stop_confirmed or (self.robot.state=='disabled' and not self.robot._enabled)
                self.preparation.restore_step(safe_to_restore=safe)
            except RuntimeError as exc:
                self.reason=str(exc);return self.status()
        if self.robot.state not in ('acquiring','armed','holding'):
            self.reason=self.robot.reason;self.session=None;self.just_enabled=False
            return self.status()
        try:
            self.check_phase='acquiring_input' if self.robot.state=='acquiring' else 'running_input'
            frame=self._input(frame,neutral=self.robot.state=='acquiring' or self.initial_hold_pending)
            if frame.session!=self.session:raise ValueError('tracking_session_changed_explicit_reenable_required')
            if self.robot.state=='acquiring':
                result=self.robot.poll_enable();self.reason=result.reason
                if result.accepted:
                    if self.preparation:self._begin_hold()
                    else:self.just_enabled=True
                elif self.robot.state!='acquiring':self._remember_failure(result.reason)
            elif self.initial_hold_pending:self._settle_hold()
            elif self.robot.check_watchdog():self.reason=self.robot.reason
        except (RuntimeError,ValueError) as exc:
            self._remember_failure(exc)
            self.request('disable',None);self.reason=str(exc)
        return self.status()

    def _begin_hold(self):
        self.initial_hold_pending=True;self.hold_started=self.clock();self.stable_since=None
        self.hold_diagnostics=None
        self.hold_target=tuple(self.robot.commanded_joint_positions)
        self.reason='measured_initial_hold_pending'

    def _settle_hold(self):
        now=self.clock();state=self.robot.read_state();measured=state.joint_positions
        target=self.robot.commanded_joint_positions
        if self.hold_target is None or max(abs(a-b) for a,b in zip(target,self.hold_target))>1e-9:
            raise RuntimeError('initial_hold_target_changed')
        error=max(abs(a-b) for a,b in zip(measured,target))
        hybrid=bool(getattr(getattr(self.robot,'transport',None),'holding_profile',None))
        self.hold_diagnostics=dict(elapsed_s=now-self.hold_started,
            maximum_error_rad=error,limit_rad=min(self.robot.limits.feedback_lead),
            measured_positions=list(measured),accepted_target=list(target),
            differences_rad=[a-b for a,b in zip(measured,target)],
            motor_ids=list(range(11,18))+list(range(21,28)),
            criterion='500ms_frozen_target_hybrid_handoff' if hybrid else 'position_error_below_0.004rad',
            within_0_004rad_target_accuracy=error<.004,
            note='Bounded in-memory diagnostics; frozen target; absolute feedback lead protection unchanged.')
        if error>min(self.robot.limits.feedback_lead):
            raise RuntimeError('initial_hold_feedback_corridor_exceeded')
        result=self.robot.hold()
        if not result.accepted:raise RuntimeError('initial_hold_rejected:'+result.reason)
        if hybrid:
            # Original remote-style handoff: retain the frozen measured seed
            # for 0.5s, then admit the clutch. Absolute feedback lead, native
            # raw alignment, measured feedback, E-stop and watchdogs remain.
            # A compliant PD equilibrium need not exactly equal its target.
            if now-self.hold_started>=.5:
                self.initial_hold_pending=False;self.just_enabled=True;self.reason='hybrid_frozen_target_handoff_complete'
        elif error<.004:
            if self.stable_since is None:self.stable_since=now
            if now-self.stable_since>=.3:
                self.initial_hold_pending=False;self.just_enabled=True;self.reason='initial_hold_measured_stable'
        else:self.stable_since=None
        if self.initial_hold_pending and now-self.hold_started>3.:raise RuntimeError('initial_hold_did_not_settle')

    def consume_enable_ack(self):
        """Each observed acquisition ACK reseeds the controller exactly once."""
        pending=self.just_enabled
        self.just_enabled=False
        return pending

    def status(self):
        return dict(state='preparing' if self.preparing else 'settling' if self.initial_hold_pending else self.robot.state,
                    attempt_id=self.attempt_id,attempt_started_at=self.attempt_started_at,
                    actual_output_count=self.attempt_output_count(),initial_output_alignment=self.current_alignment(),
                    hardware_motion_enabled=self.robot.hardware_writes_enabled,
                    initial_hold_pending=self.initial_hold_pending,reason=self.reason,session=self.session,
                    initial_hold_diagnostics=self.hold_diagnostics,
                    end_effector_reference=self.reference,
                    last_failure=self.last_failure,
                    just_enabled=self.just_enabled,preparation=self.preparation.status() if self.preparation else None)
