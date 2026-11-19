"""Private native-ROS manufacturer session and explicitly reviewed driver loader.

A bundle selects a local reviewed Python provider plus explicit calibration;
it is not a permission file. Runtime factory acknowledgements and explicit
operator enable are still required by ManufacturerAdapter.
"""
from dataclasses import dataclass, fields, replace
import hashlib
import inspect
import json
import keyword
import math
from pathlib import Path
import re
import sys
import threading
import time
from types import ModuleType
import uuid

from .configuration import read_json_yaml
from .hands import HandCalibration
from .manufacturer import ManufacturerAdapter,ManufacturerLimits,ROSFactoryTransport,hand_schema
from .manufacturer_contract import FactoryControlDriver,FactoryStatus
from .manufacturer_feedback import ManufacturerFeedback
from .model import DualArmModel
from .robot import ReadOnlyRobot
from .tiangong_protocol import ArmCommandFormatter

EVIDENCE_KEYS=frozenset(('controller','version','interface_reference'))
MOTION_TOPICS=frozenset(('/arm/cmd','/left_hand/set_motor_multi','/right_hand/set_motor_multi',
    '/head/cmd','/waist/cmd','/hric/robot/cmd_vel','/hric/robot/stand_cmd',
    '/hric/robot/fsm_state_cmd','/hric/robot/fsm_resume_cmd'))


def _evidence(value):
    if not isinstance(value,dict) or set(value)!=EVIDENCE_KEYS or any(not isinstance(v,str) or not v.strip() for v in value.values()):
        raise ValueError('manufacturer_driver_requires_controller_version_and_interface_reference')
    return value.copy()


def _reject_driver(driver,reason):
    closer=getattr(driver,'close',None)
    if callable(closer):
        try:closer()
        except Exception as exc:raise RuntimeError(reason+'; rejected_driver_cleanup_failed:'+str(exc)) from exc
    raise RuntimeError(reason)


def _native_output_schemas(profiles):
    """Check installed command bindings without making a ROS node/publisher."""
    from rclpy.type_support import check_for_type_support
    from ros2_bridge_msgs.msg import ArmCtrl,MotorCtrl
    for cls in (ArmCtrl,MotorCtrl):check_for_type_support(cls)
    ArmCommandFormatter(ArmCtrl,MotorCtrl)
    prototype=ArmCtrl()
    if type(prototype.reserved) is not int or prototype.reserved!=0:
        raise ValueError('unverified_nonzero_reserved_default')
    if profiles:
        from brainco_hand_msgs.msg import SetMotorMulti
        check_for_type_support(SetMotorMulti);hand_schema(SetMotorMulti)


@dataclass(frozen=True)
class FactoryBundle:
    source_path: Path
    source_sha256: str
    factory_name: str
    evidence: dict
    limits: ManufacturerLimits
    profiles: dict

    def driver_factory(self,node):
        """Execute only the exact reviewed file; never extend sys.path.

        This is trusted local plugin code, not a hostile-code isolation layer.
        Its constructor must create no motion publishers or send any command.
        """
        data=self.source_path.read_bytes()
        if hashlib.sha256(data).hexdigest()!=self.source_sha256:
            raise RuntimeError('reviewed_factory_driver_changed_after_bundle_validation')
        identity=hashlib.sha256(str(self.source_path).encode()).hexdigest()[:12]
        name='_tg3_reviewed_factory_'+self.source_sha256+'_'+identity
        module=sys.modules.get(name)
        if module is None:
            module=ModuleType(name);module.__file__=str(self.source_path);module.__package__=''
            sys.modules[name]=module
            try:exec(compile(data,str(self.source_path),'exec'),module.__dict__)
            except BaseException:
                if sys.modules.get(name) is module:del sys.modules[name]
                raise
        factory=getattr(module,self.factory_name,None)
        if not inspect.isfunction(factory) or factory.__module__!=name:
            raise RuntimeError('reviewed_file_must_define_the_requested_factory_function')
        driver=factory(node)
        if not isinstance(driver,FactoryControlDriver):
            _reject_driver(driver,'factory_did_not_return_FactoryControlDriver')
        actual=driver.evidence
        if not isinstance(actual,dict) or any(actual.get(k)!=v for k,v in self.evidence.items()):
            _reject_driver(driver,'runtime_factory_driver_evidence_differs_from_reviewed_bundle')
        return driver

    def create_session(self,**kwargs):
        return FactoryRosSession(driver_factory=self.driver_factory,limits=self.limits,
            profiles=self.profiles,**kwargs)


def load_factory_bundle(path):
    """Validate an explicit JSON descriptor; loading it does not execute code.

    Required schema:
      schema = tg3_factory_bundle_v1
      driver = {path, sha256, factory, evidence}
      limits = explicit ManufacturerLimits fields (no default gains/bounds)
      hands = {left/right: complete HandCalibration fields}; may be empty.
    """
    descriptor=Path(path).expanduser().resolve(strict=True)
    value=read_json_yaml(descriptor)
    if not isinstance(value,dict) or set(value)!={'schema','driver','limits','hands'} or value['schema']!='tg3_factory_bundle_v1':
        raise ValueError('unsupported_factory_bundle_schema_or_fields')
    driver=value['driver']
    if not isinstance(driver,dict) or set(driver)!={'path','sha256','factory','evidence'}:
        raise ValueError('explicit_reviewed_factory_driver_file_hash_and_function_required')
    if (not isinstance(driver['path'],str) or not driver['path'] or '\x00' in driver['path']
        or re.match(r'^[A-Za-z][A-Za-z0-9+.-]*://',driver['path'])):
        raise ValueError('invalid_local_factory_driver_path')
    source=Path(driver['path']).expanduser()
    if not source.is_absolute():source=descriptor.parent/source
    source=source.resolve(strict=True)
    if not source.is_file() or source.suffix.lower()!='.py':raise ValueError('local_reviewed_python_file_required')
    digest=driver['sha256']
    if not isinstance(digest,str) or not re.fullmatch('[a-fA-F0-9]{64}',digest):raise ValueError('factory_driver_sha256_required')
    digest=digest.lower()
    if hashlib.sha256(source.read_bytes()).hexdigest()!=digest:raise ValueError('factory_driver_sha256_mismatch')
    factory=driver['factory']
    if not isinstance(factory,str) or not factory.isidentifier() or keyword.iskeyword(factory) or len(factory)>128:
        raise ValueError('simple_local_factory_function_name_required')
    evidence=_evidence(driver['evidence'])
    limits=value['limits']
    required={'kp','kd','lower','upper','velocity','feedback_lead','hand_raw_rate','hand_raw_lead','evidence'}
    allowed={f.name for f in fields(ManufacturerLimits)}
    if not isinstance(limits,dict) or not required<=limits.keys() or limits.keys()-allowed:
        raise ValueError('complete_explicit_manufacturer_limits_required')
    limits=ManufacturerLimits(**limits)
    # A local descriptor may tighten the actual supplied model, never widen it.
    model=DualArmModel()
    if any(low<actual[0]-1e-12 or high>actual[1]+1e-12 for low,high,actual in zip(limits.lower,limits.upper,model.limits)):
        raise ValueError('factory_bundle_limits_exceed_actual_model_soft_limits')
    hands=value['hands']
    if not isinstance(hands,dict) or not hands.keys()<={'left','right'}:raise ValueError('invalid_factory_hand_sides')
    hand_fields={f.name for f in fields(HandCalibration)}
    profiles={}
    for side,profile in hands.items():
        if not isinstance(profile,dict) or set(profile)!=hand_fields:raise ValueError('complete_measured_hand_profile_required:'+side)
        profiles[side]=HandCalibration(**profile)
        if side not in limits.hand_raw_rate or side not in limits.hand_raw_lead:
            raise ValueError('explicit_hand_rate_and_feedback_lead_required:'+side)
    return FactoryBundle(source,digest,factory,evidence,limits,profiles)


class _SessionDriver(FactoryControlDriver):
    """Latch ROS executor failure across later re-enable attempts.

    Cleanup is allowed to call the real driver's stop/release even after ROS
    failure; acquiring or using control cannot continue on a failed session.
    """
    def __init__(self,session):self.session=session
    @property
    def evidence(self):return self.session.driver.evidence
    def request_dual_arm_control(self,hand_sides=()):
        self.session._check_running();return self.session.driver.request_dual_arm_control(hand_sides)
    def read_status(self):
        self.session._check_running()
        status=self.session.driver.read_status()
        if not isinstance(status,FactoryStatus):raise RuntimeError('invalid_factory_status')
        power=self.session.feedback.snapshot(require_hands=False)
        # SDK 26.06.x documents both physical E-stop booleans. The provider
        # need not invent a separate E-stop API, and cannot override this input.
        estop=power['physical_estop']
        if status.physical_estop is True:estop=True
        return replace(status,physical_estop=estop,estop_observed_at=power['physical_estop_stamp'],
                       estop_source=power['physical_estop_source'])
    def hold(self,positions):
        self.session._check_running();return self.session.driver.hold(positions)
    def stop_and_restore(self,reason):return self.session.driver.stop_and_restore(reason)
    def release_control(self):return self.session.driver.release_control()


class FactoryRosSession:
    """ROS Context, node, actual feedback, driver and executor lifecycle.

    Construction is disarmed and requests no control. Hardware construction
    requires a real factory driver factory; observe_only starts only live read
    subscriptions and exposes ReadOnlyRobot. Other ROS contexts are untouched.
    """
    def __init__(self,*,driver_factory=None,limits=None,profiles=None,observe_only=False,
                 wait_for_feedback_s=5.,monotonic=time.monotonic):
        if type(observe_only) is not bool:raise TypeError('observe_only_must_be_boolean')
        if not observe_only and (not callable(driver_factory) or not isinstance(limits,ManufacturerLimits)):
            raise RuntimeError('reviewed_factory_driver_and_explicit_limits_required_before_ROS_creation')
        if observe_only and driver_factory is not None:raise ValueError('readonly_session_must_not_execute_factory_driver')
        if type(wait_for_feedback_s) not in (int,float) or not math.isfinite(wait_for_feedback_s) or not 0<=wait_for_feedback_s<=30:
            raise ValueError('feedback_wait_must_be_in_0_to_30_seconds')
        self.context=self.node=self.executor=self.driver=self.robot=self._thread=None
        self.feedback=ManufacturerFeedback(clock=monotonic)
        self.profiles=dict(profiles or {});self.clock=monotonic;self.observe_only=observe_only
        self._closing=threading.Event();self._failed=threading.Event();self._started=threading.Event()
        self._closed=False;self._error=None;self._close_lock=threading.RLock();self.cleanup_errors=[]
        try:
            try:
                import rclpy
                from rclpy.context import Context
                from rclpy.executors import SingleThreadedExecutor
                from rclpy.node import Node
            except ImportError as exc:raise RuntimeError('native_ROS_2_unavailable_source_Jazzy_environment') from exc
            if not observe_only:_native_output_schemas(self.profiles)
            self.context=Context();rclpy.init(args=[],context=self.context)
            self.node=Node('tg3_factory_session_'+uuid.uuid4().hex[:8],context=self.context,
                enable_rosout=False,start_parameter_services=False,use_global_arguments=False)
            self.feedback.subscribe(self.node)
            self.executor=SingleThreadedExecutor(context=self.context);self.executor.add_node(self.node)
            self._thread=threading.Thread(target=self._spin,name='tg3-factory-ros',daemon=True);self._thread.start()
            if not self._started.wait(1.):raise RuntimeError('manufacturer_ROS_executor_did_not_start')
            if observe_only:self.robot=ReadOnlyRobot(self.feedback.read_state)
            else:
                self.driver=driver_factory(self.node)
                if not isinstance(self.driver,FactoryControlDriver):raise RuntimeError('factory_did_not_return_FactoryControlDriver')
                self._assert_no_initial_motion_publishers()
                self.robot=ManufacturerAdapter(self.feedback,
                    lambda:ROSFactoryTransport(self.node,self.profiles,limits,clock=monotonic),
                    limits,self.profiles,driver=_SessionDriver(self),clock=monotonic)
            self._check_running()
            if wait_for_feedback_s:self.wait_for_feedback(wait_for_feedback_s)
        except BaseException as exc:
            try:self.close()
            except Exception as cleanup:raise RuntimeError(str(exc)+'; session_cleanup_failed:'+str(cleanup)) from exc
            raise

    @classmethod
    def observe(cls,*,wait_for_feedback_s=0.,monotonic=time.monotonic):
        return cls(observe_only=True,wait_for_feedback_s=wait_for_feedback_s,monotonic=monotonic)

    def _assert_no_initial_motion_publishers(self):
        created=[p.topic_name for p in self.node.publishers if p.topic_name in MOTION_TOPICS]
        if created:raise RuntimeError('factory_driver_created_motion_publishers_during_construction:'+','.join(created))

    def _spin(self):
        self._started.set()
        try:
            while not self._closing.is_set() and self.context.ok():
                self.executor.spin_once(timeout_sec=.05)
            if not self._closing.is_set():raise RuntimeError('manufacturer_ROS_context_stopped_unexpectedly')
        except BaseException as exc:
            if not self._closing.is_set():
                self._error=exc;self._failed.set()
                robot=self.robot
                if isinstance(robot,ManufacturerAdapter):
                    try:robot.disarm('manufacturer_ROS_executor_failed')
                    except Exception:pass

    def _check_running(self):
        if self._failed.is_set():raise RuntimeError('manufacturer_ROS_executor_failed:'+str(self._error)) from self._error
        if self._closed or self._closing.is_set():raise RuntimeError('manufacturer_ROS_session_closed')
        if self._thread is not None and not self._thread.is_alive():raise RuntimeError('manufacturer_ROS_executor_not_running')

    def wait_for_feedback(self,timeout_s=5.):
        if type(timeout_s) not in (int,float) or not math.isfinite(timeout_s) or not 0<=timeout_s<=30:
            raise ValueError('feedback_wait_must_be_in_0_to_30_seconds')
        deadline=time.monotonic()+timeout_s
        while True:
            self._check_running()
            snapshot=self.feedback.snapshot(require_hands=self.profiles)
            if snapshot['ready']:return snapshot
            if time.monotonic()>=deadline:raise RuntimeError('manufacturer_feedback_wait_timeout:'+','.join(snapshot['issues']))
            self._closing.wait(min(.02,max(0.,deadline-time.monotonic())))

    def status(self):
        """Last-known lifecycle and live receipt-age diagnostics; no commands."""
        robot=self.robot
        return dict(mode='observe_only' if self.observe_only else 'manufacturer_hardware',
            closed=self._closed,executor_failed=self._failed.is_set(),
            executor_error=str(self._error) if self._error else None,
            executor_running=bool(self._thread and self._thread.is_alive() and not self._closing.is_set() and not self._failed.is_set()),
            hardware_writes_enabled=bool(isinstance(robot,ManufacturerAdapter) and robot.hardware_writes_enabled),
            control_state=robot.state if isinstance(robot,ManufacturerAdapter) else 'read_only',
            control_reason=robot.reason if isinstance(robot,ManufacturerAdapter) else 'observation_only',
            feedback=self.feedback.snapshot(require_hands=self.profiles),cleanup_errors=list(self.cleanup_errors))

    def close(self):
        with self._close_lock:
            if self._closed:return
            errors=[]
            # Keep the executor alive while the driver waits for factory stop
            # acknowledgements. ManufacturerAdapter clears motion output first.
            if isinstance(self.robot,ManufacturerAdapter):
                try:
                    result=self.robot.disarm('manufacturer_session_closed')
                    if not result.accepted:errors.append(result.reason)
                except BaseException as exc:errors.append('manufacturer_disarm_failed:'+str(exc))
                transport=self.robot.transport
                if transport is not None:
                    try:transport.close()
                    except Exception as exc:errors.append('motion_transport_cleanup_failed:'+str(exc))
            if self.driver is not None:
                closer=getattr(self.driver,'close',None)
                if callable(closer):
                    try:closer()
                    except Exception as exc:errors.append('factory_driver_cleanup_failed:'+str(exc))
            self._closing.set()
            if self.executor is not None:
                try:self.executor.shutdown(timeout_sec=2.)
                except Exception as exc:errors.append('ROS_executor_shutdown_failed:'+str(exc))
            if self._thread is not None and self._thread is not threading.current_thread():self._thread.join(2.)
            if self._thread is not None and self._thread.is_alive():
                if self.context is not None and self.context.ok():
                    try:self.context.shutdown()
                    except Exception as exc:errors.append('ROS_context_shutdown_failed:'+str(exc))
                if self._thread is not threading.current_thread():self._thread.join(2.)
            if self._thread is not None and self._thread.is_alive():errors.append('manufacturer_ROS_thread_did_not_stop')
            try:self.feedback.close()
            except Exception as exc:errors.append('manufacturer_feedback_cleanup_failed:'+str(exc))
            if self.node is not None:
                try:self.node.destroy_node()
                except Exception as exc:errors.append('ROS_node_destroy_failed:'+str(exc))
            if self.context is not None and self.context.ok():
                try:self.context.shutdown()
                except Exception as exc:errors.append('ROS_context_shutdown_failed:'+str(exc))
            self._closed=True;self.cleanup_errors=errors
            if errors:raise RuntimeError(';'.join(errors))

    def __enter__(self):self._check_running();return self
    def __exit__(self,*_exc):self.close()
