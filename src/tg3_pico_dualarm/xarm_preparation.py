"""Nonblocking, explicit-admission preparation and reversible factory modes.

No joint or hand publisher lives here. Every service has a bounded future;
FSM requests are limited to three frames. STOP or physical E-stop is never
overwritten on restoration. Only the managed supervisor creates XARM.
"""
from . import safety as motion_policy
import re
import time
import math
import json
import os
from pathlib import Path
from .xarm_adapter import _controllers, _arm_controller, _resources, JOINT_NAMES


class XArmPreparation:
    def __init__(self,node,feedback,*,clock=time.monotonic,types=None,arm_mode=3,command_guard=None):
        if arm_mode not in (0,3):raise ValueError('unreviewed_arm_preparation_mode')
        self.requested_arm_mode=arm_mode;self.command_guard=command_guard
        if types is None:
            from std_msgs.msg import String
            from sensor_msgs.msg import Joy
            from eai_manipulator_msgs.srv import Mode
            from controller_manager_msgs.srv import ListControllers,SwitchController
            from rclpy.qos import qos_profile_sensor_data
            types=dict(string=String,joy=Joy,mode=Mode,list=ListControllers,switch=SwitchController,qos=qos_profile_sensor_data)
        self.node,self.feedback,self.clock,self.types=node,feedback,clock,types
        self.publisher=node.create_publisher(types['string'],'/hric/robot/fsm_state_cmd',1)
        self.subscription=node.create_subscription(types['joy'],'/sbus_data',self._joy,types.get('qos',1))
        self.clients={key:node.create_client(types[kind],name) for key,kind,name in
            (('list','list','/controller_manager/list_controllers'),('switch','switch','/controller_manager/switch_controller'),
             ('get','mode','/EAIHardware/get_arm_mode'),('set','mode','/EAIHardware/set_arm_mode'))}
        self.remote=None;self.pending=None;self.phase='idle';self.reason='explicit_operator_enable_required'
        self.original_body=None;self.original_arm=None;self.dirty=False;self.frames=0
        self.deadline=None;self.next_frame=0.;self.stable_since=None;self.restored=True
        self.arm_mode_after_body=None;self.position_mode_confirmed_at=None

    def _joy(self,message):self.remote=(tuple(message.axes),self.clock())

    def _journal(self):
        directory=os.environ.get('TG3_SUPERVISED_SESSION')
        if not directory:return
        path=Path(directory)/'preparation.json';temporary=path.with_suffix('.tmp')
        temporary.write_text(json.dumps(self.status()),encoding='utf-8');temporary.chmod(0o600)
        temporary.replace(path)

    def physical_guard(self):
        if motion_policy.LIMITS_MANAGED_EXTERNALLY:return
        now=self.clock()
        if self.remote is None or not 0<=now-self.remote[1]<=.15:raise RuntimeError('physical_remote_missing_or_stale')
        axes=self.remote[0]
        if len(axes)<12 or any(not math.isfinite(v) for v in axes) or any(abs(v)>.12 for v in axes[:8]) or any(v!=-1 for v in axes[8:12]):
            raise RuntimeError('physical_remote_changed_stop_VR_before_factory_operation')

    def _body(self):
        self.physical_guard();snap=self.feedback.snapshot(require_hands=False)
        if snap['physical_estop'] is not False:raise RuntimeError('physical_estop_active_or_unknown')
        mode=snap['mode']
        if mode is None or self.feedback.read_state().mode=='UNKNOWN':raise RuntimeError('fresh_body_mode_required')
        return mode,snap

    def _start(self,key,value=None):
        if self.pending is not None:raise RuntimeError('factory_mode_service_already_pending')
        client=self.clients[key]
        if not client.service_is_ready():raise RuntimeError('factory_mode_service_missing:'+key)
        request=self.types[key if key in ('list','switch') else 'mode'].Request()
        if key=='switch':
            if hasattr(request,'deactivate_controllers'):
                request.deactivate_controllers=list(value);request.activate_controllers=[]
            else:
                request.stop_controllers=list(value);request.start_controllers=[]
            request.strictness=2;request.timeout.sec=0;request.timeout.nanosec=150000000
        elif key!='list':request.mode=0 if value is None else value
        self.pending=(key,client.call_async(request),self.clock()+.15)

    def _poll(self):
        if self.pending is None:return None
        key,future,deadline=self.pending
        if self.clock()>deadline:raise RuntimeError('factory_mode_service_timeout:'+key)
        if not future.done():return None
        self.pending=None;response=future.result()
        if response is None:raise RuntimeError('empty_factory_mode_response:'+key)
        if key=='switch' and getattr(response,'ok',None) is not True:raise RuntimeError('factory_deactivation_rejected')
        if key not in ('list','switch') and getattr(response,'result',None) is not True:
            raise RuntimeError('factory_mode_rejected:'+str(getattr(response,'info','')))
        return key,response

    @staticmethod
    def _mode(response):
        match=re.search(r'arm_mode\s*(?:为|is|:|=)\s*([0-9]+)\b',response.info)
        if not match or int(match[1]) not in (0,3):raise RuntimeError('unverified_factory_arm_mode_readback')
        return int(match[1])

    def begin(self):
        if self.phase not in ('idle','restored'):raise RuntimeError('factory_preparation_or_restoration_pending')
        mode,snap=self._body()
        if mode['current_state'] not in ('SWR','HBWALK') or not snap['ready']:
            raise RuntimeError('settled_SWR_or_HBWALK_required_before_preparation')
        if self.command_guard:self.command_guard.begin()
        self.original_body=mode['current_state'];self.original_arm=None
        self.arm_mode_after_body=None;self.position_mode_confirmed_at=None
        self.phase='stationary' if self.command_guard else 'inactive';self.deadline=self.clock()+5.
        self.reason='waiting_factory_zero_walk_feedback' if self.command_guard else 'checking_inactive_factory_controllers'
        self.restored=False;self._journal()
        if not self.command_guard:self._start('list')

    def _fsm(self,command):
        if self.frames>=3:return
        now=self.clock()
        if now<self.next_frame:return
        value=self.types['string']();value.data=command;self.publisher.publish(value)
        self.frames+=1;self.next_frame=now+.04

    def step(self):
        if self.phase=='ready':
            if self.command_guard:self.command_guard.check()
            self.physical_guard();return True
        if self.deadline is None or self.clock()>self.deadline:raise RuntimeError('factory_preparation_timeout')
        mode,snap=self._body()
        if mode['current_state'] not in ('SWR','HBWALK'):raise RuntimeError('body_mode_changed_during_preparation')
        if self.command_guard:
            if self.phase=='stationary':
                if self.command_guard.poll_ready():
                    self.phase='inactive';self.reason='checking_inactive_factory_controllers';self._start('list')
                return False
            self.command_guard.check()
        if self.phase in ('post_body_mode','set_position','verify_position') and mode['current_state']!='HBWALK':
            raise RuntimeError('settled_HBWALK_lost_before_position_mode_confirmation')
        reply=self._poll()
        if reply:
            key,response=reply
            if self.phase in ('inactive','inactive_check'):
                controllers=_controllers(response);active=[]
                for name,item in controllers.items():
                    if item['state']=='active' and _arm_controller(name,item):
                        if not item['claimed'] or not _resources(item['claimed'])<=set(JOINT_NAMES):
                            raise RuntimeError('cannot_deactivate_nonarm_or_unknown_resources:'+name)
                        active.append(name)
                if active:
                    if self.phase=='inactive_check':raise RuntimeError('factory_deactivation_not_observed')
                    self.phase='deactivate';self._start('switch',active)
                else:self.phase='read_mode';self._start('get')
            elif self.phase=='deactivate':self.phase='inactive_check';self._start('list')
            elif self.phase=='read_mode':
                self.original_arm=self._mode(response)
                self.phase='body';self.frames=0;self.next_frame=0.;self.stable_since=None
            elif self.phase=='post_body_mode':
                self.arm_mode_after_body=self._mode(response)
                self.phase='set_position';self.dirty=True;self._journal();self._start('set',self.requested_arm_mode)
            elif self.phase=='set_position':self.phase='verify_position';self._start('get')
            elif self.phase=='verify_position':
                if self._mode(response)!=self.requested_arm_mode:raise RuntimeError('factory_requested_arm_mode_not_confirmed')
                self.position_mode_confirmed_at=self.clock()
                self.phase='ready';self.reason='body_then_arm_mode_confirmed_waiting_arm_enable';return True
        if self.phase=='body':
            if mode['current_state']=='SWR':
                # Journal before any body mutation too. No leg enable, gains,
                # walking velocity publisher or balance-controller replacement.
                if not self.dirty:self.dirty=True;self._journal()
                self._fsm('gotoHBWALK')
            if mode['current_state']=='HBWALK' and snap['ready']:
                if self.stable_since is None:self.stable_since=self.clock()
                if self.clock()-self.stable_since>=.3:
                    # Body transitions may affect arm mode. Observe that state,
                    # then set and read back position mode AFTER settling.
                    self.phase='post_body_mode';self._start('get')
            else:self.stable_since=None
        return False

    def cancel(self):
        if self.phase in ('idle','restored') or self.phase.startswith('restore_'):return
        self.phase='restore_pending';self.reason='factory_mode_restore_pending'

    def restore_step(self,*,safe_to_restore):
        if self.phase in ('idle','restored'):return True
        if not safe_to_restore:return False
        if self.phase not in ('restore_pending','restore_mode','restore_readback','restore_body'):
            return False
        if self.phase=='restore_pending':
            # A mode-setting future may execute server-side after a local
            # cancellation. Observe its bounded completion before restoration.
            if self.pending is not None:
                if self._poll() is None:return False
            self.deadline=self.clock()+5.
            if self.dirty and self.original_arm is not None:
                self.phase='restore_mode';self._start('set',self.original_arm)
            else:self.phase='restore_body';self.frames=0;self.next_frame=0.
        if self.clock()>self.deadline:raise RuntimeError('factory_mode_restore_timeout')
        reply=self._poll()
        if reply:
            if self.phase=='restore_mode':self.phase='restore_readback';self._start('get')
            elif self.phase=='restore_readback':
                if self._mode(reply[1])!=self.original_arm:raise RuntimeError('original_arm_mode_restore_unconfirmed')
                self.phase='restore_body';self.frames=0;self.next_frame=0.
        if self.phase=='restore_body':
            # Never replace physical STOP, other operator modes, missing
            # E-stop feedback or radio loss with a request to resume SWR.
            try:mode,snap=self._body()
            except RuntimeError:return False
            external_body_request=bool(self.command_guard and self.command_guard.external_request)
            if self.original_body=='SWR' and mode['current_state']=='HBWALK' and not external_body_request:
                self._fsm('gotoSTART');return False
            if self.original_body=='SWR' and mode['current_state']=='SWR' and not snap['ready']:return False
            if self.command_guard:self.command_guard.end()
            self.phase='restored'
            self.reason='factory_arm_restored_external_body_preserved' if mode['current_state']!=self.original_body else 'factory_modes_restored'
            self.restored=True;self.dirty=False
            self._journal()
            return True
        return False

    def status(self):
        return dict(phase=self.phase,reason=self.reason,restored=self.restored,
            original_body=self.original_body,original_arm_mode=self.original_arm,mode_change_requested=self.dirty,
            required_body_mode='HBWALK',arm_mode_after_body=self.arm_mode_after_body,
            position_mode_confirmed_at=self.position_mode_confirmed_at,
            requested_arm_mode=self.requested_arm_mode,
            stationary=self.command_guard.status() if self.command_guard else None)

    def close(self):
        if self.pending is not None:self.pending[1].cancel();self.pending=None
        self.node.destroy_subscription(self.subscription);self.node.destroy_publisher(self.publisher)
        for client in self.clients.values():self.node.destroy_client(client)
        if self.command_guard:self.command_guard.close()
