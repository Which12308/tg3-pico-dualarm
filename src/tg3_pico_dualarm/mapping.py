"""Calibrated relative SE(3) targets with independent physical clutch state."""
from dataclasses import dataclass, field
import math
from types import MappingProxyType
import numpy as np

SIDES = ('left', 'right')


def rigid_transform(value):
    result=np.array(value,dtype=float,copy=True)
    if (result.shape!=(4,4) or not np.all(np.isfinite(result))
            or not np.allclose(result[3],[0.,0.,0.,1.],atol=1e-8)
            or not np.allclose(result[:3,:3].T@result[:3,:3],np.eye(3),atol=1e-7)
            or not np.isclose(np.linalg.det(result[:3,:3]),1.,atol=1e-7)):
        raise ValueError('tool calibration must contain rigid SE(3) transforms')
    result.setflags(write=False)
    return result


def tool_transforms(value):
    if not hasattr(value,'keys') or set(value)!=set(SIDES):
        raise ValueError('tool calibration must identify left and right separately')
    return MappingProxyType({side:rigid_transform(value[side]) for side in SIDES})


def identity_tools():return {side:np.eye(4) for side in SIDES}


def rotation(q):
    q = np.asarray(q, dtype=float)
    if q.shape != (4,) or not np.all(np.isfinite(q)) or abs(np.linalg.norm(q)-1) > .02:
        raise ValueError('invalid quaternion')
    x,y,z,w = q/np.linalg.norm(q)
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
                     [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
                     [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])


def pose_matrix(pose):
    result = np.eye(4)
    result[:3,:3] = rotation(pose.orientation_xyzw)
    p = np.asarray(pose.position, dtype=float)
    if p.shape != (3,) or not np.all(np.isfinite(p)):
        raise ValueError('invalid position')
    result[:3,3] = p
    return result


def scaled_rotation(matrix, scale):
    """Scale a shortest rotation using a stable matrix-to-quaternion conversion."""
    if scale == 1:
        return matrix.copy()
    # An eigenvector avoids division by sin(angle) close to pi.
    w = math.sqrt(max(0., 1+np.trace(matrix)))/2
    if w > 1e-7:
        vector = np.array([matrix[2,1]-matrix[1,2], matrix[0,2]-matrix[2,0],
                           matrix[1,0]-matrix[0,1]])/(4*w)
    else:
        values, vectors = np.linalg.eig(matrix)
        vector = np.real(vectors[:,np.argmin(abs(values-1))])
        vector /= np.linalg.norm(vector)
    angle = 2*math.atan2(float(np.linalg.norm(vector)),w)
    if np.linalg.norm(vector) < 1e-12:
        return np.eye(3)
    vector /= np.linalg.norm(vector)
    return rotation((* (vector*math.sin(angle*scale/2)), math.cos(angle*scale/2)))


@dataclass(frozen=True)
class Calibration:
    xr_to_robot: np.ndarray
    translation_scale: float = 1.
    orientation_scale: float = 1.
    source: str = 'head_heading'
    controller_to_hand: dict = field(default_factory=identity_tools)
    tcp_to_hand: dict = field(default_factory=identity_tools)
    tool_offsets_calibrated: bool = False
    tool_reference: str = 'legacy'

    def __post_init__(self):
        r = np.array(self.xr_to_robot,dtype=float,copy=True)
        if r.shape != (3,3) or not np.all(np.isfinite(r)) or not np.allclose(r.T@r,np.eye(3),atol=1e-7) or not np.isclose(np.linalg.det(r),1):
            raise ValueError('calibration must be a proper rotation')
        if not .01 <= self.translation_scale <= 1 or not 0 <= self.orientation_scale <= 1:
            raise ValueError('invalid mapping scale')
        r.setflags(write=False)
        object.__setattr__(self,'xr_to_robot',r)
        object.__setattr__(self,'controller_to_hand',tool_transforms(self.controller_to_hand))
        object.__setattr__(self,'tcp_to_hand',tool_transforms(self.tcp_to_hand))
        if type(self.tool_offsets_calibrated) is not bool:
            raise ValueError('tool calibration state must be explicit')
        if self.tool_reference not in ('legacy','urdf_tcp','measured_grasp_point'):raise ValueError('unknown_tool_reference')

    @classmethod
    def from_head(cls,head,**kwargs):
        forward = rotation(head.orientation_xyzw)@np.array([0.,0.,-1.])
        forward[1] = 0
        if np.linalg.norm(forward) < .25:
            raise ValueError('look horizontally to calibrate heading')
        forward /= np.linalg.norm(forward)
        up=np.array([0.,1.,0.]); right=np.cross(forward,up)
        return cls(np.diag([1.,-1.,1.])@np.column_stack((forward,right,up)).T,**kwargs)

    def as_dict(self):
        return dict(xr_to_robot=self.xr_to_robot.tolist(),translation_scale=self.translation_scale,
                    orientation_scale=self.orientation_scale,source=self.source,
                    controller_to_hand={s:t.tolist() for s,t in self.controller_to_hand.items()},
                    tcp_to_hand={s:t.tolist() for s,t in self.tcp_to_hand.items()},
                    tool_offset_calibrated=self.tool_offsets_calibrated,
                    tool_reference=self.tool_reference,
                    tool_calibration_source=('defined_openxr_grip_to_URDF_TCP' if self.tool_reference=='urdf_tcp' else
                        'configured_transforms' if self.tool_offsets_calibrated else 'unmeasured_identity_or_config'))


@dataclass
class MappingResult:
    targets: dict = field(default_factory=dict)
    active: dict = field(default_factory=dict)
    relatch: dict = field(default_factory=dict)
    reasons: dict = field(default_factory=dict)
    limits: dict = field(default_factory=dict)


def _clamp_position(pose,center,radius):
    offset=pose[:3,3]-center;distance=float(np.linalg.norm(offset))
    if distance<=radius:return False
    pose[:3,3]=center+offset*(radius/distance);return True


def _clamp_rotation(pose,reference,angle_limit):
    relative=pose[:3,:3]@reference[:3,:3].T
    angle=math.acos(float(np.clip((np.trace(relative)-1)/2,-1,1)))
    if angle<=angle_limit:return False
    pose[:3,:3]=scaled_rotation(relative,angle_limit/angle)@reference[:3,:3];return True


class RelativeMapper:
    def __init__(self,calibration,mode='pose',max_displacement=.20,max_controller_speed=1.,*,policy=None,
                 max_target_lead=.03,max_target_lead_rotation=.15):
        from .input_policy import NORMAL
        self.policy=NORMAL if policy is None else policy
        if mode not in ('pose','position'):
            raise ValueError('mode must be pose or position')
        # The arm moves far slower than a hand (0.02 rad/s joint cap on XARM).
        # Targets are kept within this distance/angle of the measured TCP so
        # IK never chases a pose the arm cannot reach soon, and a hand that
        # runs ahead does not end the clutch.
        for value in (max_target_lead,max_target_lead_rotation):
            if isinstance(value,bool) or not isinstance(value,(int,float)) or not math.isfinite(value) or value<=0:
                raise ValueError('invalid_target_lead_limit')
        if max_target_lead>max_displacement:raise ValueError('invalid_target_lead_limit')
        self.max_target_lead,self.max_target_lead_rotation=float(max_target_lead),float(max_target_lead_rotation)
        self.calibration,self.mode = calibration,mode
        self.max_displacement,self.max_controller_speed=max_displacement,max_controller_speed
        self._session=None; self._sequence=-1;self._barrier=-float('inf')
        self._states={side:dict(released=False,anchor=None,last=None,blocked=False) for side in SIDES}

    def invalidate(self,now=None):
        if now is not None:self._barrier=max(self._barrier,now)
        for state in self._states.values():
            state.update(released=False,anchor=None,last=None,blocked=False)

    def within_workspace(self,side,tcp_pose):
        anchor=self._states[side]['anchor']
        if anchor is None:return True
        origin=anchor[1]@np.linalg.inv(self.calibration.tcp_to_hand[side])
        target=rigid_transform(tcp_pose)
        return np.linalg.norm(target[:3,3]-origin[:3,3])<=self.max_displacement+1e-9

    def update(self,frame,current_poses,now):
        result=MappingResult()
        from .input_validation import validate_frame
        try:validate_frame(frame,now,self.policy)
        except ValueError:
            self.invalidate(now)
            for side in SIDES:
                result.active[side]=False;result.relatch[side]=False;result.reasons[side]='invalid_tracking_frame'
            return result
        if frame is None or not frame.focused or not 0 <= now-frame.received_at+frame.source_age_s <= self.policy.total_age_s or frame.received_at<=self._barrier:
            self.invalidate(now)
            for side in SIDES:
                result.active[side]=False;result.relatch[side]=False;result.reasons[side]='tracking_stale_or_unfocused'
            return result
        if frame.session != self._session:
            self.invalidate();self._session=frame.session;self._sequence=-1
        fresh=frame.sequence>self._sequence
        if frame.sequence<self._sequence:
            self.invalidate()
            for side in SIDES:
                result.active[side]=False;result.relatch[side]=False;result.reasons[side]='replayed_frame'
            return result
        self._sequence=frame.sequence
        for side in SIDES:
            controller=getattr(frame,side);state=self._states[side]
            result.active[side]=False;result.relatch[side]=False
            if not controller.connected or not controller.tracked or controller.pose is None:
                state.update(released=False,anchor=None,last=None,blocked=False)
                result.reasons[side]='controller_invalid';continue
            if controller.release_required:
                state.update(released=False,anchor=None,last=None)
                result.reasons[side]='fresh_release_required';continue
            try:
                controller_pose=pose_matrix(controller.pose)
            except ValueError:
                state.update(released=False,anchor=None,last=None,blocked=True)
                result.reasons[side]='invalid_controller_pose';continue
            if controller.grip<=.1:
                if fresh and controller.trigger<=.1 and not controller.buttons:
                    state.update(released=True,anchor=None,last=None,blocked=False)
                result.reasons[side]='released';continue
            if controller.grip<.9:
                # An analog squeeze necessarily crosses intermediate values.
                # Preserve a previously observed neutral release while rising
                # toward activation. Dropping an active clutch still requires
                # a complete release before a new anchor can be accepted.
                if state['anchor'] is not None:state['released']=False
                state.update(anchor=None,last=None)
                result.reasons[side]='grip_not_fully_pressed';continue
            if state['blocked'] or not state['released']:
                result.reasons[side]='fresh_release_required';continue
            if state['anchor'] is None:
                if not fresh:
                    result.reasons[side]='new_frame_required';continue
                controller_hand=controller_pose@self.calibration.controller_to_hand[side]
                robot_hand=rigid_transform(current_poses[side])@self.calibration.tcp_to_hand[side]
                state['anchor']=(controller_hand.copy(),robot_hand.copy())
                result.relatch[side]=True
            if fresh and state['last'] is not None:
                last_position,last_time=state['last']
                span=frame.source_time_s-last_time
                distance=float(np.linalg.norm(controller_pose[:3,3]-last_position))
                if span<=0 or distance>.08 or distance/span>self.max_controller_speed:
                    state.update(released=False,anchor=None,last=None,blocked=True)
                    result.reasons[side]='controller_jump';continue
            if fresh:
                state['last']=(controller_pose[:3,3].copy(),frame.source_time_s)
            start,robot_start=state['anchor']
            controller_hand=controller_pose@self.calibration.controller_to_hand[side]
            delta=self.calibration.translation_scale*self.calibration.xr_to_robot@(controller_hand[:3,3]-start[:3,3])
            limits=[]
            if np.linalg.norm(delta)>self.max_displacement:
                # Hold the target on the workspace sphere instead of ending the
                # clutch; it follows again as soon as the hand comes back.
                delta=delta*(self.max_displacement/np.linalg.norm(delta));limits.append('workspace_clamped')
            target=robot_start.copy();target[:3,3]+=delta
            if self.mode=='pose':
                r=self.calibration.xr_to_robot
                delta_r=r@controller_hand[:3,:3]@start[:3,:3].T@r.T
                target[:3,:3]=scaled_rotation(delta_r,self.calibration.orientation_scale)@robot_start[:3,:3]
            tcp_target=target@np.linalg.inv(self.calibration.tcp_to_hand[side])
            start_tcp=robot_start@np.linalg.inv(self.calibration.tcp_to_hand[side])
            if _clamp_position(tcp_target,start_tcp[:3,3],self.max_displacement) and 'workspace_clamped' not in limits:
                limits.append('workspace_clamped')
            current=rigid_transform(current_poses[side])
            lead=_clamp_position(tcp_target,current[:3,3],self.max_target_lead)
            lead=_clamp_rotation(tcp_target,current,self.max_target_lead_rotation) or lead
            if lead:
                limits.append('target_lead_limited')
                # Measured TCP may sit just outside the sphere; stay inside it.
                _clamp_position(tcp_target,start_tcp[:3,3],self.max_displacement)
            result.targets[side]=tcp_target;result.active[side]=True;result.reasons[side]='active'
            result.limits[side]=tuple(limits)
        return result
