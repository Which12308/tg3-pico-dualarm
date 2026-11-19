"""Bound joint and actual Cartesian increments before the robot boundary."""
from dataclasses import dataclass
import math
import numpy as np


class SafetyError(RuntimeError): pass


@dataclass(frozen=True)
class SafetyLimits:
    joint_speed: float = .24
    joint_acceleration: float = .50
    feedback_lead: float = .035
    translation_speed: float = .04
    rotation_speed: float = .15
    feedback_timeout: float = .25

    def __post_init__(self):
        if any(not math.isfinite(v) or v<=0 for v in vars(self).values()):
            raise ValueError('invalid safety limits')
        if (self.joint_speed>.30 or self.joint_acceleration>.50 or self.feedback_lead>.035
                or self.translation_speed>.04 or self.rotation_speed>.15 or self.feedback_timeout>.25):
            raise ValueError('limits exceed this unvalidated candidate operating envelope')


class JointSafetyFilter:
    """Acceleration-bounded active increments, including braking and reversal.

    A clutch release, input loss or emergency disarm is handled by the control
    layer's explicit immediate hold. Those stop events do not use this active
    trajectory filter and must not be described as acceleration-limited motion.
    """
    def __init__(self,model,limits=None,*,boundary_hold=False):
        self.model=model; self.config=limits or SafetyLimits()
        if boundary_hold and self.config.joint_speed>.02:
            raise ValueError('boundary_hold_requires_low_speed_candidate')
        self.boundary_hold=boundary_hold;self.last_diagnostics={}

    def step(self,proposed,measured,previous,previous_velocity,dt):
        values=[np.asarray(v,dtype=float) for v in (proposed,measured,previous,previous_velocity)]
        if any(v.shape!=(14,) or not np.all(np.isfinite(v)) for v in values):
            raise SafetyError('nonfinite_or_wrong_joint_state')
        proposed,measured,previous,previous_velocity=values
        self.last_diagnostics={'boundary_holds':[]}
        c=self.config
        if not math.isfinite(dt) or not 0<dt<=.05: raise SafetyError('tick_overrun')
        limits=np.asarray(self.model.limits)
        if np.any(measured<limits[:,0]) or np.any(measured>limits[:,1]):
            raise SafetyError('measured_outside_limits')
        if np.max(abs(previous-measured))>c.feedback_lead+1e-8:
            raise SafetyError('previous_target_outside_feedback_window')
        result=previous.copy();velocity=np.zeros(14)
        for side,offset in (('left',0),('right',7)):
            sl=slice(offset,offset+7)
            requested=(proposed[sl]-previous[sl])/dt
            low=np.maximum(limits[sl,0],measured[sl]-c.feedback_lead)
            high=np.minimum(limits[sl,1],measured[sl]+c.feedback_lead)
            acceleration=c.joint_acceleration*dt
            # Deceleration and sign changes are bounded by the same velocity
            # box as acceleration. Scaling an increment toward zero after this
            # projection could violate that box, so it is never done below.
            velocity_low=np.maximum.reduce((np.full(7,-c.joint_speed),
                previous_velocity[sl]-acceleration,(low-previous[sl])/dt))
            velocity_high=np.minimum.reduce((np.full(7,c.joint_speed),
                previous_velocity[sl]+acceleration,(high-previous[sl])/dt))
            if np.any(velocity_low>velocity_high):
                if self.boundary_hold and np.all(previous[sl]>=low) and np.all(previous[sl]<=high):
                    # Explicit protective hold takes priority over continuing
                    # an infeasible active braking trajectory. Freeze the last
                    # accepted target, never chase feedback or project a jump.
                    # This stop is NOT claimed to obey the active acceleration
                    # bound. All position/lead/speed bounds remain enforced.
                    self.last_diagnostics['boundary_holds'].append(dict(side=side,
                        reason='position_boundary_protective_hold',acceleration_override=True))
                    result[sl]=previous[sl];velocity[sl]=0.
                    continue
                raise SafetyError('acceleration_and_position_bounds_infeasible')
            requested=np.clip(requested,velocity_low,velocity_high)
            arm=self.model.arms[side]
            before=arm.fk(previous[sl])

            def cartesian_allowed(candidate_velocity):
                candidate=previous[sl]+candidate_velocity*dt
                after=arm.fk(candidate)
                distance=float(np.linalg.norm(after[:3,3]-before[:3,3]))
                angle=math.acos(float(np.clip((np.trace(after[:3,:3]@before[:3,:3].T)-1)/2,-1,1)))
                return distance<=c.translation_speed*dt+1e-10 and angle<=c.rotation_speed*dt+1e-10

            selected=requested
            if not cartesian_allowed(selected):
                # The nearest-to-zero feasible velocity brakes as much as the
                # acceleration and hard position bounds permit. Every point on
                # this segment remains in their convex intersection. Actual FK
                # validates the Cartesian bound; no Jacobian-only claim is made.
                braking=np.clip(np.zeros(7),velocity_low,velocity_high)
                if not cartesian_allowed(braking):
                    raise SafetyError('acceleration_and_cartesian_bounds_infeasible')
                safe,unsafe=0.,1.
                selected=braking
                for _ in range(16):
                    fraction=(safe+unsafe)/2
                    trial=braking+fraction*(requested-braking)
                    if cartesian_allowed(trial):
                        safe=fraction;selected=trial
                    else:unsafe=fraction
            candidate=previous[sl]+selected*dt
            actual_velocity=(candidate-previous[sl])/dt
            if (np.any(actual_velocity<velocity_low-1e-10)
                    or np.any(actual_velocity>velocity_high+1e-10)
                    or not cartesian_allowed(actual_velocity)):
                raise SafetyError('filtered_velocity_outside_feasible_bounds')
            result[sl]=candidate;velocity[sl]=actual_velocity
        if np.any(result<limits[:,0]-1e-9) or np.any(result>limits[:,1]+1e-9):
            raise SafetyError('filtered_target_outside_limits')
        return result,velocity
