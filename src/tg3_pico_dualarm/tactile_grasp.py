"""BrainCo Revo 2 contact-driven bottle grip using the installed ROS driver."""
from dataclasses import dataclass


@dataclass(frozen=True)
class TactileCalibration:
    """Values measured in the installed driver's raw units, per physical finger."""
    current_mode: int
    thumb_position: int
    position_tolerance: int
    contact_threshold: tuple
    target_force: tuple
    force_deadband: tuple
    current_min: tuple
    current_max: tuple
    current_step: tuple
    closing_sign: tuple
    opposition_current: int
    support_fingers: tuple
    evidence: str

    def __post_init__(self):
        for field in ('contact_threshold', 'target_force', 'force_deadband',
                      'current_min', 'current_max', 'current_step', 'closing_sign'):
            values = tuple(getattr(self, field))
            if len(values) != 5 or any(type(v) is not int for v in values):
                raise ValueError(field + ' requires five measured integers')
            object.__setattr__(self, field, values)
        if type(self.current_mode) is not int or self.current_mode < 2:
            raise ValueError('Provide the installed ROS driver current-mode enum')
        if type(self.thumb_position) is not int or type(self.position_tolerance) is not int or self.position_tolerance < 0:
            raise ValueError('Provide the measured thumb swivel position and tolerance')
        if type(self.opposition_current) is not int:
            raise ValueError('Provide the measured signed thumb opposition current')
        if not isinstance(self.evidence, str) or not self.evidence.strip():
            raise ValueError('Provide the source of the ROS enum and bottle measurements')
        support = tuple(self.support_fingers)
        if (any(type(i) is not int or not 0 <= i < 5 for i in support)
                or len(set(support)) < 3 or 0 not in support):
            raise ValueError('Specify thumb and at least two measured load-bearing finger pads')
        object.__setattr__(self, 'support_fingers', support)
        for contact, target, band, low, high, step, sign in zip(
                self.contact_threshold, self.target_force, self.force_deadband,
                self.current_min, self.current_max, self.current_step, self.closing_sign):
            if not (0 < contact <= target - band and band > 0 and 0 < low <= high
                    and 0 < step <= high and sign in (-1, 1)):
                raise ValueError('Invalid measured tactile calibration')

    def match_positions(self, profile):
        if not profile.lower[1] <= self.thumb_position <= profile.upper[1]:
            raise ValueError('Thumb swivel position is outside the hand calibration')
        for motor in FLEX_MOTORS:
            if profile.opened[motor] == profile.closed[motor]:
                raise ValueError('A five-finger grip requires travel for each finger')


# Driver motor order: thumb flexion, thumb opposition, index, middle, ring, little.
FLEX_MOTORS = (0, 2, 3, 4, 5)


def touch_values(message):
    """Parse the five finger records from the robot's TouchStatus message."""
    if len(message.data) != 5:
        raise ValueError('Expected one tactile record per finger')
    return dict(
        normal=tuple(max(f.normal_force1, f.normal_force2, f.normal_force3) for f in message.data),
        shear=tuple(max(f.tangential_force1, f.tangential_force2, f.tangential_force3) for f in message.data),
        sequence=tuple(f.status >> 8 for f in message.data),
    )


class TactileGrasp:
    def __init__(self, config, positions):
        self.config = config
        self.positions = positions
        self.reset()

    def reset(self):
        self.phase = 'idle'
        self.current = self.config.current_min
        self.last_sequence = None
        self.finger_targets = {}
        self.thumb_ready = False

    def propose(self, operation, closure, measured, previous, touch):
        """Return phase, position target and optional signed current target."""
        config = self.config
        profile = self.positions
        measured = tuple(round(position) for position in measured)
        clear = all(abs(measured[motor] - profile.opened[motor]) <= config.position_tolerance
                    for motor in FLEX_MOTORS)
        if operation == 'open':
            opening = self.phase == 'opening_thumb'
            self.reset()
            if clear or opening:
                self.phase = 'opening_thumb'
                return self.phase, profile.opened, None
            self.phase = 'opening_fingers'
            target = list(profile.opened)
            target[1] = measured[1]
            return self.phase, tuple(target), None
        if touch is None:
            return 'waiting_for_tactile', None, None
        normal = touch['normal']
        contacts = tuple(force >= threshold for force, threshold in zip(normal, config.contact_threshold))
        if self.phase == 'holding':
            if touch['sequence'] == self.last_sequence:
                return self.phase, None, None
            self.last_sequence = touch['sequence']
            self.current = tuple(
                max(lo, min(hi, value + step if force < target - band else
                            value - step if force > target + band else value))
                for value, force, target, band, lo, hi, step in zip(
                    self.current, normal, config.target_force, config.force_deadband,
                    config.current_min, config.current_max, config.current_step))
            return self.phase, measured, self.motor_currents()
        if operation != 'close':
            return 'hold_position', None, None

        if not self.thumb_ready:
            self.thumb_ready = abs(measured[1] - config.thumb_position) <= config.position_tolerance
            if not self.thumb_ready:
                if not clear and self.phase != 'opposing_thumb':
                    self.phase = 'opening_fingers'
                    target = list(profile.opened)
                    target[1] = measured[1]
                    return self.phase, tuple(target), None
                self.phase = 'opposing_thumb'
                target = list(measured)
                target[1] = config.thumb_position
                return self.phase, tuple(target), None

        if all(contacts[i] for i in config.support_fingers):
            self.phase = 'holding'
            self.last_sequence = touch['sequence']
            return self.phase, measured, self.motor_currents()

        self.phase = 'closing'
        target = list(profile.positions(closure))
        target[1] = config.thumb_position
        for motor, contact in zip(FLEX_MOTORS, contacts):
            if contact:
                # Keep the first contact position while the other fingers wrap.
                target[motor] = self.finger_targets.setdefault(motor, measured[motor])
            else:
                self.finger_targets.pop(motor, None)
                direction = profile.closed[motor] - profile.opened[motor]
                if (target[motor] - previous[motor]) * direction < 0:
                    target[motor] = previous[motor]
        return self.phase, tuple(target), None

    def motor_currents(self):
        thumb, index, middle, ring, little = (
            value * sign for value, sign in zip(self.current, self.config.closing_sign))
        return thumb, self.config.opposition_current, index, middle, ring, little
