"""Verified Tiangong 3 arm geometry; all poses use waist_pitch_link.

The supplied URDF has unresolved package mesh paths. The generated Placo model
contains kinematics only; this module does not claim mesh collision checking.
"""
from __future__ import annotations

from dataclasses import dataclass
import hashlib
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np

MODEL_SHA256 = 'ddeb5f101da23e73d4d9f1c9d7e29b890b4a81a2443b747336e31f673b47225c'
MODEL_DIR = Path(__file__).resolve().parent / 'assets'
BASE_FRAME = 'waist_pitch_link'
SIDES = ('left', 'right')
JOINT_NAMES_BY_SIDE = {
    side: tuple(f'{name}_{letter}_joint' for name in
                ('shoulder_pitch', 'shoulder_roll', 'shoulder_yaw', 'elbow_pitch',
                 'elbow_yaw', 'wrist_pitch', 'wrist_roll'))
    for side, letter in (('left', 'l'), ('right', 'r'))
}
JOINT_NAMES = JOINT_NAMES_BY_SIDE['left'] + JOINT_NAMES_BY_SIDE['right']
MOTOR_IDS_BY_SIDE = {'left': tuple(range(11, 18)), 'right': tuple(range(21, 28))}
MOTOR_IDS = MOTOR_IDS_BY_SIDE['left'] + MOTOR_IDS_BY_SIDE['right']
# Read from the actual factory joint_control_service ARM_JOINT_RANGE, not
# inferred by reflecting joint IDs. The elbow is tighter than URDF mechanics.
FACTORY_LIMITS = {
    'left': np.array([[-2.84489, 2.84489], [-.191986, 3.33358], [-2.84489, 2.84489],
                      [-2.46, .191986], [-2.84489, 2.84489], [-1.32645, 1.32645],
                      [-1.32645, 1.32645]]),
    'right': np.array([[-2.84489, 2.84489], [-3.33358, .191986], [-2.84489, 2.84489],
                       [-2.46, .191986], [-2.84489, 2.84489], [-1.32645, 1.32645],
                       [-1.32645, 1.32645]]),
}


def finite_vector(values, count):
    result = np.asarray(values, dtype=float)
    if result.shape != (count,) or not np.all(np.isfinite(result)):
        raise ValueError(f'Expected {count} finite numbers')
    return result


def rotation(axis, angle):
    x, y, z = axis
    cross = np.array([[0., -z, y], [z, 0., -x], [-y, x, 0.]])
    return np.eye(3) + np.sin(angle)*cross + (1-np.cos(angle))*(cross@cross)


def origin_transform(element):
    transform = np.eye(4)
    if element is not None:
        transform[:3, 3] = finite_vector([float(v) for v in element.get('xyz', '0 0 0').split()], 3)
        roll, pitch, yaw = finite_vector([float(v) for v in element.get('rpy', '0 0 0').split()], 3)
        transform[:3, :3] = (rotation([0, 0, 1], yaw) @ rotation([0, 1, 0], pitch)
                                @ rotation([1, 0, 0], roll))
    return transform


def default_q():
    """An observed offline reference, not a command or universal zero pose."""
    return np.array([.198, .142, .024, -.463, -.002, .008, -.005,
                     .174, -.144, -.013, -.379, .002, .004, .005])


@dataclass(frozen=True)
class Joint:
    name: str
    child: str
    origin: np.ndarray
    axis: np.ndarray


class ArmModel:
    def __init__(self, side, filename=None):
        if side not in SIDES:
            raise ValueError('side must be left or right')
        self.side = side
        self.base_frame = BASE_FRAME
        self.tip_frame = side + '_tcp_link'
        self.joint_names = JOINT_NAMES_BY_SIDE[side]
        self.motor_ids = MOTOR_IDS_BY_SIDE[side]
        self.filename = Path(filename) if filename else MODEL_DIR/'tiangong3.urdf'
        raw = self.filename.read_bytes()
        self.sha256 = hashlib.sha256(raw).hexdigest()
        if self.sha256 != MODEL_SHA256:
            raise ValueError('URDF differs from the geometry supplied by this robot')
        root = ET.fromstring(raw)
        if root.get('name') != 'tiangong3':
            raise ValueError('Unexpected robot model name')
        by_child = {j.find('child').get('link'): j for j in root.findall('joint')}
        chain, link = [], self.tip_frame
        while link != BASE_FRAME:
            if link not in by_child or len(chain) >= 8:
                raise ValueError('Unexpected arm chain')
            item = by_child[link]
            chain.append(item)
            link = item.find('parent').get('link')
        chain.reverse()
        if len(chain) != 8 or tuple(j.get('name') for j in chain[:7]) != self.joint_names:
            raise ValueError('Joint ordering differs from verified motor mapping')
        self.chain_xml = tuple(chain)
        self.joints, limits = [], []
        for i, item in enumerate(chain[:7]):
            if item.get('type') != 'revolute' or item.find('mimic') is not None:
                raise ValueError('Unsupported joint type')
            axis = finite_vector([float(v) for v in item.find('axis').get('xyz').split()], 3)
            if not np.isclose(np.linalg.norm(axis), 1.):
                raise ValueError('Joint axis must be normalized')
            self.joints.append(Joint(item.get('name'), item.find('child').get('link'),
                                     origin_transform(item.find('origin')), axis))
            limit = item.find('limit')
            limits.append([max(float(limit.get('lower')), FACTORY_LIMITS[side][i, 0])+.05,
                           min(float(limit.get('upper')), FACTORY_LIMITS[side][i, 1])-.05])
        if chain[-1].get('type') != 'fixed':
            raise ValueError('Expected fixed TCP joint')
        self.tip = origin_transform(chain[-1].find('origin'))
        self.limits = np.asarray(limits)
        self.shoulder_origin = self.joints[0].origin[:3, 3].copy()
        # Triangle-inequality bound from the actual chain offsets. It rejects
        # certainly unreachable positions, but is not a sufficient IK test.
        self.maximum_reach = float(sum(np.linalg.norm(j.origin[:3, 3])
                                       for j in self.joints[1:])
                                   + np.linalg.norm(self.tip[:3, 3]))

    def within_geometric_reach(self, position, tolerance=1e-8):
        position = finite_vector(position, 3)
        return bool(np.linalg.norm(position-self.shoulder_origin)
                    <= self.maximum_reach+tolerance)

    def pose_and_jacobian(self, q):
        q = finite_vector(q, 7)
        transform = np.eye(4)
        points, axes = [], []
        for joint, angle in zip(self.joints, q):
            transform = transform @ joint.origin
            points.append(transform[:3, 3].copy())
            axes.append(transform[:3, :3] @ joint.axis)
            turn = np.eye(4); turn[:3, :3] = rotation(joint.axis, angle)
            transform = transform @ turn
        transform = transform @ self.tip
        jacobian = np.zeros((6, 7))
        for i, (point, axis) in enumerate(zip(points, axes)):
            jacobian[:3, i] = np.cross(axis, transform[:3, 3]-point)
            jacobian[3:, i] = axis
        return transform, jacobian

    def fk(self, q):
        return self.pose_and_jacobian(q)[0]

    def links(self, q):
        """Link-frame positions, usable for separately labelled coarse guards."""
        q = finite_vector(q, 7)
        transform = np.eye(4)
        positions = {BASE_FRAME: np.zeros(3)}
        for joint, angle in zip(self.joints, q):
            transform = transform @ joint.origin
            turn = np.eye(4); turn[:3, :3] = rotation(joint.axis, angle)
            transform = transform @ turn
            positions[joint.child] = transform[:3, 3].copy()
        positions[self.tip_frame] = (transform @ self.tip)[:3, 3].copy()
        return positions


class DualArmModel:
    def __init__(self, modelroot=None):
        self.modelroot = Path(modelroot) if modelroot else MODEL_DIR
        filename = self.modelroot/'tiangong3.urdf'
        self.arms = {side: ArmModel(side, filename) for side in SIDES}
        self.limits = np.vstack([self.arms[s].limits for s in SIDES])
        self.joint_names = JOINT_NAMES
        self.motor_ids = MOTOR_IDS
        self.base_frame = BASE_FRAME

    def fk(self, q):
        q = finite_vector(q, 14)
        return {side: self.arms[side].fk(q[i*7:(i+1)*7]) for i, side in enumerate(SIDES)}

    def restrict_joint_limits(self,lower,upper):
        lower=finite_vector(lower,14);upper=finite_vector(upper,14)
        if (np.any(lower<self.limits[:,0]-1e-12) or np.any(upper>self.limits[:,1]+1e-12)
                or np.any(lower>=upper)):
            raise ValueError('manufacturer_limits_must_tighten_actual_model')
        self.limits=np.column_stack((lower,upper))
        for i,side in enumerate(SIDES):self.arms[side].limits=self.limits[i*7:(i+1)*7].copy()
        return self

    def kinematic_urdf(self, velocity_limit=.24):
        if not np.isfinite(velocity_limit) or not 0 < velocity_limit <= .30:
            raise ValueError('Invalid model velocity limit')
        root = ET.Element('robot', name='tiangong3_dualarm_kinematic')
        ET.SubElement(root, 'link', name=BASE_FRAME)
        for side in SIDES:
            arm = self.arms[side]
            for i, item in enumerate(arm.chain_xml):
                ET.SubElement(root, 'link', name=item.find('child').get('link'))
                joint = ET.fromstring(ET.tostring(item))
                if i < 7:
                    limit = joint.find('limit')
                    limit.set('lower', str(arm.limits[i, 0]))
                    limit.set('upper', str(arm.limits[i, 1]))
                    limit.set('velocity', str(velocity_limit))
                root.append(joint)
        raw = ET.tostring(root, encoding='utf-8', xml_declaration=True)
        suffix = hashlib.sha256(raw).hexdigest()[:12]
        path = self.modelroot/f'tiangong3_dualarm_kinematic_{suffix}.urdf'
        if path.exists():
            if path.read_bytes() != raw:
                raise ValueError('Extracted model changed')
        else:
            path.write_bytes(raw)
        return path
