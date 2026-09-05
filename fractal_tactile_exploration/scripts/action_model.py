"""Action data structures and probe-direction pose geometry."""

from dataclasses import dataclass
import math

import numpy as np
from geometry_msgs.msg import PoseStamped


@dataclass
class TactileAction:
    """Continuous action: location, probe direction, spin, and motion profile."""

    position: np.ndarray
    probe_direction: np.ndarray
    spin: float
    linear_velocity: np.ndarray
    normal_force: float
    duration: float
    mode: str = 'press'

    @property
    def vector(self):
        return np.concatenate((
            self.position, self.probe_direction, [self.spin],
            self.linear_velocity, [self.normal_force, self.duration]))


@dataclass
class ContactObservation:
    """One dense Omega observation and its inferred interaction regime."""

    position: np.ndarray
    compliance: float
    normal_force: float
    taxel_force: float
    taxel_field: np.ndarray
    action: TactileAction
    regime: str


@dataclass
class LocalMap:
    """Local submap anchored in the global tactile-map frame."""

    anchor: np.ndarray
    samples: list


class OmegaPressAction:
    """Convert a direction-based tactile action into a Cartesian pose."""

    name = 'omega_press'

    @staticmethod
    def to_pose(frame_id, action):
        pose = PoseStamped()
        pose.header.frame_id = frame_id
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = action.position

        pad_normal = -np.asarray(action.probe_direction, dtype=float)
        pad_normal /= max(np.linalg.norm(pad_normal), 1e-9)
        reference = np.array([1.0, 0.0, 0.0])
        if abs(np.dot(reference, pad_normal)) > 0.95:
            reference = np.array([0.0, 1.0, 0.0])
        tangent_x = reference - np.dot(reference, pad_normal) * pad_normal
        tangent_x /= np.linalg.norm(tangent_x)
        tangent_y = np.cross(pad_normal, tangent_x)
        spin_cos, spin_sin = math.cos(action.spin), math.sin(action.spin)
        rotated_x = spin_cos * tangent_x + spin_sin * tangent_y
        rotated_y = -spin_sin * tangent_x + spin_cos * tangent_y
        rotation = np.column_stack((rotated_x, rotated_y, pad_normal))
        pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w = (
            OmegaPressAction._rotation_to_quaternion(rotation))
        return pose

    @staticmethod
    def _rotation_to_quaternion(rotation):
        trace = np.trace(rotation)
        quaternion = np.empty(4)
        if trace > 0.0:
            scale = 2.0 * math.sqrt(trace + 1.0)
            quaternion[3] = 0.25 * scale
            quaternion[0] = (rotation[2, 1] - rotation[1, 2]) / scale
            quaternion[1] = (rotation[0, 2] - rotation[2, 0]) / scale
            quaternion[2] = (rotation[1, 0] - rotation[0, 1]) / scale
            return quaternion

        index = int(np.argmax(np.diag(rotation)))
        next_index = (index + 1) % 3
        last_index = (index + 2) % 3
        scale = 2.0 * math.sqrt(max(
            1.0 + rotation[index, index]
            - rotation[next_index, next_index]
            - rotation[last_index, last_index], 1e-9))
        quaternion[index] = 0.25 * scale
        quaternion[3] = (rotation[last_index, next_index]
                         - rotation[next_index, last_index]) / scale
        quaternion[next_index] = (rotation[next_index, index]
                                  + rotation[index, next_index]) / scale
        quaternion[last_index] = (rotation[last_index, index]
                                  + rotation[index, last_index]) / scale
        return quaternion
