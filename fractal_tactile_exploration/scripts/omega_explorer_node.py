#!/usr/bin/env python3
"""ROS I/O for the editable FRACTAL Omega tactile exploration policy."""

import math

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from franka_msgs.msg import FrankaState
from geometry_msgs.msg import PoseStamped
from paxini_hardware.msg import TactileSensor
from tf2_ros import Buffer, TransformException, TransformListener

from fractal_tactile_exploration.msg import FractalMapSample
from mock_square_test import MockSquareTest
from moveit_executor import MoveItExecutor
from omega_math import ExplorationPolicy, GaussianProcessMap


class OmegaExplorer(Node):
    def __init__(self):
        super().__init__('omega_explorer_node')
        self.map = GaussianProcessMap(self)
        self.policy = ExplorationPolicy(self, self.map)
        self.contact_limit = self.declare_parameter('safety.contact_threshold', 1.0).value
        self.hard_limit = self.declare_parameter('safety.hard_force_limit', 5.0).value
        self.force_weight = self.declare_parameter('fusion.franka_force_weight', 1.0).value
        self.taxel_weight = self.declare_parameter('fusion.omega_taxel_weight', 1.0).value
        self.force = 0.0
        self.targets = self.create_publisher(PoseStamped, '~/next_omega_target', 10)
        self.retract = self.create_publisher(PoseStamped, '~/emergency_retract_target', 10)
        self.samples = self.create_publisher(FractalMapSample, '~/map_sample', 10)
        self.create_subscription(FrankaState, '/franka_robot_state_broadcaster/robot_state', self.on_franka, 10)
        self.create_subscription(TactileSensor, '/paxini/L5325_omega/tactile_sensor', self.on_omega, 10)
        self.moveit = MoveItExecutor(self)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.mock_square_test = MockSquareTest(self, self.moveit)

    def on_franka(self, state):
        self.force = float(np.linalg.norm(state.k_f_ext_hat_k[:3]))

    def on_omega(self, tactile):
        try:
            tip_transform = self.tf_buffer.lookup_transform(
                self.policy.frame_id, self.moveit.tool_link, Time())
        except TransformException:
            return
        taxel_sum = sum(math.sqrt(t.force.x**2 + t.force.y**2 + t.force.z**2) for t in tactile.taxels)
        taxel_field = np.asarray([
            component
            for taxel in tactile.taxels
            for component in (taxel.force.x, taxel.force.y, taxel.force.z)
        ], dtype=float)
        taxel_positions = np.asarray([
            (taxel.position.x, taxel.position.y, taxel.position.z)
            for taxel in tactile.taxels
        ], dtype=float)
        taxel_normals = np.asarray([
            (taxel.normal.x, taxel.normal.y, taxel.normal.z)
            for taxel in tactile.taxels
        ], dtype=float)
        metric = self.force_weight * self.force + self.taxel_weight * taxel_sum
        if self.force >= self.hard_limit:
            self.get_logger().error('Hard force reached; controller watchdog is also stopping torque output.')
            return
        if metric < self.contact_limit: return
        compliance = min(1.0, taxel_sum / max(metric, 1e-9))
        translation = tip_transform.transform.translation
        orientation = tip_transform.transform.rotation
        roll, pitch, yaw = self.quaternion_to_rpy(orientation)
        self.map.add(
            translation.x, translation.y, translation.z, compliance,
            normal_force=self.force, taxel_force=taxel_sum,
            taxel_field=taxel_field, taxel_positions=taxel_positions,
            taxel_normals=taxel_normals)
        sample = FractalMapSample(); sample.header = tactile.header
        sample.header.frame_id = self.policy.frame_id
        sample.position.x, sample.position.y, sample.position.z = (
            translation.x, translation.y, translation.z)
        sample.roll, sample.pitch, sample.yaw = roll, pitch, yaw
        sample.franka_force, sample.contact_metric, sample.compliance = self.force, metric, compliance
        self.samples.publish(sample)
        target = self.policy.next_pose()
        target.header.stamp = tactile.header.stamp
        self.targets.publish(target)

    @staticmethod
    def quaternion_to_rpy(quaternion):
        sin_roll = 2.0 * (quaternion.w * quaternion.x + quaternion.y * quaternion.z)
        cos_roll = 1.0 - 2.0 * (quaternion.x ** 2 + quaternion.y ** 2)
        roll = math.atan2(sin_roll, cos_roll)
        sin_pitch = 2.0 * (quaternion.w * quaternion.y - quaternion.z * quaternion.x)
        pitch = math.asin(max(-1.0, min(1.0, sin_pitch)))
        sin_yaw = 2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y)
        cos_yaw = 1.0 - 2.0 * (quaternion.y ** 2 + quaternion.z ** 2)
        yaw = math.atan2(sin_yaw, cos_yaw)
        return roll, pitch, yaw


def main():
    rclpy.init()
    rclpy.spin(OmegaExplorer())
    rclpy.shutdown()
if __name__ == '__main__': main()