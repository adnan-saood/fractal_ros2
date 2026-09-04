#!/usr/bin/env python3
"""Editable FRACTAL action-conditioned Omega tactile exploration policy."""
import math
import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from franka_msgs.msg import FrankaState
from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (BoundingVolume, Constraints, MoveItErrorCodes,
                             OrientationConstraint, PositionConstraint)
from paxini_hardware.msg import TactileSensor
from shape_msgs.msg import SolidPrimitive
from fractal_tactile_exploration.msg import FractalMapSample


class GaussianProcessMap:
    """Two transparent scalar GP fields: surface height and compliance."""
    def __init__(self, node):
        self.length = node.declare_parameter('geometry.length_scale', 0.015).value
        self.signal = node.declare_parameter('geometry.signal_stddev', 0.002).value
        self.height_prior = node.declare_parameter('geometry.prior', 0.0).value
        self.material_length = node.declare_parameter('material.length_scale', 0.020).value
        self.material_signal = node.declare_parameter('material.signal_stddev', 0.25).value
        self.material_prior = node.declare_parameter('material.prior', 0.5).value
        self.noise = node.declare_parameter('sensor.measurement_stddev', 0.0005).value
        self.samples = []

    def add(self, x, y, height, compliance): self.samples.append((x, y, height, compliance))
    def predict(self, x, y):
        if not self.samples:
            return self.height_prior, self.signal ** 2, self.material_prior, self.material_signal ** 2
        points = np.array([[sample[0], sample[1]] for sample in self.samples])
        query = np.array([x, y])
        def field(length, signal, prior, value_index):
            distances = np.sum((points[:, None] - points[None, :]) ** 2, axis=2)
            covariance = signal ** 2 * np.exp(-0.5 * distances / length ** 2) + np.eye(len(points)) * self.noise ** 2
            cross = signal ** 2 * np.exp(-0.5 * np.sum((points - query) ** 2, axis=1) / length ** 2)
            values = np.array([sample[value_index] for sample in self.samples]) - prior
            alpha = np.linalg.solve(covariance, values)
            mean = prior + cross @ alpha
            variance = max(0.0, signal ** 2 - cross @ np.linalg.solve(covariance, cross))
            return mean, variance
        height, height_variance = field(self.length, self.signal, self.height_prior, 2)
        compliance, compliance_variance = field(self.material_length, self.material_signal, self.material_prior, 3)
        return height, height_variance, compliance, compliance_variance


class OmegaPressAction:
    """Current action model; add EliteFingerAction with the same interface later."""
    name = 'omega_press'
    def candidate_pose(self, x, y, z, yaw):
        pose = PoseStamped()
        pose.header.frame_id = 'tactile_map'
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = x, y, z
        pose.pose.orientation.w, pose.pose.orientation.z = math.cos(yaw / 2), math.sin(yaw / 2)
        return pose


class OmegaExplorer(Node):
    def __init__(self):
        super().__init__('omega_explorer_node')
        self.map = GaussianProcessMap(self)
        self.xs = self.declare_parameter('candidate_x', [-0.05, 0.0, 0.05]).value
        self.ys = self.declare_parameter('candidate_y', [-0.05, 0.0, 0.05]).value
        self.approach_z = self.declare_parameter('approach_height', 0.03).value
        self.yaw = self.declare_parameter('nominal_yaw', 0.0).value
        self.info_weight = self.declare_parameter('exploration_weight', 1.0).value
        self.affordance_weight = self.declare_parameter('affordance_weight', 1.5).value
        self.flat_target = self.declare_parameter('affordance.target_flatness', 0.95).value
        self.firm_target = self.declare_parameter('affordance.target_firmness', 0.95).value
        self.contact_limit = self.declare_parameter('safety.contact_threshold', 1.0).value
        self.hard_limit = self.declare_parameter('safety.hard_force_limit', 5.0).value
        self.force_weight = self.declare_parameter('fusion.franka_force_weight', 1.0).value
        self.taxel_weight = self.declare_parameter('fusion.omega_taxel_weight', 1.0).value
        self.mock_square_enabled = self.declare_parameter('motion_test.enabled', False).value
        self.mock_square_frame = self.declare_parameter('motion_test.frame_id', 'world').value
        self.mock_square_xy = self.declare_parameter(
            'motion_test.square_xy', [0.30, -0.05, 0.40, -0.05, 0.40, 0.05, 0.30, 0.05]).value
        self.mock_square_z = self.declare_parameter('motion_test.z', 0.50).value
        self.mock_square_orientation = self.declare_parameter(
            'motion_test.orientation_xyzw', [0.0, 1.0, 0.0, 0.0]).value
        self.mock_square_position_tolerance = self.declare_parameter(
            'motion_test.position_tolerance', 0.005).value
        self.mock_square_orientation_tolerance = self.declare_parameter(
            'motion_test.orientation_tolerance', 0.05).value
        self.mock_square_start_delay = self.declare_parameter(
            'motion_test.start_delay', 2.0).value
        self.force, self.position = 0.0, None
        self.action = OmegaPressAction()
        self.targets = self.create_publisher(PoseStamped, '~/next_omega_target', 10)
        self.retract = self.create_publisher(PoseStamped, '~/emergency_retract_target', 10)
        self.samples = self.create_publisher(FractalMapSample, '~/map_sample', 10)
        self.create_subscription(FrankaState, '/franka_robot_state_broadcaster/robot_state', self.on_franka, 10)
        self.create_subscription(TactileSensor, '/paxini/L5325_omega/tactile_sensor', self.on_omega, 10)
        self.move_group_client = ActionClient(self, MoveGroup, '/move_action')
        self.square_waypoint_index = 0
        self.square_timer = None
        if self.mock_square_enabled:
            self.validate_mock_square_parameters()
            self.square_timer = self.create_timer(
                self.mock_square_start_delay, self.start_mock_square_test)

    def validate_mock_square_parameters(self):
        if len(self.mock_square_xy) < 8 or len(self.mock_square_xy) % 2:
            raise ValueError('motion_test.square_xy must contain at least four XY coordinate pairs.')
        if len(self.mock_square_orientation) != 4:
            raise ValueError('motion_test.orientation_xyzw must contain four values.')

    def start_mock_square_test(self):
        if not self.move_group_client.server_is_ready():
            self.get_logger().info('Waiting for MoveIt /move_action before starting square test.')
            return
        self.square_timer.cancel()
        self.get_logger().info(
            f'Starting mock XY square test with {len(self.mock_square_xy) // 2} waypoints.')
        self.send_next_square_waypoint()

    def send_next_square_waypoint(self):
        if self.square_waypoint_index >= len(self.mock_square_xy) // 2:
            self.get_logger().info('Mock XY square test completed.')
            return

        point_index = 2 * self.square_waypoint_index
        x, y = self.mock_square_xy[point_index:point_index + 2]
        goal = MoveGroup.Goal()
        request = goal.request
        request.group_name = 'panda_arm'
        request.num_planning_attempts = 5
        request.allowed_planning_time = 5.0
        request.max_velocity_scaling_factor = 0.1
        request.max_acceleration_scaling_factor = 0.1
        request.goal_constraints = [self.make_tool_pose_constraint(x, y)]
        request.start_state.is_diff = True
        goal.planning_options.plan_only = False

        self.get_logger().info(
            f'Planning square waypoint {self.square_waypoint_index + 1}: '
            f'x={x:.3f}, y={y:.3f}, z={self.mock_square_z:.3f}.')
        self.move_group_client.send_goal_async(goal).add_done_callback(self.on_square_goal_response)

    def make_tool_pose_constraint(self, x, y):
        target_pose = Pose()
        target_pose.position.x = x
        target_pose.position.y = y
        target_pose.position.z = self.mock_square_z
        (target_pose.orientation.x, target_pose.orientation.y,
         target_pose.orientation.z, target_pose.orientation.w) = self.mock_square_orientation

        position_constraint = PositionConstraint()
        position_constraint.header.frame_id = self.mock_square_frame
        position_constraint.link_name = 'omega_contact_tip'
        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        sphere.dimensions = [self.mock_square_position_tolerance]
        position_constraint.constraint_region = BoundingVolume(
            primitives=[sphere], primitive_poses=[target_pose])
        position_constraint.weight = 1.0

        orientation_constraint = OrientationConstraint()
        orientation_constraint.header.frame_id = self.mock_square_frame
        orientation_constraint.link_name = 'omega_contact_tip'
        orientation_constraint.orientation = target_pose.orientation
        orientation_constraint.absolute_x_axis_tolerance = self.mock_square_orientation_tolerance
        orientation_constraint.absolute_y_axis_tolerance = self.mock_square_orientation_tolerance
        orientation_constraint.absolute_z_axis_tolerance = self.mock_square_orientation_tolerance
        orientation_constraint.weight = 1.0

        constraint = Constraints()
        constraint.position_constraints = [position_constraint]
        constraint.orientation_constraints = [orientation_constraint]
        return constraint

    def on_square_goal_response(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('MoveIt rejected the mock square waypoint goal.')
            return
        goal_handle.get_result_async().add_done_callback(self.on_square_goal_result)

    def on_square_goal_result(self, future):
        result = future.result().result
        if result.error_code.val != MoveItErrorCodes.SUCCESS:
            self.get_logger().error(
                f'Mock square waypoint {self.square_waypoint_index + 1} failed with '
                f'MoveIt error code {result.error_code.val}.')
            return
        self.square_waypoint_index += 1
        self.send_next_square_waypoint()

    def on_franka(self, state):
        self.force = float(np.linalg.norm(state.k_f_ext_hat_k[:3]))
        self.position = state.o_t_ee[12:15]

    def on_omega(self, tactile):
        if self.position is None: return
        taxel_sum = sum(math.sqrt(t.force.x**2 + t.force.y**2 + t.force.z**2) for t in tactile.taxels)
        metric = self.force_weight * self.force + self.taxel_weight * taxel_sum
        if self.force >= self.hard_limit:
            self.get_logger().error('Hard force reached; controller watchdog is also stopping torque output.')
            return
        if metric < self.contact_limit: return
        compliance = min(1.0, taxel_sum / max(metric, 1e-9))
        self.map.add(*self.position[:2], self.position[2], compliance)
        sample = FractalMapSample(); sample.header = tactile.header
        sample.position.x, sample.position.y, sample.position.z = self.position
        sample.yaw, sample.franka_force, sample.contact_metric, sample.compliance = self.yaw, self.force, metric, compliance
        self.samples.publish(sample)
        self.publish_next(tactile.header.stamp)

    def publish_next(self, stamp):
        best = None
        for x in self.xs:
            for y in self.ys:
                _, gv, compliance, cv = self.map.predict(x, y)
                flatness = 1.0 - min(1.0, math.sqrt(gv) / max(self.approach_z, 1e-9))
                firmness = 1.0 - min(1.0, max(0.0, compliance))
                affordance = min(flatness / self.flat_target, firmness / self.firm_target)
                score = self.info_weight * (gv + cv) + self.affordance_weight * affordance
                best = max(best, (score, x, y)) if best else (score, x, y)
        target = self.action.candidate_pose(best[1], best[2], self.approach_z, self.yaw)
        target.header.stamp = stamp
        self.targets.publish(target)


def main():
    rclpy.init(); rclpy.spin(OmegaExplorer()); rclpy.shutdown()
if __name__ == '__main__': main()