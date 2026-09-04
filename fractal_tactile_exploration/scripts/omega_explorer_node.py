#!/usr/bin/env python3
"""Editable FRACTAL action-conditioned Omega tactile exploration policy."""
import math
import numpy as np
import rclpy
from rclpy.node import Node
from franka_msgs.msg import FrankaState
from geometry_msgs.msg import PoseStamped
from paxini_hardware.msg import TactileSensor
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
        self.force, self.position = 0.0, None
        self.action = OmegaPressAction()
        self.targets = self.create_publisher(PoseStamped, '~/next_omega_target', 10)
        self.retract = self.create_publisher(PoseStamped, '~/emergency_retract_target', 10)
        self.samples = self.create_publisher(FractalMapSample, '~/map_sample', 10)
        self.create_subscription(FrankaState, '/franka_robot_state_broadcaster/robot_state', self.on_franka, 10)
        self.create_subscription(TactileSensor, '/paxini/L5325_omega/tactile_sensor', self.on_omega, 10)

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