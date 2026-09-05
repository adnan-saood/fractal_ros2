#!/usr/bin/env python3
"""Small smoke test for the lifecycle probe event sequence."""

import rclpy
from geometry_msgs.msg import PoseStamped
from fractal_tactile_exploration.msg import FractalMapSample
from rclpy.node import Node


class SimulationTopicTest(Node):
    def __init__(self):
        super().__init__('simulation_topic_test')
        self.expected_samples = self.declare_parameter('expected_samples', 1).value
        self.timeout = self.declare_parameter('timeout', 90.0).value
        self.targets = 0
        self.samples = 0
        self.started = self.get_clock().now()
        self.create_subscription(
            PoseStamped, '/omega_explorer_node/next_omega_target', self.on_target, 10)
        self.create_subscription(
            FractalMapSample, '/omega_explorer_node/map_sample', self.on_sample, 10)
        self.create_timer(0.25, self.check)

    def on_target(self, _message):
        self.targets += 1

    def on_sample(self, _message):
        self.samples += 1

    def check(self):
        elapsed = (self.get_clock().now() - self.started).nanoseconds * 1e-9
        if self.samples >= self.expected_samples:
            self.get_logger().info(
                f'Simulation smoke test passed: {self.samples} samples, {self.targets} targets.')
            raise SystemExit(0)
        if elapsed >= self.timeout:
            self.get_logger().error(
                f'Simulation smoke test timed out: {self.samples}/{self.expected_samples} samples, '
                f'{self.targets} targets.')
            raise SystemExit(1)


def main():
    rclpy.init()
    rclpy.spin(SimulationTopicTest())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
