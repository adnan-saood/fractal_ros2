#!/usr/bin/env python3
"""ROS I/O for the editable FRACTAL Omega tactile exploration policy."""

import math
from enum import Enum, auto

import numpy as np
import rclpy
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn
from rclpy.time import Time
from franka_msgs.msg import FrankaState
from geometry_msgs.msg import PoseStamped
from paxini_hardware.msg import TactileSensor
from tf2_ros import Buffer, TransformException, TransformListener

from fractal_tactile_exploration.msg import FractalMapSample
from belief_model import GaussianProcessMap
from exploration_policy import ExplorationPolicy
from moveit_executor import MoveItExecutor


class ExplorationState(Enum):
    START = auto()
    CALCULATE_NEXT = auto()
    MOVE_TO_APPROACH = auto()
    PROBE_DOWN = auto()
    RETRACT = auto()
    EMERGENCY_RETRACT = auto()
    UPDATE = auto()
    FINISHED = auto()


class OmegaExplorer(LifecycleNode):
    def __init__(self):
        super().__init__('omega_explorer_node')
        self.state = ExplorationState.START
        self.force = 0.0
        self.probe_observation = None
        self.current_action = None
        self.probe_count = 0
        self.state_timer = None

    def on_configure(self, _state):
        self.map = GaussianProcessMap(self)
        self.policy = ExplorationPolicy(self, self.map)
        self.contact_limit = self.declare_parameter('safety.contact_threshold', 1.0).value
        self.hard_limit = self.declare_parameter('safety.hard_force_limit', 5.0).value
        self.force_weight = self.declare_parameter('fusion.franka_force_weight', 1.0).value
        self.taxel_weight = self.declare_parameter('fusion.omega_taxel_weight', 1.0).value
        self.retract_distance = self.declare_parameter('safety.retract_distance', 0.03).value
        self.probe_timeout = self.declare_parameter('exploration.probe_timeout', 5.0).value
        self.max_probes = self.declare_parameter('exploration.max_probes', 0).value
        self.targets = self.create_publisher(PoseStamped, '~/next_omega_target', 10)
        self.retract = self.create_publisher(PoseStamped, '~/emergency_retract_target', 10)
        self.samples = self.create_publisher(FractalMapSample, '~/map_sample', 10)
        self.create_subscription(
            FrankaState, '/franka_robot_state_broadcaster/robot_state', self.on_franka, 10)
        self.create_subscription(
            TactileSensor, '/paxini/L5325_omega/tactile_sensor', self.on_omega, 10)
        self.moveit = MoveItExecutor(self)
        self.moveit.frame_id = self.policy.frame_id
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.state_timer = self.create_timer(0.05, self.run_state_machine)
        self.state_timer.cancel()
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, _state):
        self.state = ExplorationState.START
        self.state_timer.reset()
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, _state):
        if self.state_timer is not None:
            self.state_timer.cancel()
        self.state = ExplorationState.FINISHED
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, _state):
        if self.state_timer is not None:
            self.destroy_timer(self.state_timer)
            self.state_timer = None
        return TransitionCallbackReturn.SUCCESS

    def run_state_machine(self):
        if self.state == ExplorationState.START:
            if not self.moveit.server_is_ready():
                return
            self.state = ExplorationState.CALCULATE_NEXT
        if self.state == ExplorationState.CALCULATE_NEXT:
            if self.max_probes > 0 and self.probe_count >= self.max_probes:
                self.state = ExplorationState.FINISHED
                self.get_logger().info('Exploration completed.')
                return
            self.current_action = self.policy.next_action(seed_downward=self.probe_count == 0)
            self.state = ExplorationState.MOVE_TO_APPROACH
            self.send_pose(self.policy.action_model.to_pose(
                self.policy.frame_id, self.current_action), self.on_approach_complete)
        elif self.state == ExplorationState.UPDATE:
            self.update_belief()
        elif self.state == ExplorationState.EMERGENCY_RETRACT:
            return
        elif self.state == ExplorationState.PROBE_DOWN:
            elapsed = (self.get_clock().now() - self.probe_started).nanoseconds * 1e-9
            if elapsed >= self.probe_timeout:
                self.get_logger().warning('Probe timed out without contact; retracting.')
                self.begin_retract()
        elif self.state == ExplorationState.FINISHED:
            return

    def send_pose(self, pose, on_complete):
        pose.header.stamp = self.get_clock().now().to_msg()
        self.moveit.execute_pose(pose.pose, on_complete)
        self.targets.publish(pose)

    def on_approach_complete(self, succeeded):
        if self.state != ExplorationState.MOVE_TO_APPROACH:
            return
        if not succeeded:
            self.get_logger().error('Approach motion failed; exploration stopped.')
            self.state = ExplorationState.FINISHED
            return
        self.state = ExplorationState.PROBE_DOWN
        self.probe_started = self.get_clock().now()
        probe_pose = self.policy.action_model.to_pose(
            self.policy.frame_id, self.current_action)
        probe_pose.pose.position.x += self.current_action.probe_direction[0] * self.policy.max_probe_distance
        probe_pose.pose.position.y += self.current_action.probe_direction[1] * self.policy.max_probe_distance
        probe_pose.pose.position.z += self.current_action.probe_direction[2] * self.policy.max_probe_distance
        self.send_pose(probe_pose, self.on_probe_motion_complete)

    def on_probe_motion_complete(self, succeeded):
        if self.state != ExplorationState.PROBE_DOWN:
            return
        if not succeeded:
            self.get_logger().error('Probe descent failed; exploration stopped.')
            self.state = ExplorationState.FINISHED
            return
        if self.probe_observation is None:
            self.get_logger().warning('Probe motion completed without contact; retracting.')
            self.begin_retract()

    def on_probe_cancelled(self, succeeded):
        if self.state != ExplorationState.PROBE_DOWN:
            return
        if not succeeded:
            self.get_logger().error('Could not cancel probe motion after contact.')
            self.state = ExplorationState.FINISHED
            return
        self.begin_retract()

    def on_retract_complete(self, succeeded):
        if self.state != ExplorationState.RETRACT:
            return
        if not succeeded:
            self.get_logger().error('Retract motion failed; exploration stopped.')
            self.state = ExplorationState.FINISHED
            return
        self.state = ExplorationState.UPDATE

    def on_emergency_retract_complete(self, succeeded):
        if self.state != ExplorationState.EMERGENCY_RETRACT:
            return
        if not succeeded:
            self.get_logger().error('Emergency retract motion failed.')
        self.state = ExplorationState.FINISHED

    def start_emergency_retract(self, pose, cancelled):
        if self.state != ExplorationState.EMERGENCY_RETRACT:
            return
        if not cancelled:
            self.get_logger().error('Could not cancel motion for emergency retract.')
            self.state = ExplorationState.FINISHED
            return
        self.send_pose(pose, self.on_emergency_retract_complete)
        self.retract.publish(pose)

    def begin_retract(self):
        if self.state != ExplorationState.PROBE_DOWN:
            return
        self.state = ExplorationState.RETRACT
        retract_pose = self.retract_pose()
        self.send_pose(retract_pose, self.on_retract_complete)

    def retract_pose(self):
        retract_pose = self.policy.action_model.to_pose(
            self.policy.frame_id, self.current_action)
        if self.probe_observation is None:
            base = self.current_action.position + (
                self.current_action.probe_direction * self.policy.max_probe_distance)
        else:
            base = np.asarray(self.probe_observation['position'], dtype=float)
        retract_position = base - self.current_action.probe_direction * self.retract_distance
        retract_pose.pose.position.x, retract_pose.pose.position.y, retract_pose.pose.position.z = retract_position
        return retract_pose

    def update_belief(self):
        observation = self.probe_observation
        if observation is None:
            self.get_logger().warning('Probe ended without contact; calculating a new action.')
        else:
            self.map.add(*observation['position'], observation['compliance'],
                         normal_force=observation['normal_force'],
                         taxel_force=observation['taxel_force'],
                         taxel_field=observation['taxel_field'],
                         taxel_positions=observation['taxel_positions'],
                         taxel_normals=observation['taxel_normals'])
            self.samples.publish(observation['sample'])
            self.probe_count += 1
        self.probe_observation = None
        self.state = ExplorationState.CALCULATE_NEXT

    def on_franka(self, state):
        self.force = float(np.linalg.norm(state.k_f_ext_hat_k[:3]))

    def on_omega(self, tactile):
        if self.state != ExplorationState.PROBE_DOWN or self.probe_observation is not None:
            return
        taxel_sum = sum(math.sqrt(t.force.x**2 + t.force.y**2 + t.force.z**2) for t in tactile.taxels)
        metric = self.force_weight * self.force + self.taxel_weight * taxel_sum
        if self.force >= self.hard_limit:
            self.get_logger().error('Hard force reached; controller watchdog is also stopping torque output.')
            self.probe_observation = None
            self.state = ExplorationState.EMERGENCY_RETRACT
            retract_pose = self.retract_pose()
            self.moveit.cancel_active(
                lambda succeeded: self.start_emergency_retract(retract_pose, succeeded))
            return
        if metric < self.contact_limit:
            return
        try:
            tip_transform = self.tf_buffer.lookup_transform(
                self.policy.frame_id, self.moveit.tool_link, Time())
        except TransformException:
            return
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
        compliance = min(1.0, taxel_sum / max(metric, 1e-9))
        translation = tip_transform.transform.translation
        orientation = tip_transform.transform.rotation
        roll, pitch, yaw = self.quaternion_to_rpy(orientation)
        sample = FractalMapSample(); sample.header = tactile.header
        sample.header.frame_id = self.policy.frame_id
        sample.position.x, sample.position.y, sample.position.z = (
            translation.x, translation.y, translation.z)
        sample.roll, sample.pitch, sample.yaw = roll, pitch, yaw
        sample.franka_force, sample.contact_metric, sample.compliance = self.force, metric, compliance
        self.probe_observation = {
            'position': (translation.x, translation.y, translation.z),
            'compliance': compliance,
            'normal_force': self.force,
            'taxel_force': taxel_sum,
            'taxel_field': taxel_field,
            'taxel_positions': taxel_positions,
            'taxel_normals': taxel_normals,
            'sample': sample,
        }
        self.moveit.cancel_active(self.on_probe_cancelled)

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