"""Continuous action optimization over the tactile factor-graph belief."""

import math

import numpy as np

from action_model import OmegaPressAction, TactileAction
from gp_models import REGIMES


class ExplorationPolicy:
    """Optimize position, probe direction, spin, and interaction settings."""

    def __init__(self, node, belief):
        self.belief = belief
        self.frame_id = node.declare_parameter('map_frame', 'tactile_map').value
        self.approach_offset = node.declare_parameter('approach_height', 0.03).value
        self.max_probe_distance = node.declare_parameter(
            'action.max_probe_distance', 0.40).value
        self.position_bounds = np.asarray(node.declare_parameter(
            'action.position_bounds', [-0.20, 0.20, -0.20, 0.20]).value, dtype=float)
        self.probe_cone_angle = math.radians(node.declare_parameter(
            'action.probe_cone_angle_deg', 30.0).value)
        self.spin_bounds = np.asarray(node.declare_parameter(
            'action.yaw_bounds', [-math.pi, math.pi]).value, dtype=float)
        self.velocity_bounds = np.asarray(node.declare_parameter(
            'action.velocity_bounds', [-0.02, 0.02]).value, dtype=float)
        self.force_bounds = np.asarray(node.declare_parameter(
            'action.normal_force_bounds', [0.1, 2.0]).value, dtype=float)
        self.duration_bounds = np.asarray(node.declare_parameter(
            'action.duration_bounds', [0.1, 2.0]).value, dtype=float)
        self.lambda_information = node.declare_parameter(
            'objective.lambda_information', 1.0).value
        self.lambda_affordance = node.declare_parameter(
            'objective.lambda_affordance', 1.0).value
        self.lambda_cost = node.declare_parameter(
            'objective.lambda_cost', 0.1).value
        self.action_model = OmegaPressAction()
        self.last_action = None

    def _clip_action(self, action):
        action.position[0] = np.clip(
            action.position[0], self.position_bounds[0], self.position_bounds[1])
        action.position[1] = np.clip(
            action.position[1], self.position_bounds[2], self.position_bounds[3])
        action.probe_direction = self._clip_probe_direction(action.probe_direction)
        action.spin = float(np.clip(action.spin, *self.spin_bounds))
        action.linear_velocity = np.clip(action.linear_velocity, *self.velocity_bounds)
        action.normal_force = float(np.clip(action.normal_force, *self.force_bounds))
        action.duration = float(np.clip(action.duration, *self.duration_bounds))
        return action

    def _clip_probe_direction(self, direction):
        direction = np.asarray(direction, dtype=float)
        direction /= max(np.linalg.norm(direction), 1e-9)
        angle = math.acos(np.clip(-direction[2], -1.0, 1.0))
        if angle <= self.probe_cone_angle:
            return direction
        horizontal = direction.copy()
        horizontal[2] = 0.0
        horizontal_norm = np.linalg.norm(horizontal)
        if horizontal_norm < 1e-9:
            horizontal = np.array([1.0, 0.0, 0.0])
        else:
            horizontal /= horizontal_norm
        return np.array([
            math.sin(self.probe_cone_angle) * horizontal[0],
            math.sin(self.probe_cone_angle) * horizontal[1],
            -math.cos(self.probe_cone_angle)])

    def _initial_action(self):
        position = np.zeros(3)
        position[:2] = np.mean(self.position_bounds.reshape(2, 2), axis=1)
        state_mean, _ = self.belief.predict_state(position)
        position[2] = state_mean[0] + self.approach_offset
        return TactileAction(
            position=position,
            probe_direction=np.array([0.0, 0.0, -1.0]),
            spin=0.0,
            linear_velocity=np.zeros(3),
            normal_force=float(np.mean(self.force_bounds)),
            duration=float(np.mean(self.duration_bounds)))

    def _objective(self, action):
        action = self._clip_action(action)
        state_mean, _ = self.belief.predict_state(action.position)
        information = self.belief.predictive_information_gain(action.position, action)
        regime_probabilities = self.belief.interaction_prior(state_mean, action)
        affordance = self._expected_affordance(action, regime_probabilities)
        cost = self._action_cost(action)
        return (self.lambda_information * information
                + self.lambda_affordance * affordance
                - self.lambda_cost * cost)

    @staticmethod
    def _expected_affordance(action, regime_probabilities):
        rewards = {'static': 1.0, 'elastic': 0.5}
        reward = sum(
            regime_probabilities[index] * rewards[regime]
            for index, regime in enumerate(REGIMES))
        return float(reward - 0.25 * action.normal_force ** 2)

    @staticmethod
    def _action_cost(action):
        movement = np.linalg.norm(action.position)
        orientation = math.acos(np.clip(-action.probe_direction[2], -1.0, 1.0))
        speed = np.linalg.norm(action.linear_velocity)
        return float(
            movement + 0.25 * orientation + 0.1 * abs(action.spin)
            + speed * action.duration + 0.1 * action.duration)

    def next_action(self, seed_downward=False):
        """Optimize an action with deterministic coordinate ascent."""
        best = self._initial_action()
        best_score = self._objective(best)
        steps = np.array([
            0.02, 0.02, 0.02, 0.1, 0.1, 0.1, 0.2,
            0.01, 0.01, 0.01, 0.5, 0.1])
        if seed_downward:
            steps[3:7] = 0.0
        for _ in range(5):
            improved = False
            for index, step in enumerate(steps):
                for direction in (-1.0, 1.0):
                    values = best.vector.copy()
                    values[index] += direction * step
                    candidate = TactileAction(
                        position=values[:3],
                        probe_direction=values[3:6],
                        spin=values[6],
                        linear_velocity=values[7:10],
                        normal_force=values[10],
                        duration=values[11])
                    score = self._objective(candidate)
                    if score > best_score:
                        best, best_score, improved = candidate, score, True
            steps *= 0.5
            if not improved:
                break
        state_mean, _ = self.belief.predict_state(best.position)
        best.position[2] = state_mean[0] + self.approach_offset
        self.last_action = best
        return best

    def next_pose(self):
        return self.action_model.to_pose(self.frame_id, self.next_action())
