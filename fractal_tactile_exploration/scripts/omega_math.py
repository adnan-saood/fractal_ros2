"""Belief-space action generation for the generative tactile field.

This module follows source_material/algorithm.tex.  The implementation keeps
factor values explicit so the contact model can later be replaced by a learned
or differentiable forward model without changing the action policy.
"""

from dataclasses import dataclass
import math

import numpy as np
from geometry_msgs.msg import PoseStamped


REGIMES = ('static', 'elastic')


@dataclass
class TactileAction:
    """Continuous action u: pose, motion profile, and interaction command."""

    position: np.ndarray
    rpy: np.ndarray
    linear_velocity: np.ndarray
    normal_force: float
    duration: float
    mode: str = 'press'

    @property
    def vector(self):
        return np.concatenate((self.position, self.rpy, self.linear_velocity,
                               [self.normal_force, self.duration]))


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
    """Local submap Li anchored in the global tactile-map frame."""

    anchor: np.ndarray
    samples: list


class GaussianProcessField:
    """Scalar GP posterior for one component of s(x)."""

    def __init__(self, length_scale, signal_stddev, prior, noise):
        self.length_scale = max(float(length_scale), 1e-9)
        self.signal_variance = float(signal_stddev) ** 2
        self.prior = float(prior)
        self.noise_variance = max(float(noise) ** 2, 1e-12)
        self.points = []
        self.values = []

    def add(self, point, value):
        self.points.append(np.asarray(point, dtype=float))
        self.values.append(float(value))

    def predict(self, point):
        point = np.asarray(point, dtype=float)
        if not self.points:
            return self.prior, self.signal_variance
        points = np.asarray(self.points)
        values = np.asarray(self.values) - self.prior
        distances = np.sum((points[:, None] - points[None, :]) ** 2, axis=2)
        covariance = self.signal_variance * np.exp(
            -0.5 * distances / self.length_scale ** 2)
        covariance += np.eye(len(points)) * self.noise_variance
        cross = self.signal_variance * np.exp(
            -0.5 * np.sum((points - point) ** 2, axis=1) / self.length_scale ** 2)
        alpha = np.linalg.solve(covariance, values)
        mean = self.prior + cross @ alpha
        variance = self.signal_variance - cross @ np.linalg.solve(covariance, cross)
        return float(mean), max(0.0, float(variance))


class LearnedTactileForwardModel:
    """Online surrogate for h(s, eta, u) with an analytic cold-start prior.

    Features are [height, compliance, action.vector] and outputs are the
    flattened [Fx, Fy, Fz] field for every Omega taxel. The model
    is deliberately small and inspectable; it can be replaced by a neural or
    differentiable contact model without changing FactorGraphBelief.
    """

    def __init__(self, node):
        self.ridge = node.declare_parameter(
            'forward_model.ridge', 1e-3).value
        self.taxel_count = int(node.declare_parameter(
            'forward_model.taxel_count', 239).value)
        self.output_size = self.taxel_count * 3
        self.taxel_positions = None
        self.taxel_normals = None
        self.samples = {regime: [] for regime in REGIMES}

    def set_taxel_geometry(self, positions, normals):
        positions = np.asarray(positions, dtype=float)
        normals = np.asarray(normals, dtype=float)
        if positions.shape != (self.taxel_count, 3) or normals.shape != (self.taxel_count, 3):
            raise ValueError('Taxel geometry must have shape (taxel_count, 3).')
        normal_lengths = np.linalg.norm(normals, axis=1, keepdims=True)
        fallback = np.zeros_like(normals)
        fallback[:, 2] = 1.0
        self.taxel_positions = positions
        self.taxel_normals = np.divide(
            normals, normal_lengths, out=fallback, where=normal_lengths > 1e-9)

    @staticmethod
    def features(state_mean, action):
        return np.concatenate(([state_mean[0], state_mean[1], 1.0], action.vector))

    def analytic_prediction(self, state_mean, action, regime):
        _, compliance = state_mean
        speed = float(np.linalg.norm(action.linear_velocity))
        regime_gain = {'static': 1.0, 'elastic': 1.1}[regime]
        normal_force = regime_gain * action.normal_force
        per_taxel_force = normal_force * (0.5 + 0.5 * (1.0 - compliance))
        if self.taxel_normals is None:
            normals = np.zeros((self.taxel_count, 3), dtype=float)
            normals[:, 2] = 1.0
        else:
            normals = self.taxel_normals
        field = per_taxel_force * normals / max(self.taxel_count, 1)
        field[:, 0] += action.linear_velocity[0] * speed / max(self.taxel_count, 1)
        field[:, 1] += action.linear_velocity[1] * speed / max(self.taxel_count, 1)
        return field.reshape(-1)

    def predict(self, state_mean, action, regime):
        features = self.features(state_mean, action)
        samples = self.samples[regime]
        if len(samples) < len(features):
            return self.analytic_prediction(state_mean, action, regime)
        design = np.asarray([sample[0] for sample in samples])
        targets = np.asarray([sample[1] for sample in samples])
        regularizer = self.ridge * np.eye(design.shape[1])
        coefficients = np.linalg.solve(
            design.T @ design + regularizer, design.T @ targets)
        return features @ coefficients

    def observe(self, state_mean, action, regime, taxel_field):
        target = np.asarray(taxel_field, dtype=float).reshape(-1)
        if target.size != self.output_size:
            raise ValueError(
                f'Expected {self.output_size} dense taxel values, got {target.size}.')
        self.samples[regime].append((self.features(state_mean, action), target))


class FactorGraphBelief:
    """Belief over static state, local-map anchors, and latent regimes.

    The stored factors correspond to the source formulation:
    kinematic, measurement, interaction-prior, and spatial-consistency factors.
    ``update`` currently uses an analytic contact surrogate; its interfaces are
    intentionally the same ones a GTSAM or learned generative backend needs.
    """

    def __init__(self, node):
        noise = node.declare_parameter('sensor.measurement_stddev', 0.0005).value
        self.geometry = GaussianProcessField(
            node.declare_parameter('geometry.length_scale', 0.015).value,
            node.declare_parameter('geometry.signal_stddev', 0.002).value,
            node.declare_parameter('geometry.prior', 0.0).value, noise)
        self.material = GaussianProcessField(
            node.declare_parameter('material.length_scale', 0.020).value,
            node.declare_parameter('material.signal_stddev', 0.25).value,
            node.declare_parameter('material.prior', 0.5).value, noise)
        self.forward_model = LearnedTactileForwardModel(node)
        self.local_maps = []
        self.observations = []
        self.factors = []

    def predict_state(self, position):
        point = np.asarray(position, dtype=float)[:2]
        height, height_variance = self.geometry.predict(point)
        material, material_variance = self.material.predict(point)
        return np.array([height, material]), np.array([height_variance, material_variance])

    def interaction_prior(self, state_mean, action):
        """p(eta | s(x), u), using force, speed, and compliance cues."""
        height, compliance = state_mean
        speed = float(np.linalg.norm(action.linear_velocity))
        force = max(action.normal_force, 0.0)
        logits = np.array([
            1.5 - 3.0 * speed - 1.5 * force,
            1.0 + 2.0 * (1.0 - compliance) - 0.5 * speed,
        ])
        logits -= np.max(logits)
        probabilities = np.exp(logits)
        return probabilities / np.sum(probabilities)

    def observation_model(self, state_mean, action, regime):
        """E[z | s, eta, u] as a flattened dense taxel XYZ field."""
        return self.forward_model.predict(state_mean, action, regime)

    def expected_observation(self, state_mean, action):
        probabilities = self.interaction_prior(state_mean, action)
        observations = np.asarray([
            self.observation_model(state_mean, action, regime) for regime in REGIMES])
        return probabilities @ observations, probabilities, observations

    def update(self, position, compliance, normal_force, taxel_force,
               taxel_field, action):
        """Add measurement and interaction factors, then update local GP beliefs."""
        position = np.asarray(position, dtype=float)
        state_mean, _ = self.predict_state(position)
        probabilities = self.interaction_prior(state_mean, action)
        regime = REGIMES[int(np.argmax(probabilities))]
        observation = ContactObservation(
            position, float(compliance), float(normal_force), float(taxel_force),
            np.asarray(taxel_field, dtype=float), action, regime)
        self.observations.append(observation)
        self.forward_model.observe(state_mean, action, regime, taxel_field)
        self.geometry.add(position[:2], position[2])
        self.material.add(position[:2], compliance)
        local_map = LocalMap(anchor=position.copy(), samples=[observation])
        self.local_maps.append(local_map)
        self.factors.extend([
            ('interaction_prior', probabilities),
            ('measurement', observation),
            ('kinematic', position.copy()),
        ])
        self._add_spatial_factors(local_map)
        return regime

    def _add_spatial_factors(self, new_map):
        for old_map in self.local_maps[:-1]:
            distance = np.linalg.norm(new_map.anchor - old_map.anchor)
            if distance < 3.0 * self.geometry.length_scale:
                self.factors.append(('spatial_consistency', old_map, new_map))

    def predictive_information_gain(self, position, action):
        """Approximate I(s, eta; z | u, B) for the proposed action."""
        state_mean, state_variance = self.predict_state(position)
        _, regime_probabilities, regime_observations = self.expected_observation(
            state_mean, action)
        observation_mean = regime_probabilities @ regime_observations
        centered = regime_observations - observation_mean
        regime_variance = np.sum(
            regime_probabilities[:, None] * centered ** 2, axis=0)
        measurement_noise = np.full(regime_variance.shape, 0.05)
        prior_observation_variance = regime_variance + measurement_noise ** 2
        state_uncertainty = np.full(
            prior_observation_variance.shape,
            state_variance[0] + state_variance[1])
        return float(0.5 * np.sum(np.log1p(state_uncertainty / prior_observation_variance)))


class OmegaPressAction:
    """Convert a continuous action pose to a ROS message for MoveIt."""

    name = 'omega_press'

    @staticmethod
    def to_pose(frame_id, action):
        pose = PoseStamped()
        pose.header.frame_id = frame_id
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = action.position
        roll, pitch, yaw = action.rpy
        half_roll, half_pitch, half_yaw = roll / 2.0, pitch / 2.0, yaw / 2.0
        pose.pose.orientation.x = (
            math.sin(half_roll) * math.cos(half_pitch) * math.cos(half_yaw)
            - math.cos(half_roll) * math.sin(half_pitch) * math.sin(half_yaw))
        pose.pose.orientation.y = (
            math.cos(half_roll) * math.sin(half_pitch) * math.cos(half_yaw)
            + math.sin(half_roll) * math.cos(half_pitch) * math.sin(half_yaw))
        pose.pose.orientation.z = (
            math.cos(half_roll) * math.cos(half_pitch) * math.sin(half_yaw)
            - math.sin(half_roll) * math.sin(half_pitch) * math.cos(half_yaw))
        pose.pose.orientation.w = (
            math.cos(half_roll) * math.cos(half_pitch) * math.cos(half_yaw)
            + math.sin(half_roll) * math.sin(half_pitch) * math.sin(half_yaw))
        return pose


class ExplorationPolicy:
    """Continuously optimize u over the current factor-graph belief."""

    def __init__(self, node, belief):
        self.belief = belief
        self.frame_id = node.declare_parameter('map_frame', 'tactile_map').value
        self.approach_offset = node.declare_parameter('approach_height', 0.03).value
        self.position_bounds = np.asarray(node.declare_parameter(
            'action.position_bounds', [-0.20, 0.20, -0.20, 0.20]).value, dtype=float)
        self.roll_bounds = np.asarray(node.declare_parameter(
            'action.roll_bounds', [-0.5, 0.5]).value, dtype=float)
        self.pitch_bounds = np.asarray(node.declare_parameter(
            'action.pitch_bounds', [-0.5, 0.5]).value, dtype=float)
        self.yaw_bounds = np.asarray(node.declare_parameter(
            'action.yaw_bounds', [-math.pi, math.pi]).value, dtype=float)
        self.velocity_bounds = np.asarray(node.declare_parameter(
            'action.velocity_bounds', [-0.02, 0.02]).value, dtype=float)
        self.force_bounds = np.asarray(node.declare_parameter(
            'action.normal_force_bounds', [0.1, 2.0]).value, dtype=float)
        self.duration_bounds = np.asarray(node.declare_parameter(
            'action.duration_bounds', [0.1, 2.0]).value, dtype=float)
        self.lambda_information = node.declare_parameter('objective.lambda_information', 1.0).value
        self.lambda_affordance = node.declare_parameter('objective.lambda_affordance', 1.0).value
        self.lambda_cost = node.declare_parameter('objective.lambda_cost', 0.1).value
        self.action_model = OmegaPressAction()
        self.last_action = None

    def _clip_action(self, action):
        action.position[0] = np.clip(action.position[0], self.position_bounds[0], self.position_bounds[1])
        action.position[1] = np.clip(action.position[1], self.position_bounds[2], self.position_bounds[3])
        action.rpy[0] = np.clip(action.rpy[0], *self.roll_bounds)
        action.rpy[1] = np.clip(action.rpy[1], *self.pitch_bounds)
        action.rpy[2] = np.clip(action.rpy[2], *self.yaw_bounds)
        action.linear_velocity = np.clip(action.linear_velocity, *self.velocity_bounds)
        action.normal_force = float(np.clip(action.normal_force, *self.force_bounds))
        action.duration = float(np.clip(action.duration, *self.duration_bounds))
        return action

    def _initial_action(self):
        position = np.zeros(3)
        position[:2] = np.mean(self.position_bounds.reshape(2, 2), axis=1)
        state_mean, _ = self.belief.predict_state(position)
        position[2] = state_mean[0] + self.approach_offset
        return TactileAction(
            position=position, rpy=np.zeros(3), linear_velocity=np.zeros(3),
            normal_force=float(np.mean(self.force_bounds)),
            duration=float(np.mean(self.duration_bounds)))

    def _objective(self, action):
        action = self._clip_action(action)
        state_mean, state_variance = self.belief.predict_state(action.position)
        information = self.belief.predictive_information_gain(action.position, action)
        regime_probabilities = self.belief.interaction_prior(state_mean, action)
        affordance = self._expected_affordance(action, regime_probabilities)
        cost = self._action_cost(action)
        return (self.lambda_information * information
                + self.lambda_affordance * affordance
                - self.lambda_cost * cost)

    def _expected_affordance(self, action, regime_probabilities):
        rewards = {'static': 1.0, 'elastic': 0.5}
        reward = sum(regime_probabilities[i] * rewards[regime] for i, regime in enumerate(REGIMES))
        return float(reward - 0.25 * action.normal_force ** 2)

    def _action_cost(self, action):
        movement = np.linalg.norm(action.position)
        orientation = np.linalg.norm(action.rpy)
        speed = np.linalg.norm(action.linear_velocity)
        return float(movement + 0.25 * orientation + speed * action.duration + 0.1 * action.duration)

    def next_action(self):
        """Optimize a continuous action with deterministic coordinate ascent."""
        best = self._initial_action()
        best_score = self._objective(best)
        steps = np.array([
            0.02, 0.02, 0.02, 0.1, 0.1, 0.1,
            0.01, 0.01, 0.01, 0.5, 0.1])
        for _ in range(5):
            improved = False
            for index, step in enumerate(steps):
                for direction in (-1.0, 1.0):
                    values = best.vector.copy()
                    values[index] += direction * step
                    candidate = TactileAction(
                        position=values[:3], rpy=values[3:6],
                        linear_velocity=values[6:9], normal_force=values[9],
                        duration=values[10])
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


class GaussianProcessMap(FactorGraphBelief):
    """Compatibility facade for the existing ROS node.

    ``add`` remains available while the underlying implementation now stores
    factor-graph belief, local submaps, latent regimes, and GP state.
    """

    def add(self, x, y, height, compliance, normal_force=0.0,
            taxel_force=0.0, taxel_field=None, taxel_positions=None,
            taxel_normals=None):
        action = getattr(self, 'last_action', None)
        if action is None:
            action = TactileAction(
                position=np.array([x, y, height]), rpy=np.zeros(3),
                linear_velocity=np.zeros(3), normal_force=0.0, duration=0.1)
        if taxel_field is None:
            taxel_field = np.zeros(self.forward_model.output_size)
        if self.forward_model.taxel_normals is None and taxel_positions is not None:
            self.forward_model.set_taxel_geometry(taxel_positions, taxel_normals)
        return self.update(
            np.array([x, y, height]), compliance, normal_force, taxel_force,
            taxel_field, action)
