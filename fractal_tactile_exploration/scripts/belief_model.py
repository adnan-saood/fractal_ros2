"""Factor-graph belief and map updates for tactile exploration."""

import numpy as np

from action_model import ContactObservation, LocalMap
from gp_models import GaussianProcessField, LearnedTactileForwardModel, REGIMES


class FactorGraphBelief:
    """Belief over geometry, material, local maps, and interaction regimes."""

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
        """Estimate p(eta | s(x), u) from force, speed, and compliance."""
        _, compliance = state_mean
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
        return self.forward_model.predict(state_mean, action, regime)

    def expected_observation(self, state_mean, action):
        probabilities = self.interaction_prior(state_mean, action)
        observations = np.asarray([
            self.observation_model(state_mean, action, regime)
            for regime in REGIMES])
        return probabilities @ observations, probabilities, observations

    def update(self, position, compliance, normal_force, taxel_force,
               taxel_field, action):
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
        return float(0.5 * np.sum(
            np.log1p(state_uncertainty / prior_observation_variance)))


class GaussianProcessMap(FactorGraphBelief):
    """Compatibility facade for the node's existing map update interface."""

    def add(self, x, y, height, compliance, normal_force=0.0,
            taxel_force=0.0, taxel_field=None, taxel_positions=None,
            taxel_normals=None):
        from action_model import TactileAction

        action = getattr(self, 'last_action', None)
        if action is None:
            action = TactileAction(
                position=np.array([x, y, height]),
                probe_direction=np.array([0.0, 0.0, -1.0]), spin=0.0,
                linear_velocity=np.zeros(3), normal_force=0.0, duration=0.1)
        if taxel_field is None:
            taxel_field = np.zeros(self.forward_model.output_size)
        if self.forward_model.taxel_normals is None and taxel_positions is not None:
            self.forward_model.set_taxel_geometry(taxel_positions, taxel_normals)
        return self.update(
            np.array([x, y, height]), compliance, normal_force, taxel_force,
            taxel_field, action)
