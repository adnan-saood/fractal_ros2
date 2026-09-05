"""Gaussian-process fields and the learned dense tactile forward model."""

import numpy as np

from action_model import TactileAction


REGIMES = ('static', 'elastic')


class GaussianProcessField:
    """Scalar RBF Gaussian process for one component of the tactile state."""

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
    """Online ridge-regression surrogate for dense Omega observations."""

    def __init__(self, node):
        self.ridge = node.declare_parameter('forward_model.ridge', 1e-3).value
        self.taxel_count = int(node.declare_parameter(
            'forward_model.taxel_count', 239).value)
        self.output_size = self.taxel_count * 3
        self.taxel_positions = None
        self.taxel_normals = None
        self.samples = {regime: [] for regime in REGIMES}

    def set_taxel_geometry(self, positions, normals):
        positions = np.asarray(positions, dtype=float)
        normals = np.asarray(normals, dtype=float)
        expected_shape = (self.taxel_count, 3)
        if positions.shape != expected_shape or normals.shape != expected_shape:
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
