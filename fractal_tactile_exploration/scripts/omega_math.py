"""Compatibility exports for the tactile exploration math modules.

New code should import from the focused modules directly. This facade keeps
existing node and downstream imports stable during the refactor.
"""

from action_model import (
    ContactObservation,
    LocalMap,
    OmegaPressAction,
    TactileAction,
)
from belief_model import FactorGraphBelief, GaussianProcessMap
from exploration_policy import ExplorationPolicy
from gp_models import GaussianProcessField, LearnedTactileForwardModel, REGIMES

__all__ = [
    'ContactObservation',
    'LocalMap',
    'OmegaPressAction',
    'TactileAction',
    'FactorGraphBelief',
    'GaussianProcessMap',
    'ExplorationPolicy',
    'GaussianProcessField',
    'LearnedTactileForwardModel',
    'REGIMES',
]
