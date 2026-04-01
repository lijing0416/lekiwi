"""Teleoperator base classes and helpers.

This module provides the abstract `Teleoperator` base class expected by
teleoperator implementations in `lerobot.teleoperators.*`.

Note: an example script was moved to `teleoperator_example.py` to avoid
name collisions causing circular imports.
"""
from __future__ import annotations

import abc
from typing import Any

from .config import TeleoperatorConfig


class Teleoperator(abc.ABC):
    """Abstract base class for teleoperators.

    Subclasses should implement the abstract properties/methods used across
    the package (see `lerobot.teleoperators.*` for usages).
    """

    # Class attributes that specific teleoperators override
    config_class = TeleoperatorConfig
    name: str = "teleoperator"

    def __init__(self, config: TeleoperatorConfig):
        self.config = config

    @property
    @abc.abstractmethod
    def action_features(self) -> dict[str, Any]:
        raise NotImplementedError

    @property
    @abc.abstractmethod
    def feedback_features(self) -> dict[str, Any]:
        raise NotImplementedError

    @property
    @abc.abstractmethod
    def is_connected(self) -> bool:
        raise NotImplementedError

    @property
    @abc.abstractmethod
    def is_calibrated(self) -> bool:
        raise NotImplementedError

    @abc.abstractmethod
    def connect(self, *args, **kwargs) -> None:
        raise NotImplementedError

    @abc.abstractmethod
    def calibrate(self) -> None:
        raise NotImplementedError

    @abc.abstractmethod
    def configure(self) -> None:
        raise NotImplementedError

    @abc.abstractmethod
    def get_action(self) -> dict[str, Any]:
        raise NotImplementedError

    @abc.abstractmethod
    def send_feedback(self, feedback: dict[str, Any]) -> None:
        raise NotImplementedError

    @abc.abstractmethod
    def disconnect(self) -> None:
        raise NotImplementedError

    def __repr__(self) -> str:
        return f"<{self.__class__.__name__} name={self.name} config={getattr(self, 'config', None)}>"


# Backwards-compatible alias expected by other modules that import
# `teleoperator` from this module.
teleoperator = Teleoperator