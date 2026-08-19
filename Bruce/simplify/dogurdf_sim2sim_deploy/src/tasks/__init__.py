"""Task specifications."""

from .base import ObservationSpec, TaskSpec
from .velocity import build_velocity_task, velocity_observation_spec

__all__ = [
    "ObservationSpec",
    "TaskSpec",
    "build_velocity_task",
    "velocity_observation_spec",
]
