"""Task specifications for legged robot environments."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Callable, Mapping

import jax
import jax.numpy as jp


CommandSampler = Callable[[jax.Array], tuple[jax.Array, jax.Array]]


@dataclass(frozen=True)
class ObservationSpec:
    actor_size: int
    critic_size: int
    odom_size: int | None = None


@dataclass(frozen=True)
class TaskSpec:
    """Task-level metadata and extension hooks."""

    name: str
    command_dim: int
    observation: ObservationSpec
    reward_names: tuple[str, ...]
    termination_names: tuple[str, ...]
    uses_phase_machine: bool = False

    def initial_phase_info(self) -> Mapping[str, jp.ndarray | float | bool]:
        """Optional task phase state included in env info."""
        return {}
