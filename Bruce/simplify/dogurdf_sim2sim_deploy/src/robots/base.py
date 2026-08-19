"""Robot-level specifications used by MJX legged environments."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Mapping, Sequence


@dataclass(frozen=True)
class BodyGroups:
    """Named body groups used by rewards, termination, and contact logic."""

    feet: tuple[str, ...]
    penalized_contacts: tuple[str, ...]
    nonfoot_contacts: tuple[str, ...]


@dataclass(frozen=True)
class RobotSpec:
    """Static robot metadata independent from a specific task."""

    name: str
    asset_path: Path
    num_dof: int
    num_actions: int
    joint_names: tuple[str, ...]
    default_joint_angles: Mapping[str, float]
    joint_lower: tuple[float, ...]
    joint_upper: tuple[float, ...]
    torque_limits: tuple[float, ...]
    body_groups: BodyGroups
    mjx_to_policy_idx: tuple[int, ...]
    policy_to_mjx_idx: tuple[int, ...]

    def default_pose(self, ordered_names: Sequence[str] | None = None) -> tuple[float, ...]:
        """Return default joint pose in the requested order."""
        names = tuple(ordered_names or self.joint_names)
        return tuple(self.default_joint_angles[name] for name in names)
