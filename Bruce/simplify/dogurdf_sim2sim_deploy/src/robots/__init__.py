"""Robot specifications."""

from .base import BodyGroups, RobotSpec
from .dogurdf import (
    DOGURDF_JOINT_NAMES,
    DOGURDF_LEG_JOINT_NAMES,
    DOGURDF_WHEEL_JOINT_NAMES,
    NUM_JOINTS,
    NUM_LEG_JOINTS,
    NUM_WHEELS,
    WHEEL_RADIUS,
    WHEEL_VEL_LIMIT,
    build_dogurdf_spec,
)

__all__ = [
    "BodyGroups",
    "RobotSpec",
    "DOGURDF_JOINT_NAMES",
    "DOGURDF_LEG_JOINT_NAMES",
    "DOGURDF_WHEEL_JOINT_NAMES",
    "NUM_JOINTS",
    "NUM_LEG_JOINTS",
    "NUM_WHEELS",
    "WHEEL_RADIUS",
    "WHEEL_VEL_LIMIT",
    "build_dogurdf_spec",
]
