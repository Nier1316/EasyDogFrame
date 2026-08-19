"""dogurdf wheel-legged quadruped robot specification.

16 DoF: 4 legs x (hip, thigh, calf, wheel).  The legs are position-controlled
and the wheels are velocity-controlled, so the policy vector is grouped as
"12 leg joints, then 4 wheel joints" while MJX orders joints per-leg.
"""

from __future__ import annotations

from pathlib import Path

from .base import BodyGroups, RobotSpec


LEG_PREFIXES = ("FL", "FR", "RL", "RR")

# Policy / Isaac order: all leg joints first (grouped per leg), wheels last.
DOGURDF_LEG_JOINT_NAMES = tuple(
    f"{leg}_{joint}_joint"
    for leg in LEG_PREFIXES
    for joint in ("hip", "thigh", "calf")
)
DOGURDF_WHEEL_JOINT_NAMES = tuple(f"{leg}_wheel_joint" for leg in LEG_PREFIXES)
DOGURDF_JOINT_NAMES = DOGURDF_LEG_JOINT_NAMES + DOGURDF_WHEEL_JOINT_NAMES

# MJX order, as reported by the compiled model: per leg hip/thigh/calf/wheel.
DOGURDF_MJX_JOINT_NAMES = tuple(
    f"{leg}_{joint}_joint"
    for leg in LEG_PREFIXES
    for joint in ("hip", "thigh", "calf", "wheel")
)

NUM_JOINTS = len(DOGURDF_JOINT_NAMES)          # 16
NUM_LEG_JOINTS = len(DOGURDF_LEG_JOINT_NAMES)  # 12
NUM_WHEELS = len(DOGURDF_WHEEL_JOINT_NAMES)    # 4

# Nominal stance, calibrated against the model: places each wheel centre
# directly under its hip (x offset 1e-4 m) with the torso at z = 0.45 m.
# Raised from 0.347 on 2026-08-18: user wanted a taller ride height, then
# dialed 0.55 back to 0.45. Only calf changed (-0.70 -> -0.35); thigh stays
# at 0.20 so the stance stays close to the original. Measured via FK:
# (thigh=0.20, calf=-0.35) sits the torso at 0.449 m and still gives ~0.10 m
# of wheel lift with 0.35 rad of calf fold.
NOMINAL_HIP = 0.0
NOMINAL_THIGH = 0.20
NOMINAL_CALF = -0.35
NOMINAL_TORSO_HEIGHT = 0.45

WHEEL_RADIUS = 0.113
# Motor limit 12.5 rad/s -> 12.5 * 0.113 = 1.41 m/s top speed.
WHEEL_VEL_LIMIT = 12.5
LEG_VEL_LIMIT = 14.0

LEG_TORQUE_LIMIT = 150.0
WHEEL_TORQUE_LIMIT = 53.0

# Joint limits straight from urdf/dogurdf.urdf.  Wheels are `continuous` there,
# i.e. unbounded; the large sentinel keeps clip() a no-op for them.
JOINT_LIMITS = {"hip": (-0.6, 0.6), "thigh": (-0.7, 1.75), "calf": (-1.0, 0.35)}
WHEEL_LIMIT = 1.0e6


def _policy_to_mjx_permutation() -> tuple[tuple[int, ...], tuple[int, ...]]:
    """Build the index permutations between policy order and MJX order.

    ``mjx_to_policy_idx[i]`` is the MJX slot feeding policy slot *i*, so
    ``mjx_array[mjx_to_policy_idx]`` reorders MJX -> policy.  ``policy_to_mjx_idx``
    is its inverse, used to push policy-order arrays back into MJX order.
    """
    mjx_slot_of = {name: i for i, name in enumerate(DOGURDF_MJX_JOINT_NAMES)}
    mjx_to_policy = tuple(mjx_slot_of[name] for name in DOGURDF_JOINT_NAMES)

    policy_slot_of = {name: i for i, name in enumerate(DOGURDF_JOINT_NAMES)}
    policy_to_mjx = tuple(policy_slot_of[name] for name in DOGURDF_MJX_JOINT_NAMES)
    return mjx_to_policy, policy_to_mjx


def build_dogurdf_spec(project_root: Path) -> RobotSpec:
    """Build the 16-DoF dogurdf wheel-legged robot spec."""
    default_joint_angles = {}
    for leg in LEG_PREFIXES:
        default_joint_angles[f"{leg}_hip_joint"] = NOMINAL_HIP
        default_joint_angles[f"{leg}_thigh_joint"] = NOMINAL_THIGH
        default_joint_angles[f"{leg}_calf_joint"] = NOMINAL_CALF
        default_joint_angles[f"{leg}_wheel_joint"] = 0.0

    joint_lower = tuple(
        JOINT_LIMITS[j][0] for _ in LEG_PREFIXES for j in ("hip", "thigh", "calf")
    ) + (-WHEEL_LIMIT,) * NUM_WHEELS
    joint_upper = tuple(
        JOINT_LIMITS[j][1] for _ in LEG_PREFIXES for j in ("hip", "thigh", "calf")
    ) + (WHEEL_LIMIT,) * NUM_WHEELS

    mjx_to_policy_idx, policy_to_mjx_idx = _policy_to_mjx_permutation()

    return RobotSpec(
        name="dogurdf",
        asset_path=project_root / "assets/bot_model/dogurdf/dogurdf.xml",
        num_dof=NUM_JOINTS,
        num_actions=NUM_JOINTS,
        joint_names=DOGURDF_JOINT_NAMES,
        default_joint_angles=default_joint_angles,
        joint_lower=joint_lower,
        joint_upper=joint_upper,
        torque_limits=(LEG_TORQUE_LIMIT,) * NUM_LEG_JOINTS
        + (WHEEL_TORQUE_LIMIT,) * NUM_WHEELS,
        body_groups=BodyGroups(
            # The wheels are this robot's ground-contact bodies, so they take
            # the "feet" role throughout the reward / contact machinery.
            feet=tuple(f"{leg}_wheel_link" for leg in LEG_PREFIXES),
            penalized_contacts=("torso",)
            + tuple(
                f"{leg}_{part}_link"
                for leg in LEG_PREFIXES
                for part in ("hip", "thigh", "calf", "wheel")
            ),
            nonfoot_contacts=("torso",)
            + tuple(
                f"{leg}_{part}_link"
                for leg in LEG_PREFIXES
                for part in ("hip", "thigh", "calf")
            ),
        ),
        mjx_to_policy_idx=mjx_to_policy_idx,
        policy_to_mjx_idx=policy_to_mjx_idx,
    )


__all__ = [
    "DOGURDF_JOINT_NAMES",
    "DOGURDF_LEG_JOINT_NAMES",
    "DOGURDF_MJX_JOINT_NAMES",
    "DOGURDF_WHEEL_JOINT_NAMES",
    "LEG_PREFIXES",
    "NOMINAL_CALF",
    "NOMINAL_HIP",
    "NOMINAL_THIGH",
    "NOMINAL_TORSO_HEIGHT",
    "NUM_JOINTS",
    "NUM_LEG_JOINTS",
    "NUM_WHEELS",
    "WHEEL_RADIUS",
    "WHEEL_VEL_LIMIT",
    "build_dogurdf_spec",
]
