"""Velocity tracking task specification."""

from __future__ import annotations

from .base import ObservationSpec, TaskSpec


def velocity_observation_spec(
    num_joints: int,
    num_position_joints: int,
    num_feet: int,
    command_dim: int = 3,
) -> ObservationSpec:
    """Observation sizes implied by a robot's joint split.

    Mirrors the layout built in
    ``components/tasks/velocity/observations.py``: position-controlled joints
    contribute position + velocity, velocity-controlled joints (wheels)
    contribute velocity only.
    """
    actor = (
        3  # base linear velocity
        + 3  # base angular velocity
        + 3  # projected gravity
        + num_position_joints  # joint position relative to default
        + num_joints  # joint velocity (all joints)
        + num_joints  # last action
        + command_dim
    )
    # Privileged adds foot height / air time / contact flag / contact forces.
    critic = actor + num_feet * 3 + num_feet * 3
    odom = 2 + num_position_joints + num_joints + 3
    return ObservationSpec(actor_size=actor, critic_size=critic, odom_size=odom)


def build_velocity_task(
    num_joints: int = 16,
    num_position_joints: int = 12,
    num_feet: int = 4,
) -> TaskSpec:
    """Build the velocity task spec (defaults to the wheel-legged robot)."""
    return TaskSpec(
        name="velocity",
        command_dim=3,
        observation=velocity_observation_spec(
            num_joints, num_position_joints, num_feet,
        ),
        reward_names=(
            "tracking_lin_vel",
            "tracking_ang_vel",
            "lateral_vel",
            "yaw_rate_error",
            "upright",
            "pose_hipx",
            "pose_hipy",
            "pose_knee",
            "joint_mirror",
            "bad_orientation",
            "base_height",
            "dof_pos_limits",
            "action_rate",
            "feet_air_time",
            "foot_clearance",
            "foot_swing_height",
            "slip",
            "soft_landing",
        ),
        termination_names=("fell_over", "illegal_contact", "nan", "timeout"),
    )
