"""dogurdf stand-up (recovery) experiment."""

from __future__ import annotations

from pathlib import Path

from cfg.dogurdf_config import get_dogurdf_standup_config, get_dogurdf_ppo_config
from robots import build_dogurdf_spec
from tasks import build_velocity_task

from .base import ExperimentSpec


def get_experiment(project_root: Path | None = None) -> ExperimentSpec:
    if project_root is None:
        project_root = Path(__file__).resolve().parents[3]

    return ExperimentSpec(
        name="standup",
        robot=build_dogurdf_spec(project_root),
        task=build_velocity_task(),
        env_config_builder=get_dogurdf_standup_config,
        ppo_config_builder=get_dogurdf_ppo_config,
        env_class_path="envs.backends.generic_env:GenericEnv",
        component_name="velocity",
        notes="16-DoF wheel-legged stand-up from a belly-down prone pose (6 s).",
    )
