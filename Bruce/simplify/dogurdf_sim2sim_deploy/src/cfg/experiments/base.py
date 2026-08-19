"""Experiment descriptors used by the environment and trainer factories."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Callable

from ml_collections import config_dict

from robots import RobotSpec
from tasks import TaskSpec


ConfigBuilder = Callable[[], config_dict.ConfigDict]
PpoConfigBuilder = Callable[[], config_dict.ConfigDict]
RobotSpecBuilder = Callable[[], RobotSpec]
TaskSpecBuilder = Callable[[], TaskSpec]


@dataclass(frozen=True)
class ExperimentSpec:
    name: str
    robot: RobotSpec
    task: TaskSpec
    env_config_builder: ConfigBuilder
    ppo_config_builder: PpoConfigBuilder
    env_class_path: str
    component_name: str = "velocity"
    notes: str = ""

    def build_env_config(self) -> config_dict.ConfigDict:
        return self.env_config_builder()

    def build_ppo_config(self) -> config_dict.ConfigDict:
        return self.ppo_config_builder()
