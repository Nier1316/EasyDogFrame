"""Shared MLP actor-critic networks."""

from __future__ import annotations

from typing import Tuple

import flax.linen as nn
import jax.numpy as jnp
import numpy as np


class ActorCriticMLP(nn.Module):
    """Pure MLP asymmetric Actor-Critic."""

    action_size: int
    actor_hidden_dims: Tuple[int, ...] = (512, 256, 128)
    critic_hidden_dims: Tuple[int, ...] = (512, 256, 128)
    init_noise_std: float = 1.0
    activation: str = "elu"

    def setup(self):
        self.actor_mlp = [
            nn.Dense(dim, kernel_init=nn.initializers.orthogonal(scale=np.sqrt(2.0)))
            for dim in self.actor_hidden_dims
        ]
        self.actor_out = nn.Dense(
            self.action_size,
            kernel_init=nn.initializers.orthogonal(scale=0.01),
        )
        self.critic_mlp = [
            nn.Dense(dim, kernel_init=nn.initializers.orthogonal(scale=np.sqrt(2.0)))
            for dim in self.critic_hidden_dims
        ]
        self.critic_out = nn.Dense(1, kernel_init=nn.initializers.orthogonal(scale=0.01))
        self.std = self.param(
            "std",
            nn.initializers.constant(self.init_noise_std),
            (self.action_size,),
        )

    def _get_activation(self):
        if self.activation == "tanh":
            return nn.tanh
        if self.activation == "relu":
            return nn.relu
        return nn.elu

    def __call__(self, actor_obs, critic_obs):
        act_fn = self._get_activation()
        x = actor_obs
        for layer in self.actor_mlp:
            x = act_fn(layer(x))
        action_mean = self.actor_out(x)
        action_std = jnp.abs(self.std) + 1e-8

        x = critic_obs
        for layer in self.critic_mlp:
            x = act_fn(layer(x))
        value = self.critic_out(x).squeeze(-1)
        return action_mean, action_std, value

    def act(self, actor_obs):
        act_fn = self._get_activation()
        x = actor_obs
        for layer in self.actor_mlp:
            x = act_fn(layer(x))
        return self.actor_out(x)
