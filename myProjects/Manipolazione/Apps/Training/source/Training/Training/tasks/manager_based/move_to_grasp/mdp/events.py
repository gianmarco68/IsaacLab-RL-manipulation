# Copyright (c) 2022-2026, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Event terms for the move_to_grasp task."""

from __future__ import annotations

from typing import TYPE_CHECKING

import torch

from isaaclab.managers import ManagerTermBaseCfg

from .claw_geometry import ClawGeometry

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedRLEnv


class close_fingers_when_in_position(ClawGeometry):
    """Closes the fingers once the gripper has held the clamping pose still for long enough.

    The policy never commands the fingers: closing is how the environment checks that the pose it
    reached is actually a grasp, so success depends only on where the gripper stopped.

    Once triggered the fingers stay closed until the episode resets, and the joint targets survive
    because no action term writes to those joints.
    """

    def __init__(self, cfg: ManagerTermBaseCfg, env: ManagerBasedRLEnv):
        super().__init__(cfg, env)
        gripper = env.cfg.gripper
        robot = env.scene["robot"]
        flat_names = [name for pair in gripper.finger_joint_names for name in pair]
        flat_closed = [value for pair in gripper.closed_finger_pos for value in pair]
        self._finger_joint_ids, _ = robot.find_joints(flat_names, preserve_order=True)
        self._closed_pos = torch.tensor(flat_closed, dtype=torch.float32, device=env.device)
        self._steps_held = torch.zeros(env.num_envs, dtype=torch.long, device=env.device)
        self._closed = torch.zeros(env.num_envs, dtype=torch.bool, device=env.device)

    def reset(self, env_ids: torch.Tensor):
        self._steps_held[env_ids] = 0
        self._closed[env_ids] = False

    def __call__(
        self,
        env: ManagerBasedRLEnv,
        env_ids: torch.Tensor | None,
        height_tol: float,
        vel_tol: float,
        hold_steps: int,
    ):
        ready = self._ready_to_clamp(env, height_tol) & self.is_still(env, vel_tol)
        # the counter restarts whenever the pose is lost, so a lucky fly-through never triggers
        self._steps_held = torch.where(ready, self._steps_held + 1, torch.zeros_like(self._steps_held))

        closing = ready & (self._steps_held >= hold_steps) & ~self._closed
        if torch.any(closing):
            targets = self._closed_pos.unsqueeze(0).expand(int(closing.sum()), -1)
            env.scene["robot"].set_joint_position_target(
                targets, joint_ids=self._finger_joint_ids, env_ids=closing.nonzero().squeeze(-1)
            )
            self._closed |= closing

    @property
    def closed(self) -> torch.Tensor:
        """Which environments have already had their fingers closed."""
        return self._closed
