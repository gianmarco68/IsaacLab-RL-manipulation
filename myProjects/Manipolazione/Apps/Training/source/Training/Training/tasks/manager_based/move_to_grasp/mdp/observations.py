# Copyright (c) 2022-2026, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Observation terms for the move_to_grasp task.

The finger joints are already covered by the generic joint observations, so only the quantities the
policy cannot infer from joint angles alone are added here.
"""

from __future__ import annotations

from typing import TYPE_CHECKING

import torch

from .claw_geometry import ClawGeometry
from .stages import NUM_STAGES

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedRLEnv


class grasp_to_target_error(ClawGeometry):
    """Reports, for every claw, how far it still is from the spot it must clamp.

    Four numbers per claw in the order the motion resolves them: guide-to-side angle, sideways offset,
    height difference, offset along the guide. Height keeps its sign, so the policy can tell whether
    it has to go down or back up.
    """

    def __call__(self, env: ManagerBasedRLEnv) -> torch.Tensor:
        angle, perp, height, along = self._errors(env)
        return torch.cat([angle, perp, height, along], dim=-1)


def task_stage(env: ManagerBasedRLEnv) -> torch.Tensor:
    """Reports which stage the environment is in, one-hot.

    Which axes the policy can actually move changes with the stage, so without this the same
    observation would stand for different consequences.

    The stages are owned by the action terms, which have not run yet the first time observations are
    read, so a missing tracker means everyone is still in the first stage.
    """
    tracker = getattr(env, "_move_to_grasp_stages", None)
    if tracker is None:
        return torch.nn.functional.one_hot(
            torch.zeros(env.num_envs, dtype=torch.long, device=env.device), num_classes=NUM_STAGES
        ).float()
    return torch.nn.functional.one_hot(tracker.stage, num_classes=NUM_STAGES).float()


class gripper_tilt(ClawGeometry):
    """Reports how far the plane of the guides has tipped away from horizontal, in radians.

    The four errors above are top-view projections and stay the same whatever the tilt, so without
    this the policy has no way to see it.
    """

    def __call__(self, env: ManagerBasedRLEnv) -> torch.Tensor:
        return self._tilt(env).unsqueeze(-1)


class finger_contact_forces(ClawGeometry):
    """Reports how hard each finger is pressing on the workpiece, one number per finger.

    Without it the contact reward would be invisible to the policy, which could not tell a grasp from
    a near miss.
    """

    def __call__(self, env: ManagerBasedRLEnv) -> torch.Tensor:
        forces = [
            torch.linalg.norm(env.scene[name].data.force_matrix_w.torch, dim=-1).flatten(1).max(dim=1).values
            for pair in self._contact_sensor_names
            for name in pair
        ]
        return torch.stack(forces, dim=-1)
