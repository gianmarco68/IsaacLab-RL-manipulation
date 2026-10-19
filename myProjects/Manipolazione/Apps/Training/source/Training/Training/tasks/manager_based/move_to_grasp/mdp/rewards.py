# Copyright (c) 2022-2026, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Reward terms for the move_to_grasp task.

The motion is split in stages: line the guides up with the sides of the workpiece, slide each claw
over the spot it must clamp, lower the gripper onto it, then hold still until the fingers close on
it. Each stage is multiplied by a boolean gate built from the previous stage's errors, so a stage
only pays once the one before it is satisfied.
"""

from __future__ import annotations

from typing import TYPE_CHECKING

import torch

from .claw_geometry import ClawGeometry

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedRLEnv


class guides_parallel_to_sides(ClawGeometry):
    """Rewards every guide for pointing along its target side, seen from above.

    Averaged over the guides so the value stays in [0, 1] whatever their number.
    """

    def __call__(self, env: ManagerBasedRLEnv, std: float) -> torch.Tensor:
        angle, _, _, _ = self._errors(env)
        return torch.mean(1.0 - torch.tanh(angle / std), dim=-1)


class guides_over_sides(ClawGeometry):
    """Rewards every guide for lying on top of its target side, seen from above.

    Measures only the offset perpendicular to the guide; the distance along it is covered by the
    slide joint. `std` is a distance in metres, unlike the angular error of guides_parallel_to_sides.
    """

    def __call__(self, env: ManagerBasedRLEnv, std: float) -> torch.Tensor:
        _, perp, _, _ = self._errors(env)
        return torch.mean(1.0 - torch.tanh(perp / std), dim=-1)


class still_when_aligned(ClawGeometry):
    """Rewards stopping the arm, weighted by how close the guides are to lying on their sides.

    The slide stage only opens once the gripper stands still while aligned; without a reason to stop,
    exploration noise keeps the arm moving and the gate is never met. Keeps paying after promotion,
    so advancing a stage never costs reward.
    """

    def __call__(
        self, env: ManagerBasedRLEnv, angle_std: float, perp_std: float, vel_std: float
    ) -> torch.Tensor:
        angle, perp, _, _ = self._errors(env)
        closeness = torch.mean((1.0 - torch.tanh(angle / angle_std)) * (1.0 - torch.tanh(perp / perp_std)), dim=-1)
        speed = torch.abs(env.scene["robot"].data.joint_vel.torch[:, self._moving_joint_ids]).max(dim=-1).values
        return closeness * (1.0 - torch.tanh(speed / vel_std))


class claw_slide_alignment(ClawGeometry):
    """Rewards each slide joint for bringing its grasp point over its grasp target, seen from above.

    Covers only the offset parallel to the guide, the one component a slide can remove; the vertical
    gap stays and is closed by the descent.
    """

    def __call__(self, env: ManagerBasedRLEnv, std: float) -> torch.Tensor:
        _, _, _, along = self._errors(env)
        gate = (self._stage(env) >= 1).float()
        return gate * torch.mean(1.0 - torch.tanh(torch.abs(along) / std), dim=-1)


class grasp_points_at_target_height(ClawGeometry):
    """Rewards lowering every grasp point to the height of its grasp target.

    Pays nothing until everything matches in top view, so the gripper only descends once each claw is
    already above the spot it must clamp; descending earlier drives the fingers into the workpiece.
    """

    def __call__(self, env: ManagerBasedRLEnv, std: float) -> torch.Tensor:
        _, _, height, _ = self._errors(env)
        gate = (self._stage(env) >= 2).float()
        return gate * torch.mean(1.0 - torch.tanh(torch.abs(height) / std), dim=-1)


class align_grasp_around_target(ClawGeometry):
    """Rewards each claw for having its grasp target between its two fingers rather than beside them.

    Shares the descent's gate, since the fingers end up straddling the workpiece as a result of going
    down. All-or-nothing, so it cannot steer the motion towards the condition: it pays for a pose the
    other terms produce, and carries a small weight for that reason.
    """

    def __call__(self, env: ManagerBasedRLEnv) -> torch.Tensor:
        gate = (self._stage(env) >= 2).float()
        return gate * torch.mean(self._grasp_target_between_fingers(env).float(), dim=-1)


class hold_still_at_target(ClawGeometry):
    """Rewards standing still once every claw is in position to clamp.

    Without it nothing would stop the policy from crossing the correct pose at speed; the fingers
    only close after the gripper has held that pose, so lingering there has to pay.
    """

    def __call__(self, env: ManagerBasedRLEnv, vel_std: float, height_tol: float) -> torch.Tensor:
        gate = self._ready_to_clamp(env, height_tol).float()
        robot = env.scene["robot"]
        speed = torch.linalg.norm(robot.data.joint_vel.torch[:, self._moving_joint_ids], dim=-1)
        return gate * (1.0 - torch.tanh(speed / vel_std))


class both_fingers_in_contact(ClawGeometry):
    """Rewards each claw for touching the workpiece with both fingers at once: the task objective.

    Reads the per-partner contact force, so pressing against the ground or against the gripper itself
    does not count. Gated on being in position as well, otherwise knocking the workpiece while
    misaligned would register as a success.
    """

    def __call__(
        self, env: ManagerBasedRLEnv, force_threshold: float, height_tol: float
    ) -> torch.Tensor:
        gate = self._ready_to_clamp(env, height_tol).float()
        touching = [
            self._finger_touches(env, thumb, force_threshold)
            & self._finger_touches(env, forefinger, force_threshold)
            for thumb, forefinger in self._contact_sensor_names
        ]
        return gate * torch.mean(torch.stack(touching, dim=-1).float(), dim=-1)
