# Copyright (c) 2022-2026, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Action terms for the move_to_grasp task.

Both terms read the same stage and write only the axes that stage owns. An axis a stage does not own
is not merely left uncommanded: its target is latched, so it holds its position instead of following
whatever the arm drifts to.
"""

from __future__ import annotations

from collections.abc import Sequence
from typing import TYPE_CHECKING

import torch

from isaaclab.envs.mdp.actions.joint_actions import JointPositionAction
from isaaclab.envs.mdp.actions.task_space_actions import DifferentialInverseKinematicsAction
from isaaclab.utils.math import quat_apply, quat_from_angle_axis, quat_inv, quat_mul

from .stages import ALIGN, DESCEND, SLIDE, get_stage_tracker

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedEnv

    from .actions_cfg import StagedJointPositionActionCfg, StagedLevelIKActionCfg

# farthest the position target may run ahead of the measured tool position, in metres
_MAX_TARGET_LEAD = 0.03


class StagedLevelIKAction(DifferentialInverseKinematicsAction):
    """Relative-pose IK that holds the tool level and owns the horizontal axes, then the vertical one.

    Two things the stock term cannot do. It accumulates every axis onto its own stored target rather
    than onto the measured pose, so a zero command holds position instead of drifting with whatever
    the arm sags or oscillates into; the stock term rebuilds its target from the measured pose every
    step, which makes a zero command mean "keep what you currently have", including any drift. And it
    exposes every axis at once, which lets a policy dive before it has aligned.
    """

    cfg: StagedLevelIKActionCfg

    def __init__(self, cfg: StagedLevelIKActionCfg, env: ManagerBasedEnv):
        super().__init__(cfg, env)
        axis = torch.tensor(env.cfg.gripper.level_axis_in_ee, dtype=torch.float32, device=self.device)
        self._level_axis = axis / torch.linalg.norm(axis)
        # which end of the vertical the tool's level axis points at, frozen on first use
        self._vertical_sign: torch.Tensor | None = None
        self._pos_des = torch.zeros(self.num_envs, 3, device=self.device)
        self._quat_des = torch.zeros(self.num_envs, 4, device=self.device)
        # targets are seeded from the pose itself, which only exists once the simulation is stepping
        self._needs_seed = torch.ones(self.num_envs, dtype=torch.bool, device=self.device)

    def reset(self, env_ids: Sequence[int] | None = None) -> None:
        super().reset(env_ids)
        self._needs_seed[env_ids] = True

    def process_actions(self, actions: torch.Tensor):
        self._raw_actions[:] = actions
        self._processed_actions[:] = self.raw_actions * self._scale
        if self.cfg.clip is not None:
            self._processed_actions = torch.clamp(
                self._processed_actions, min=self._clip[:, :, 0], max=self._clip[:, :, 1]
            )

        stage = self._stage_tracker().update(self._env)
        ee_pos_curr, ee_quat_curr = self._compute_frame_pose()
        up = self._world_up_in_root()

        seed = self._needs_seed.unsqueeze(-1)
        seeded_quat = self._level_quat_from_measured(ee_quat_curr, up, self._processed_actions[:, 5])
        self._pos_des = torch.where(seed, ee_pos_curr, self._pos_des)
        self._quat_des = torch.where(seed, seeded_quat, self._quat_des)
        self._needs_seed[:] = False

        # translating, turning and descending all accumulate onto the stored target rather than onto
        # the measured pose: that is what makes a zero action hold position instead of drifting with
        # whatever the arm sags or oscillates into between one step and the next. Outside its stage
        # an axis is not recomputed at all, so it keeps the target it was left at.
        horizontal = (stage == ALIGN).unsqueeze(-1)
        vertical = (stage == DESCEND).unsqueeze(-1)
        self._pos_des[:, :2] = torch.where(
            horizontal, self._pos_des[:, :2] + self._processed_actions[:, :2], self._pos_des[:, :2]
        )
        self._pos_des[:, 2:3] = torch.where(
            vertical, self._pos_des[:, 2:3] + self._processed_actions[:, 2:3], self._pos_des[:, 2:3]
        )
        # bounded lead over the measured pose, so a gripper blocked by contact never winds up a target
        lead = self._pos_des - ee_pos_curr
        lead_norm = torch.linalg.norm(lead, dim=-1, keepdim=True).clamp(min=1e-9)
        self._pos_des = ee_pos_curr + lead * torch.clamp(_MAX_TARGET_LEAD / lead_norm, max=1.0)
        # rotating a level target about the world vertical leaves it level, so turning never needs to
        # re-measure the pose or re-run the levelling correction
        yaw_quat = quat_from_angle_axis(self._processed_actions[:, 5], up)
        self._quat_des = torch.where(horizontal, quat_mul(yaw_quat, self._quat_des), self._quat_des)

        self._ik_controller.ee_pos_des = self._pos_des
        self._ik_controller.ee_quat_des = self._quat_des

    def _stage_tracker(self):
        """The environment's stage machine, built from this term's gate settings."""
        return get_stage_tracker(
            self._env, self.cfg.stage_gate, self.cfg.stage_hold_steps, self.cfg.stage_vel_tol
        )

    def _world_up_in_root(self) -> torch.Tensor:
        """The world vertical, expressed in the root frame the IK works in."""
        root_quat = self._asset.data.root_quat_w.torch
        world_up = torch.zeros_like(root_quat[:, :3])
        world_up[:, 2] = 1.0
        return quat_apply(quat_inv(root_quat), world_up)

    def _level_quat_from_measured(
        self, ee_quat_curr: torch.Tensor, up: torch.Tensor, yaw_delta: torch.Tensor
    ) -> torch.Tensor:
        """Seeds a fresh target: the tool's level axis put back onto the vertical, then the yaw command.

        Used only to seed a target after reset, when there is no stored one yet to accumulate onto.
        The correction is the shortest rotation onto the vertical, which leaves the rotation about it
        untouched, so levelling never fights the yaw the policy asks for. Quaternions throughout: at
        the wrist pose this arm works in, pitch sits at 89 degrees and euler angles are degenerate.
        """
        axis_now = quat_apply(ee_quat_curr, self._level_axis.expand_as(up))
        if self._vertical_sign is None:
            self._vertical_sign = torch.sign(torch.sum(axis_now * up, dim=-1, keepdim=True))
        goal = self._vertical_sign * up

        cross = torch.linalg.cross(axis_now, goal, dim=-1)
        angle = torch.atan2(torch.linalg.norm(cross, dim=-1), torch.sum(axis_now * goal, dim=-1))
        align = quat_from_angle_axis(angle, cross)
        # yaw turns about the world vertical, not about goal, so a positive command always reads as
        # counter-clockwise from above whichever end of the axis the tool points at
        return quat_mul(quat_from_angle_axis(yaw_delta, up), quat_mul(align, ee_quat_curr))


class StagedJointPositionAction(JointPositionAction):
    """Joint position action the policy may only write during one stage of the task.

    Outside that stage the last target written is re-issued, so the joints hold where they stopped:
    dropping the command instead would send them back to the offset and undo the stage's work.
    """

    cfg: StagedJointPositionActionCfg

    def __init__(self, cfg: StagedJointPositionActionCfg, env: ManagerBasedEnv):
        super().__init__(cfg, env)
        self._held_target = self._default_target()

    def _default_target(self) -> torch.Tensor:
        """Target of a zero action: the joints where the stage found them. Offset may be a scalar."""
        return torch.zeros_like(self._raw_actions) + self._offset

    def reset(self, env_ids: Sequence[int] | None = None) -> None:
        super().reset(env_ids)
        self._held_target[env_ids] = self._default_target()[env_ids]

    def process_actions(self, actions: torch.Tensor):
        super().process_actions(actions)
        stage = get_stage_tracker(
            self._env, self.cfg.stage_gate, self.cfg.stage_hold_steps, self.cfg.stage_vel_tol
        ).update(self._env)
        live = (stage == SLIDE).unsqueeze(-1)
        self._held_target = torch.where(live, self._processed_actions, self._held_target)
        self._processed_actions = self._held_target
