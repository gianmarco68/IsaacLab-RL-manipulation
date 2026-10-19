# Copyright (c) 2022-2026, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Geometry shared by the reward, observation and event terms of the move_to_grasp task.

A claw is one grasp unit: two fingers, a point between them where they clamp, and optionally a slide
joint carrying it along a guide. Every claw is paired with one grasp target on the workpiece.

Terms subclass ClawGeometry so frame and joint names are resolved to indices once, and so the same
error decomposition and the same gates are used everywhere.
"""

from __future__ import annotations

from typing import TYPE_CHECKING

import torch

from isaaclab.managers import ManagerTermBase, ManagerTermBaseCfg
from isaaclab.utils.math import matrix_from_quat

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedRLEnv

_EPS = 1e-6


def normalize(v: torch.Tensor) -> torch.Tensor:
    """Scale a vector to unit length, leaving a near-zero vector untouched instead of exploding."""
    return v / torch.clamp(torch.linalg.norm(v, dim=-1, keepdim=True), min=_EPS)


def flatten(v: torch.Tensor) -> torch.Tensor:
    """Project a direction onto the horizontal plane and renormalize, giving its top-view direction."""
    return normalize(torch.stack([v[..., 0], v[..., 1], torch.zeros_like(v[..., 2])], dim=-1))


class ClawGeometry(ManagerTermBase):
    """Resolves the gripper description to indices once, then serves errors and gates to every term.

    Frame order in a FrameTransformer follows sorted prim paths, not declaration order, so every
    lookup goes through target_frame_names.index().
    """

    def __init__(self, cfg: ManagerTermBaseCfg, env: ManagerBasedRLEnv):
        super().__init__(cfg, env)
        gripper = env.cfg.gripper
        ee_names = env.scene["ee_frame"].data.target_frame_names
        object_names = env.scene["object_frame"].data.target_frame_names

        self._point_idx = [ee_names.index(name) for name in gripper.grasp_point_frame_names]
        self._ee_idx = ee_names.index("ee")
        self._target_idx = [object_names.index(name) for name in gripper.grasp_target_frame_names]
        self._finger_idx = [
            [ee_names.index(name) for name in pair] for pair in gripper.finger_frame_names
        ]
        self._contact_sensor_names = gripper.contact_sensor_names
        self._guide_axis = torch.tensor(gripper.guide_axis_in_ee, dtype=torch.float32, device=env.device)
        level_axis = torch.tensor(gripper.level_axis_in_ee, dtype=torch.float32, device=env.device)
        self._level_axis = level_axis / torch.linalg.norm(level_axis)
        self._num_claws = len(self._point_idx)

        # joints the policy can move, used to tell whether the gripper is standing still
        robot = env.scene["robot"]
        moving_joints = list(env.cfg.arm.joint_names_expr)
        if gripper.has_positioning_dofs:
            moving_joints += list(gripper.slide_joint_names)
        self._moving_joint_ids, _ = robot.find_joints(moving_joints)

    def _errors(self, env: ManagerBasedRLEnv):
        """Splits each claw's offset from its grasp target into four independent parts: angle between
        guide and side in radians, offset perpendicular to the guide, height difference, offset along
        the guide, all three distances in metres.

        Perpendicular and along together rebuild the horizontal offset exactly, so no part is counted
        twice. Height keeps its sign; the others are magnitudes.
        """
        ee_frame = env.scene["ee_frame"].data
        object_frame = env.scene["object_frame"].data
        ee_rot = matrix_from_quat(ee_frame.target_quat_w.torch[:, self._ee_idx])

        angle, perp, height, along = [], [], [], []
        for claw in range(self._num_claws):
            point_w = ee_frame.target_pos_w.torch[:, self._point_idx[claw]]
            target_w = object_frame.target_pos_w.torch[:, self._target_idx[claw]]
            target_rot = matrix_from_quat(object_frame.target_quat_w.torch[:, self._target_idx[claw]])

            # guide direction rides the ee frame; side direction is the grasp target frame's x axis
            guide_h = flatten(torch.matmul(ee_rot, self._guide_axis[claw].unsqueeze(-1)).squeeze(-1))
            side_h = flatten(target_rot[..., 0])

            delta = target_w - point_w
            delta_h = torch.stack([delta[:, 0], delta[:, 1], torch.zeros_like(delta[:, 2])], dim=-1)
            along_c = torch.sum(delta_h * guide_h, dim=-1)

            # angle between two lines, in radians. atan2 stays accurate near zero, where acos loses
            # precision, and the absolute value makes 0 deg and 180 deg equally parallel
            cross = torch.linalg.norm(torch.linalg.cross(guide_h, side_h, dim=-1), dim=-1)
            dot = torch.abs(torch.sum(guide_h * side_h, dim=-1))
            angle.append(torch.atan2(cross, dot))
            perp.append(torch.linalg.norm(delta_h - along_c.unsqueeze(-1) * guide_h, dim=-1))
            height.append(delta[:, 2])
            along.append(along_c)

        return tuple(torch.stack(t, dim=-1) for t in (angle, perp, height, along))

    def _is_aligned(self, env: ManagerBasedRLEnv, angle_tol: float, perp_tol: float) -> torch.Tensor:
        """Gate reached when every guide lies on its target side, parallel and with no sideways
        offset. Height and position along the guide are not checked here.
        """
        angle, perp, _, _ = self._errors(env)
        return torch.all((angle < angle_tol) & (perp < perp_tol), dim=-1)

    def _is_slide_aligned(
        self, env: ManagerBasedRLEnv, angle_tol: float, perp_tol: float, along_tol: float
    ) -> torch.Tensor:
        """Gate reached when every guide lies on its side and every slide has carried its grasp point
        over its grasp target. Everything matches in top view; only the height gap is left.
        """
        _, _, _, along = self._errors(env)
        return self._is_aligned(env, angle_tol, perp_tol) & torch.all(
            torch.abs(along) < along_tol, dim=-1
        )

    def _is_in_position(
        self,
        env: ManagerBasedRLEnv,
        angle_tol: float,
        perp_tol: float,
        height_tol: float,
        along_tol: float,
    ) -> torch.Tensor:
        """Gate reached when every claw is ready for its fingers to close: guide on its side, grasp
        point at the target height and over the target, target between the two fingers.

        Purely geometric. Standing still is required by the closing event, not here.
        """
        _, _, height, _ = self._errors(env)
        return (
            self._is_slide_aligned(env, angle_tol, perp_tol, along_tol)
            & torch.all(torch.abs(height) < height_tol, dim=-1)
            & torch.all(self._grasp_target_between_fingers(env), dim=-1)
        )

    def _stage(self, env: ManagerBasedRLEnv) -> torch.Tensor:
        """Stage the action terms are enforcing, zero until they have run for the first time.

        Gating a reward on the stage rather than on a fresh tolerance check is what keeps it lit: the
        stage is latched, so the term cannot switch off every time the pose wobbles out of a
        centimetre-wide window it is no longer allowed to correct.
        """
        tracker = getattr(env, "_move_to_grasp_stages", None)
        if tracker is None:
            return torch.zeros(env.num_envs, dtype=torch.long, device=env.device)
        return tracker.stage

    def _ready_to_clamp(self, env: ManagerBasedRLEnv, height_tol: float) -> torch.Tensor:
        """The descent has been reached, the grasp points are at target height, and each target sits
        between its two fingers. The geometric part of the gate that the stage does not already cover.
        """
        _, _, height, _ = self._errors(env)
        return (
            (self._stage(env) >= 2)
            & torch.all(torch.abs(height) < height_tol, dim=-1)
            & torch.all(self._grasp_target_between_fingers(env), dim=-1)
        )

    def is_aligned(self, env: ManagerBasedRLEnv, angle_tol: float, perp_tol: float) -> torch.Tensor:
        """The first gate, exposed for the stage machine: every guide lies on its target side."""
        return self._is_aligned(env, angle_tol, perp_tol)

    def is_ready_to_descend(
        self, env: ManagerBasedRLEnv, angle_tol: float, perp_tol: float, along_tol: float
    ) -> torch.Tensor:
        """The slide gate, exposed for the action term: everything matches in top view, only the
        height gap is left. Sharing it keeps the staged reward and the staged action in step.
        """
        return self._is_slide_aligned(env, angle_tol, perp_tol, along_tol)

    def _tilt(self, env: ManagerBasedRLEnv) -> torch.Tensor:
        """Angle in radians between the gripper's level axis and the world vertical: zero when the
        plane the guides lie in is horizontal, whichever way round the axis points.

        The error decomposition works on top-view projections and cannot see this, so tilt has to be
        measured on its own.
        """
        ee_rot = matrix_from_quat(env.scene["ee_frame"].data.target_quat_w.torch[:, self._ee_idx])
        axis_w = torch.matmul(ee_rot, self._level_axis.unsqueeze(-1)).squeeze(-1)
        return torch.atan2(torch.linalg.norm(axis_w[:, :2], dim=-1), torch.abs(axis_w[:, 2]))

    def is_still(self, env: ManagerBasedRLEnv, vel_tol: float) -> torch.Tensor:
        """True where every joint the policy drives has stopped, which means the gripper has too."""
        robot = env.scene["robot"]
        speed = torch.abs(robot.data.joint_vel.torch[:, self._moving_joint_ids])
        return torch.all(speed < vel_tol, dim=-1)

    def _grasp_target_between_fingers(self, env: ManagerBasedRLEnv) -> torch.Tensor:
        """True where the grasp target sits between a claw's two fingers, measured along the line
        joining them.
        """
        ee_frame = env.scene["ee_frame"].data
        object_frame = env.scene["object_frame"].data
        between = []
        for claw in range(self._num_claws):
            target_w = object_frame.target_pos_w.torch[:, self._target_idx[claw]]
            thumb_w = ee_frame.target_pos_w.torch[:, self._finger_idx[claw][0]]
            fore_w = ee_frame.target_pos_w.torch[:, self._finger_idx[claw][1]]
            finger_axis = normalize(thumb_w - fore_w)
            # opposite signs mean the point sits between the two fingers
            from_thumb = torch.sum((target_w - thumb_w) * finger_axis, dim=-1)
            from_fore = torch.sum((target_w - fore_w) * finger_axis, dim=-1)
            between.append((from_thumb * from_fore) < 0.0)
        return torch.stack(between, dim=-1)

    def _finger_touches(self, env: ManagerBasedRLEnv, sensor_name: str, threshold: float) -> torch.Tensor:
        """True where the named finger sensor pushes on the filtered workpiece harder than threshold."""
        force = env.scene[sensor_name].data.force_matrix_w.torch
        return torch.linalg.norm(force, dim=-1).flatten(1).max(dim=1).values > threshold
