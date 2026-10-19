# Copyright (c) 2022-2026, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Termination terms for the move_to_grasp task.

Both end episodes that can no longer lead anywhere, so the failure shows up in the episode length
instead of costing the full five seconds in silence.
"""

from __future__ import annotations

from typing import TYPE_CHECKING

import torch

from .claw_geometry import ClawGeometry

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedRLEnv


class descended_before_aligned(ClawGeometry):
    """Ends the episode when a grasp point has sunk past its target without being aligned over it.

    At that point the claw is beside the workpiece rather than around it, and the fingers can only
    push it away.
    """

    def __call__(self, env: ManagerBasedRLEnv, depth_tol: float) -> torch.Tensor:
        _, _, height, _ = self._errors(env)
        # height is target minus grasp point, so a positive value means the point is below the target
        too_low = torch.any(height > depth_tol, dim=-1)
        return too_low & (self._stage(env) < 2)


class gripper_not_level(ClawGeometry):
    """Ends the episode when the plane the guides lie in has tipped away from horizontal.

    The action term holds that plane level, so this fires only when something has overpowered the IK.
    """

    def __call__(self, env: ManagerBasedRLEnv, tilt_tol: float) -> torch.Tensor:
        return self._tilt(env) > tilt_tol
