# Copyright (c) 2022-2026, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Action configurations for the move_to_grasp task.

Kept apart from actions.py because environment configurations are built before the simulation app
exists: naming the term as a string leaves its module, which pulls in Omniverse, unimported until the
action manager actually needs it.
"""

from __future__ import annotations

from typing import TYPE_CHECKING

from isaaclab.envs.mdp.actions.actions_cfg import (
    DifferentialInverseKinematicsActionCfg,
    JointPositionActionCfg,
)
from isaaclab.utils.configclass import configclass

if TYPE_CHECKING:
    from .actions import StagedJointPositionAction, StagedLevelIKAction


@configclass
class StageGateCfg:
    """What the two staged terms need to agree on to see the same stage. Identical on both."""

    stage_gate: dict = {}
    """Tolerances advancing the stage: angle_tol and perp_tol open the slide, along_tol the descent."""

    stage_hold_steps: int = 5
    """Steps the current stage's tolerances must hold before the next stage takes over."""

    stage_vel_tol: float = 0.05
    """Joint speed below which the gripper counts as stopped, in rad/s. Promotion needs it too."""


@configclass
class StagedLevelIKActionCfg(DifferentialInverseKinematicsActionCfg, StageGateCfg):
    """Relative IK action that is levelled, and horizontal in one stage and vertical in another."""

    class_type: type[StagedLevelIKAction] | str = "{DIR}.actions:StagedLevelIKAction"


@configclass
class StagedJointPositionActionCfg(JointPositionActionCfg, StageGateCfg):
    """Joint position action writable only during the sliding stage."""

    class_type: type[StagedJointPositionAction] | str = "{DIR}.actions:StagedJointPositionAction"
