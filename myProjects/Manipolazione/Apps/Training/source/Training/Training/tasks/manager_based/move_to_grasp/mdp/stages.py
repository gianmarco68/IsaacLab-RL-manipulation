# Copyright (c) 2022-2026, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Stage machine decoupling the three movements of the move_to_grasp task.

Gating rewards is not enough to order a motion: a reward worth nothing still leaves the axis free to
move. Here the stage decides which axes the action terms may write at all, so aligning, sliding and
descending cannot overlap and each one stops by construction when its stage ends.
"""

from __future__ import annotations

from typing import TYPE_CHECKING

import torch

from .claw_geometry import ClawGeometry

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedRLEnv

ALIGN, SLIDE, DESCEND = 0, 1, 2
NUM_STAGES = 3


class StageTracker:
    """Per-environment stage, advanced once its gate has held long enough and never moved back.

    Monotonic on purpose: letting a stage lapse would hand the axes back and forth every time noise
    crosses a tolerance, which is the oscillation the stages exist to remove.
    """

    def __init__(self, env: ManagerBasedRLEnv, tolerances: dict, hold_steps: int, vel_tol: float):
        self._geometry = ClawGeometry(None, env)
        self._angle_tol = tolerances["angle_tol"]
        self._perp_tol = tolerances["perp_tol"]
        self._along_tol = tolerances["along_tol"]
        self._hold_steps = hold_steps
        self._vel_tol = vel_tol
        self.stage = torch.zeros(env.num_envs, dtype=torch.long, device=env.device)
        self._held = torch.zeros(env.num_envs, dtype=torch.long, device=env.device)
        self._last_step = -1

    def update(self, env: ManagerBasedRLEnv) -> torch.Tensor:
        """Recomputes the stage once per environment step, and returns it.

        Both action terms ask for it in the same step, and the episode counter tells the two calls
        apart from a genuinely new step.
        """
        step = int(env.common_step_counter)
        if step == self._last_step:
            return self.stage
        self._last_step = step

        # a freshly reset environment starts over: its episode counter is back to zero
        just_reset = env.episode_length_buf == 0
        self.stage = torch.where(just_reset, torch.zeros_like(self.stage), self.stage)
        self._held = torch.where(just_reset, torch.zeros_like(self._held), self._held)

        aligned = self._geometry.is_aligned(env, self._angle_tol, self._perp_tol)
        slid = self._geometry.is_ready_to_descend(env, self._angle_tol, self._perp_tol, self._along_tol)
        # standing still is part of the gate: promotion is irreversible and freezes the axes that
        # could still fix the pose, so it must not happen while the gripper is flying through
        gate_met = (
            torch.where(self.stage == ALIGN, aligned, slid)
            & self._geometry.is_still(env, self._vel_tol)
            & (self.stage < DESCEND)
        )

        self._held = torch.where(gate_met, self._held + 1, torch.zeros_like(self._held))
        advance = self._held >= self._hold_steps
        self.stage = torch.where(advance, self.stage + 1, self.stage)
        self._held = torch.where(advance, torch.zeros_like(self._held), self._held)
        return self.stage


def get_stage_tracker(
    env: ManagerBasedRLEnv, tolerances: dict, hold_steps: int, vel_tol: float
) -> StageTracker:
    """Returns the environment's tracker, building it on first use.

    Both action terms need the same one, and neither can own it: they are built before the sensors
    the tracker reads are initialised.
    """
    tracker = getattr(env, "_move_to_grasp_stages", None)
    if tracker is None:
        tracker = StageTracker(env, tolerances, hold_steps, vel_tol)
        env._move_to_grasp_stages = tracker
    return tracker
