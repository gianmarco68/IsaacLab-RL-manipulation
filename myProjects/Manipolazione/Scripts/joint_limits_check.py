# Copyright (c) 2022-2026, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Diagnostic: print initial joint position vs. hard joint limits for the arm joints.

One-off script to check whether the CAD-imported robot spawns with J1-J6 already at/near
their physical joint limits (would explain instant `joints_out_of_bounds` termination).
"""

import argparse
import contextlib
import sys

import gymnasium as gym
import torch

import isaaclab_tasks  # noqa: F401

with contextlib.suppress(ImportError):
    import isaaclab_tasks_experimental  # noqa: F401
from isaaclab_tasks.utils import (
    add_launcher_args,
    launch_simulation,
    resolve_task_config,
    setup_preset_cli,
)

parser = argparse.ArgumentParser(description="Print joint limits vs. initial pose for the arm.")
parser.add_argument("--num_envs", type=int, default=1, help="Number of environments to simulate.")
parser.add_argument("--task", type=str, default=None, help="Name of the task.")
add_launcher_args(parser)
parser.set_defaults(visualizer=[])
args_cli, hydra_args = setup_preset_cli(parser)
sys.argv = [sys.argv[0]] + hydra_args

import Training.tasks  # noqa: F401

ARM_JOINTS = [
    "J1", "J2", "J3", "J4", "J5", "J6",
    "RightClawSlide", "LeftClawSlide",
    "RightThumb", "RightForefinger", "LeftThumb", "LeftForefinger",
    "LeftClawRevolution",
]


def main():
    env_cfg, _ = resolve_task_config(args_cli.task, "")

    with launch_simulation(env_cfg, args_cli):
        env_cfg.scene.num_envs = args_cli.num_envs
        env = gym.make(args_cli.task, cfg=env_cfg)
        env.reset()

        robot = env.unwrapped.scene["robot"]
        joint_ids, joint_names = robot.find_joints(ARM_JOINTS)

        joint_pos = robot.data.joint_pos[0, joint_ids]
        limits = robot.data.joint_pos_limits[0, joint_ids]
        soft_limits = robot.data.soft_joint_pos_limits[0, joint_ids]

        print("\n[JOINT LIMITS CHECK] env 0, arm joints:")
        for name, pos, lim, soft in zip(joint_names, joint_pos, limits, soft_limits):
            lower, upper = lim[0].item(), lim[1].item()
            soft_lower, soft_upper = soft[0].item(), soft[1].item()
            p = pos.item()
            margin_lower = p - soft_lower
            margin_upper = soft_upper - p
            print(
                f"  {name:20s} pos={p:9.4f}  hard=[{lower:9.4f}, {upper:9.4f}]  "
                f"soft=[{soft_lower:9.4f}, {soft_upper:9.4f}]  "
                f"margin=[{margin_lower:9.4f}, {margin_upper:9.4f}]"
            )

        env.close()


if __name__ == "__main__":
    main()
