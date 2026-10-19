# Copyright (c) 2022-2026, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Diagnostic: print the initial claw-to-object distance to size the grasp_distance reward's std."""

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

parser = argparse.ArgumentParser(description="Print initial claw-to-object distance.")
parser.add_argument("--num_envs", type=int, default=1, help="Number of environments to simulate.")
parser.add_argument("--task", type=str, default=None, help="Name of the task.")
add_launcher_args(parser)
parser.set_defaults(visualizer=[])
args_cli, hydra_args = setup_preset_cli(parser)
sys.argv = [sys.argv[0]] + hydra_args

import Training.tasks  # noqa: F401


def main():
    env_cfg, _ = resolve_task_config(args_cli.task, "")

    with launch_simulation(env_cfg, args_cli):
        env_cfg.scene.num_envs = args_cli.num_envs
        env = gym.make(args_cli.task, cfg=env_cfg)
        env.reset()

        object_ = env.unwrapped.scene["object"]
        ee_frame = env.unwrapped.scene["ee_frame"]

        object_pos = object_.data.root_pos_w[0]
        print("\n[GRASP DISTANCE CHECK] env 0:")
        print(f"  object pos_w        = {object_pos.tolist()}")
        for i, name in enumerate(["right_gripper", "left_gripper"]):
            claw_pos = ee_frame.data.target_pos_w[0, i]
            dist = torch.linalg.norm(claw_pos - object_pos).item()
            print(f"  {name:15s} pos_w = {claw_pos.tolist()}  distance to object = {dist:.4f} m")

        env.close()


if __name__ == "__main__":
    main()
