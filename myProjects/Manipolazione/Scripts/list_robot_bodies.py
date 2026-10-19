# Copyright (c) 2022-2026, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Diagnostic: print the robot articulation's actual rigid-body link names and their world pose."""

import argparse
import contextlib
import sys

import gymnasium as gym

import isaaclab_tasks  # noqa: F401

with contextlib.suppress(ImportError):
    import isaaclab_tasks_experimental  # noqa: F401
from isaaclab_tasks.utils import add_launcher_args, launch_simulation, resolve_task_config, setup_preset_cli

parser = argparse.ArgumentParser(description="List robot articulation body names.")
parser.add_argument("--task", type=str, default=None, help="Name of the task.")
add_launcher_args(parser)
parser.set_defaults(visualizer=[])
args_cli, hydra_args = setup_preset_cli(parser)
sys.argv = [sys.argv[0]] + hydra_args

import Training.tasks  # noqa: F401


def main():
    env_cfg, _ = resolve_task_config(args_cli.task, "")

    with launch_simulation(env_cfg, args_cli):
        env_cfg.scene.num_envs = 1
        env = gym.make(args_cli.task, cfg=env_cfg)
        env.reset()

        robot = env.unwrapped.scene["robot"]
        pos_w = robot.data.body_pos_w.torch[0]

        print("\n[ROBOT BODIES]")
        for i, name in enumerate(robot.data.body_names):
            p = pos_w[i]
            print(f"  {i:2d}  {name:30s} pos_w=({p[0]:.4f}, {p[1]:.4f}, {p[2]:.4f})")

        env.close()


if __name__ == "__main__":
    main()
