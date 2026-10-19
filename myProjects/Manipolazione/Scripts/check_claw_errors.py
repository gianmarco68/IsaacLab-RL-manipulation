# Copyright (c) 2022-2026, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Diagnostic: print the per-claw error decomposition, to size the three staged reward std values.

Also drives RightClawSlide alone: slide_error must shrink while lateral_error and standoff_error
stay put, which is what catches a wrong sign or order in the approach_axis cross product.
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

parser = argparse.ArgumentParser(description="Print the per-claw error decomposition.")
parser.add_argument("--num_envs", type=int, default=1, help="Number of environments to simulate.")
parser.add_argument("--task", type=str, default="Apps-v0", help="Name of the task.")
add_launcher_args(parser)
parser.set_defaults(visualizer=[])
args_cli, hydra_args = setup_preset_cli(parser)
sys.argv = [sys.argv[0]] + hydra_args

from isaaclab.managers import SceneEntityCfg

import Training.tasks  # noqa: F401

OUT = "C:/Projects/myProjects/Manipolazione/Scripts/logs/claw_errors_output.txt"
lines = []


def say(text=""):
    print(text)
    lines.append(text)


def report(env, rewards, params, label):
    (right_point, right_err), (left_point, left_err) = rewards._both_claw_errors(
        env, params["object_cfg"], params["staffa_half_side"], params["grasp_height"],
        params["right_face_offset"], params["left_face_offset"],
        params["right_claw_cfg"], params["left_claw_cfg"],
    )
    say(f"[{label}]")
    for name, cfg, frames, point, err in (
        ("right", params["right_claw_cfg"], rewards.RIGHT_GRIP_FRAMES, right_point, right_err),
        ("left", params["left_claw_cfg"], rewards.LEFT_GRIP_FRAMES, left_point, left_err),
    ):
        finger_axis, slide_axis, approach_axis, grip_center = rewards._claw_axes(env, cfg, frames)
        lateral, slide, standoff = err
        lateral_norm = torch.linalg.norm(lateral, dim=-1)[0].item()
        say(f"  {name} claw")
        say(f"    grip_center    = {[round(v, 6) for v in grip_center[0].tolist()]}")
        say(f"    grasp point    = {[round(v, 6) for v in point[0].tolist()]}")
        say(f"    finger_axis    = {[round(v, 4) for v in finger_axis[0].tolist()]}")
        say(f"    slide_axis     = {[round(v, 4) for v in slide_axis[0].tolist()]}")
        say(f"    approach_axis  = {[round(v, 4) for v in approach_axis[0].tolist()]}")
        say(f"    lateral_error  = {lateral_norm:+.6f}")
        say(f"    slide_error    = {slide[0].item():+.6f}")
        say(f"    standoff_error = {standoff[0].item():+.6f}")
        total = (lateral_norm**2 + slide[0].item() ** 2 + standoff[0].item() ** 2) ** 0.5
        say(f"    total distance = {total:.6f}")
    return right_err, left_err


def main():
    env_cfg, _ = resolve_task_config(args_cli.task, "")

    with launch_simulation(env_cfg, args_cli):
        env_cfg.scene.num_envs = args_cli.num_envs
        env = gym.make(args_cli.task, cfg=env_cfg)
        env.reset()
        unwrapped = env.unwrapped

        rewards = sys.modules["Training.tasks.manager_based.apps_pick&place_staffe.mdp.rewards"]
        params = dict(unwrapped.reward_manager.get_term_cfg("align_guides_to_sides").params)
        # object_cfg is a signature default, so it never appears in the term params
        params.setdefault("object_cfg", SceneEntityCfg("object"))
        params["object_cfg"].resolve(unwrapped.scene)

        report(unwrapped, rewards, params, "PRE-GRASP")

        # drive RightClawSlide alone: action index 6 is the first of slides_action
        say()
        say("[DRIVING RightClawSlide ALONE]")
        action = torch.zeros((unwrapped.num_envs, unwrapped.action_space.shape[1]), device=unwrapped.device)
        action[:, 6] = 1.0
        for _ in range(60):
            env.step(action)
        report(unwrapped, rewards, params, "AFTER RIGHT SLIDE")

        with open(OUT, "w", encoding="utf-8") as handle:
            handle.write("\n".join(lines) + "\n")
        print(f"\nwritten to {OUT}")
        env.close()


if __name__ == "__main__":
    main()
