# Copyright (c) 2022-2026, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Compute OffsetCfg.pos values for the ee_frame targets, without using the GUI.

Adds temporary FrameTransformer targets on the fingertips/sensor to read their REAL runtime world
pose. A direct pxr.Usd query on the stage returns the static default pose instead of the physics-
simulated one, because Isaac Lab writes runtime poses to Fabric, not back to the classic USD stage.

For each grasp point this converts the desired world-space point into the *local* frame of the
tracked reference prim (right_fixed/left_fixed/sensor itself), since FrameTransformerCfg.OffsetCfg.pos
is "w.r.t. the parent frame" (the tracked prim), not world space.
"""

import argparse
import contextlib
import sys

import gymnasium as gym
import torch

import isaaclab_tasks  # noqa: F401

with contextlib.suppress(ImportError):
    import isaaclab_tasks_experimental  # noqa: F401
from isaaclab.sensors import FrameTransformerCfg, OffsetCfg
from isaaclab_tasks.utils import add_launcher_args, launch_simulation, resolve_task_config, setup_preset_cli

parser = argparse.ArgumentParser(description="Compute ee_frame OffsetCfg.pos values.")
parser.add_argument("--task", type=str, default=None, help="Name of the task.")
add_launcher_args(parser)
parser.set_defaults(visualizer=[])
args_cli, hydra_args = setup_preset_cli(parser)
sys.argv = [sys.argv[0]] + hydra_args

import Training.tasks  # noqa: F401

COMPONENTS = "{ENV_REGEX_NS}/Robot/m710id_50m/gripper/gripper_body/components"

EXTRA_FRAMES = {
    "right_forefinger": f"{COMPONENTS}/right_claw/right_forefinger",
    "right_thumb": f"{COMPONENTS}/right_claw/right_thumb",
    "left_forefinger": f"{COMPONENTS}/left_claw/left_forefinger",
    "left_thumb": f"{COMPONENTS}/left_claw/left_thumb",
    "sensor": f"{COMPONENTS}/sensor",
}


def quat_conjugate(q: torch.Tensor) -> torch.Tensor:
    """(w, x, y, z) -> inverse rotation, for a unit quaternion."""
    return q * torch.tensor([1.0, -1.0, -1.0, -1.0], device=q.device)


def quat_rotate(q: torch.Tensor, v: torch.Tensor) -> torch.Tensor:
    """Rotate vector v (3,) by quaternion q (w, x, y, z)."""
    w, x, y, z = q[0], q[1], q[2], q[3]
    qvec = torch.stack([x, y, z])
    uv = torch.cross(qvec, v)
    uuv = torch.cross(qvec, uv)
    return v + 2.0 * (w * uv + uuv)


def main():
    env_cfg, _ = resolve_task_config(args_cli.task, "")

    with launch_simulation(env_cfg, args_cli):
        env_cfg.scene.num_envs = 1
        # add the fingertip/sensor prims as extra ee_frame targets, offset 0 -- just to read their
        # real runtime world pose the same way right_gripper/left_gripper already are read
        for name, path in EXTRA_FRAMES.items():
            env_cfg.scene.ee_frame.target_frames.append(
                FrameTransformerCfg.FrameCfg(prim_path=path, name=name, offset=OffsetCfg())
            )

        env = gym.make(args_cli.task, cfg=env_cfg)
        env.reset()

        ee_frame = env.unwrapped.scene["ee_frame"]
        names = ee_frame.data.target_frame_names
        pos_w = ee_frame.data.target_pos_w.torch[0]  # (num_frames, 3)
        quat_w = ee_frame.data.target_quat_w.torch[0]  # (num_frames, 4), (w, x, y, z)

        def get(name: str):
            i = names.index(name)
            return pos_w[i], quat_w[i]

        print("\n[EE FRAME OFFSETS]")
        for side in ("right", "left"):
            forefinger_pos, _ = get(f"{side}_forefinger")
            thumb_pos, _ = get(f"{side}_thumb")
            grasp_point_world = (forefinger_pos + thumb_pos) / 2.0
            fixed_pos, fixed_quat = get(f"{side}_gripper")
            local_offset = quat_rotate(quat_conjugate(fixed_quat), grasp_point_world - fixed_pos)
            print(
                f"  {side}_gripper: grasp_point_world={grasp_point_world.tolist()}"
                f"  offset(local)={local_offset.tolist()}"
            )

        sensor_pos, sensor_quat = get("sensor")
        # offset from sensor onto itself is (0,0,0) by construction -- sensor bbox center would need
        # its own bbox measurement; use its own origin as tcp reference point (offset 0) for now
        print(f"  tcp (sensor): pos_w={sensor_pos.tolist()}  offset(local)=(0.0, 0.0, 0.0)")

        env.close()


if __name__ == "__main__":
    main()
