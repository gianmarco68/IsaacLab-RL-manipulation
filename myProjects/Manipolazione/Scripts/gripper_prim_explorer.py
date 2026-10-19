# Copyright (c) 2022-2026, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Diagnostic: list prims under the gripper body with local transform and world bbox.

Helps pick a TCP/camera-mount candidate prim, and measures the offset between
right_fixed/left_fixed and the actual fingertip contact surface.
"""

import argparse
import contextlib
import sys

import isaaclab_tasks  # noqa: F401

with contextlib.suppress(ImportError):
    import isaaclab_tasks_experimental  # noqa: F401
from isaaclab_tasks.utils import add_launcher_args, launch_simulation, resolve_task_config, setup_preset_cli

parser = argparse.ArgumentParser(description="Explore gripper prim hierarchy.")
parser.add_argument("--task", type=str, default=None, help="Name of the task.")
add_launcher_args(parser)
parser.set_defaults(visualizer=[])
args_cli, hydra_args = setup_preset_cli(parser)
sys.argv = [sys.argv[0]] + hydra_args

import Training.tasks  # noqa: F401

GRIPPER_ROOT = "/World/m710id_50m/gripper"


def main():
    env_cfg, _ = resolve_task_config(args_cli.task, "")

    with launch_simulation(env_cfg, args_cli):
        from pxr import Usd, UsdGeom

        stage = Usd.Stage.Open(
            "C:\\Projects\\myProjects\\Manipolazione\\Apps\\Simulation\\robotApps.usd"
        )
        gripper_prim = stage.GetPrimAtPath(GRIPPER_ROOT)
        if not gripper_prim.IsValid():
            print(f"[ERROR] prim not found: {GRIPPER_ROOT}")
            return

        cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_], useExtentsHint=True)

        print(f"\n[GRIPPER PRIM EXPLORER] under {GRIPPER_ROOT}:")
        for prim in Usd.PrimRange(gripper_prim):
            xformable = UsdGeom.Xformable(prim)
            local_pos = "n/a"
            if xformable:
                local_transform = xformable.GetLocalTransformation()
                t = local_transform.ExtractTranslation()
                local_pos = f"({t[0]:.4f}, {t[1]:.4f}, {t[2]:.4f})"

            bbox = cache.ComputeWorldBound(prim)
            box_range = bbox.ComputeAlignedRange()
            if not box_range.IsEmpty():
                bmin, bmax = box_range.GetMin(), box_range.GetMax()
                bbox_str = f"min=({bmin[0]:.3f},{bmin[1]:.3f},{bmin[2]:.3f}) max=({bmax[0]:.3f},{bmax[1]:.3f},{bmax[2]:.3f})"
            else:
                bbox_str = "empty"

            depth = prim.GetPath().pathString.count("/") - GRIPPER_ROOT.count("/")
            print(f"  {'  ' * depth}{prim.GetName():25s} local_pos={local_pos:25s} world_bbox: {bbox_str}")


if __name__ == "__main__":
    main()
