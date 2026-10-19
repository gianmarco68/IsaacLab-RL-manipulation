"""Raise the measured grasp offsets by a fixed height and print them ready to paste.

Paste into the Script Editor with the simulation PLAYING and the robot still at the grasp pose the
offsets below were measured at. No contact reading is involved: only the orientation of the claw
bodies and of the workpiece is needed.

The offsets live in three different frames. In the workpiece frame the local z is the vertical, so
the shift adds straight to it. In a claw frame and in the flange frame it does not, so the world
shift is rotated into each of those frames first.
"""

import os

import numpy as np
import omni.usd
from pxr import UsdPhysics
from isaacsim.core.experimental.prims import Articulation, XformPrim

OUT = "C:/Projects/myProjects/Manipolazione/Scripts/logs/shift_grasp_offsets_up.txt"

ROBOT_PATH = "/World/m710id_50m"
STAFFA_BODY_PATH = "/World/staffa_quadrata/structure"
FLANGE_NAME = "J6_link"
SLIDE_JOINTS = {"right": "RightClawSlide", "left": "LeftClawSlide"}

# how far up the workpiece's own vertical the reference point moves
SHIFT_M = 0.033

# measured at the grasp pose, from the contact points
MEASURED = {
    "right_grasp_point": (0.001667, 0.000120, 0.226791),
    "left_grasp_point": (0.004914, -0.007540, 0.272276),
    "right_grasp_target": (-0.008267, -0.217419, 0.050539),
    "left_grasp_target": (-0.216563, -0.006709, 0.052556),
    "ee_offset": (0.509288, -0.054103, -0.056016),
}

TARGET_ROT_IN_OBJECT = {
    "right": (1.0, 0.0, 0.0, 0.0),
    "left": (0.70710678, 0.0, 0.0, 0.70710678),
}


def quat_to_matrix(q):
    """wxyz quaternion -> 3x3 rotation matrix."""
    w, x, y, z = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ])


def world_pose(path):
    pos, quat = XformPrim(path).get_world_poses()
    return np.array(pos.numpy()[0], dtype=float), np.array(quat.numpy()[0], dtype=float)


lines = []


def say(text=""):
    print(text)
    lines.append(text)


def fmt(v, digits=6):
    return "(" + ", ".join(f"{x:.{digits}f}" for x in np.asarray(v, dtype=float).ravel()) + ")"


stage = omni.usd.get_context().get_stage()
robot = Articulation(ROBOT_PATH)
link_path = dict(zip(robot.link_names, robot.link_paths[0]))

claw_body = {}
for prim in stage.Traverse():
    if not prim.IsA(UsdPhysics.PrismaticJoint):
        continue
    name = prim.GetName()
    side = next((s for s, j in SLIDE_JOINTS.items() if j == name), None)
    if side is None:
        continue
    targets = UsdPhysics.PrismaticJoint(prim).GetBody1Rel().GetTargets()
    claw_body[side] = targets[0].pathString if targets else None

_, staffa_quat = world_pose(STAFFA_BODY_PATH)
staffa_rot = quat_to_matrix(staffa_quat)
# the workpiece's own up direction, so the shift stays correct even if the ring is not level
shift_world = staffa_rot @ np.array([0.0, 0.0, SHIFT_M])

_, flange_quat = world_pose(link_path[FLANGE_NAME])
flange_rot = quat_to_matrix(flange_quat)

say("=" * 78)
say(f"[SHIFT]  {SHIFT_M * 1000:.1f} mm along the workpiece vertical")
say("=" * 78)
say(f"  in world coordinates = {fmt(shift_world)}")

shifted = {}
for side in ("right", "left"):
    _, claw_quat = world_pose(claw_body[side])
    in_claw = quat_to_matrix(claw_quat).T @ shift_world
    shifted[side + "_grasp_point"] = np.array(MEASURED[side + "_grasp_point"]) + in_claw
    shifted[side + "_grasp_target"] = np.array(MEASURED[side + "_grasp_target"]) + np.array(
        [0.0, 0.0, SHIFT_M]
    )
    say(f"  in the {side} claw frame = {fmt(in_claw)}")
shifted["ee_offset"] = np.array(MEASURED["ee_offset"]) + flange_rot.T @ shift_world
say(f"  in the flange frame  = {fmt(flange_rot.T @ shift_world)}")

say()
say("=" * 78)
say("[PASTE INTO move_to_grasp_apps_cfg.py]")
say("=" * 78)
say(f"  ee_offset={fmt(shifted['ee_offset'])},")
for side in ("right", "left"):
    say(f"  {side}_grasp_point  OffsetCfg(pos={fmt(shifted[side + '_grasp_point'])})")
for side in ("right", "left"):
    say(f"  {side}_grasp_target OffsetCfg(pos={fmt(shifted[side + '_grasp_target'])},"
        f" rot={fmt(TARGET_ROT_IN_OBJECT[side], 8)})")

os.makedirs(os.path.dirname(OUT), exist_ok=True)
with open(OUT, "w") as handle:
    handle.write("\n".join(lines))
print(f"written to {OUT}")
