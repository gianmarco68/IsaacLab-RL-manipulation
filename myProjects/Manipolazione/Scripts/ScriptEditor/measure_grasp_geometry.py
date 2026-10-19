"""Measure every geometric quantity the staged grasp reward needs.

Paste into the Isaac Sim Script Editor of the standalone install and run it TWICE, with the simulation
PLAYING: once with the robot in the pre-grasp pose, once in the pose where the staffa is grasped.

Reads through isaacsim.core.experimental.prims, whose backend is fabric while playing: a direct pxr
query would return the authored default pose, not the live one.

Quaternions from get_world_poses() are wxyz; Isaac Lab uses xyzw.
"""

import numpy as np
from isaacsim.core.experimental.prims import Articulation, XformPrim

# ---------------------------------------------------------------- config
ROBOT_PATH = "/World/m710id_50m"
GRIPPER = ROBOT_PATH + "/gripper/gripper_body/components"
FINGERS = {
    "right_thumb": GRIPPER + "/right_claw/right_thumb",
    "right_forefinger": GRIPPER + "/right_claw/right_forefinger",
    "left_thumb": GRIPPER + "/left_claw/rotor/left_thumb",
    "left_forefinger": GRIPPER + "/left_claw/rotor/left_forefinger",
}
STAFFA_PATH = "/World/staffa_quadrata"
HALF_SIDE = 0.225
HALF_HEIGHT = 0.0813
OUT = "C:/Projects/myProjects/Manipolazione/Scripts/logs/grasp_geometry_output.txt"

lines = []


def say(text=""):
    print(text)
    lines.append(text)


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


# ---------------------------------------------------------------- joints
robot = Articulation(ROBOT_PATH)
dof_pos = np.array(robot.get_dof_positions().numpy()[0], dtype=float)
say("[JOINTS]")
for name, value in zip(robot.dof_names, dof_pos):
    say(f"  {name:<20} {value:+.6f}   ({np.degrees(value):+8.3f} deg)")

# ---------------------------------------------------------------- poses
staffa_pos, staffa_quat = world_pose(STAFFA_PATH)
staffa_rot = quat_to_matrix(staffa_quat)
say()
say("[STAFFA]")
say(f"  pos_w  = {np.round(staffa_pos, 6).tolist()}")
say(f"  quat_w = {np.round(staffa_quat, 6).tolist()}   (wxyz)")

finger_pos, finger_rot = {}, {}
say()
say("[FINGERS]  world pose, and position in the staffa frame")
for name, path in FINGERS.items():
    pos, quat = world_pose(path)
    finger_pos[name] = pos
    finger_rot[name] = quat_to_matrix(quat)
    local = staffa_rot.T @ (pos - staffa_pos)
    say(f"  {name:<18} pos_w={np.round(pos, 6).tolist()}")
    say(f"  {'':<18} quat_w={np.round(quat, 6).tolist()}")
    say(f"  {'':<18} in staffa frame={np.round(local, 6).tolist()}")

# ---------------------------------------------------------------- per claw
FACES = {"+x": (0, +1), "-x": (0, -1), "+y": (1, +1), "-y": (1, -1)}

say()
say("[CLAWS]")
for claw, (thumb, forefinger) in {
    "right": ("right_thumb", "right_forefinger"),
    "left": ("left_thumb", "left_forefinger"),
}.items():
    thumb_pos, fore_pos = finger_pos[thumb], finger_pos[forefinger]
    gap = float(np.linalg.norm(thumb_pos - fore_pos))
    mid = (thumb_pos + fore_pos) / 2.0
    mid_local = staffa_rot.T @ (mid - staffa_pos)

    say(f"  {claw} claw")
    say(f"    finger gap          = {gap:.6f}   <- bar thickness, at the grasped pose")
    say(f"    grip midpoint local = {np.round(mid_local, 6).tolist()}")

    # nearest face of the square ring, and the nearest point on it
    best = None
    for face, (axis, sign) in FACES.items():
        point = mid_local.copy()
        point[axis] = sign * HALF_SIDE
        other = 1 - axis
        point[other] = np.clip(mid_local[other], -HALF_SIDE, HALF_SIDE)
        point[2] = np.clip(mid_local[2], -HALF_HEIGHT, HALF_HEIGHT)
        distance = float(np.linalg.norm(point - mid_local))
        if best is None or distance < best[2]:
            best = (face, point, distance)
    face, grasp_local, distance = best
    grasp_w = staffa_rot @ grasp_local + staffa_pos
    say(f"    nearest face        = {face}, distance {distance:.6f}")
    say(f"    grasp point local   = {np.round(grasp_local, 6).tolist()}  -> face offset {grasp_local[1 - FACES[face][0]]:+.6f}")

    # OffsetCfg that moves the whole claw onto the grasp point. Shifting by (grasp - midpoint),
    # the same world vector for both fingers, keeps them separated: mapping both onto the grasp
    # point itself would collapse them and make finger_axis degenerate.
    shift = grasp_w - mid
    for finger in (thumb, forefinger):
        offset = finger_rot[finger].T @ shift
        say(f"    OffsetCfg {finger:<18} pos=({offset[0]:+.6f}, {offset[1]:+.6f}, {offset[2]:+.6f})")

with open(OUT, "w", encoding="utf-8") as handle:
    handle.write("\n".join(lines) + "\n")
say()
say(f"written to {OUT}")
