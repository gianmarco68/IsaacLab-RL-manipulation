"""Measure every robot-specific value the move_to_grasp task needs, and print it ready to paste.

Paste into the Isaac Sim Script Editor of the standalone install, with the simulation PLAYING, and
run it TWICE:

  run 1, robot at the PRE-GRASP pose  -> take the [PASS 1] block
  run 2, robot at the GRASP pose, fingers closed on the ring -> take the [PASS 2] block

Why not the Property panel: while playing, PhysX moves bodies through Fabric and the USD attributes
keep their authored values, so the panel reads zero. These queries go through
isaacsim.core.experimental.prims, whose backend is Fabric during play.

Quaternions from get_world_poses() are wxyz, which is also what Isaac Lab's OffsetCfg expects.
Prim paths are never guessed: links come from the articulation, and the body of a claw is whichever
body its slide joint moves.
"""

import numpy as np
import omni.usd
from pxr import UsdGeom, UsdPhysics
from isaacsim.core.experimental.prims import Articulation, XformPrim

# ---------------------------------------------------------------- scene layout
ROBOT_PATH = "/World/m710id_50m"
STAFFA_PATH = "/World/staffa_quadrata"
# the child prim that actually carries RigidBodyAPI: object_frame is declared on this prim, not on
# the outer root, and the two do not share the same origin or orientation
STAFFA_BODY_PATH = STAFFA_PATH + "/structure"
FLANGE_NAME = "J6_link"
SLIDE_JOINTS = ["RightClawSlide", "LeftClawSlide"]
FINGER_LINKS = {
    "right": ("right_thumb", "right_forefinger"),
    "left": ("left_thumb", "left_forefinger"),
}

# grasp targets, from the ring geometry: middle of two adjacent sides, on the top face
HALF_SIDE = 0.225
TOP_FACE = 0.0813
GRASP_TARGETS_LOCAL = {
    "right": np.array([0.0, -HALF_SIDE, TOP_FACE]),
    "left": np.array([-HALF_SIDE, 0.0, TOP_FACE]),
}

OUT = "C:/Projects/myProjects/Manipolazione/Scripts/logs/move_to_grasp_geometry.txt"

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


def rpy_deg(q):
    """wxyz quaternion -> roll, pitch, yaw in degrees."""
    w, x, y, z = q
    roll = np.arctan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = np.arcsin(np.clip(2 * (w * y - z * x), -1.0, 1.0))
    yaw = np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return np.degrees([roll, pitch, yaw])


def world_pose(path):
    pos, quat = XformPrim(path).get_world_poses()
    return np.array(pos.numpy()[0], dtype=float), np.array(quat.numpy()[0], dtype=float)


def to_local(rot_world, origin_world, point_world):
    """Express a world point in the frame of a body, which is what OffsetCfg stores."""
    return rot_world.T @ (point_world - origin_world)


def matrix_to_quat(m):
    """3x3 rotation matrix -> wxyz quaternion, the format OffsetCfg.rot expects."""
    tr = m[0, 0] + m[1, 1] + m[2, 2]
    if tr > 0:
        s = np.sqrt(tr + 1.0) * 2
        return np.array([0.25 * s, (m[2, 1] - m[1, 2]) / s, (m[0, 2] - m[2, 0]) / s, (m[1, 0] - m[0, 1]) / s])
    i = np.argmax([m[0, 0], m[1, 1], m[2, 2]])
    if i == 0:
        s = np.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2
        return np.array([(m[2, 1] - m[1, 2]) / s, 0.25 * s, (m[0, 1] + m[1, 0]) / s, (m[0, 2] + m[2, 0]) / s])
    if i == 1:
        s = np.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2
        return np.array([(m[0, 2] - m[2, 0]) / s, (m[0, 1] + m[1, 0]) / s, 0.25 * s, (m[1, 2] + m[2, 1]) / s])
    s = np.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2
    return np.array([(m[1, 0] - m[0, 1]) / s, (m[0, 2] + m[2, 0]) / s, (m[1, 2] + m[2, 1]) / s, 0.25 * s])


def fmt(v, digits=6):
    return "(" + ", ".join(f"{x:.{digits}f}" for x in np.asarray(v).ravel()) + ")"


stage = omni.usd.get_context().get_stage()
robot = Articulation(ROBOT_PATH)
link_path = dict(zip(robot.link_names, robot.link_paths[0]))

# prismatic joints move in metres and revolute ones in radians, so they cannot be printed alike
prismatic = {}
for prim in stage.Traverse():
    if not prim.IsA(UsdPhysics.PrismaticJoint):
        continue
    joint = UsdPhysics.PrismaticJoint(prim)
    targets = joint.GetBody1Rel().GetTargets()
    prismatic[prim.GetName()] = {
        "axis": joint.GetAxisAttr().Get(),
        "body": targets[0].pathString if targets else None,
    }

say("[LINKS]  name and prim path")
for name, path in link_path.items():
    say(f"  {name:<20} {path}")

say()
say("[JOINTS]")
dof_pos = np.array(robot.get_dof_positions().numpy()[0], dtype=float)
for name, value in zip(robot.dof_names, dof_pos):
    if name in prismatic:
        say(f"  {name:<22} {value:+.6f} m")
    else:
        say(f"  {name:<22} {value:+.6f} rad  ({np.degrees(value):+8.3f} deg)")

say()
say("[SLIDE JOINTS]  the body a slide moves is the claw that carries its fingers")
for name in SLIDE_JOINTS:
    info = prismatic.get(name)
    say(f"  {name:<18} axis={info['axis'] if info else '?'}  body={info['body'] if info else 'NOT FOUND'}")

if FLANGE_NAME not in link_path:
    say(f"\nFLANGE_NAME {FLANGE_NAME!r} is not a link. Pick one from [LINKS] and run again.")
    with open(OUT, "w") as handle:
        handle.write("\n".join(lines))
    raise SystemExit("flange not identified")

flange_path = link_path[FLANGE_NAME]
flange_pos, flange_quat = world_pose(flange_path)
flange_rot = quat_to_matrix(flange_quat)

# ---------------------------------------------------------------- PASS 1
say()
say("=" * 70)
say("[PASS 1]  read these with the robot at the PRE-GRASP pose")
say("=" * 70)

roll, pitch, yaw = rpy_deg(flange_quat)
say()
say("[TOOL ORIENTATION]  raw roll/pitch/yaw of the flange, for reference only")
say(f"  roll={roll:+8.3f}  pitch={pitch:+8.3f}  yaw={yaw:+8.3f}   (degrees)")
say("  Near +-90 pitch these three numbers are ambiguous (gimbal lock): roll and yaw stop being")
say("  separable. The [LEVEL CHECK] below tests the same thing without that ambiguity.")

say()
say("[GUIDE AXES]  prismatic joint axis, authored in USD, expressed in the flange frame")
guide_axes, guide_axes_world = {}, {}
for name in SLIDE_JOINTS:
    info = prismatic.get(name)
    if info is None or info["body"] is None:
        say(f"  {name}: joint or its body not found")
        continue
    axis_local = {"X": [1.0, 0, 0], "Y": [0, 1.0, 0], "Z": [0, 0, 1.0]}[info["axis"]]
    _, body_quat = world_pose(info["body"])
    axis_w = quat_to_matrix(body_quat) @ np.array(axis_local)
    guide_axes_world[name] = axis_w
    guide_axes[name] = flange_rot.T @ axis_w
    say(f"  {name:<18} axis={info['axis']}  in flange frame = {fmt(guide_axes[name])}")

if len(guide_axes) == 2:
    a, b = guide_axes[SLIDE_JOINTS[0]], guide_axes[SLIDE_JOINTS[1]]
    angle = np.degrees(np.arccos(np.clip(abs(np.dot(a, b)), -1.0, 1.0)))
    say(f"  angle between the two guides = {angle:.3f} deg")
    if abs(angle - 90.0) > 5.0:
        say("  WARNING: the guides are not perpendicular, so no single yaw can align both to")
        say("  two adjacent sides of a square workpiece.")

say()
say("[LEVEL CHECK]  are the two guides horizontal, independent of any frame convention")
say("  Tests the world Z component of each guide directly, sidestepping the Euler ambiguity above.")
level = True
for name, axis_w in guide_axes_world.items():
    tilt = np.degrees(np.arcsin(np.clip(abs(axis_w[2]), -1.0, 1.0)))
    say(f"  {name:<18} tilt from horizontal = {tilt:.3f} deg")
    level &= tilt < 3.0
say("  LEVEL" if level else "  WARNING: at least one guide is not horizontal; the gripper is tilted.")

say()
say("[STAFFA]  confirms origin at centre, side 0.45, half thickness 0.0813")
staffa_pos, staffa_quat = world_pose(STAFFA_PATH)
say(f"  pos_w  = {fmt(staffa_pos)}")
say(f"  quat_w = {fmt(staffa_quat)}   (wxyz)")
staffa_prim = stage.GetPrimAtPath(STAFFA_PATH)
if staffa_prim.IsValid():
    cache = UsdGeom.BBoxCache(0, [UsdGeom.Tokens.default_])
    box = cache.ComputeLocalBound(staffa_prim).ComputeAlignedRange()
    say(f"  local bbox min = {fmt(list(box.GetMin()))}")
    say(f"  local bbox max = {fmt(list(box.GetMax()))}")

say()
say("[GRASP TARGET FRAMES]  where the two grasp targets sit and point, in the frame of the child")
say("  prim that actually carries RigidBodyAPI: the outer root above is only used to know where")
say("  the ring's geometry is in the world, since the two prims do not share pose.")
staffa_rot = quat_to_matrix(staffa_quat)
staffa_body_pos, staffa_body_quat = world_pose(STAFFA_BODY_PATH)
staffa_body_rot = quat_to_matrix(staffa_body_quat)
# desired world orientation of each target: the ring's own frame, optionally turned 90 deg about
# its own z so the target's x axis runs along that side
TARGET_ROT_IN_STAFFA = {
    "right": np.eye(3),
    "left": quat_to_matrix(np.array([0.70710678, 0.0, 0.0, 0.70710678])),
}
grasp_target_offsets = {}
for side, local in GRASP_TARGETS_LOCAL.items():
    target_w = staffa_pos + staffa_rot @ local
    target_rot_w = staffa_rot @ TARGET_ROT_IN_STAFFA[side]
    offset_pos = to_local(staffa_body_rot, staffa_body_pos, target_w)
    offset_rot = matrix_to_quat(staffa_body_rot.T @ target_rot_w)
    grasp_target_offsets[side] = (offset_pos, offset_rot)
    say(f"  {side:<6} target world pos = {fmt(target_w)}")
    say(f"  {'':<6} offset pos (local to structure) = {fmt(offset_pos)}")
    say(f"  {'':<6} offset rot (local to structure) = {fmt(offset_rot)}   (wxyz)")

# ---------------------------------------------------------------- PASS 2
say()
say("=" * 70)
say("[PASS 2]  read these with the robot at the GRASP pose, fingers closed on the ring")
say("=" * 70)

staffa_rot = quat_to_matrix(staffa_quat)
targets_w = {side: staffa_pos + staffa_rot @ local for side, local in GRASP_TARGETS_LOCAL.items()}
claw_body = {
    "right": prismatic.get(SLIDE_JOINTS[0], {}).get("body"),
    "left": prismatic.get(SLIDE_JOINTS[1], {}).get("body"),
}

say()
say("[GRASP POINT OFFSETS]  chosen so each grasp point lands on its grasp target at this pose")
offsets = {}
for side, path in claw_body.items():
    if path is None:
        say(f"  {side}: claw body unknown")
        continue
    claw_pos, claw_quat = world_pose(path)
    offsets[side] = to_local(quat_to_matrix(claw_quat), claw_pos, targets_w[side])
    say(f"  {side:<6} body   = {path}")
    say(f"  {'':<6} target = {fmt(targets_w[side])}")
    say(f"  {'':<6} offset = {fmt(offsets[side])}")

say()
say("[FINGER SPREAD]  sanity check using the RAW finger link origins, no offset applied")
for side, (thumb, fore) in FINGER_LINKS.items():
    thumb_pos, _ = world_pose(link_path[thumb])
    fore_pos, _ = world_pose(link_path[fore])
    axis = thumb_pos - fore_pos
    axis = axis / max(np.linalg.norm(axis), 1e-9)
    from_thumb = float(np.dot(targets_w[side] - thumb_pos, axis))
    from_fore = float(np.dot(targets_w[side] - fore_pos, axis))
    between = "YES" if from_thumb * from_fore < 0 else "NO"
    say(f"  {side:<6} gap={np.linalg.norm(thumb_pos - fore_pos):.6f} m   target between fingers: {between}")

say()
say("[FINGER FRAME OFFSETS]  shift both fingers by the same world vector so their midpoint lands")
say("  exactly on the grasp target, keeping the true separation between them (a naive offset of")
say("  target-minus-origin for EACH finger would collapse both onto the same point instead).")
finger_offsets = {}
for side, (thumb, fore) in FINGER_LINKS.items():
    thumb_pos, thumb_quat = world_pose(link_path[thumb])
    fore_pos, fore_quat = world_pose(link_path[fore])
    shift_w = targets_w[side] - (thumb_pos + fore_pos) / 2.0
    thumb_offset = quat_to_matrix(thumb_quat).T @ shift_w
    fore_offset = quat_to_matrix(fore_quat).T @ shift_w
    finger_offsets[side] = (thumb_offset, fore_offset)
    say(f"  {side:<6} thumb offset      = {fmt(thumb_offset)}")
    say(f"  {'':<6} forefinger offset = {fmt(fore_offset)}")

midpoint_w = (targets_w["right"] + targets_w["left"]) / 2.0
ee_offset = to_local(flange_rot, flange_pos, midpoint_w)
say()
say("[EE OFFSET]  flange to the midpoint of the two grasp targets, the yaw pivot")
say(f"  ee_offset={fmt(ee_offset)}")

# ---------------------------------------------------------------- paste-ready
say()
say("=" * 70)
say("[PASTE INTO move_to_grasp_apps_cfg.py]")
say("=" * 70)
say(f"  ee_body_name={FLANGE_NAME!r},")
say(f"  ee_offset={fmt(ee_offset)},                       # PASS 2")
say(f"  pregrasp_ee_rp=({np.radians(roll):.6f}, {np.radians(pitch):.6f}),    # PASS 1")
if len(guide_axes) == 2:
    say("  guide_axis_in_ee=[")
    say(f"      {fmt(guide_axes[SLIDE_JOINTS[0]])},")
    say(f"      {fmt(guide_axes[SLIDE_JOINTS[1]])},")
    say("  ],                                             # PASS 1")
for side in ("right", "left"):
    if side in offsets:
        say(f"  {side}_grasp_point offset = {fmt(offsets[side])}    # PASS 2")
        say(f"  {side}_grasp_point prim   = {claw_body[side]}")
say()
say("  # grasp targets, PASS 1 (static geometry, does not depend on robot pose)")
for side in ("right", "left"):
    pos, rot = grasp_target_offsets[side]
    say(f"  {side}_grasp_target pos = {fmt(pos)}")
    say(f"  {side}_grasp_target rot = {fmt(rot)}")

with open(OUT, "w") as handle:
    handle.write("\n".join(lines))
say()
say(f"written to {OUT}")
