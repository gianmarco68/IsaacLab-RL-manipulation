"""Measure and verify every frame offset the move_to_grasp task needs.

Paste into the Script Editor of the standalone Isaac Sim, with the simulation PLAYING, and run it
once at the pre-grasp pose and once at the grasp pose. The pose is detected automatically, each run
is saved to its own file, and the paste-ready block appears as soon as both exist.

While playing, PhysX moves bodies through Fabric and the USD attributes keep their authored values,
so the Property panel reads zero. These queries go through isaacsim.core.experimental.prims, whose
backend is Fabric during play. Quaternions are wxyz, the format OffsetCfg expects.
"""

import json
import os

import numpy as np
import omni.usd
from pxr import UsdGeom, UsdPhysics
from isaacsim.core.experimental.prims import Articulation, XformPrim

# ---------------------------------------------------------------- scene layout
ROBOT_PATH = "/World/m710id_50m"
STAFFA_PATH = "/World/staffa_quadrata"
# the child prim carrying RigidBodyAPI: every object frame is declared on this prim, and it does not
# share pose with the outer root
STAFFA_BODY_PATH = STAFFA_PATH + "/structure"
FLANGE_NAME = "J6_link"
SLIDE_JOINTS = {"right": "RightClawSlide", "left": "LeftClawSlide"}
FINGER_LINKS = {
    "right": ("right_thumb", "right_forefinger"),
    "left": ("left_thumb", "left_forefinger"),
}

# a run counts as the grasp pose when both fingertip midpoints are this close to their target
GRASP_DISTANCE_M = 0.10
# the bbox reaches the outer surface of the bar; this pulls each target back onto the bar axis
WIRE_RADIUS_M = 0.0
# set to "grasp" or "pregrasp" to override the automatic detection below
FORCE_PASS = None

# offsets currently written in move_to_grasp_apps_cfg.py, checked against the measurement below
CURRENT_CFG = {
    "ee_offset": (0.481494, -0.055081, -0.054198),
    "right_grasp_point": (0.059495, -0.003599, 0.197887),
    "left_grasp_point": (-0.116437, -0.018366, 0.245592),
    "right_grasp_target": (0.0, -0.225, 0.0813),
    "left_grasp_target": (-0.225, 0.0, 0.0813),
}

# direction each grasped side runs in, inside the workpiece frame: the -y side runs along x, the
# -x side along y
SIDE_AXIS_IN_OBJECT = {
    "right": np.array([1.0, 0.0, 0.0]),
    "left": np.array([0.0, 1.0, 0.0]),
}

# orientation of each target inside the workpiece frame: x is turned to run along the grasped side
TARGET_ROT_IN_OBJECT = {
    "right": np.array([1.0, 0.0, 0.0, 0.0]),
    "left": np.array([0.70710678, 0.0, 0.0, 0.70710678]),
}

LOG_DIR = "C:/Projects/myProjects/Manipolazione/Scripts/logs"
STATE = {
    "pregrasp": LOG_DIR + "/move_to_grasp_pass_pregrasp.json",
    "grasp": LOG_DIR + "/move_to_grasp_pass_grasp.json",
}
REPORT = LOG_DIR + "/move_to_grasp_geometry_v2.txt"

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


def matrix_to_quat(m):
    """3x3 rotation matrix -> wxyz quaternion (Shepperd's method, stable for every rotation)."""
    tr = m[0, 0] + m[1, 1] + m[2, 2]
    if tr > 0:
        s = np.sqrt(tr + 1.0) * 2
        return np.array([0.25 * s, (m[2, 1] - m[1, 2]) / s, (m[0, 2] - m[2, 0]) / s, (m[1, 0] - m[0, 1]) / s])
    i = int(np.argmax([m[0, 0], m[1, 1], m[2, 2]]))
    if i == 0:
        s = np.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2
        return np.array([(m[2, 1] - m[1, 2]) / s, 0.25 * s, (m[0, 1] + m[1, 0]) / s, (m[0, 2] + m[2, 0]) / s])
    if i == 1:
        s = np.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2
        return np.array([(m[0, 2] - m[2, 0]) / s, (m[0, 1] + m[1, 0]) / s, 0.25 * s, (m[1, 2] + m[2, 1]) / s])
    s = np.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2
    return np.array([(m[1, 0] - m[0, 1]) / s, (m[0, 2] + m[2, 0]) / s, (m[1, 2] + m[2, 1]) / s, 0.25 * s])


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
    """Express a world point in a body's frame, which is what OffsetCfg stores."""
    return rot_world.T @ (point_world - origin_world)


def to_world(rot_world, origin_world, offset_local):
    """Where an OffsetCfg lands in the world, the inverse of to_local."""
    return origin_world + rot_world @ np.asarray(offset_local, dtype=float)


def fmt(v, digits=6):
    return "(" + ", ".join(f"{x:.{digits}f}" for x in np.asarray(v, dtype=float).ravel()) + ")"


def err_mm(a, b):
    """Componentwise difference in millimetres, for comparing an offset against the cfg."""
    return fmt((np.asarray(a, dtype=float) - np.asarray(b, dtype=float)) * 1000.0, 2)


stage = omni.usd.get_context().get_stage()
robot = Articulation(ROBOT_PATH)
link_path = dict(zip(robot.link_names, robot.link_paths[0]))

# the body a slide joint moves is the claw that carries that side's fingers
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
claw_body = {side: prismatic.get(name, {}).get("body") for side, name in SLIDE_JOINTS.items()}

flange_path = link_path[FLANGE_NAME]
flange_pos, flange_quat = world_pose(flange_path)
flange_rot = quat_to_matrix(flange_quat)

# ---------------------------------------------------------------- workpiece geometry
# measured in the frame of the rigid body itself, so the result is a constant of the asset: it does
# not depend on the robot, on the pose, or on where the workpiece was placed in the scene
say("=" * 78)
say("[WORKPIECE]  grasp targets read from the geometry, not assumed")
say("=" * 78)
body_prim = stage.GetPrimAtPath(STAFFA_BODY_PATH)
cache = UsdGeom.BBoxCache(0, [UsdGeom.Tokens.default_])
box = cache.ComputeUntransformedBound(body_prim).ComputeAlignedRange()
bmin = np.array(list(box.GetMin()))
bmax = np.array(list(box.GetMax()))
centre = (bmin + bmax) / 2.0
half = (bmax - bmin) / 2.0
say(f"  prim      = {STAFFA_BODY_PATH}")
say(f"  bbox min  = {fmt(bmin)}")
say(f"  bbox max  = {fmt(bmax)}")
say(f"  centre    = {fmt(centre)}   half extents = {fmt(half)}")
if abs(half[0] - half[1]) > 0.005:
    say(f"  WARNING: the footprint is not square ({2 * half[0]:.4f} x {2 * half[1]:.4f} m).")

# middle of two adjacent sides, on the top face, expressed in the rigid body's own frame
grasp_target_local = {
    "right": centre + np.array([0.0, -(half[1] - WIRE_RADIUS_M), half[2] - WIRE_RADIUS_M]),
    "left": centre + np.array([-(half[0] - WIRE_RADIUS_M), 0.0, half[2] - WIRE_RADIUS_M]),
}
for side, local in grasp_target_local.items():
    say(f"  {side:<6} measured offset = {fmt(local)}"
        f"   vs cfg: {err_mm(CURRENT_CFG[side + '_grasp_target'], local)} mm")

staffa_body_pos, staffa_body_quat = world_pose(STAFFA_BODY_PATH)
staffa_body_rot = quat_to_matrix(staffa_body_quat)
target_w = {s: to_world(staffa_body_rot, staffa_body_pos, l) for s, l in grasp_target_local.items()}

# ---------------------------------------------------------------- pose detection
say()
say("=" * 78)
say("[POSE]  which pass this run is")
say("=" * 78)
finger_mid = {}
spread = {}
between = {}
for side, (thumb, fore) in FINGER_LINKS.items():
    thumb_pos, _ = world_pose(link_path[thumb])
    fore_pos, _ = world_pose(link_path[fore])
    finger_mid[side] = (thumb_pos + fore_pos) / 2.0
    spread[side] = float(np.linalg.norm(thumb_pos - fore_pos))
    axis = (thumb_pos - fore_pos) / max(spread[side], 1e-9)
    from_thumb = float(np.dot(target_w[side] - thumb_pos, axis))
    from_fore = float(np.dot(target_w[side] - fore_pos, axis))
    between[side] = from_thumb * from_fore < 0

dof = dict(zip(robot.dof_names, np.array(robot.get_dof_positions().numpy()[0], dtype=float)))
gap = {s: float(np.linalg.norm(target_w[s] - finger_mid[s])) for s in FINGER_LINKS}
for side in FINGER_LINKS:
    say(f"  {side:<6} fingertip midpoint to target = {gap[side] * 1000:8.1f} mm"
        f"   finger spread = {spread[side] * 1000:6.1f} mm"
        f"   target between fingers: {'YES' if between[side] else 'NO'}")

is_grasp = all(gap[s] < GRASP_DISTANCE_M for s in FINGER_LINKS)
pass_name = FORCE_PASS if FORCE_PASS else ("grasp" if is_grasp else "pregrasp")
say(f"  -> pass: {pass_name.upper()}" + ("  (forced)" if FORCE_PASS else "  (detected)"))
if pass_name == "grasp" and not all(between.values()):
    say("  WARNING: at least one target is not between the fingers. The grasp point offsets would")
    say("  silently absorb that error. Correct the pose before trusting this run.")
say("  Wrong guess? Set FORCE_PASS at the top of the script and run again.")

# ---------------------------------------------------------------- per-pass measurements
record = {"pass": pass_name, "joints": {k: float(v) for k, v in dof.items()}}

if pass_name == "pregrasp":
    say()
    say("=" * 78)
    say("[PRE-GRASP MEASUREMENTS]")
    say("=" * 78)
    roll, pitch, yaw = rpy_deg(flange_quat)
    say(f"  flange rpy = {roll:+8.3f} {pitch:+8.3f} {yaw:+8.3f} deg   (ambiguous near +-90 pitch)")
    record["pregrasp_ee_rp"] = [float(np.radians(roll)), float(np.radians(pitch))]

    say()
    say("  [GUIDE AXES]  prismatic axis authored in USD, expressed in the flange frame")
    axes = {}
    level = True
    for side, name in SLIDE_JOINTS.items():
        info = prismatic[name]
        axis_local = {"X": [1.0, 0, 0], "Y": [0, 1.0, 0], "Z": [0, 0, 1.0]}[info["axis"]]
        _, body_quat = world_pose(info["body"])
        axis_w = quat_to_matrix(body_quat) @ np.array(axis_local)
        axes[side] = flange_rot.T @ axis_w
        tilt = np.degrees(np.arcsin(np.clip(abs(axis_w[2]), -1.0, 1.0)))
        level &= tilt < 3.0
        say(f"  {name:<18} in flange frame = {fmt(axes[side])}   tilt = {tilt:.3f} deg")
    angle = np.degrees(np.arccos(np.clip(abs(np.dot(axes["right"], axes["left"])), -1.0, 1.0)))
    say(f"  angle between the guides = {angle:.3f} deg"
        + ("" if abs(angle - 90.0) < 5.0 else "   WARNING: not perpendicular"))
    say("  LEVEL" if level else "  WARNING: the gripper is tilted.")
    record["guide_axis_in_ee"] = [axes["right"].tolist(), axes["left"].tolist()]

else:
    say()
    say("=" * 78)
    say("[GRASP MEASUREMENTS]  valid only if this pose really holds the workpiece")
    say("=" * 78)
    say("  The grasp point is where a claw's two fingers meet, so it is measured from the fingers")
    say("  themselves. Solving it against the workpiece instead would bake today's centring error")
    say("  into the offset permanently; that variant is printed only as a cross-check.")
    points = {}
    for side, path in claw_body.items():
        claw_pos, claw_quat = world_pose(path)
        claw_rot = quat_to_matrix(claw_quat)
        points[side] = to_local(claw_rot, claw_pos, finger_mid[side])
        from_target = to_local(claw_rot, claw_pos, target_w[side])
        say(f"  {side:<6} body   = {path}")
        say(f"  {'':<6} offset = {fmt(points[side])}"
            f"   vs cfg: {err_mm(CURRENT_CFG[side + '_grasp_point'], points[side])} mm")
        # the residual splits into a part along the grasped side, which is this grasp's centring
        # error and must stay out of the offset, and a part across it, which would mean the finger
        # link origins do not sit where the fingers actually touch
        axis_w = staffa_body_rot @ SIDE_AXIS_IN_OBJECT[side]
        resid = finger_mid[side] - target_w[side]
        along = float(np.dot(resid, axis_w))
        across = resid - along * axis_w
        say(f"  {'':<6} centring error along the side = {along * 1000:+8.1f} mm   (expected, excluded)")
        say(f"  {'':<6} across the side = {np.linalg.norm(across) * 1000:8.1f} mm  {fmt(across * 1000, 1)}")
        if np.linalg.norm(across) > 0.01:
            say(f"  {'':<6} WARNING: over 10 mm across the side. The finger link origins are not at")
            say(f"  {'':<6} the contact point, so this offset is not where the claw actually grips.")
    # the yaw pivot sits between the two claws, so it is read off the gripper, not the workpiece
    midpoint_w = (finger_mid["right"] + finger_mid["left"]) / 2.0
    ee_offset = to_local(flange_rot, flange_pos, midpoint_w)
    say(f"  ee_offset = {fmt(ee_offset)}   vs cfg: {err_mm(CURRENT_CFG['ee_offset'], ee_offset)} mm")
    record["right_grasp_point"] = points["right"].tolist()
    record["left_grasp_point"] = points["left"].tolist()
    record["ee_offset"] = ee_offset.tolist()

record["right_grasp_target"] = grasp_target_local["right"].tolist()
record["left_grasp_target"] = grasp_target_local["left"].tolist()

# ---------------------------------------------------------------- verify what the cfg has today
say()
say("=" * 78)
say("[CHECK CURRENT CFG]  where today's offsets actually land, at this pose")
say("=" * 78)
for side in ("right", "left"):
    landed = to_world(staffa_body_rot, staffa_body_pos, CURRENT_CFG[side + "_grasp_target"])
    say(f"  {side}_grasp_target lands {fmt(landed)}   should be {fmt(target_w[side])}"
        f"   off by {np.linalg.norm(landed - target_w[side]) * 1000:8.1f} mm")
for side in ("right", "left"):
    claw_pos, claw_quat = world_pose(claw_body[side])
    landed = to_world(quat_to_matrix(claw_quat), claw_pos, CURRENT_CFG[side + "_grasp_point"])
    say(f"  {side}_grasp_point  lands {fmt(landed)}   fingertip midpoint {fmt(finger_mid[side])}"
        f"   off by {np.linalg.norm(landed - finger_mid[side]) * 1000:8.1f} mm")
ee_landed = to_world(flange_rot, flange_pos, CURRENT_CFG["ee_offset"])
centre_w = (finger_mid["right"] + finger_mid["left"]) / 2.0
say(f"  ee              lands {fmt(ee_landed)}   between the two claws {fmt(centre_w)}"
    f"   off by {np.linalg.norm(ee_landed - centre_w) * 1000:8.1f} mm")

# ---------------------------------------------------------------- accumulate and emit
os.makedirs(LOG_DIR, exist_ok=True)
with open(STATE[pass_name], "w") as handle:
    json.dump(record, handle, indent=2)

saved = {}
for name, path in STATE.items():
    if os.path.exists(path):
        with open(path) as handle:
            saved[name] = json.load(handle)

say()
say("=" * 78)
if len(saved) < 2:
    missing = ", ".join(n for n in STATE if n not in saved)
    say(f"[SAVED]  {pass_name} run stored. Still missing: {missing}.")
    say("Move the robot to that pose and run this script again for the paste-ready block.")
else:
    pre = saved["pregrasp"]
    gr = saved["grasp"]
    say("[PASTE INTO move_to_grasp_apps_cfg.py]")
    say("=" * 78)
    say("# ArmDescriptorCfg")
    say(f"  ee_offset={fmt(gr['ee_offset'])},")
    say(f"  pregrasp_ee_rp=({pre['pregrasp_ee_rp'][0]:.6f}, {pre['pregrasp_ee_rp'][1]:.6f}),")
    say("# GripperDescriptorCfg")
    say(f"  guide_axis_in_ee=[{fmt(pre['guide_axis_in_ee'][0])}, {fmt(pre['guide_axis_in_ee'][1])}],")
    say("# ee_frame target_frames, offset relative to each claw's sliding body")
    for side in ("right", "left"):
        say(f"  {side}_grasp_point  OffsetCfg(pos={fmt(gr[side + '_grasp_point'])})")
    say("# object_frame target_frames, offset relative to the structure prim")
    for side in ("right", "left"):
        say(f"  {side}_grasp_target OffsetCfg(pos={fmt(gr[side + '_grasp_target'])},"
            f" rot={fmt(TARGET_ROT_IN_OBJECT[side], 8)})")

with open(REPORT, "w") as handle:
    handle.write("\n".join(lines))
say()
say(f"report written to {REPORT}")
