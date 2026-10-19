"""Measure every frame offset the move_to_grasp task needs, from the real finger contacts.

Paste into the Script Editor of the standalone Isaac Sim, with the simulation PLAYING, and run it
once at the pre-grasp pose and once at the grasp pose, in any order. The pose is detected from the
contacts themselves, each run is saved to its own file, and the paste-ready block appears as soon
as both exist.

The grasp point is the midpoint of the segment the two fingers are actually clamping, read from
PhysX contact data rather than from link origins, so it depends neither on where the finger prims
have their origin nor on where the workpiece drifted to. The grasp target is placed on that same
point, expressed in the workpiece frame: the goal becomes reproducing this verified grasp.

A contact view holds only the bodies that declare the contact-report schema, so its rows are
resolved against that subset at runtime instead of being assumed. It also has no data until PhysX
has stepped with it in place, which is why the script runs asynchronously over several frames.
"""

import asyncio
import json
import os

import numpy as np
import omni.kit.app
import omni.usd
from pxr import PhysxSchema, Usd, UsdGeom, UsdPhysics
from isaacsim.core.experimental.prims import Articulation, RigidPrim, XformPrim

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

WARMUP_FRAMES = 20
# contacts flicker between steps, so the best frame of this many is kept per finger
SAMPLE_FRAMES = 40
MAX_CONTACT_COUNT = 1024
# set to "grasp" or "pregrasp" to override the automatic detection below
FORCE_PASS = None
# the bbox reaches the outer surface of the bar; this pulls the reference midpoint onto the bar axis
WIRE_RADIUS_M = 0.0

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
REPORT = LOG_DIR + "/move_to_grasp_geometry_v4.txt"

lines = []


def say(text=""):
    print(text)
    lines.append(text)


def flush():
    os.makedirs(LOG_DIR, exist_ok=True)
    with open(REPORT, "w") as handle:
        handle.write("\n".join(lines))


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


def world_scale(prim):
    """Per-axis scale from the prim's frame to the world, needed to read authored geometry in metres."""
    matrix = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    rows = np.array([[matrix[r][c] for c in range(3)] for r in range(3)])
    return np.linalg.norm(rows, axis=1)


async def main():
    stage = omni.usd.get_context().get_stage()
    robot = Articulation(ROBOT_PATH)
    link_path = dict(zip(robot.link_names, robot.link_paths[0]))
    finger_names = [name for side in ("right", "left") for name in FINGER_LINKS[side]]

    # ------------------------------------------------------------ contact schema
    # PhysX only fills contact buffers for bodies carrying the schema, and applying it is a stage
    # edit that takes effect on the next Stop/Play, not during the running simulation
    say("=" * 78)
    say("[CONTACT SCHEMA]")
    say("=" * 78)
    applied = []
    for name in finger_names:
        prim = stage.GetPrimAtPath(link_path[name])
        if prim.HasAPI(PhysxSchema.PhysxContactReportAPI):
            say(f"  {name:<18} already reporting")
            continue
        PhysxSchema.PhysxContactReportAPI.Apply(prim).CreateThresholdAttr().Set(0.0)
        applied.append(name)
        say(f"  {name:<18} schema applied now")
    if applied:
        say()
        say("  Press Stop, then Play, redo the grasp and run again. Nothing measured or saved.")
        flush()
        return

    # ------------------------------------------------------------ read the contacts
    # the view holds only the bodies that report contact, so its row order is that subset in link
    # order, which is resolved here rather than assumed
    reporting = [
        name for name in robot.link_names
        if stage.GetPrimAtPath(link_path[name]).HasAPI(PhysxSchema.PhysxContactReportAPI)
    ]
    view = RigidPrim(
        [link_path[name] for name in reporting],
        contact_filter_paths=[STAFFA_BODY_PATH],
        max_contact_count=MAX_CONTACT_COUNT,
    )

    for _ in range(WARMUP_FRAMES):
        await omni.kit.app.get_app().next_update_async()

    _f, points_w, _n, _d, counts_w, starts_w = view.get_contact_force_data()
    rows = int(np.asarray(counts_w.numpy()).reshape(len(reporting), -1).shape[0])
    say()
    say("=" * 78)
    say("[CONTACT VIEW]  rows belong to the bodies that report contact, in link order")
    say("=" * 78)
    say(f"  bodies in the view ({rows}): {', '.join(reporting)}")
    missing = [n for n in finger_names if n not in reporting]
    if missing:
        say(f"  WARNING: not in the view: {', '.join(missing)}. Their contacts cannot be read.")

    best_count = {name: 0 for name in reporting}
    best_point = {}
    for _ in range(SAMPLE_FRAMES):
        _f, points_w, _n, _d, counts_w, starts_w = view.get_contact_force_data()
        points = np.asarray(points_w.numpy()).reshape(-1, 3)
        counts = np.asarray(counts_w.numpy()).reshape(len(reporting), -1)[:, 0]
        starts = np.asarray(starts_w.numpy()).reshape(len(reporting), -1)[:, 0]
        for index, name in enumerate(reporting):
            count = int(counts[index])
            start = int(starts[index])
            if count <= 0 or count > len(points) or start + count > len(points):
                continue
            hits = points[start:start + count]
            hits = hits[np.isfinite(hits).all(axis=1)]
            if len(hits) > best_count[name]:
                best_count[name] = len(hits)
                best_point[name] = hits.mean(axis=0)
        await omni.kit.app.get_app().next_update_async()

    say()
    say("=" * 78)
    say(f"[CONTACTS]  best frame of {SAMPLE_FRAMES}, against {STAFFA_BODY_PATH}")
    say("=" * 78)
    for name in reporting:
        if name in best_point:
            say(f"  {name:<18} {best_count[name]:4d} contact(s)   mean point = {fmt(best_point[name])}")
        else:
            say(f"  {name:<18} no contact")

    # ------------------------------------------------------------ pose detection
    gripping = {side: all(n in best_point for n in FINGER_LINKS[side]) for side in FINGER_LINKS}
    is_grasp = all(gripping.values())
    pass_name = FORCE_PASS if FORCE_PASS else ("grasp" if is_grasp else "pregrasp")
    say()
    say(f"  both fingers in contact: right={gripping['right']}  left={gripping['left']}")
    say(f"  -> pass: {pass_name.upper()}" + ("  (forced)" if FORCE_PASS else "  (detected)"))
    if pass_name == "grasp" and not is_grasp:
        say("  WARNING: forced to grasp but a finger reports no contact. Aborting.")
        flush()
        return

    flange_pos, flange_quat = world_pose(link_path[FLANGE_NAME])
    flange_rot = quat_to_matrix(flange_quat)
    staffa_body_pos, staffa_body_quat = world_pose(STAFFA_BODY_PATH)
    staffa_body_rot = quat_to_matrix(staffa_body_quat)

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

    dof = dict(zip(robot.dof_names, np.array(robot.get_dof_positions().numpy()[0], dtype=float)))
    record = {"pass": pass_name, "joints": {k: float(v) for k, v in dof.items()}}
    grip_w = {}

    # ------------------------------------------------------------ workpiece reference geometry
    # authored geometry is in the prim's local units, so it is scaled to metres before use; this
    # block only reports how far off the side midpoint the grasp landed
    say()
    say("=" * 78)
    say("[WORKPIECE]  geometric reference, for comparison only")
    say("=" * 78)
    body_prim = stage.GetPrimAtPath(STAFFA_BODY_PATH)
    scale = world_scale(body_prim)
    cache = UsdGeom.BBoxCache(0, [UsdGeom.Tokens.default_])
    box = cache.ComputeUntransformedBound(body_prim).ComputeAlignedRange()
    bmin = np.array(list(box.GetMin())) * scale
    bmax = np.array(list(box.GetMax())) * scale
    centre = (bmin + bmax) / 2.0
    half = (bmax - bmin) / 2.0
    say(f"  world scale of the prim = {fmt(scale, 8)}")
    say(f"  centre = {fmt(centre)}   half extents = {fmt(half)} m")
    if abs(half[0] - half[1]) > 0.005:
        say(f"  WARNING: the footprint is not square ({2 * half[0]:.4f} x {2 * half[1]:.4f} m).")
    reference_local = {
        "right": centre + np.array([0.0, -(half[1] - WIRE_RADIUS_M), half[2] - WIRE_RADIUS_M]),
        "left": centre + np.array([-(half[0] - WIRE_RADIUS_M), 0.0, half[2] - WIRE_RADIUS_M]),
    }

    # ------------------------------------------------------------ per-pass measurements
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
        say("[GRASP MEASUREMENTS]  from the clamped segment, not from link origins")
        say("=" * 78)
        say("  Each finger's contacts are averaged first, then the two averages are averaged, so a")
        say("  finger touching in many points does not outweigh one touching in a single point.")
        for side, (thumb, fore) in FINGER_LINKS.items():
            grip_w[side] = (best_point[thumb] + best_point[fore]) / 2.0
            say(f"  {side:<6} clamped midpoint (world) = {fmt(grip_w[side])}")
            reference_w = to_world(staffa_body_rot, staffa_body_pos, reference_local[side])
            resid = grip_w[side] - reference_w
            axis_w = staffa_body_rot @ SIDE_AXIS_IN_OBJECT[side]
            along = float(np.dot(resid, axis_w))
            across = resid - along * axis_w
            say(f"  {'':<6} off the side midpoint: {along * 1000:+8.1f} mm along the side,"
                f" {np.linalg.norm(across) * 1000:6.1f} mm across   (accepted, this grasp is the goal)")

        for side in ("right", "left"):
            claw_pos, claw_quat = world_pose(claw_body[side])
            point_offset = to_local(quat_to_matrix(claw_quat), claw_pos, grip_w[side])
            target_offset = to_local(staffa_body_rot, staffa_body_pos, grip_w[side])
            record[side + "_grasp_point"] = point_offset.tolist()
            record[side + "_grasp_target"] = target_offset.tolist()
            say()
            say(f"  {side}_grasp_point  from {claw_body[side]}")
            say(f"  {'':<6} offset = {fmt(point_offset)}"
                f"   vs cfg: {err_mm(CURRENT_CFG[side + '_grasp_point'], point_offset)} mm")
            say(f"  {side}_grasp_target from {STAFFA_BODY_PATH}")
            say(f"  {'':<6} offset = {fmt(target_offset)}"
                f"   vs cfg: {err_mm(CURRENT_CFG[side + '_grasp_target'], target_offset)} mm")

        # the yaw pivot sits between the two claws, so it is read off the gripper, not the workpiece
        ee_offset = to_local(flange_rot, flange_pos, (grip_w["right"] + grip_w["left"]) / 2.0)
        record["ee_offset"] = ee_offset.tolist()
        say()
        say(f"  ee_offset = {fmt(ee_offset)}   vs cfg: {err_mm(CURRENT_CFG['ee_offset'], ee_offset)} mm")

    # ------------------------------------------------------------ verify what the cfg has today
    say()
    say("=" * 78)
    say("[CHECK CURRENT CFG]  where today's offsets actually land, at this pose")
    say("=" * 78)
    for side in ("right", "left"):
        landed = to_world(staffa_body_rot, staffa_body_pos, CURRENT_CFG[side + "_grasp_target"])
        extra = (f"   off the clamped midpoint by {np.linalg.norm(landed - grip_w[side]) * 1000:.1f} mm"
                 if side in grip_w else "")
        say(f"  {side}_grasp_target lands {fmt(landed)}{extra}")
    for side in ("right", "left"):
        claw_pos, claw_quat = world_pose(claw_body[side])
        landed = to_world(quat_to_matrix(claw_quat), claw_pos, CURRENT_CFG[side + "_grasp_point"])
        extra = (f"   off the clamped midpoint by {np.linalg.norm(landed - grip_w[side]) * 1000:.1f} mm"
                 if side in grip_w else "")
        say(f"  {side}_grasp_point  lands {fmt(landed)}{extra}")

    # ------------------------------------------------------------ accumulate and emit
    os.makedirs(LOG_DIR, exist_ok=True)
    with open(STATE[pass_name], "w") as handle:
        json.dump(record, handle, indent=2)

    saved = {}
    for name, path in STATE.items():
        if os.path.exists(path):
            with open(path) as handle:
                saved[name] = json.load(handle)

    needed = {
        "pregrasp": ("pregrasp_ee_rp", "guide_axis_in_ee"),
        "grasp": ("ee_offset", "right_grasp_point", "left_grasp_point",
                  "right_grasp_target", "left_grasp_target"),
    }
    todo = [f"{p}:{k}" for p, keys in needed.items() for k in keys if k not in saved.get(p, {})]

    say()
    say("=" * 78)
    if todo:
        say(f"[SAVED]  {pass_name} run stored. Still missing: {', '.join(todo)}.")
        say("Move the robot to the other pose and run this script again.")
    else:
        pre, gr = saved["pregrasp"], saved["grasp"]
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

    flush()
    say()
    say(f"report written to {REPORT}")


asyncio.ensure_future(main())
