"""Measure the grasp point and grasp target from the real contacts between fingers and workpiece.

Paste into the Script Editor of the standalone Isaac Sim, with the simulation PLAYING and the
fingers clamped on the workpiece.

Contacts come from PhysX's contact-report event stream, which delivers the prim path of each
colliding mesh together with the world position of every contact. Nothing has to be matched by row
index or by proximity: a contact is attributed to a finger because PhysX names that finger.

At the grasp pose the grasp point and the grasp target are the same world point, the middle of the
segment the two fingers clamp. It is then written twice, as an offset from the claw body and as an
offset from the workpiece body, which is what makes it reproducible at any other pose.
"""

import asyncio
import json
import os

import numpy as np
import omni.kit.app
import omni.usd
from omni.physx import get_physx_simulation_interface
from pxr import PhysicsSchemaTools, PhysxSchema, Usd, UsdGeom, UsdPhysics, UsdUtils
from isaacsim.core.experimental.prims import Articulation, RigidPrim, XformPrim

# ---------------------------------------------------------------- scene layout
ROBOT_PATH = "/World/m710id_50m"
STAFFA_PATH = "/World/staffa_quadrata"
STAFFA_BODY_PATH = STAFFA_PATH + "/structure"
FLANGE_NAME = "J6_link"
SLIDE_JOINTS = {"right": "RightClawSlide", "left": "LeftClawSlide"}
FINGER_LINKS = {
    "right": ("right_thumb", "right_forefinger"),
    "left": ("left_thumb", "left_forefinger"),
}

# frames to listen for contact events; PhysX emits them once per physics step
LISTEN_FRAMES = 60
# also make the workpiece report: its contacts with the ground are a known reference that tells a
# silent stream apart from fingers that are not really touching
REPORT_ON_WORKPIECE = True

# the fingers grip well below the top of the stack, so the reference point is lifted onto the top
# face, keeping the position along and across the side that the contacts measured
LIFT_TO_TOP_FACE = True
# subtract the bar radius to sit on the axis of the top ring instead of on its outer surface
TOP_FACE_INSET_M = 0.0

# orientation of each target inside the workpiece frame: x is turned to run along the grasped side
TARGET_ROT_IN_OBJECT = {
    "right": (1.0, 0.0, 0.0, 0.0),
    "left": (0.70710678, 0.0, 0.0, 0.70710678),
}

LOG_DIR = "C:/Projects/myProjects/Manipolazione/Scripts/logs"
PREGRASP_JSON = LOG_DIR + "/move_to_grasp_pass_pregrasp.json"
GRASP_JSON = LOG_DIR + "/move_to_grasp_pass_grasp.json"
REPORT = LOG_DIR + "/measure_grasp_from_contacts.txt"

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


def world_pose(path):
    pos, quat = XformPrim(path).get_world_poses()
    return np.array(pos.numpy()[0], dtype=float), np.array(quat.numpy()[0], dtype=float)


def to_local(rot_world, origin_world, point_world):
    """Express a world point in a body's frame, which is what OffsetCfg stores."""
    return rot_world.T @ (point_world - origin_world)


def to_world(rot_world, origin_world, offset_local):
    """Where an offset lands in the world, the inverse of to_local."""
    return origin_world + rot_world @ np.asarray(offset_local, dtype=float)


def world_scale(prim):
    """Per-axis scale from the prim's frame to the world, needed to read authored geometry in metres."""
    matrix = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    rows = np.array([[matrix[r][c] for c in range(3)] for r in range(3)])
    return np.linalg.norm(rows, axis=1)


def fmt(v, digits=6):
    return "(" + ", ".join(f"{x:.{digits}f}" for x in np.asarray(v, dtype=float).ravel()) + ")"


async def main():
    stage = omni.usd.get_context().get_stage()
    robot = Articulation(ROBOT_PATH)
    link_path = dict(zip(robot.link_names, robot.link_paths[0]))
    finger_path = {name: link_path[name] for side in FINGER_LINKS for name in FINGER_LINKS[side]}

    # ------------------------------------------------------------ what the fingers declare
    # a mesh with CollisionAPI still generates no contact when collisionEnabled is false, which is
    # how touching CAD parts are usually kept apart, and it looks exactly like a missing collider
    say("=" * 78)
    say("[FINGER COLLIDERS]  how many are enabled, which is what decides whether contact happens")
    say("=" * 78)
    for name, path in finger_path.items():
        enabled = disabled = 0
        for prim in Usd.PrimRange(stage.GetPrimAtPath(path)):
            if not prim.HasAPI(UsdPhysics.CollisionAPI):
                continue
            attr = UsdPhysics.CollisionAPI(prim).GetCollisionEnabledAttr()
            if attr and attr.Get() is False:
                disabled += 1
            else:
                enabled += 1
        report = stage.GetPrimAtPath(path).HasAPI(PhysxSchema.PhysxContactReportAPI)
        say(f"  {name:<18} {enabled:3d} enabled, {disabled:3d} disabled   contact report: {report}")

    # the schema is what makes PhysX emit events for a body; it is read when the simulation starts
    targets = dict(finger_path)
    if REPORT_ON_WORKPIECE:
        targets["workpiece"] = STAFFA_BODY_PATH
    applied = []
    for name, path in targets.items():
        prim = stage.GetPrimAtPath(path)
        if prim.HasAPI(PhysxSchema.PhysxContactReportAPI):
            continue
        PhysxSchema.PhysxContactReportAPI.Apply(prim).CreateThresholdAttr().Set(0.0)
        applied.append(name)
    if applied:
        say()
        say(f"  Contact report applied to: {', '.join(applied)}.")
        say("  PhysX reads it at simulation start: press Stop, then Play, redo the grasp, run again.")
        flush()
        return

    # ------------------------------------------------------------ sleep state
    # PhysX puts a resting island to sleep and then stops simulating it: no contact events, no
    # contact data, however hard the fingers are pressing
    stage_id = UsdUtils.StageCache.Get().GetId(stage).ToLongInt()
    physx = get_physx_simulation_interface()
    encoded = {name: PhysicsSchemaTools.sdfPathToInt(path) for name, path in targets.items()}
    say()
    say("=" * 78)
    say("[SLEEP STATE]")
    say("=" * 78)
    for name in targets:
        say(f"  {name:<18} sleeping = {physx.is_sleeping(stage_id, encoded[name])}")

    # ------------------------------------------------------------ listen to the contact stream
    finger_of_path = {path: name for name, path in targets.items()}
    hits = {name: [] for name in targets}
    other_side = {}

    def on_contact(headers, data):
        """Collect every contact where one of the two colliders sits under a finger link."""
        for header in headers:
            path0 = PhysicsSchemaTools.intToSdfPath(header.collider0).pathString
            path1 = PhysicsSchemaTools.intToSdfPath(header.collider1).pathString
            for mine, theirs in ((path0, path1), (path1, path0)):
                name = next((n for p, n in finger_of_path.items() if mine.startswith(p)), None)
                if name is None:
                    continue
                other_side.setdefault(name, set()).add(theirs)
                start = header.contact_data_offset
                for index in range(start, start + header.num_contact_data):
                    position = data[index].position
                    hits[name].append((position.x, position.y, position.z))

    # the stream reports transitions, so a grasp established before subscribing stays silent; the
    # tensor view below reports the current state instead, and the two cover each other
    view = RigidPrim(
        list(targets.values()), contact_filter_paths=[STAFFA_BODY_PATH], max_contact_count=1024
    )
    subscription = physx.subscribe_contact_report_events(on_contact)
    for _ in range(LISTEN_FRAMES):
        for name in targets:
            physx.wake_up(stage_id, encoded[name])
        await omni.kit.app.get_app().next_update_async()
    subscription = None

    say()
    say("=" * 78)
    say("[TENSOR VIEW]  current contact state, independent of the event stream")
    say("=" * 78)
    _f, points_w, _n, _d, counts_w, starts_w = view.get_contact_force_data()
    points = np.asarray(points_w.numpy()).reshape(-1, 3)
    counts = np.asarray(counts_w.numpy()).ravel()
    starts = np.asarray(starts_w.numpy()).ravel()
    names = list(targets)
    for index, count in enumerate(counts):
        label = names[index] if index < len(names) else f"row {index}"
        count, start = int(count), int(starts[index])
        if count <= 0 or start + count > len(points):
            say(f"  {label:<18} {count} contact(s)")
            continue
        block = points[start:start + count]
        block = block[np.isfinite(block).all(axis=1)]
        say(f"  {label:<18} {len(block)} contact(s)   mean = {fmt(block.mean(axis=0)) if len(block) else '-'}")
        if len(block) and not len(hits[label]):
            hits[label] = [tuple(row) for row in block]

    say()
    say("=" * 78)
    say(f"[CONTACT EVENTS]  collected over {LISTEN_FRAMES} frames")
    say("=" * 78)
    for name in targets:
        points = np.array(hits[name]).reshape(-1, 3)
        say(f"  {name:<18} {len(points):5d} contact point(s)")
        for path in sorted(other_side.get(name, ())):
            say(f"  {'':<18}   touching {path}")

    if not any(len(hits[name]) for name in finger_path):
        say()
        if hits.get("workpiece"):
            say("  The workpiece reports contact but no finger does: the fingers are not actually")
            say("  touching it. Check what the workpiece is touching, listed above.")
        else:
            say("  Nothing reports, not even the workpiece resting on the ground, so PhysX is not")
            say("  emitting events in this run: press Stop, then Play, redo the grasp, run again.")
        flush()
        return

    # ------------------------------------------------------------ the clamped midpoint
    # each finger is averaged first and the two averages are then averaged, so a finger reporting
    # many contacts does not pull the midpoint onto its own side
    say()
    say("=" * 78)
    say("[CLAMPED MIDPOINT]  middle of the segment each pair of fingers holds")
    say("=" * 78)
    grip_w = {}
    for side, (thumb, fore) in FINGER_LINKS.items():
        a = np.array(hits[thumb]).reshape(-1, 3)
        b = np.array(hits[fore]).reshape(-1, 3)
        if not len(a) or not len(b):
            say(f"  {side:<6} only one finger reports contact: {len(a)} / {len(b)}. Skipped.")
            continue
        grip_w[side] = (a.mean(axis=0) + b.mean(axis=0)) / 2.0
        say(f"  {side:<6} {FINGER_LINKS[side][0]} mean = {fmt(a.mean(axis=0))}")
        say(f"  {'':<6} {FINGER_LINKS[side][1]} mean = {fmt(b.mean(axis=0))}")
        say(f"  {'':<6} midpoint = {fmt(grip_w[side])}   finger separation ="
            f" {np.linalg.norm(a.mean(axis=0) - b.mean(axis=0)) * 1000:.1f} mm")

    if len(grip_w) < 2:
        say()
        say("  Both claws are needed. Fix the grasp and run again.")
        flush()
        return

    # ------------------------------------------------------------ turn it into offsets
    prismatic = {}
    for prim in stage.Traverse():
        if prim.IsA(UsdPhysics.PrismaticJoint):
            joint = UsdPhysics.PrismaticJoint(prim)
            targets = joint.GetBody1Rel().GetTargets()
            prismatic[prim.GetName()] = targets[0].pathString if targets else None
    claw_body = {side: prismatic[name] for side, name in SLIDE_JOINTS.items()}

    staffa_pos, staffa_quat = world_pose(STAFFA_BODY_PATH)
    staffa_rot = quat_to_matrix(staffa_quat)
    flange_pos, flange_quat = world_pose(link_path[FLANGE_NAME])
    flange_rot = quat_to_matrix(flange_quat)

    say()
    say("=" * 78)
    say("[OFFSETS]  the same world point written against two different prims")
    say("=" * 78)
    record = {
        "pass": "grasp",
        "joints": {k: float(v) for k, v in
                   zip(robot.dof_names, np.array(robot.get_dof_positions().numpy()[0], dtype=float))},
    }
    # the contacts sit low on the stack, so the reference point is raised to the top face along the
    # workpiece's own vertical; both offsets are derived from the raised point, which keeps them
    # coincident at this pose
    if LIFT_TO_TOP_FACE:
        body_prim = stage.GetPrimAtPath(STAFFA_BODY_PATH)
        box = UsdGeom.BBoxCache(0, [UsdGeom.Tokens.default_]).ComputeUntransformedBound(
            body_prim).ComputeAlignedRange()
        top_face = float((np.array(list(box.GetMax())) * world_scale(body_prim))[2])
        for side in ("right", "left"):
            local = to_local(staffa_rot, staffa_pos, grip_w[side])
            raised = np.array([local[0], local[1], top_face - TOP_FACE_INSET_M])
            say(f"  {side:<6} raised {(raised[2] - local[2]) * 1000:+.1f} mm to the top face"
                f"  (z {local[2]:.6f} -> {raised[2]:.6f} in the workpiece frame)")
            grip_w[side] = to_world(staffa_rot, staffa_pos, raised)

    for side in ("right", "left"):
        claw_pos, claw_quat = world_pose(claw_body[side])
        point_offset = to_local(quat_to_matrix(claw_quat), claw_pos, grip_w[side])
        target_offset = to_local(staffa_rot, staffa_pos, grip_w[side])
        record[side + "_grasp_point"] = point_offset.tolist()
        record[side + "_grasp_target"] = target_offset.tolist()
        say(f"  {side}_grasp_point  from {claw_body[side]}")
        say(f"  {'':<6} {fmt(point_offset)}")
        say(f"  {side}_grasp_target from {STAFFA_BODY_PATH}")
        say(f"  {'':<6} {fmt(target_offset)}")

    ee_offset = to_local(flange_rot, flange_pos, (grip_w["right"] + grip_w["left"]) / 2.0)
    record["ee_offset"] = ee_offset.tolist()
    say(f"  ee_offset from {FLANGE_NAME}")
    say(f"  {'':<6} {fmt(ee_offset)}")

    # ------------------------------------------------------------ emit
    os.makedirs(LOG_DIR, exist_ok=True)
    with open(GRASP_JSON, "w") as handle:
        json.dump(record, handle, indent=2)

    pre = {}
    if os.path.exists(PREGRASP_JSON):
        with open(PREGRASP_JSON) as handle:
            pre = json.load(handle)

    say()
    say("=" * 78)
    say("[PASTE INTO move_to_grasp_apps_cfg.py]")
    say("=" * 78)
    say("# ArmDescriptorCfg")
    say(f"  ee_offset={fmt(ee_offset)},")
    if "pregrasp_ee_rp" in pre:
        say(f"  pregrasp_ee_rp=({pre['pregrasp_ee_rp'][0]:.6f}, {pre['pregrasp_ee_rp'][1]:.6f}),")
    if "guide_axis_in_ee" in pre:
        say("# GripperDescriptorCfg")
        say(f"  guide_axis_in_ee=[{fmt(pre['guide_axis_in_ee'][0])}, {fmt(pre['guide_axis_in_ee'][1])}],")
    if not pre:
        say("  # pre-grasp values missing: run the pre-grasp pass to fill them")
    say("# ee_frame target_frames, offset relative to each claw's sliding body")
    for side in ("right", "left"):
        say(f"  {side}_grasp_point  OffsetCfg(pos={fmt(record[side + '_grasp_point'])})")
    say("# object_frame target_frames, offset relative to the structure prim")
    for side in ("right", "left"):
        say(f"  {side}_grasp_target OffsetCfg(pos={fmt(record[side + '_grasp_target'])},"
            f" rot={fmt(TARGET_ROT_IN_OBJECT[side], 8)})")

    flush()
    say()
    say(f"report written to {REPORT}")


asyncio.ensure_future(main())
