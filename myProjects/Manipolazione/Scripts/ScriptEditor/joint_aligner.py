# Anchors a selected Joint on the CENTRE OF BODY1'S MESHES, writing the matching point into
# localPos0 (Body0's frame) and localPos1 (Body1's frame).
#
# CAD-imported parts keep whatever origin the CAD had — often shared with a neighbouring part and
# far from the piece itself — so anchoring on Body1's origin puts the joint in the wrong place.
# Anchoring on the mesh centre fixes that WITHOUT moving any prim's origin, so no local frame is
# redefined and no existing joint is invalidated (see prim_center_origin.py for the destructive
# alternative, worth using only before a joint exists).
#
# The joint zero is unaffected: both localPos are set to the SAME world point, so the two frames
# coincide in the pose the assembly is authored in — which stays joint position 0. Limits are
# measured from there (SETUP_SCENARIO_CAD.md §8).
#
# Does not touch localRot0/localRot1: axis alignment is a separate step (§7.2), done by hand.
#
# ANCHOR_MODE:
#   "mesh_centre"  centre of Body1's bounding box — the default, right for most joints
#   "body1_origin" Body1's origin — for when the CAD origin is already
#                  exactly on the intended axis (e.g. a rotor whose origin sits on its axis)
#   "world_point"  an explicit WORLD_POINT (metres, stage units) — use when the anchor is a
#                  specific feature (e.g. a travel-limit stop) that doesn't sit at Body1's mesh
#                  centre or origin. Read WORLD_POINT with the Measure Tool: Point-to-Point,
#                  Snap: Vertex on the feature, Display XYZ -> World, Units -> Meter.

ANCHOR_MODE = "body1_origin"
WORLD_POINT = (0.0, 0.0, 0.0)  # only used when ANCHOR_MODE == "world_point"

import omni.usd
from pxr import Usd, UsdGeom, Gf, UsdPhysics

ctx = omni.usd.get_context()
stage = ctx.get_stage()
sel = ctx.get_selection().get_selected_prim_paths()

if not sel:
    print("ERROR: no prim selected. Select the joint in the Stage panel before running.")
else:
    joint_prim = stage.GetPrimAtPath(sel[0])
    joint = UsdPhysics.Joint(joint_prim)

    if not joint:
        print(f"ERROR: {sel[0]} is not a UsdPhysics Joint (type: {joint_prim.GetTypeName()}).")
    else:
        body0_targets = joint.GetBody0Rel().GetTargets()
        body1_targets = joint.GetBody1Rel().GetTargets()

        if not body0_targets or not body1_targets:
            print("ERROR: the joint needs both Body0 and Body1 set.")
        else:
            body0 = stage.GetPrimAtPath(body0_targets[0])
            body1 = stage.GetPrimAtPath(body1_targets[0])

            xform_cache = UsdGeom.XformCache()
            body0_world = xform_cache.GetLocalToWorldTransform(body0)
            body1_world = xform_cache.GetLocalToWorldTransform(body1)

            if ANCHOR_MODE == "mesh_centre":
                bbox_cache = UsdGeom.BBoxCache(
                    Usd.TimeCode.Default(), ["default", "render", "proxy", "guide"]
                )
                world_range = bbox_cache.ComputeWorldBound(body1).ComputeAlignedRange()
                if world_range.IsEmpty():
                    raise RuntimeError(f"Body1 ({body1.GetPath()}) has no geometry to centre on.")
                anchor_world = world_range.GetMidpoint()
            elif ANCHOR_MODE == "world_point":
                anchor_world = Gf.Vec3d(*WORLD_POINT)
            else:
                anchor_world = body1_world.Transform(Gf.Vec3d(0, 0, 0))

            local0 = body0_world.GetInverse().Transform(anchor_world)
            local1 = body1_world.GetInverse().Transform(anchor_world)

            joint.GetLocalPos0Attr().Set(Gf.Vec3f(local0))
            joint.GetLocalPos1Attr().Set(Gf.Vec3f(local1))

            print(f"Joint  : {joint_prim.GetPath()} [{joint_prim.GetTypeName()}]")
            print(f"Body0  : {body0.GetPath()}")
            print(f"Body1  : {body1.GetPath()}")
            print(f"Anchor : {anchor_world}  (mode: {ANCHOR_MODE})")
            print(f"  localPos0 = {local0}")
            print(f"  localPos1 = {local1}")
