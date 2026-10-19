# Read-only. Select a Joint prim and run: prints the anchor point in WORLD space, computed from
# localPos0 + Body0's current world transform, and separately from localPos1 + Body1's — i.e. the
# actual numeric position PhysX uses, independent of whatever the viewport gizmo is (or isn't)
# currently drawing. If the joint is well-aligned the two should be the same point (or very close);
# if they differ, that's the real, numeric version of the "anchors don't coincide" problem.

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

            local_pos0 = joint.GetLocalPos0Attr().Get()
            local_pos1 = joint.GetLocalPos1Attr().Get()

            anchor0_world = body0_world.Transform(Gf.Vec3d(local_pos0))
            anchor1_world = body1_world.Transform(Gf.Vec3d(local_pos1))
            gap = (anchor1_world - anchor0_world).GetLength()

            print(f"Joint  : {joint_prim.GetPath()} [{joint_prim.GetTypeName()}]")
            print(f"Body0  : {body0.GetPath()}")
            print(f"  localPos0 = {local_pos0}  ->  world = {anchor0_world}")
            print(f"Body1  : {body1.GetPath()}")
            print(f"  localPos1 = {local_pos1}  ->  world = {anchor1_world}")
            print(f"Gap between anchors (world) = {gap:.6f}  (stage units — should be ~0 if aligned)")
