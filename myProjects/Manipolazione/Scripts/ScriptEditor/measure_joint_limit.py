# Select a Joint prim, set WORLD_POINT to a feature read with the Measure Tool (same procedure as
# joint_aligner's "world_point" ANCHOR_MODE: Point-to-Point, Snap: Vertex, Display XYZ -> World,
# Units -> Meter), and run. Prints the signed distance from the joint's zero position to that point,
# projected onto the joint's translation axis — this is exactly the value physics:lowerLimit /
# physics:upperLimit expects.
#
# Why not bounding boxes (measure_rail_travel.py): that script assumes the relevant reference is a
# bbox edge of the whole prim. Here the reference is an arbitrary CAD feature (e.g. a mechanical
# stop) that sits neither at the prim's origin nor at a bbox extreme — same class of problem
# joint_aligner solves for anchoring, just applied to limits instead of the joint frame.
#
# Requires the joint to already have localPos0/localPos1 set (run joint_aligner first): the zero
# position is defined by those, not recomputed here.

WORLD_POINT = (-0.16120717, -0.1933, 0.32150804) # read from the Measure Tool, metres, world space

import omni.usd
from pxr import Usd, UsdGeom, Gf, UsdPhysics

ctx = omni.usd.get_context()
stage = ctx.get_stage()
sel = ctx.get_selection().get_selected_prim_paths()

AXIS_LOCAL = {
    "X": Gf.Vec3d(1, 0, 0),
    "Y": Gf.Vec3d(0, 1, 0),
    "Z": Gf.Vec3d(0, 0, 1),
}

if not sel:
    print("ERROR: no prim selected. Select the joint in the Stage panel before running.")
else:
    joint_prim = stage.GetPrimAtPath(sel[0])
    joint = UsdPhysics.Joint(joint_prim)

    if not joint:
        print(f"ERROR: {sel[0]} is not a UsdPhysics Joint (type: {joint_prim.GetTypeName()}).")
    else:
        prismatic = UsdPhysics.PrismaticJoint(joint_prim)
        if not prismatic:
            print(f"ERROR: {sel[0]} is not a PrismaticJoint (type: {joint_prim.GetTypeName()}).")
        else:
            body0_targets = joint.GetBody0Rel().GetTargets()
            if not body0_targets:
                print("ERROR: the joint needs Body0 set.")
            else:
                body0 = stage.GetPrimAtPath(body0_targets[0])
                local_pos0 = joint.GetLocalPos0Attr().Get()
                local_rot0 = joint.GetLocalRot0Attr().Get()
                axis_token = prismatic.GetAxisAttr().Get()

                xform_cache = UsdGeom.XformCache()
                body0_world = xform_cache.GetLocalToWorldTransform(body0)

                # Joint zero, in world space: Body0's world transform applied to localPos0.
                anchor_world = body0_world.Transform(Gf.Vec3d(local_pos0))

                # Joint axis, in world space: localRot0 rotates the schema's local axis into
                # Body0's frame, then Body0's world rotation carries it into world space.
                # (localRot0 is a Quatf; Gf.Rotation is built from a Quatd and exposes TransformDir.)
                rot0d = Gf.Quatd(float(local_rot0.GetReal()), Gf.Vec3d(local_rot0.GetImaginary()))
                axis_body0 = Gf.Rotation(rot0d).TransformDir(AXIS_LOCAL[axis_token])
                axis_world = body0_world.TransformDir(axis_body0)
                axis_world = axis_world.GetNormalized()

                target_world = Gf.Vec3d(*WORLD_POINT)
                offset = target_world - anchor_world
                limit_value = Gf.Dot(offset, axis_world)

                print(f"Joint       : {joint_prim.GetPath()} [{joint_prim.GetTypeName()}]")
                print(f"Body0       : {body0.GetPath()}")
                print(f"Axis        : {axis_token}  (world = {axis_world})")
                print(f"Zero (world): {anchor_world}")
                print(f"Target      : {target_world}")
                print(f"=> limit value (lowerLimit or upperLimit, sign depends on direction) = {limit_value:.6f}")
