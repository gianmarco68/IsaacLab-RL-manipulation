# Read-only. Run this WHILE THE SIMULATION IS PAUSED (not Stopped — Stop resets to the rest pose,
# which defeats the purpose). Select a Joint prim and run: computes the CURRENT position along the
# joint's axis, i.e. the same scalar physics:lowerLimit/upperLimit constrain, directly comparable to
# them. Useful because the Property panel often doesn't expose a live "State" readout for a
# PrismaticJoint in this Kit version.
#
# Math: at the authored zero pose, anchor0_world == anchor1_world (that's what joint_aligner.py
# guarantees) so the projected difference is 0 by construction. As body1 moves relative to body0,
# this tracks the live joint position exactly.

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
                local_rot0 = joint.GetLocalRot0Attr().Get()
                axis_token = prismatic.GetAxisAttr().Get()
                lower_limit = prismatic.GetLowerLimitAttr().Get()
                upper_limit = prismatic.GetUpperLimitAttr().Get()

                anchor0_world = body0_world.Transform(Gf.Vec3d(local_pos0))
                anchor1_world = body1_world.Transform(Gf.Vec3d(local_pos1))

                rot0d = Gf.Quatd(float(local_rot0.GetReal()), Gf.Vec3d(local_rot0.GetImaginary()))
                axis_body0 = Gf.Rotation(rot0d).TransformDir(AXIS_LOCAL[axis_token])
                axis_world = body0_world.TransformDir(axis_body0).GetNormalized()

                current_position = Gf.Dot(anchor1_world - anchor0_world, axis_world)

                print(f"Joint            : {joint_prim.GetPath()}")
                print(f"Axis             : {axis_token}")
                print(f"Current position : {current_position:.6f}  (stage units, along axis)")
                print(f"Lower / Upper    : {lower_limit} / {upper_limit}")
                if lower_limit is not None and upper_limit is not None:
                    if current_position <= lower_limit + 1e-4 or current_position >= upper_limit - 1e-4:
                        print("=> AT (or past) the configured limit — the joint itself is the stop.")
                    else:
                        print("=> WITHIN the configured range — something else is stopping the body (collision, no drive, etc).")
