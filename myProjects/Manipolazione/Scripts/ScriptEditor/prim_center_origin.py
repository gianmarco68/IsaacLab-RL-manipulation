# Moves each selected prim's origin to the centre of its own meshes, WITHOUT moving the geometry
# in world space: the prim's transform is shifted and every direct child is compensated by the
# opposite amount.
#
# CAD-imported parts keep whatever origin the CAD had — usually the global origin, far from the
# part itself. That breaks joint_aligner.py, which anchors the joint on Body1's origin, and puts
# the move/rotate gizmo nowhere near the piece.
#
# Math (USD row-vector convention: pointInParent = pointInLocal * M):
#   c = bbox centre expressed in the prim's local space
#   M_prim' = Translate(c)  * M_prim      -> the origin lands on c
#   M_child' = M_child * Translate(-c)    -> geometry stays put
#
# ⚠️ This redefines the prim's local frame, so any Joint that already references it is invalidated:
#    localPos0/localPos1 and localRot0/localRot1 are expressed in the bodies' local spaces. Re-run
#    joint_aligner.py (and re-check the axis) on every joint touching a re-centred prim.
#
# ⚠️ The bbox centre is not necessarily the rotation axis. For a Revolute joint the anchor must sit
#    on the real axis of rotation, which for a rotor is rarely the centre of its bounding box —
#    there, centring the origin makes the gizmo usable but is not by itself the correct anchor.

import omni.usd
from pxr import Usd, UsdGeom, Gf

ctx = omni.usd.get_context()
stage = ctx.get_stage()
sel = ctx.get_selection().get_selected_prim_paths()

if not sel:
    print("ERROR: select one or more prims whose origin should be moved to their mesh centre.")
else:
    bbox_cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default", "render", "proxy", "guide"])
    xform_cache = UsdGeom.XformCache()

    def set_local_matrix(prim, matrix):
        xformable = UsdGeom.Xformable(prim)
        xformable.ClearXformOpOrder()
        xformable.AddTransformOp().Set(matrix)

    for path in sel:
        prim = stage.GetPrimAtPath(path)
        xformable = UsdGeom.Xformable(prim)
        if not xformable:
            print(f"SKIP {path}: not Xformable.")
            continue

        world_range = bbox_cache.ComputeWorldBound(prim).ComputeAlignedRange()
        if world_range.IsEmpty():
            print(f"SKIP {path}: no geometry to centre on.")
            continue

        prim_world = xform_cache.GetLocalToWorldTransform(prim)
        centre_local = prim_world.GetInverse().Transform(world_range.GetMidpoint())

        if Gf.Vec3d(centre_local).GetLength() < 1e-9:
            print(f"SKIP {path}: origin is already at the mesh centre.")
            continue

        to_centre = Gf.Matrix4d().SetTranslate(Gf.Vec3d(centre_local))
        from_centre = Gf.Matrix4d().SetTranslate(-Gf.Vec3d(centre_local))

        children = [c for c in prim.GetChildren() if UsdGeom.Xformable(c)]
        for child in children:
            child_xformable = UsdGeom.Xformable(child)
            set_local_matrix(child, child_xformable.GetLocalTransformation() * from_centre)

        set_local_matrix(prim, to_centre * xformable.GetLocalTransformation())

        print(f"{path}: origin moved by {centre_local}, {len(children)} child(ren) compensated.")
