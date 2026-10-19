# Moves each selected prim's LOCAL ROTATION to identity, WITHOUT moving the geometry in world
# space: the prim's transform loses its rotation component, and every direct child is compensated
# by the inverse amount, exactly like prim_center_origin.py does for translation.
#
# CAD-imported parts sometimes keep a stray local rotation from the source file, unrelated to the
# part's own natural orientation (visible symptom: the local-space bounding box gizmo looks
# oversized and tilted by some odd angle in the viewport, instead of a tight box aligned with the
# mesh — because the box is axis-aligned in a local frame that isn't aligned with the geometry).
#
# Math (USD row-vector convention: pointInParent = pointInLocal * M):
#   M_new  = Transform(translation=t_old, scale=s_old, rotation=identity)
#   comp   = M_old * M_new^-1                  -> what rotation "leaked out" of the prim
#   M_child' = M_child * comp                  -> children absorb it, geometry stays put
#
# ⚠️ Same caveat as prim_center_origin.py: redefines the prim's local frame, invalidating any Joint
#    that already references it (Local Rotation 0/1 are expressed in the bodies' local spaces).
#    Re-run joint_aligner.py AND re-check the axis (Local Rotation 0/1, print_joint_axis_world.py)
#    on every joint touching a prim processed by this script.

import omni.usd
from pxr import Usd, UsdGeom, Gf

ctx = omni.usd.get_context()
stage = ctx.get_stage()
sel = ctx.get_selection().get_selected_prim_paths()

if not sel:
    print("ERROR: select one or more prims whose local rotation should be zeroed.")
else:
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

        old_matrix = xformable.GetLocalTransformation()
        old_transform = Gf.Transform(old_matrix)

        if old_transform.GetRotation().GetAngle() < 1e-6:
            print(f"SKIP {path}: local rotation is already ~identity.")
            continue

        new_transform = Gf.Transform()
        new_transform.SetTranslation(old_transform.GetTranslation())
        new_transform.SetScale(old_transform.GetScale())
        # Rotation left as identity (default).
        new_matrix = new_transform.GetMatrix()

        compensation = old_matrix * new_matrix.GetInverse()

        children = [c for c in prim.GetChildren() if UsdGeom.Xformable(c)]
        for child in children:
            child_xformable = UsdGeom.Xformable(child)
            set_local_matrix(child, child_xformable.GetLocalTransformation() * compensation)

        set_local_matrix(prim, new_matrix)

        print(f"{path}: local rotation zeroed ({old_transform.GetRotation().GetAngle():.2f} deg removed), {len(children)} child(ren) compensated.")
