# For each selected prim (one per independently-moving part, e.g. right_forefinger,
# right_thumb, right_fixed, finger_guide, left_forefinger, left_thumb, left_fixed — select
# all of them at once with Ctrl+click), ensures:
#   - RigidBodyAPI is applied, with Kinematic Enabled = True (safety net: keeps the part
#     frozen in place until its Joint is built, instead of falling under gravity)
#   - CollisionAPI + Convex Hull approximation is applied to every descendant Mesh that
#     doesn't already have one
# Writes directly via pxr, so it isn't affected by Kit's UI restriction that hides "Rigid
# Body" from the quick-add menu when a prim already sits inside another RigidBody's subtree.
#
# NOTE: the static base group (bottom_panel/guide_panel/cover under `base`) needs SDF Mesh
# instead of Convex Hull once it has RigidBody applied — fix that separately afterward.

import omni.usd
from pxr import Usd, UsdGeom, UsdPhysics

ctx = omni.usd.get_context()
stage = ctx.get_stage()
sel = ctx.get_selection().get_selected_prim_paths()

if not sel:
    print("ERROR: select one or more prims to fix up (each an independently moving part).")
else:
    for path in sel:
        prim = stage.GetPrimAtPath(path)

        rb_api = UsdPhysics.RigidBodyAPI.Apply(prim)
        rb_api.CreateKinematicEnabledAttr().Set(True)

        added = 0
        for desc in Usd.PrimRange(prim):
            if desc.IsA(UsdGeom.Mesh) and not desc.HasAPI(UsdPhysics.CollisionAPI):
                UsdPhysics.CollisionAPI.Apply(desc)
                mesh_collision = UsdPhysics.MeshCollisionAPI.Apply(desc)
                mesh_collision.CreateApproximationAttr().Set("convexHull")
                added += 1

        print(f"{path}: RigidBody ensured (kinematic=True), Collider added to {added} mesh(es).")
