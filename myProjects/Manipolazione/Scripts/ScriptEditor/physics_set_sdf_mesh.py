# For each selected prim (e.g. `base`), sets the Collider Approximation to SDF Mesh on every
# descendant Mesh that already has CollisionAPI applied. Collider properties only appear in the
# Property panel on Mesh leaf prims, not on the Xform groups (base/bottom_panel/guide_panel/cover)
# that contain them, so selecting the group there shows nothing — this script walks the subtree
# instead, avoiding manually re-selecting every one of the ~100 meshes involved.
#
# SDF Mesh chosen over the default Convex Hull because guide_panel has non-convex geometry
# (grooves/mechanical stops) where the carriages (right_fixed/left_fixed) need precise contact —
# see workspace_notes/SETUP_SCENARIO_CAD.md §6 role 1, and the Omni Physics collider docs
# (SDF is the only way to use non-convex geometry directly on a dynamic/kinematic rigid body
# without falling back to convex decomposition).

import omni.usd
from pxr import Usd, UsdGeom, UsdPhysics

ctx = omni.usd.get_context()
stage = ctx.get_stage()
sel = ctx.get_selection().get_selected_prim_paths()

if not sel:
    print("ERROR: select one or more prims (e.g. `base`) whose Mesh descendants should get SDF Mesh.")
else:
    changed = 0
    for path in sel:
        prim = stage.GetPrimAtPath(path)
        for desc in Usd.PrimRange(prim):
            if desc.IsA(UsdGeom.Mesh) and desc.HasAPI(UsdPhysics.CollisionAPI):
                mesh_collision = UsdPhysics.MeshCollisionAPI.Apply(desc)
                mesh_collision.CreateApproximationAttr().Set("sdf")
                changed += 1

    print(f"Set Approximation=SDF Mesh on {changed} mesh(es).")
