# For each selected prim, reverts the Collider Approximation to Convex Hull on every descendant
# Mesh that has CollisionAPI applied. Companion/undo script for physics_set_sdf_mesh.py.
#
# SDF colliders are not used for joint end-stops (SETUP_SCENARIO_CAD.md §8): joint travel comes from
# authored physics:lowerLimit/upperLimit taken from CAD dimensions, which is exact, cannot be tunneled
# through, and costs nothing — whereas SDF contact generation on a mesh-dense assembly can overflow
# PhysX's gpuCollisionStackSize (64 MiB default). Convex Hull is the approximation to use.

import omni.usd
from pxr import Usd, UsdGeom, UsdPhysics

ctx = omni.usd.get_context()
stage = ctx.get_stage()
sel = ctx.get_selection().get_selected_prim_paths()

if not sel:
    print("ERROR: select one or more prims whose Mesh descendants should revert to Convex Hull.")
else:
    changed = 0
    for path in sel:
        prim = stage.GetPrimAtPath(path)
        for desc in Usd.PrimRange(prim):
            if desc.IsA(UsdGeom.Mesh) and desc.HasAPI(UsdPhysics.CollisionAPI):
                mesh_collision = UsdPhysics.MeshCollisionAPI.Apply(desc)
                mesh_collision.CreateApproximationAttr().Set("convexHull")
                changed += 1

    print(f"Set Approximation=Convex Hull on {changed} mesh(es).")
