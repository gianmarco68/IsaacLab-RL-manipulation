# Select one or more prims and run: prints each one's world-space position (origin of its local
# frame transformed to world), read via GetLocalToWorldTransform — not the Property panel, which
# defaults to Local and is easy to misread on CAD-imported prims with inherited rotations.

import omni.usd
from pxr import Usd, UsdGeom, Gf

ctx = omni.usd.get_context()
stage = ctx.get_stage()
sel = ctx.get_selection().get_selected_prim_paths()

if not sel:
    print("ERROR: no prim selected.")
else:
    xform_cache = UsdGeom.XformCache()
    for path in sel:
        prim = stage.GetPrimAtPath(path)
        if not UsdGeom.Xformable(prim):
            print(f"{path}: not Xformable, skipped.")
            continue
        world = xform_cache.GetLocalToWorldTransform(prim)
        pos = world.Transform(Gf.Vec3d(0, 0, 0))
        print(f"{path}  world position = ({pos[0]:.6f}, {pos[1]:.6f}, {pos[2]:.6f})")
