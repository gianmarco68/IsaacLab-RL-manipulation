# For each selected Joint prim, enables collision between the two bodies it connects
# (`physics:collisionEnabled`, defaults to False in the USD Physics Joint schema — PhysX
# disables collision between directly-jointed bodies to avoid an immediate contact explosion
# at the shared anchor point). Needed for the collision-based limit measurement method
# (workspace_notes/SETUP_SCENARIO_CAD.md §8): without this, the moving body tunnels straight
# through the static one's collider no matter how accurate the geometry is, because PhysX
# never generates a contact between the pair in the first place.

import omni.usd
from pxr import UsdPhysics

ctx = omni.usd.get_context()
stage = ctx.get_stage()
sel = ctx.get_selection().get_selected_prim_paths()

if not sel:
    print("ERROR: select one or more Joint prims to enable collision on.")
else:
    changed = 0
    for path in sel:
        prim = stage.GetPrimAtPath(path)
        joint = UsdPhysics.Joint(prim)
        if not joint:
            print(f"SKIP {path}: not a UsdPhysics Joint prim.")
            continue
        joint.CreateCollisionEnabledAttr().Set(True)
        changed += 1

    print(f"Enabled collision on {changed} joint(s).")
