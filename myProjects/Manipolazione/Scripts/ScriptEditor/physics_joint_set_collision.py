# Sets `physics:collisionEnabled` on each selected Joint prim. Flip the constant below.
#
# The USD Physics Joint schema defaults this to False: PhysX disables collision between two
# directly-jointed bodies because they normally start touching at the shared anchor, and a CAD
# assembly is modelled in contact — leaving it on makes PhysX try to depenetrate mating faces and
# generate an enormous contact set (this is what requested ~840 MB of gpuCollisionStackSize here,
# against a 64 MiB default, after which contacts were dropped and the carriage tunneled through
# the rail).
#
# Keep this False. Joint travel is declared with authored physics:lowerLimit/upperLimit taken from
# CAD dimensions, not measured by collision (SETUP_SCENARIO_CAD.md §8).

ENABLE_COLLISION = False

import omni.usd
from pxr import UsdPhysics

ctx = omni.usd.get_context()
stage = ctx.get_stage()
sel = ctx.get_selection().get_selected_prim_paths()

if not sel:
    print("ERROR: select one or more Joint prims.")
else:
    changed = 0
    for path in sel:
        prim = stage.GetPrimAtPath(path)
        joint = UsdPhysics.Joint(prim)
        if not joint:
            print(f"SKIP {path}: not a UsdPhysics Joint prim.")
            continue
        joint.CreateCollisionEnabledAttr().Set(ENABLE_COLLISION)
        changed += 1

    print(f"Set collisionEnabled={ENABLE_COLLISION} on {changed} joint(s).")
