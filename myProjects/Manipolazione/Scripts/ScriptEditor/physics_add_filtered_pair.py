# Select exactly two prims (two rigid bodies) and run: applies FilteredPairsAPI so PhysX never
# generates contacts between them, while both keep colliding normally with everything else.
#
# Use for pairs that are permanently mated in the CAD and whose relative motion is already fully
# constrained by a Joint — e.g. the carriage and the rail it slides on. Their travel comes from
# the joint's authored limits, so contact between them buys nothing and costs a great deal: CAD
# parts are modelled in contact, so PhysX spends every step trying to depenetrate mating faces.
#
# This is the targeted alternative to switching collision off wholesale on a body: unlike removing
# CollisionAPI, the parts still collide with the workpiece and the outside world.

import omni.usd
from pxr import UsdPhysics

ctx = omni.usd.get_context()
stage = ctx.get_stage()
sel = ctx.get_selection().get_selected_prim_paths()

if len(sel) != 2:
    print(f"ERROR: select exactly two prims (got {len(sel)}).")
else:
    path_a, path_b = sel
    prim_a = stage.GetPrimAtPath(path_a)

    filtered = UsdPhysics.FilteredPairsAPI.Apply(prim_a)
    rel = filtered.CreateFilteredPairsRel()
    existing = rel.GetTargets()
    if path_b in existing:
        print(f"Already filtered: {path_a} <-> {path_b}")
    else:
        rel.AddTarget(path_b)
        print(f"Filtered pair added: {path_a} <-> {path_b}")
