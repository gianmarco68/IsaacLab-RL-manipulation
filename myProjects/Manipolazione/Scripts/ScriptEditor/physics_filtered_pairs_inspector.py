# Read-only. Lists every FilteredPairsAPI on the stage (or under the selected prims): which prim
# carries it and which prims it targets. Use it to check the coverage of SETUP_SCENARIO_CAD.md §9
# — every pair of rigid bodies that is mated in the CAD must be protected either by a Joint
# (collisionEnabled=False, the default) or by a filtered pair, otherwise PhysX tries to depenetrate
# the mating faces and floods gpuCollisionStackSize.
#
# Note the API is asymmetric in how it is authored — it lives on one prim of the pair and targets
# the other — but the filtering itself works both ways, so a pair listed once is fully covered.

import omni.usd
from pxr import Usd, UsdPhysics

ctx = omni.usd.get_context()
stage = ctx.get_stage()
sel = ctx.get_selection().get_selected_prim_paths()

roots = [stage.GetPrimAtPath(p) for p in sel] if sel else [stage.GetPseudoRoot()]

found = 0
for root in roots:
    for prim in Usd.PrimRange(root):
        if not prim.HasAPI(UsdPhysics.FilteredPairsAPI):
            continue
        rel = UsdPhysics.FilteredPairsAPI(prim).GetFilteredPairsRel()
        targets = rel.GetTargets() if rel else []
        print(f"{prim.GetPath()}")
        for t in targets:
            print(f"    <-> {t}")
        found += 1

if found == 0:
    print("No FilteredPairsAPI found.")
else:
    print(f"\n{found} prim(s) carrying FilteredPairsAPI.")
