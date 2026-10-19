# Removes every FilteredPairsAPI found under the selection (or the whole stage if nothing is
# selected): clears each prim's FilteredPairsRel targets and removes the API itself. Companion to
# physics_filtered_pairs_inspector.py (read-only listing) and physics_add_filtered_pair.py (adds
# one pair) — use this to wipe an inconsistent set of filters and rebuild only the pairs that are
# actually mated in the CAD (see SETUP_SCENARIO_CAD.md §9), instead of hunting them down one by one.

import omni.usd
from pxr import Usd, UsdPhysics

ctx = omni.usd.get_context()
stage = ctx.get_stage()
sel = ctx.get_selection().get_selected_prim_paths()

roots = [stage.GetPrimAtPath(p) for p in sel] if sel else [stage.GetPseudoRoot()]

cleared = 0
for root in roots:
    for prim in Usd.PrimRange(root):
        if not prim.HasAPI(UsdPhysics.FilteredPairsAPI):
            continue
        rel = UsdPhysics.FilteredPairsAPI(prim).GetFilteredPairsRel()
        targets = rel.GetTargets() if rel else []
        print(f"{prim.GetPath()}: removing {len(targets)} target(s)")
        prim.RemoveAPI(UsdPhysics.FilteredPairsAPI)
        cleared += 1

print(f"\n{cleared} prim(s) cleared." if cleared else "No FilteredPairsAPI found.")
