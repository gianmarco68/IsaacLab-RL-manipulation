# Lists every prim in the stage whose name matches the selected prim's name (case-insensitive
# substring), across the whole stage tree, with their full paths and world-space translate/scale.
# Useful after the Robot Assembler's "Begin Assemble", which may leave the original gripper prim in
# the stage alongside a second copy moved onto the FANUC flange; the two overlapping or misaligned
# copies can read visually as one oversized prim rather than as two (see
# workspace_notes/SETUP_SCENARIO_CAD.md §11).

import omni.usd
from pxr import Usd, UsdGeom

ctx = omni.usd.get_context()
stage = ctx.get_stage()
sel = ctx.get_selection().get_selected_prim_paths()

if not sel:
    print("ERROR: select the prim whose name you want to search for (e.g. the gripper root).")
else:
    needle = stage.GetPrimAtPath(sel[0]).GetName().lower()
    print(f"--- Prims matching name '{needle}' ---")
    matches = []
    for prim in stage.Traverse():
        if needle in prim.GetName().lower():
            matches.append(prim)

    if len(matches) <= 1:
        print(f"Only {len(matches)} match(es) — no duplication by name.")
    else:
        print(f"{len(matches)} matches found:")

    for prim in matches:
        xformable = UsdGeom.Xformable(prim)
        world = xformable.ComputeLocalToWorldTransform(Usd.TimeCode.Default()) if xformable else None
        translate = world.ExtractTranslation() if world else None
        print(f"{prim.GetPath()}  active={prim.IsActive()}  world_translate={translate}")
