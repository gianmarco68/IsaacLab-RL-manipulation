# Prints the world-space bounding box (min, max, size) of the selected prim, computed from actual
# geometry via UsdGeom.BBoxCache — not read off the viewport by eye. A model 1000x too small can
# look "about right" in the viewport (see workspace_notes/SETUP_SCENARIO_CAD.md §3), and the same
# illusion can hide a too-large result after Robot Assembler's "Begin Assemble", so trust this number
# over the on-screen impression. Run it
# once on a known-good prim (e.g. the FANUC flange or a part with a documented size) to sanity check,
# then on the gripper root.

import omni.usd
from pxr import Usd, UsdGeom

ctx = omni.usd.get_context()
stage = ctx.get_stage()
sel = ctx.get_selection().get_selected_prim_paths()

if not sel:
    print("ERROR: select a prim to measure its world-space bounding box.")
else:
    cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_, UsdGeom.Tokens.render])
    for path in sel:
        prim = stage.GetPrimAtPath(path)
        bbox = cache.ComputeWorldBound(prim)
        rng = bbox.ComputeAlignedBox()
        size = rng.GetMax() - rng.GetMin()
        print(f"{path}: min={rng.GetMin()} max={rng.GetMax()} size={size}")
