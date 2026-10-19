# Read-only. Reports the stage's unit metadata (metersPerUnit / kilogramsPerUnit) alongside the
# actual magnitude of the raw mesh point data, to decide whether the mm->m conversion is carried
# as a 0.001 scale op above the colliders or is baked into the geometry itself.
#
# NVIDIA flagged the scale-above-colliders arrangement as a risk factor in its own right (not just
# a wrong scale, which was already ruled out): a scaled collider hierarchy is worth eliminating by
# baking the unit conversion into the geometry and declaring metersPerUnit instead.
#
# Note this conflicts with SETUP_SCENARIO_CAD.md §3, which prescribes exactly that 0.001 root
# scale — hence this report, to see what is actually on the stage before changing the procedure.

import omni.usd
from pxr import Usd, UsdGeom

ctx = omni.usd.get_context()
stage = ctx.get_stage()

print("=== Stage metadata ===")
print(f"metersPerUnit    = {UsdGeom.GetStageMetersPerUnit(stage)}")
print(f"upAxis           = {UsdGeom.GetStageUpAxis(stage)}")
print(f"defaultPrim      = {stage.GetDefaultPrim().GetPath() if stage.GetDefaultPrim() else None}")

sel = ctx.get_selection().get_selected_prim_paths()
roots = [stage.GetPrimAtPath(p) for p in sel] if sel else [stage.GetPseudoRoot()]

print("\n=== Raw point-data extent of the first few meshes ===")
print("(local-space, BEFORE any parent scale: ~hundreds => data is in mm; ~fractions => meters)")

shown = 0
for root in roots:
    for prim in Usd.PrimRange(root):
        if shown >= 5:
            break
        if not prim.IsA(UsdGeom.Mesh):
            continue
        points = UsdGeom.Mesh(prim).GetPointsAttr().Get()
        if not points:
            continue
        xs = [p[0] for p in points]
        ys = [p[1] for p in points]
        zs = [p[2] for p in points]
        size = (max(xs) - min(xs), max(ys) - min(ys), max(zs) - min(zs))
        print(f"{prim.GetPath()}: local size = ({size[0]:.4f}, {size[1]:.4f}, {size[2]:.4f})")
        shown += 1
