# Run WHILE PAUSED (not Stopped), after selecting the prim that's stuck (e.g. left_fixed).
# Scans the whole stage for other RigidBody prims whose world bounding box is within MARGIN of the
# selected prim's current world bounding box — i.e. candidates for an unfiltered collision that's
# stopping it, without guessing/disabling bodies one by one.

MARGIN = 0.01  # metres — how close two bboxes must be to be reported

import omni.usd
from pxr import Usd, UsdGeom, UsdPhysics

ctx = omni.usd.get_context()
stage = ctx.get_stage()
sel = ctx.get_selection().get_selected_prim_paths()

if not sel:
    print("ERROR: select the prim that's stuck (e.g. left_fixed).")
else:
    target = stage.GetPrimAtPath(sel[0])
    bbox_cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default", "render", "proxy", "guide"])

    target_range = bbox_cache.ComputeWorldBound(target).ComputeAlignedRange()
    if target_range.IsEmpty():
        print(f"ERROR: {sel[0]} has no geometry to check.")
    else:
        t_min, t_max = target_range.GetMin(), target_range.GetMax()

        def expanded_overlap(a_min, a_max, b_min, b_max, margin):
            for i in range(3):
                if a_max[i] + margin < b_min[i] or b_max[i] + margin < a_min[i]:
                    return False
            return True

        print(f"Target: {sel[0]}  world bbox [{t_min}] .. [{t_max}]")
        print(f"Nearby RigidBody prims (within {MARGIN} m):")

        found = 0

        def walk(prim):
            global found
            if prim != target and prim.HasAPI(UsdPhysics.RigidBodyAPI):
                r = bbox_cache.ComputeWorldBound(prim).ComputeAlignedRange()
                if not r.IsEmpty():
                    if expanded_overlap(t_min, t_max, r.GetMin(), r.GetMax(), MARGIN):
                        print(f"  {prim.GetPath()}")
                        found += 1
            for child in prim.GetChildren():
                walk(child)

        walk(stage.GetPseudoRoot())

        if found == 0:
            print("  (none — try increasing MARGIN, or the obstruction isn't a RigidBody prim itself)")
