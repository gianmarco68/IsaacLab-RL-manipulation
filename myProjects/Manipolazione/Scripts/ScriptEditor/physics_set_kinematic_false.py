# For each selected prim (the independently-moving parts, once their Joint is built and ready
# to test), sets Kinematic Enabled = False so the Joint's drive/limits actually control the body
# instead of it being frozen in place. Do NOT include `base` in the selection — it stays
# Kinematic=True permanently as the static root of the chain (role 3 in
# workspace_notes/SETUP_SCENARIO_CAD.md §6), not a moving body.
#
# Two kinematic bodies connected by a Joint are both inert to the solver, which is why
# PhysX refuses to create/drive a Joint between them ("cannot create a joint between static
# bodies") until the moving side is switched to Kinematic=False.

import omni.usd
from pxr import UsdPhysics

ctx = omni.usd.get_context()
stage = ctx.get_stage()
sel = ctx.get_selection().get_selected_prim_paths()

if not sel:
    print("ERROR: select the moving-body prim(s) whose Joint is ready to test.")
else:
    changed = 0
    for path in sel:
        prim = stage.GetPrimAtPath(path)
        if not prim.HasAPI(UsdPhysics.RigidBodyAPI):
            print(f"SKIP {path}: no RigidBodyAPI applied.")
            continue
        rb_api = UsdPhysics.RigidBodyAPI(prim)
        rb_api.CreateKinematicEnabledAttr().Set(False)
        changed += 1

    print(f"Set Kinematic Enabled=False on {changed} body/bodies.")
