"""Report where the articulation root sits in the robot USD, and what Isaac Lab would find.

Paste into the Script Editor with the cell scene open. Isaac Lab resolves the robot to the prim
given in the cfg and then searches that subtree for exactly one ArticulationRootAPI: none, or more
than one, and the environment fails to build.
"""

import omni.usd
from pxr import UsdPhysics

# the prim the task cfg spawns the robot under, and the robot root inside the USD
SPAWN_ROOTS = ["/World/m710id_50m", "/World"]

stage = omni.usd.get_context().get_stage()

print("=" * 78)
print("[ARTICULATION ROOTS]  every prim on the stage declaring one")
print("=" * 78)
roots = []
for prim in stage.Traverse():
    if prim.HasAPI(UsdPhysics.ArticulationRootAPI):
        roots.append(prim.GetPath().pathString)
        api = UsdPhysics.ArticulationRootAPI(prim)
        enabled = api.GetArticulationEnabledAttr()
        print(f"  {prim.GetPath().pathString}   enabled = {enabled.Get() if enabled else 'unset'}")
if not roots:
    print("  none on the whole stage")

print()
print("=" * 78)
print("[WHAT ISAAC LAB WOULD SEE]  one root per spawned robot is required")
print("=" * 78)
for spawn in SPAWN_ROOTS:
    under = [r for r in roots if r == spawn or r.startswith(spawn + "/")]
    print(f"  under {spawn}: {len(under)}")
    for path in under:
        print(f"      {path}")
