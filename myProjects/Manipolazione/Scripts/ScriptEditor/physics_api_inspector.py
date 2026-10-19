# Reports, for the selected prim(s) (or the whole stage if nothing is selected), which physics
# APIs are currently applied down the subtree: RigidBody (+ kinematic flag) and Collider
# (+ approximation) on every Mesh descendant. Use this before adding Joints to catch missing
# APIs (e.g. a moving part with RigidBody but no Collider, which PhysX otherwise only flags
# with a mass/inertia warning at Play time).

import omni.usd
from pxr import Usd, UsdGeom, UsdPhysics

ctx = omni.usd.get_context()
stage = ctx.get_stage()
sel = ctx.get_selection().get_selected_prim_paths()

roots = [stage.GetPrimAtPath(p) for p in sel] if sel else [stage.GetPseudoRoot()]

# Also written to a file: the Script Editor console can't be copied from and
# long output scrolls out of view, so the file lets it be read outside Isaac Sim.
OUTPUT_PATH = r"C:\Projects\myProjects\Manipolazione\Scripts\logs\physics_api_inspector_output.txt"

lines = []


def report(prim):
    tags = []

    if prim.HasAPI(UsdPhysics.RigidBodyAPI):
        rb = UsdPhysics.RigidBodyAPI(prim)
        kinematic = rb.GetKinematicEnabledAttr().Get()
        tags.append(f"RigidBody(kinematic={kinematic})")

    if prim.HasAPI(UsdPhysics.CollisionAPI):
        collision = UsdPhysics.CollisionAPI(prim)
        enabled = collision.GetCollisionEnabledAttr().Get()
        mesh_collision = UsdPhysics.MeshCollisionAPI(prim)
        approx = mesh_collision.GetApproximationAttr().Get() if mesh_collision else None
        tags.append(f"Collider(enabled={enabled}, approximation={approx})")

    # Only report prims that actually carry a physics API: with dozens of plain
    # mesh/xform prims in the tree, including everything buries the relevant lines.
    # Full path (instead of indentation) keeps each line self-contained once filtered.
    if tags:
        lines.append(f"{prim.GetPath()} [{prim.GetTypeName()}]  <- {', '.join(tags)}")

    for child in prim.GetChildren():
        report(child)


for root in roots:
    report(root)

with open(OUTPUT_PATH, "w") as f:
    f.write("\n".join(lines) + "\n")

print(f"Wrote {len(lines)} line(s) to {OUTPUT_PATH}")
