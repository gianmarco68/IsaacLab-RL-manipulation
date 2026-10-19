# Runs prim_center_origin.py's logic (move each prim's origin to its mesh centre, without moving
# geometry in world space) on the fixed list of left-claw moving bodies below, so it can be
# re-run without reselecting them every time.
#
# ⚠️ Same caveats as prim_center_origin.py: invalidates any Joint already referencing these prims
# (re-run joint_aligner.py after); bbox centre is NOT the rotation axis for finger_guide's Revolute
# joint to left_fixed (only useful there for making the Move gizmo usable, not as the anchor).

PATHS = [
    "/gripper/gripper_body/components/left_claw/left_fixed",
    "/gripper/gripper_body/components/left_claw/rotor/finger_guide",
    "/gripper/gripper_body/components/left_claw/rotor/left_thumb",
    "/gripper/gripper_body/components/left_claw/rotor/left_forefinger",
]

import omni.usd
from pxr import Usd, UsdGeom, Gf

ctx = omni.usd.get_context()
stage = ctx.get_stage()

bbox_cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default", "render", "proxy", "guide"])
xform_cache = UsdGeom.XformCache()


def set_local_matrix(prim, matrix):
    xformable = UsdGeom.Xformable(prim)
    xformable.ClearXformOpOrder()
    xformable.AddTransformOp().Set(matrix)


for path in PATHS:
    prim = stage.GetPrimAtPath(path)
    if not prim.IsValid():
        print(f"SKIP {path}: prim not found.")
        continue

    xformable = UsdGeom.Xformable(prim)
    if not xformable:
        print(f"SKIP {path}: not Xformable.")
        continue

    world_range = bbox_cache.ComputeWorldBound(prim).ComputeAlignedRange()
    if world_range.IsEmpty():
        print(f"SKIP {path}: no geometry to centre on.")
        continue

    prim_world = xform_cache.GetLocalToWorldTransform(prim)
    centre_local = prim_world.GetInverse().Transform(world_range.GetMidpoint())

    if Gf.Vec3d(centre_local).GetLength() < 1e-9:
        print(f"SKIP {path}: origin is already at the mesh centre.")
        continue

    to_centre = Gf.Matrix4d().SetTranslate(Gf.Vec3d(centre_local))
    from_centre = Gf.Matrix4d().SetTranslate(-Gf.Vec3d(centre_local))

    children = [c for c in prim.GetChildren() if UsdGeom.Xformable(c)]
    for child in children:
        child_xformable = UsdGeom.Xformable(child)
        set_local_matrix(child, child_xformable.GetLocalTransformation() * from_centre)

    set_local_matrix(prim, to_centre * xformable.GetLocalTransformation())

    print(f"{path}: origin moved by {centre_local}, {len(children)} child(ren) compensated.")
