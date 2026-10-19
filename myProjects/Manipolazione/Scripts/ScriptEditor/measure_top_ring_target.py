"""Place each grasp target at the middle of a side, on the axis of the topmost ring.

Paste into the Script Editor. Works on the full cell or on the workpiece USD opened alone, with or
without the simulation playing: everything is measured in the frame of the rigid body itself, from
authored geometry, so it does not depend on where the workpiece sits or on the robot.

The bounding box alone is not enough. Its top face is the outer surface of the top ring, half a bar
thickness above the ring's axis, and the axis is what a gripper closes onto. The bar radius is read
from the cross-section of the side itself rather than assumed.
"""

import os

import numpy as np
import omni.usd
from pxr import Gf, Usd, UsdGeom, UsdPhysics

STAFFA_BODY_PATH = "/World/staffa_quadrata/structure"
# fallback when the workpiece USD is opened on its own and the path above does not exist
FALLBACK_BODY_NAME = "structure"

# how wide a strip around the middle of a side is sampled, along that side
SAMPLE_SPAN_M = 0.04
# how deep below the highest point a sample is still considered part of the top ring
TOP_SLAB_M = 0.05

OUT = "C:/Projects/myProjects/Manipolazione/Scripts/logs/measure_top_ring_target.txt"

lines = []


def say(text=""):
    print(text)
    lines.append(text)


def fmt(v, digits=6):
    return "(" + ", ".join(f"{x:.{digits}f}" for x in np.asarray(v, dtype=float).ravel()) + ")"


def matrix_to_numpy(matrix):
    return np.array([[matrix[r][c] for c in range(4)] for r in range(4)])


def find_body(stage):
    """The prim the object frames are declared on: the rigid body, wherever the file puts it."""
    prim = stage.GetPrimAtPath(STAFFA_BODY_PATH)
    if prim and prim.IsValid():
        return prim
    for candidate in stage.Traverse():
        if candidate.GetName() == FALLBACK_BODY_NAME and candidate.HasAPI(UsdPhysics.RigidBodyAPI):
            return candidate
    for candidate in stage.Traverse():
        if candidate.HasAPI(UsdPhysics.RigidBodyAPI):
            return candidate
    return None


stage = omni.usd.get_context().get_stage()
body = find_body(stage)
if body is None:
    say("No rigid body found. Open the cell or the workpiece USD and run again.")
    raise SystemExit("no rigid body")

say("=" * 78)
say("[BODY]")
say("=" * 78)
say(f"  {body.GetPath().pathString}")

# every mesh point, expressed in the body's own frame and in metres: the body frame is what
# OffsetCfg is relative to, and the scale above it decides the units the points come in
body_l2w = UsdGeom.Xformable(body).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
to_body = matrix_to_numpy(body_l2w.GetInverse())
scale = np.linalg.norm(matrix_to_numpy(body_l2w)[:3, :3], axis=1)
say(f"  world scale = {fmt(scale, 8)}")

chunks = []
for prim in Usd.PrimRange(body):
    mesh = UsdGeom.Mesh(prim)
    if not mesh:
        continue
    points = mesh.GetPointsAttr().Get()
    if not points:
        continue
    local = matrix_to_numpy(
        UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    )
    raw = np.array([[p[0], p[1], p[2], 1.0] for p in points])
    chunks.append((raw @ local @ to_body)[:, :3])

if not chunks:
    say("  No mesh points under this body.")
    raise SystemExit("no geometry")

pts = np.vstack(chunks) * scale
bmin, bmax = pts.min(axis=0), pts.max(axis=0)
centre, half = (bmin + bmax) / 2.0, (bmax - bmin) / 2.0

say()
say("=" * 78)
say("[BOUNDING BOX]  in the body frame, in metres")
say("=" * 78)
say(f"  {len(pts)} points")
say(f"  min = {fmt(bmin)}")
say(f"  max = {fmt(bmax)}")
say(f"  centre = {fmt(centre)}   half extents = {fmt(half)}")
if max(half[0], half[1]) > 1.0:
    say("  WARNING: the side is over 2 m. The scale above is not being applied; the numbers below")
    say("  are in the file's own units, not metres.")

# a side is sampled in a narrow strip around its middle, so the corners and the opposite sides do
# not enter the cross-section
SIDES = {
    # name: (axis running along the side, axis across it, sign of the sampled side)
    "right": (0, 1, -1),
    "left": (1, 0, -1),
}

say()
say("=" * 78)
say("[TOP RING]  cross-section of each side at its middle")
say("=" * 78)
targets = {}
for name, (along, across, sign) in SIDES.items():
    strip = pts[np.abs(pts[:, along] - centre[along]) < SAMPLE_SPAN_M]
    edge = centre[across] + sign * half[across]
    strip = strip[np.abs(strip[:, across] - edge) < 2 * SAMPLE_SPAN_M]
    if len(strip) < 10:
        say(f"  {name:<6} only {len(strip)} points in the strip; widen SAMPLE_SPAN_M.")
        continue
    slab = strip[strip[:, 2] > strip[:, 2].max() - TOP_SLAB_M]
    # the bar is round, so its diameter across the side is the same as its height
    radius = (slab[:, across].max() - slab[:, across].min()) / 2.0
    axis_across = (slab[:, across].max() + slab[:, across].min()) / 2.0
    axis_z = slab[:, 2].max() - radius

    target = np.array(centre)
    target[along] = centre[along]
    target[across] = axis_across
    target[2] = axis_z
    targets[name] = target

    say(f"  {name:<6} {len(strip)} points in the strip, {len(slab)} in the top slab")
    say(f"  {'':<6} bar radius = {radius * 1000:.2f} mm   diameter = {radius * 2000:.2f} mm")
    say(f"  {'':<6} top surface z = {slab[:, 2].max():.6f}   ring axis z = {axis_z:.6f}")
    say(f"  {'':<6} target = {fmt(target)}")

say()
say("=" * 78)
say("[PASTE INTO move_to_grasp_apps_cfg.py]")
say("=" * 78)
ROT = {"right": (1.0, 0.0, 0.0, 0.0), "left": (0.70710678, 0.0, 0.0, 0.70710678)}
for name in ("right", "left"):
    if name not in targets:
        continue
    say(f"  {name}_grasp_target OffsetCfg(pos={fmt(targets[name])}, rot={fmt(ROT[name], 8)})")

os.makedirs(os.path.dirname(OUT), exist_ok=True)
with open(OUT, "w") as handle:
    handle.write("\n".join(lines))
print(f"written to {OUT}")
