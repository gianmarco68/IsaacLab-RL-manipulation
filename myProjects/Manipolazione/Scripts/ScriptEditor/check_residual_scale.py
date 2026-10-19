# Walks the selected prim's ancestor chain (up to the stage root) and its own xformOpOrder,
# printing every scale op found. Detects a residual ~1000x scale (see
# workspace_notes/SETUP_SCENARIO_CAD.md §3) — a leftover scale like that throws off PhysX's contact
# offset/rest offset margins and can make a rigid body tunnel through a collider instead of stopping
# cleanly on contact.
#
# Also decomposes xformOp:transform (4x4 matrix) ops for embedded scale: tools that align frames
# by computing a relative transform (e.g. the Robot Assembler's "Begin Assemble") commonly write
# a single matrix op instead of separate translate/rotate/scale ops, and a plain TypeScale check
# reports "no scale op" even when the matrix carries a non-unity scale.

import omni.usd
from pxr import Usd, UsdGeom, Gf

ctx = omni.usd.get_context()
stage = ctx.get_stage()
sel = ctx.get_selection().get_selected_prim_paths()

def matrix_scale(m):
    row0 = Gf.Vec3d(m[0][0], m[0][1], m[0][2])
    row1 = Gf.Vec3d(m[1][0], m[1][1], m[1][2])
    row2 = Gf.Vec3d(m[2][0], m[2][1], m[2][2])
    return (row0.GetLength(), row1.GetLength(), row2.GetLength())

if not sel:
    print("ERROR: select a prim (e.g. `components` or `right_fixed`) to inspect its scale chain.")
else:
    for path in sel:
        prim = stage.GetPrimAtPath(path)
        print(f"--- Scale chain for {path} (root -> prim) ---")

        chain = []
        p = prim
        while p and p != stage.GetPseudoRoot():
            chain.append(p)
            p = p.GetParent()
        chain.reverse()

        for p in chain:
            xformable = UsdGeom.Xformable(p)
            if not xformable:
                continue
            ops = xformable.GetOrderedXformOps()
            found = False
            for op in ops:
                if op.GetOpType() == UsdGeom.XformOp.TypeScale:
                    print(f"{p.GetPath()}: scale op = {op.Get()}")
                    found = True
                elif op.GetOpType() == UsdGeom.XformOp.TypeTransform:
                    value = op.Get()
                    if value is not None:
                        s = matrix_scale(Gf.Matrix4d(value))
                        if any(abs(c - 1.0) > 1e-4 for c in s):
                            print(f"{p.GetPath()}: transform matrix op embeds scale = {s}")
                            found = True
            if not found:
                print(f"{p.GetPath()}: no scale op")
