# Dumps xformOpOrder and every xformOp:* attribute on the selected prim, marking which ops are
# actually wired into xformOpOrder (and therefore part of the effective transform used by
# rendering/PhysX) versus attributes that exist on the prim but are NOT listed in xformOpOrder
# (orphaned — shown in some Property panel fields but inert). Use it when check_residual_scale.py
# reports "no scale op" on a prim while the Property panel shows a Scale value: it resolves whether
# that value is real (in the order) or an orphaned attribute.

import omni.usd
from pxr import Usd, UsdGeom

ctx = omni.usd.get_context()
stage = ctx.get_stage()
sel = ctx.get_selection().get_selected_prim_paths()

if not sel:
    print("ERROR: select a prim to dump its xform ops.")
else:
    for path in sel:
        prim = stage.GetPrimAtPath(path)
        print(f"--- {path} ---")

        xformable = UsdGeom.Xformable(prim)
        order_attr = xformable.GetXformOpOrderAttr()
        order = order_attr.Get() if order_attr and order_attr.HasAuthoredValue() else None
        print(f"xformOpOrder = {list(order) if order else '(none authored)'}")

        wired_names = set(order) if order else set()

        for prop in prim.GetProperties():
            name = prop.GetName()
            if name.startswith("xformOp:"):
                attr = prim.GetAttribute(name)
                value = attr.Get() if attr else None
                status = "WIRED" if name in wired_names else "ORPHANED (not in xformOpOrder)"
                print(f"  {name} = {value}   [{status}]")

        print("Ops actually returned by GetOrderedXformOps():")
        for op in xformable.GetOrderedXformOps():
            print(f"  {op.GetOpName()} ({op.GetOpType()}) = {op.Get()}")
