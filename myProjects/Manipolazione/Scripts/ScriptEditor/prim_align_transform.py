# Aligns MOVE_PRIM_PATH's world position + orientation to REFERENCE_PRIM_PATH's, keeping
# MOVE_PRIM_PATH's own local scale. Paths are hardcoded (not read from viewport selection) because
# selection order isn't reliably preserved — right-click a prim -> "Copy Prim Path" -> paste below.
#
# Rotation via Gf.Matrix4d.RemoveScaleShear(), not Gf.Transform decomposition, so scale leaking in
# from the parent chain can't corrupt it.
#
# Undo works via omni.kit.commands only if the prim already has a single xformOp:transform; a
# multi-op CAD stack forces a direct, non-undoable USD edit instead (a warning is printed then).

MOVE_PRIM_PATH = "/gripper/gripper_body/components/right_claw/right_forefinger/tn__501154891_3_"
REFERENCE_PRIM_PATH = "/gripper/gripper_body/components/right_claw/right_forefinger/tn__59_056_06_02_0_00052376_4_"

# True  = re-anchor mode: MOVE_PRIM_PATH's frame moves, children are compensated so their geometry
#         stays fixed in world space
# False = attach mode: MOVE_PRIM_PATH's frame moves and children move with it, unchanged, so the
#         geometry actually relocates to REFERENCE_PRIM_PATH
COMPENSATE_CHILDREN = True

import omni.usd
import omni.kit.commands
from pxr import Gf, UsdGeom

ctx = omni.usd.get_context()
stage = ctx.get_stage()

if not MOVE_PRIM_PATH or not REFERENCE_PRIM_PATH:
    print("ERROR: set MOVE_PRIM_PATH and REFERENCE_PRIM_PATH at the top of the script "
          "(Stage panel -> right-click prim -> Copy Prim Path).")
else:
    move_prim = stage.GetPrimAtPath(MOVE_PRIM_PATH)
    ref_prim = stage.GetPrimAtPath(REFERENCE_PRIM_PATH)

    if not move_prim.IsValid():
        print(f"ERROR: MOVE_PRIM_PATH {MOVE_PRIM_PATH} does not exist on the stage.")
    elif not ref_prim.IsValid():
        print(f"ERROR: REFERENCE_PRIM_PATH {REFERENCE_PRIM_PATH} does not exist on the stage.")
    else:
        xformable = UsdGeom.Xformable(move_prim)
        if not xformable:
            print(f"ERROR: {MOVE_PRIM_PATH} is not Xformable.")
        else:
            print(f"Aligning {MOVE_PRIM_PATH} -> {REFERENCE_PRIM_PATH}")

            xf_cache = UsdGeom.XformCache()

            # Reference: world position + orientation only, its scale is discarded.
            ref_world = xf_cache.GetLocalToWorldTransform(ref_prim)
            desired_world = ref_world.RemoveScaleShear()

            # Preserve the moved prim's own current local scale.
            move_local = xformable.GetLocalTransformation()
            move_scale = Gf.Transform(move_local).GetScale()

            # Back to the moved prim's parent space, then strip whatever scale/shear the parent's
            # inverse (e.g. the 0.001 unit-fix scale on the root) leaked into the linear part.
            parent_world = xf_cache.GetLocalToWorldTransform(move_prim.GetParent())
            local_rt = (desired_world * parent_world.GetInverse()).RemoveScaleShear()

            new_local = Gf.Matrix4d(1.0)
            new_local.SetScale(move_scale)
            new_local = new_local * local_rt

            # Compensation applied to every direct child so geometry stays fixed in world space:
            # Child_World unchanged  =>  Child_Local' = Child_Local * (Move_Local * Move_Local'^-1)
            compensation = move_local * new_local.GetInverse()

            def apply_matrix(prim, new_matrix, old_matrix):
                prim_xformable = UsdGeom.Xformable(prim)
                ops = prim_xformable.GetOrderedXformOps()
                if len(ops) == 1 and ops[0].GetOpType() == UsdGeom.XformOp.TypeTransform:
                    omni.kit.commands.execute(
                        "ChangeProperty",
                        prop_path=ops[0].GetAttr().GetPath(),
                        value=new_matrix,
                        prev=old_matrix,
                    )
                    return True
                prim_xformable.ClearXformOpOrder()
                prim_xformable.AddTransformOp().Set(new_matrix)
                return False

            undoable = apply_matrix(move_prim, new_local, move_local)
            if not undoable:
                print(f"WARNING: {MOVE_PRIM_PATH} had a multi-op xform stack, collapsed to a "
                      "single matrix op via direct USD edit — NOT undoable with Ctrl+Z.")

            children = [c for c in move_prim.GetChildren() if UsdGeom.Xformable(c)]
            if COMPENSATE_CHILDREN:
                for child in children:
                    child_xformable = UsdGeom.Xformable(child)
                    child_old = child_xformable.GetLocalTransformation()
                    child_new = child_old * compensation
                    child_undoable = apply_matrix(child, child_new, child_old)
                    if not child_undoable:
                        print(f"WARNING: {child.GetPath()} had a multi-op xform stack, collapsed to a "
                              "single matrix op via direct USD edit — NOT undoable with Ctrl+Z.")
                print(f"Aligned. {len(children)} child(ren) compensated to keep geometry fixed.")
            else:
                print(f"Aligned. {len(children)} child(ren) left as-is, geometry moves with the prim.")
            print("New local transform:")
            print(new_local)
