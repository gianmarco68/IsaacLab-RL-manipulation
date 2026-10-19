# Analyzes the subtree of the selected prim (or the whole stage if nothing is selected),
# distinguishing "group" prims (assembly/subassembly, kind set by the CAD Converter) from
# "component" prims (leaf part, contains the Mesh). Prints a readable indented tree
# regardless of how the Stage window is currently displaying prims (flat or hierarchical),
# and finally selects all "group" prims found so they show highlighted in the Stage panel
# for an immediate visual check.

import omni.usd
from pxr import Usd

ctx = omni.usd.get_context()
stage = ctx.get_stage()
sel = ctx.get_selection().get_selected_prim_paths()

start_prim = stage.GetPrimAtPath(sel[0]) if sel else stage.GetPseudoRoot()
print(f"Analysis root: {start_prim.GetPath()}")

group_paths = []


def walk(prim, depth=0):
    kind = Usd.ModelAPI(prim).GetKind()
    label = kind if kind else "-"
    n_children = len(prim.GetChildren())
    print("  " * depth + f"{prim.GetName()}  [{label}]  ({n_children} children)")

    if kind == "group" or kind == "assembly":
        group_paths.append(prim.GetPath().pathString)

    for child in prim.GetChildren():
        walk(child, depth + 1)


for child in start_prim.GetChildren():
    walk(child)

print(f"\nFound {len(group_paths)} group/assembly prims.")
ctx.get_selection().set_selected_prim_paths(group_paths, True)
print("Selected in the Stage panel for visual check.")
