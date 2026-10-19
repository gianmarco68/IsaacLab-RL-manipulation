# Collapses only the currently selected prims (and their descendants) in the Stage window's
# tree view, equivalent to pressing "-" on each selected prim. Uses the Stage window's public
# StageWidget.collapse(path) method (found in omni.kit.widget.stage's stage_widget.py) rather
# than touching the underlying ui.TreeView directly.

import omni.ui as ui
import omni.usd

stage_window = ui.Workspace.get_window("Stage")

if stage_window is None:
    print("ERROR: could not find the Stage window (is it open?).")
else:
    widget = stage_window.get_widget()
    ctx = omni.usd.get_context()
    selected_paths = ctx.get_selection().get_selected_prim_paths()

    if not selected_paths:
        print("ERROR: no prim selected in the Stage panel.")
    else:
        for path in selected_paths:
            widget.collapse(path)
        print(f"Collapsed {len(selected_paths)} selected prim(s) and their descendants.")
