# ScriptEditor

Utilities meant to be **pasted into the Script Editor of Isaac Sim** (the standalone copy in
`C:\isaacsim`, not the pixi one) and to act on the current selection in the Stage. They use
`omni.usd`, so they do not run outside Isaac Sim. Usage procedure and context are in
`workspace_notes/SETUP_SCENARIO_CAD.md`.

## Physics: RigidBody, Collider, Joint

| Script | Purpose |
|---|---|
| `physics_api_inspector.py` | Report of RigidBody / Collider / approximation over a subtree; also writes a `.txt`, because the Script Editor console cannot be copied |
| `physics_api_fixup.py` | RigidBody + Kinematic + Collider in one go on a multi-selection |
| `physics_set_convex_hull.py` · `physics_set_sdf_mesh.py` | Change the collider Approximation on a whole subtree |
| `physics_set_kinematic_false.py` | Kinematic = False on the moving bodies once their Joint is ready |
| `physics_joint_set_collision.py` | Sets `collisionEnabled` on Joints (constant at the top of the file) |
| `physics_joint_enable_collision.py` | Enables collision between the two bodies of each selected Joint (needed to measure limits by collision) |
| `physics_joint_inspector.py` | Body0/Body1, `collisionEnabled`, effective limits |
| `physics_add_filtered_pair.py` | `FilteredPairsAPI` between the two selected prims |
| `physics_filtered_pairs_inspector.py` | Lists every filtered pair on the stage |
| `physics_clear_filtered_pairs.py` | Removes every `FilteredPairsAPI` from the selection (or the whole stage), to start clean and rebuild only the pairs that are needed |

## Joint setup and checks

| Script | Purpose |
|---|---|
| `joint_aligner.py` | Writes `localPos0/1` so both anchors land on the same world point (3 `ANCHOR_MODE`s, see the guide §7.3) |
| `print_joint_anchor_world.py` | World anchor of Body0/Body1 plus the gap: checks alignment with numbers, not with the gizmo |
| `print_joint_current_state.py` | Current position of the Joint along its axis versus its limits — run while **paused** |
| `measure_joint_limit.py` | Distance projected on the Joint axis between its zero and a `WORLD_POINT`: the direct value for `lowerLimit` / `upperLimit` |
| `measure_rail_travel.py` | Travel and limits from bounding boxes (rail, then carriage) |
| `check_articulation_root.py` | Where the articulation root sits in the robot USD, and what Isaac Lab would find |

## Transforms, units and scale

| Script | Purpose |
|---|---|
| `stage_units_report.py` · `stage_set_meters_per_unit.py` | Diagnose and fix the stage units |
| `check_residual_scale.py` | Chain of scales from the root down to a prim (`scale` ops **and** `transform` matrices) |
| `dump_xform_ops.py` | `xformOpOrder` plus every `xformOp:*` attribute on a prim, flagging the "orphaned" ones (not in the order, hence inert) — use it when the Property panel and `check_residual_scale.py` disagree |
| `prim_center_origin.py` | Moves a prim's origin to the centre of its bbox without moving the geometry (invalidates existing Joints: redo `joint_aligner.py` afterwards) |
| `prim_zero_local_rotation.py` | Zeroes a prim's local rotation without moving the geometry (same caveat as `prim_center_origin.py`) |
| `prim_align_transform.py` | Aligns the world position and orientation of one prim to another (hardcoded paths), keeping its own local scale |
| `center_left_claw_origins.py` | `prim_center_origin.py` applied in bulk to the 4 moving bodies of the left claw |

## Inspection and measurement of the stage

| Script | Purpose |
|---|---|
| `print_prim_world_position.py` | World position of a prim, bypassing the Local/World confusion of the Property panel |
| `print_prim_world_bbox.py` | Real world-space bounding box (min/max/size) of a prim — a number, not an impression by eye |
| `find_duplicate_prims.py` | Looks for other prims with the same name as the selected one — exposes duplications |
| `find_nearby_bodies.py` | RigidBodies near or overlapping a selected prim — finds unfiltered collisions without trial and error |
| `kind_hierarchy_inspector.py` | Indented tree of "group" vs "component" prims (CAD Converter `kind`) under the selection |
| `collapse_stage_tree.py` | Collapses the selected prims in the Stage window tree |
| `find_circle_center.py` · `find_surface_plane_point.py` · `find_surface_centroid.py` | Find a precise physical point via a marker when the Measure Tool does not snap |
| `get_measurements.py` | Measurement helper |

## Grasp measurements and contact diagnostics

| Script | Purpose |
|---|---|
| `measure_grasp_geometry.py` | Every geometric quantity the staged grasp reward needs |
| `measure_grasp_from_contacts.py` | Grasp point and grasp target from the real finger/workpiece contacts |
| `measure_top_ring_target.py` | Places each grasp target at the middle of a side, on the axis of the topmost ring |
| `shift_grasp_offsets_up.py` | Raises the measured grasp offsets by a fixed height and prints them ready to paste |
| `measure_move_to_grasp.py` … `measure_move_to_grasp_v6.py` | Variants of the measurement of every frame offset the `move_to_grasp` task needs (`_v3` to `_v6` read the real finger contacts) |
| `diag_contact_stream.py` | Whether PhysX emits contact events at all in this scene |
| `diag_finger_contacts.py` | Why PhysX reports no contact between fingers and workpiece |
| `diag_which_body_touches.py` | Which rigid body of the robot PhysX credits with touching the workpiece |
