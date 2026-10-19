# Scripts

Support scripts for the Manipolazione project, split by **where they are executed**.

| Folder / file | Where it runs | Contents |
|---|---|---|
| [`ScriptEditor/`](ScriptEditor/) | Pasted into the **Script Editor of Isaac Sim** (they use `omni.usd` and act on the selection in the Stage) | ~48 USD stage utilities: physics inspection and fixes (RigidBody, Collider, Joint, `FilteredPairsAPI`), units and scale, measurements (bbox, points, Joint travel, grasp geometry), contact diagnostics. Full list in [`ScriptEditor/README.md`](ScriptEditor/README.md). |
| [`SimulationScripts/`](SimulationScripts/) | From a terminal with the Python of Isaac Sim (they open a `SimulationApp`) | NVIDIA Motion Generation examples: `trajectory_example1.py` and Tutorial 9 (gripper control, arm trajectory, follow target, pick & place with cuMotion RMPflow and with PINK IK). |
| `*.py` in this folder | From a terminal, with the **Isaac Lab** environment (they use `gymnasium`, `isaaclab_tasks`, argparse) | One-off diagnostics on the RL environment of the robot with the two-claw gripper. |

## Isaac Lab diagnostics (files at the top level)

| Script | Purpose |
|---|---|
| `check_claw_errors.py` | Prints the per-claw error decomposition (slide / lateral / standoff) to size the three reward `std` values; drives `RightClawSlide` on its own to check the sign and order of the cross product on `approach_axis` |
| `compute_ee_frame_offsets.py` | Computes the `OffsetCfg.pos` values of the `ee_frame` targets without the GUI, reading the runtime poses of fingertips and sensor through temporary `FrameTransformer`s |
| `get_grasp_distance.py` | Prints the initial claw-to-object distance to size the `std` of the `grasp_distance` reward |
| `gripper_prim_explorer.py` | Lists the prims under the gripper body with local transform and world bbox, to choose the TCP / camera-mount candidate prim |
| `joint_limits_check.py` | Compares the initial position of joints J1–J6 with the hard limits, to verify the CAD-imported robot does not spawn at a limit |
| `list_robot_bodies.py` | Prints the real link names of the articulation and their world pose |

## Notes

- The scripts in `ScriptEditor/` do not run outside Isaac Sim: `omni.*` does not exist in a plain Python.
- The scripts in `SimulationScripts/` are copies of the NVIDIA examples (Apache-2.0 license) with local modifications (for example a local asset instead of `get_assets_root_path_async`).
