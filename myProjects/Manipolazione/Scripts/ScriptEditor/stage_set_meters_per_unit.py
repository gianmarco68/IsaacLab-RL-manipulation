# Sets the stage's metersPerUnit metadata. This only changes how existing numbers are INTERPRETED
# — it rescales nothing — so it must be kept consistent with whatever scale the geometry carries.
#
# Context for this scene: the CAD Converter emitted point data in millimetres AND declared
# metersPerUnit = 0.001 (a self-consistent millimetre stage). SETUP_SCENARIO_CAD.md §3 then
# prescribed an extra Scale = 0.001 on the root prim to "convert mm to m" — a second conversion on
# top of the first, leaving PhysX with a gripper 0.27 mm tall instead of 273 mm. Tolerances like
# contact offset are tuned for metre-scale bodies, so at that size everything reads as touching
# everything, which is what requested ~840 MB of gpuCollisionStackSize.
#
# Target state: metersPerUnit = 1.0 with the 0.001 root scale kept, so the conversion happens
# exactly once and the stage is in metres — which is what the Isaac Sim asset library, the FANUC
# arm and the Robot Assembler all expect. USD does not convert units across references, so a
# millimetre gripper cannot be cleanly mounted on a metre robot.
#
# The viewport grid is drawn in stage units, not metres, so it is not a valid check of any of this:
# read the reported extents instead.

METERS_PER_UNIT = 1.0

import omni.usd
from pxr import UsdGeom

ctx = omni.usd.get_context()
stage = ctx.get_stage()

before = UsdGeom.GetStageMetersPerUnit(stage)
UsdGeom.SetStageMetersPerUnit(stage, METERS_PER_UNIT)
after = UsdGeom.GetStageMetersPerUnit(stage)

print(f"metersPerUnit: {before} -> {after}")
print("Reminder: this reinterprets existing numbers, it does not rescale geometry.")
print("With metersPerUnit=1.0 the root prim must keep Scale=0.001 for mm point data.")
