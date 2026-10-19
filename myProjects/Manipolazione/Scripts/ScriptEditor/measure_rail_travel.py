# Select exactly two prims — the rail first, then the carriage that slides on it — and run.
# Reports, per axis, the world-space extent of both and the travel left over for the carriage,
# plus where the carriage currently sits inside that travel. This turns into the PrismaticJoint's
# physics:lowerLimit / physics:upperLimit directly, since those are measured from the pose the
# assembly is authored in, not from the end of the rail.
#
# The sliding axis is the one with by far the largest travel; the other two should be ~0 (the
# carriage wraps the rail tightly across its section).
#
# This is an estimate from bounding boxes: it assumes the carriage cannot travel past the rail's
# extremes, which is true for a captive linear guide but ignores separate mechanical stops mounted
# short of the rail ends. Treat it as a first value to confirm against CAD dimensions or the
# component datasheet (the CAD Converter keeps commercial part codes in the mesh names).

import omni.usd
from pxr import Usd, UsdGeom

ctx = omni.usd.get_context()
stage = ctx.get_stage()
sel = ctx.get_selection().get_selected_prim_paths()

if len(sel) != 2:
    print(f"ERROR: select exactly two prims — rail first, then carriage (got {len(sel)}).")
else:
    rail_path, carriage_path = sel
    cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default", "render", "proxy", "guide"])

    rail_range = cache.ComputeWorldBound(stage.GetPrimAtPath(rail_path)).ComputeAlignedRange()
    car_range = cache.ComputeWorldBound(stage.GetPrimAtPath(carriage_path)).ComputeAlignedRange()

    mpu = UsdGeom.GetStageMetersPerUnit(stage)
    print(f"rail     : {rail_path}")
    print(f"carriage : {carriage_path}")
    print(f"metersPerUnit = {mpu}  (values below are in world/stage units)\n")

    for i, axis in enumerate("XYZ"):
        rail_min, rail_max = rail_range.GetMin()[i], rail_range.GetMax()[i]
        car_min, car_max = car_range.GetMin()[i], car_range.GetMax()[i]

        rail_size = rail_max - rail_min
        car_size = car_max - car_min
        travel = rail_size - car_size
        offset = car_min - rail_min  # how far along the rail the carriage already sits

        print(f"--- {axis} ---")
        print(f"  rail     [{rail_min:+.5f} .. {rail_max:+.5f}]  size {rail_size:.5f}")
        print(f"  carriage [{car_min:+.5f} .. {car_max:+.5f}]  size {car_size:.5f}")
        print(f"  travel available : {travel:.5f}   ({travel * mpu * 1000.0:.2f} mm)")
        if travel > 0:
            print(f"  => lowerLimit {-offset:+.5f} / upperLimit {travel - offset:+.5f}")
