# Select exactly two prims — a marker first (placed roughly by eye near the circular feature, no
# snapping needed), then the Mesh that contains it — and run.
#
# The circular feature (a boss/hole) usually has no prim of its own: it's just a local cluster of
# vertices inside a larger merged Mesh, so it can't be selected directly and its bbox can't be used
# (see find_nearest_vertex.py for the single-vertex case; this is the same idea generalised to a
# ring of vertices). This script gathers every Mesh vertex within RADIUS of the marker's world
# position and reports their centroid, which approximates the circle's centre for a roughly
# symmetric ring of vertices.
#
# RADIUS is the search radius in metres: wide enough to catch the whole ring, tight enough to
# exclude unrelated geometry. Start small and grow it if "points found" looks too low; the spread
# printed (min/max distance from centroid) tells you whether the cluster is a tight ring (good) or
# picked up stray points from elsewhere (bad — shrink RADIUS or move the marker closer).

RADIUS = 0.005  # metres — tune to the feature size

import omni.usd
from pxr import Usd, UsdGeom, Gf

ctx = omni.usd.get_context()
stage = ctx.get_stage()
sel = ctx.get_selection().get_selected_prim_paths()

if len(sel) != 2:
    print(f"ERROR: select exactly two prims — marker first, then the target Mesh (got {len(sel)}).")
else:
    marker_path, mesh_path = sel
    marker = stage.GetPrimAtPath(marker_path)
    mesh_prim = stage.GetPrimAtPath(mesh_path)
    mesh = UsdGeom.Mesh(mesh_prim)

    if not mesh:
        print(f"ERROR: {mesh_path} is not a Mesh (type: {mesh_prim.GetTypeName()}).")
    else:
        xform_cache = UsdGeom.XformCache()
        marker_world = xform_cache.GetLocalToWorldTransform(marker).Transform(Gf.Vec3d(0, 0, 0))
        mesh_world_xf = xform_cache.GetLocalToWorldTransform(mesh_prim)

        points = mesh.GetPointsAttr().Get()
        if not points:
            print(f"ERROR: {mesh_path} has no points.")
        else:
            nearby = []
            for p in points:
                world_p = mesh_world_xf.Transform(Gf.Vec3d(p))
                dist = (world_p - marker_world).GetLength()
                if dist <= RADIUS:
                    nearby.append(world_p)

            if not nearby:
                print(f"No points found within RADIUS={RADIUS} m of the marker. Move it closer or increase RADIUS.")
            else:
                centroid = Gf.Vec3d(0, 0, 0)
                for p in nearby:
                    centroid += p
                centroid /= len(nearby)

                spread = [(p - centroid).GetLength() for p in nearby]

                print(f"Marker      : {marker_path}  world = {marker_world}")
                print(f"Mesh        : {mesh_path}")
                print(f"Points found: {len(nearby)}  (within {RADIUS} m)")
                print(f"Centroid    : {centroid}")
                print(f"Spread      : min={min(spread):.6f}  max={max(spread):.6f}  (stage units)")
                print("If spread is roughly uniform (tight ring), centroid is a good centre estimate.")
                print("If max spread is much larger than min, RADIUS caught unrelated geometry — shrink it or move the marker.")
