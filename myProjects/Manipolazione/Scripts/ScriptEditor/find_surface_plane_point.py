# Select exactly two prims — a marker first (placed roughly by eye near the target area, no
# snapping needed), then the Mesh that contains it — and run.
#
# For a TEXTURED surface (a knurled/pyramid grip pattern, ribbing, etc.): neither vertex averaging
# (find_circle_center.py) nor area-weighted triangle centroid (find_surface_centroid.py) gives a
# clean answer, because the texture's sloped facets dominate the local geometry — there is no single
# flat patch to average. This instead does a least-squares plane fit (PCA) over the nearby vertices:
# a symmetric, repeated texture averages out in that fit, recovering the underlying base surface the
# texture sits on, not a point sitting on top of one specific bump.
#
# Prints the fitted plane's centre point (on the base surface, texture-independent) and its normal.

RADIUS = 0.005  # metres — tune to the feature size, wide enough to cover several texture repeats

import omni.usd
from pxr import Usd, UsdGeom, Gf
import numpy as np

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
            marker_np = np.array([marker_world[0], marker_world[1], marker_world[2]])

            nearby = []
            for p in points:
                world_p = mesh_world_xf.Transform(Gf.Vec3d(p))
                world_np = np.array([world_p[0], world_p[1], world_p[2]])
                if np.linalg.norm(world_np - marker_np) <= RADIUS:
                    nearby.append(world_np)

            if len(nearby) < 3:
                print(f"Only {len(nearby)} point(s) within RADIUS={RADIUS} m — need at least 3 to fit a plane. Increase RADIUS.")
            else:
                pts = np.array(nearby)
                centroid = pts.mean(axis=0)

                # PCA: the eigenvector of smallest variance is the best-fit plane's normal —
                # a symmetric texture (pyramids, ribs) contributes equally in every in-plane
                # direction, so it cancels out and doesn't bias the fit.
                centered = pts - centroid
                cov = np.cov(centered, rowvar=False)
                eigvals, eigvecs = np.linalg.eigh(cov)
                normal = eigvecs[:, 0]  # smallest eigenvalue first from eigh

                spread_along_normal = centered @ normal
                spread_in_plane = np.linalg.norm(centered - np.outer(spread_along_normal, normal), axis=1)

                print(f"Marker         : {marker_path}  world = {marker_world}")
                print(f"Mesh           : {mesh_path}")
                print(f"Points used    : {len(nearby)}  (within {RADIUS} m)")
                print(f"Plane centre   : ({centroid[0]:.6f}, {centroid[1]:.6f}, {centroid[2]:.6f})")
                print(f"Plane normal   : ({normal[0]:.6f}, {normal[1]:.6f}, {normal[2]:.6f})")
                print(f"Texture relief : {abs(spread_along_normal).max():.6f}  (max deviation from fitted plane, stage units)")
                print(f"In-plane spread: {spread_in_plane.max():.6f}  (how far the patch extends, sanity check vs RADIUS)")
