# Select exactly two prims — a marker first (placed roughly by eye near the target area, no
# snapping needed), then the Mesh that contains it — and run.
#
# Unlike find_circle_center.py (which averages raw MESH VERTICES near the marker, so the result is
# biased toward wherever the CAD tessellation happens to be denser, and can land off the surface for
# a sparse/irregular local mesh), this walks the mesh's actual FACE topology: it triangulates every
# face within RADIUS of the marker and computes the AREA-WEIGHTED centroid of those triangles. That
# centroid lies on (or extremely close to, for a locally near-planar patch) the real surface,
# because it's built from surface area rather than vertex count.
#
# Use this when the target is a patch of surface (a flat face, a rounded pad) rather than a ring of
# vertices around a hole — for a genuine circular hole/boss, find_circle_center.py's vertex-ring
# approach is still the right tool.

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
        face_vertex_counts = mesh.GetFaceVertexCountsAttr().Get()
        face_vertex_indices = mesh.GetFaceVertexIndicesAttr().Get()

        if not points or not face_vertex_counts or not face_vertex_indices:
            print(f"ERROR: {mesh_path} has no usable face topology.")
        else:
            world_points = [mesh_world_xf.Transform(Gf.Vec3d(p)) for p in points]

            total_area = 0.0
            weighted_centroid = Gf.Vec3d(0, 0, 0)
            tri_count = 0

            idx = 0
            for count in face_vertex_counts:
                face_indices = face_vertex_indices[idx : idx + count]
                idx += count

                # Fan-triangulate the face (works for tris, quads, and CAD ngons alike).
                v0 = world_points[face_indices[0]]
                for i in range(1, count - 1):
                    v1 = world_points[face_indices[i]]
                    v2 = world_points[face_indices[i + 1]]

                    tri_centroid = (v0 + v1 + v2) / 3.0
                    if (tri_centroid - marker_world).GetLength() > RADIUS:
                        continue

                    tri_area = 0.5 * Gf.Cross(v1 - v0, v2 - v0).GetLength()
                    if tri_area <= 0.0:
                        continue

                    weighted_centroid += tri_centroid * tri_area
                    total_area += tri_area
                    tri_count += 1

            if total_area <= 0.0:
                print(f"No surface found within RADIUS={RADIUS} m of the marker. Move it closer or increase RADIUS.")
            else:
                weighted_centroid /= total_area

                print(f"Marker         : {marker_path}  world = {marker_world}")
                print(f"Mesh           : {mesh_path}")
                print(f"Triangles used : {tri_count}  (within {RADIUS} m)")
                print(f"Surface area   : {total_area:.8f}  (stage units^2)")
                print(f"Centroid       : {weighted_centroid}")
