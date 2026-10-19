from pxr import UsdGeom
import omni.usd

stage = omni.usd.get_context().get_stage()
prim = stage.GetPrimAtPath("/StaffaQuadrata/structure")  # selected prim path

cache = UsdGeom.BBoxCache(0, [UsdGeom.Tokens.default_])
bbox = cache.ComputeWorldBound(prim)
range_ = bbox.ComputeAlignedBox()
print("min:", range_.GetMin(), "max:", range_.GetMax())
print("size:", range_.GetMax() - range_.GetMin())
