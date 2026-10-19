# Read-only. For each selected Joint prim (or the whole stage if nothing is selected), reports
# type, Body0/Body1, physics:collisionEnabled, and Lower/Upper Limit (for Prismatic/Revolute).
# Cross-checks what physics_joint_set_collision.py and manually-entered limits actually are on the
# USD stage, since the Property panel display can be misread and stale Play sessions can mask
# whether a schema edit actually took effect.

import omni.usd
from pxr import Usd, UsdPhysics

ctx = omni.usd.get_context()
stage = ctx.get_stage()
sel = ctx.get_selection().get_selected_prim_paths()

roots = [stage.GetPrimAtPath(p) for p in sel] if sel else [stage.GetPseudoRoot()]

# Also written to a file: the Script Editor console can't be copied from and
# long output scrolls out of view, so the file lets it be read outside Isaac Sim.
OUTPUT_PATH = r"C:\Projects\myProjects\Manipolazione\Scripts\logs\physics_joint_inspector_output.txt"

lines = []


def report(prim):
    joint = UsdPhysics.Joint(prim)
    if joint:
        body0 = joint.GetBody0Rel().GetTargets()
        body1 = joint.GetBody1Rel().GetTargets()
        collision_enabled = joint.GetCollisionEnabledAttr().Get()
        lines.append(f"{prim.GetPath()} [{prim.GetTypeName()}]")
        lines.append(f"  Body0 = {body0}")
        lines.append(f"  Body1 = {body1}")
        lines.append(f"  physics:collisionEnabled = {collision_enabled}")

        # Prismatic/Revolute expose Lower/Upper directly via type-specific schemas:
        prismatic = UsdPhysics.PrismaticJoint(prim)
        if prismatic:
            lines.append(f"  lowerLimit = {prismatic.GetLowerLimitAttr().Get()}")
            lines.append(f"  upperLimit = {prismatic.GetUpperLimitAttr().Get()}")
        revolute = UsdPhysics.RevoluteJoint(prim)
        if revolute:
            lines.append(f"  lowerLimit = {revolute.GetLowerLimitAttr().Get()}")
            lines.append(f"  upperLimit = {revolute.GetUpperLimitAttr().Get()}")

    for child in prim.GetChildren():
        report(child)


for root in roots:
    report(root)

with open(OUTPUT_PATH, "w") as f:
    f.write("\n".join(lines) + "\n")

print(f"Wrote {len(lines)} line(s) to {OUTPUT_PATH}")
