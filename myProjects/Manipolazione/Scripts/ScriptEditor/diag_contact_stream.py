"""Establish whether PhysX emits contact events at all in this scene, and why not for the fingers.

Paste into the Script Editor with the simulation PLAYING and the fingers clamped on the workpiece.

Listens to every contact event in the whole scene, not only the ones involving the gripper. That
single fact splits the possible causes apart: if the scene is silent, contact reporting is not
active at all and the simulation has to be restarted; if other pairs report while the fingers do
not, the fingers genuinely are not in contact or their pair is filtered out.
"""

import asyncio
from collections import Counter

import omni.kit.app
import omni.timeline
import omni.usd
from omni.physx import get_physx_simulation_interface
from pxr import PhysicsSchemaTools, PhysxSchema, Usd, UsdPhysics

ROBOT_PATH = "/World/m710id_50m"
STAFFA_PATH = "/World/staffa_quadrata"
FINGER_NAMES = ["right_thumb", "right_forefinger", "left_thumb", "left_forefinger"]
LISTEN_FRAMES = 60

OUT = "C:/Projects/myProjects/Manipolazione/Scripts/logs/diag_contact_stream.txt"

lines = []


def say(text=""):
    print(text)
    lines.append(text)


async def main():
    stage = omni.usd.get_context().get_stage()

    say("=" * 78)
    say("[TIMELINE]  contact events only exist while physics is stepping")
    say("=" * 78)
    timeline = omni.timeline.get_timeline_interface()
    say(f"  playing = {timeline.is_playing()}")

    say()
    say("=" * 78)
    say("[REPORT THRESHOLD]  a contact under the threshold is never reported")
    say("=" * 78)
    for prim in stage.Traverse():
        if not prim.HasAPI(PhysxSchema.PhysxContactReportAPI):
            continue
        attr = PhysxSchema.PhysxContactReportAPI(prim).GetThresholdAttr()
        value = attr.Get() if attr else None
        say(f"  {prim.GetPath().pathString:<70} threshold = {value}")

    say()
    say("=" * 78)
    say("[FILTERED PAIRS]  a filtered pair generates no contact whatsoever")
    say("=" * 78)
    found = False
    for prim in stage.Traverse():
        if not prim.HasAPI(UsdPhysics.FilteredPairsAPI):
            continue
        targets = UsdPhysics.FilteredPairsAPI(prim).GetFilteredPairsRel().GetTargets()
        if not targets:
            continue
        found = True
        say(f"  {prim.GetPath().pathString}")
        for target in targets:
            say(f"      excludes {target.pathString}")
    if not found:
        say("  none")

    say()
    say("=" * 78)
    say("[COLLISION GROUPS]  membership can exclude a pair just as effectively")
    say("=" * 78)
    groups = [p for p in stage.Traverse() if p.IsA(UsdPhysics.CollisionGroup)]
    if not groups:
        say("  none")
    for group in groups:
        rel = UsdPhysics.CollisionGroup(group).GetFilteredGroupsRel()
        say(f"  {group.GetPath().pathString}  filters {[t.pathString for t in rel.GetTargets()]}")

    # ------------------------------------------------------------ listen to everything
    seen = Counter()
    total = [0]

    def on_contact(headers, data):
        for header in headers:
            path0 = PhysicsSchemaTools.intToSdfPath(header.collider0).pathString
            path1 = PhysicsSchemaTools.intToSdfPath(header.collider1).pathString
            total[0] += header.num_contact_data
            seen[(top_level(path0), top_level(path1))] += header.num_contact_data

    def top_level(path):
        """Collapse a collider path to something readable: the prim four levels down at most."""
        parts = path.split("/")
        return "/".join(parts[:5]) if len(parts) > 5 else path

    subscription = get_physx_simulation_interface().subscribe_contact_report_events(on_contact)
    for _ in range(LISTEN_FRAMES):
        await omni.kit.app.get_app().next_update_async()
    subscription = None

    say()
    say("=" * 78)
    say(f"[ALL CONTACT EVENTS]  every pair in the scene, over {LISTEN_FRAMES} frames")
    say("=" * 78)
    say(f"  {total[0]} contact point(s) in {len(seen)} distinct pair(s)")
    for (a, b), count in seen.most_common(30):
        say(f"  {count:6d}   {a}")
        say(f"  {'':6}     vs {b}")

    say()
    say("=" * 78)
    say("[READING]")
    say("=" * 78)
    if total[0] == 0:
        say("  The whole scene is silent. Contact reporting is not active in this run: the schema")
        say("  is in the USD but PhysX never parsed it. Press Stop, then Play, redo the grasp.")
    else:
        gripper = [p for p in seen if any(ROBOT_PATH in side for side in p)]
        staffa = [p for p in seen if any(STAFFA_PATH in side for side in p)]
        say(f"  Reporting works: {len(seen)} pairs. Pairs involving the robot: {len(gripper)}.")
        say(f"  Pairs involving the workpiece: {len(staffa)}.")
        if not staffa:
            say("  The workpiece touches nothing that reports. It needs the contact-report schema")
            say("  too, or it is held by a pair that is filtered out.")

    with open(OUT, "w") as handle:
        handle.write("\n".join(lines))
    say()
    say(f"written to {OUT}")


asyncio.ensure_future(main())
