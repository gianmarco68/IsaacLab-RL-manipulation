"""Find out why PhysX reports no contact between the gripper fingers and the workpiece.

Paste into the Script Editor with the simulation PLAYING and the fingers clamped on the workpiece.
Runs asynchronously over a number of frames, because a contact view created mid-simulation has no
data until PhysX has stepped at least once with it in place.

Reads three things in order, which together isolate the cause:
  net contact impulses   -> does PhysX see the fingers touching anything at all
  filtered force matrix  -> does it attribute that contact to the filtered workpiece prim
  detailed contact data  -> are individual contact points available
"""

import asyncio

import numpy as np
import omni.kit.app
import omni.usd
from pxr import PhysxSchema, UsdPhysics
from isaacsim.core.experimental.prims import Articulation, RigidPrim

ROBOT_PATH = "/World/m710id_50m"
STAFFA_PATH = "/World/staffa_quadrata"
FINGER_NAMES = ["right_thumb", "right_forefinger", "left_thumb", "left_forefinger"]
# frames to let PhysX step with the contact view in place before the first read
WARMUP_FRAMES = 20
# contacts can be intermittent, so the readings are taken over several frames and the best is kept
SAMPLE_FRAMES = 30
MAX_CONTACT_COUNT = 512

OUT = "C:/Projects/myProjects/Manipolazione/Scripts/logs/diag_finger_contacts.txt"

lines = []


def say(text=""):
    print(text)
    lines.append(text)


def describe(prim):
    """Which physics schemas a prim declares, the ones that decide whether it can report contact."""
    tags = []
    if prim.HasAPI(UsdPhysics.RigidBodyAPI):
        tags.append("RigidBody")
    if prim.HasAPI(UsdPhysics.CollisionAPI):
        tags.append("Collision")
    if prim.HasAPI(PhysxSchema.PhysxContactReportAPI):
        tags.append("ContactReport")
    return ",".join(tags) if tags else "-"


async def main():
    stage = omni.usd.get_context().get_stage()

    say("=" * 78)
    say("[WORKPIECE SUBTREE]  which prim is the rigid body and which one carries the collider")
    say("=" * 78)
    rigid_bodies = []
    for prim in stage.GetPrimAtPath(STAFFA_PATH).GetChildren() + [stage.GetPrimAtPath(STAFFA_PATH)]:
        tags = describe(prim)
        if tags != "-":
            say(f"  {prim.GetPath().pathString:<55} {tags}")
        if prim.HasAPI(UsdPhysics.RigidBodyAPI):
            rigid_bodies.append(prim.GetPath().pathString)
    for prim in stage.Traverse():
        path = prim.GetPath().pathString
        if path.startswith(STAFFA_PATH) and prim.HasAPI(UsdPhysics.CollisionAPI):
            say(f"  collider: {path}")

    robot = Articulation(ROBOT_PATH)
    link_path = dict(zip(robot.link_names, robot.link_paths[0]))
    finger_paths = [link_path[name] for name in FINGER_NAMES]

    say()
    say("=" * 78)
    say("[FINGERS]  schemas declared on each finger link")
    say("=" * 78)
    for name, path in zip(FINGER_NAMES, finger_paths):
        say(f"  {name:<18} {describe(stage.GetPrimAtPath(path))}")
        for child in stage.GetPrimAtPath(path).GetChildren():
            if child.HasAPI(UsdPhysics.CollisionAPI):
                say(f"  {'':<18} collider child: {child.GetName()}")

    if not rigid_bodies:
        say()
        say("  No RigidBodyAPI anywhere under the workpiece: nothing can be filtered against.")
        with open(OUT, "w") as handle:
            handle.write("\n".join(lines))
        return

    # filter against every rigid body found, not just the one the task cfg names, so a filter path
    # pointing at the wrong prim shows up as zeros on that column and non-zeros on another
    say()
    say(f"  filtering against: {rigid_bodies}")
    fingers = RigidPrim(
        finger_paths, contact_filter_paths=rigid_bodies, max_contact_count=MAX_CONTACT_COUNT
    )

    for _ in range(WARMUP_FRAMES):
        await omni.kit.app.get_app().next_update_async()

    best_net = np.zeros((len(finger_paths), 3))
    best_matrix = np.zeros((len(finger_paths), len(rigid_bodies), 3))
    best_counts = np.zeros((len(finger_paths), len(rigid_bodies)), dtype=int)
    sample_points = {}

    for _ in range(SAMPLE_FRAMES):
        net = fingers.get_net_contact_forces().numpy().reshape(len(finger_paths), 3)
        matrix = fingers.get_contact_force_matrix().numpy().reshape(
            len(finger_paths), len(rigid_bodies), 3
        )
        _f, points, _n, _d, counts, starts = fingers.get_contact_force_data()
        points = points.numpy().reshape(-1, 3)
        counts = counts.numpy().reshape(len(finger_paths), len(rigid_bodies))
        starts = starts.numpy().reshape(len(finger_paths), len(rigid_bodies))

        better = np.linalg.norm(net, axis=1) > np.linalg.norm(best_net, axis=1)
        best_net[better] = net[better]
        better_m = np.linalg.norm(matrix, axis=2) > np.linalg.norm(best_matrix, axis=2)
        best_matrix[better_m] = matrix[better_m]
        for i in range(len(finger_paths)):
            for j in range(len(rigid_bodies)):
                if counts[i, j] > best_counts[i, j]:
                    best_counts[i, j] = int(counts[i, j])
                    start = int(starts[i, j])
                    sample_points[(i, j)] = points[start:start + int(counts[i, j])].mean(axis=0)
        await omni.kit.app.get_app().next_update_async()

    say()
    say("=" * 78)
    say(f"[NET CONTACT]  any contact at all on each finger, best over {SAMPLE_FRAMES} frames")
    say("=" * 78)
    for i, name in enumerate(FINGER_NAMES):
        say(f"  {name:<18} |impulse| = {np.linalg.norm(best_net[i]):.6f}   {best_net[i]}")

    say()
    say("=" * 78)
    say("[FILTERED]  contact attributed to each workpiece rigid body")
    say("=" * 78)
    for j, body in enumerate(rigid_bodies):
        say(f"  vs {body}")
        for i, name in enumerate(FINGER_NAMES):
            point = sample_points.get((i, j))
            where = f"   mean point = {point}" if point is not None else ""
            say(f"    {name:<18} |force| = {np.linalg.norm(best_matrix[i, j]):.6f}"
                f"   contacts = {best_counts[i, j]}{where}")

    say()
    say("=" * 78)
    say("[READING]")
    say("=" * 78)
    if np.linalg.norm(best_net) == 0.0:
        say("  Zero net contact on every finger: PhysX is not reporting for these bodies. Either the")
        say("  contact-report schema was applied without a Stop/Play afterwards, or the fingers are")
        say("  visually touching but not in contact (a filtered pair, or no collider on the fingers).")
    elif best_counts.sum() == 0:
        say("  The fingers do touch something, but nothing is attributed to the workpiece bodies")
        say("  listed above. The contact is against another prim: check the collider list.")
    else:
        say("  Contacts are available. The filter path that works is the one with non-zero counts.")

    with open(OUT, "w") as handle:
        handle.write("\n".join(lines))
    say()
    say(f"written to {OUT}")


asyncio.ensure_future(main())
