"""Find which rigid body of the robot PhysX credits with touching the workpiece.

Paste into the Script Editor with the simulation PLAYING and the fingers clamped on the workpiece.

A collider belongs to the nearest rigid body above it, which is not necessarily the link it sits
under. Querying contacts on the wrong body returns zero however hard the fingers squeeze, so this
asks every link at once and prints the ones that report contact.
"""

import asyncio

import numpy as np
import omni.kit.app
import omni.usd
from pxr import PhysxSchema, UsdPhysics
from isaacsim.core.experimental.prims import Articulation, RigidPrim

ROBOT_PATH = "/World/m710id_50m"
STAFFA_BODY_PATH = "/World/staffa_quadrata/structure"
FINGER_NAMES = ["right_thumb", "right_forefinger", "left_thumb", "left_forefinger"]
WARMUP_FRAMES = 20
SAMPLE_FRAMES = 40
MAX_CONTACT_COUNT = 1024

OUT = "C:/Projects/myProjects/Manipolazione/Scripts/logs/diag_which_body_touches.txt"

lines = []


def say(text=""):
    print(text)
    lines.append(text)


def owning_body(prim):
    """The nearest ancestor declaring RigidBodyAPI, which is the body a collider belongs to."""
    node = prim
    while node.IsValid() and node.GetPath().pathString != "/":
        if node.HasAPI(UsdPhysics.RigidBodyAPI):
            return node.GetPath().pathString
        node = node.GetParent()
    return None


async def main():
    stage = omni.usd.get_context().get_stage()
    robot = Articulation(ROBOT_PATH)
    link_path = dict(zip(robot.link_names, robot.link_paths[0]))

    say("=" * 78)
    say("[FINGER COLLIDERS]  full subtree of each finger, and which body owns each collider")
    say("=" * 78)
    for name in FINGER_NAMES:
        root = stage.GetPrimAtPath(link_path[name])
        found = False
        for prim in Usd_traverse(root):
            if prim.HasAPI(UsdPhysics.CollisionAPI):
                found = True
                say(f"  {name:<18} {prim.GetPath().pathString}")
                say(f"  {'':<18}   owned by {owning_body(prim)}")
        if not found:
            say(f"  {name:<18} NO COLLIDER anywhere in its subtree")

    say()
    say("=" * 78)
    say("[GRIPPER COLLIDERS]  every collider under the gripper, grouped by owning body")
    say("=" * 78)
    by_body = {}
    gripper_root = stage.GetPrimAtPath(ROBOT_PATH + "/gripper")
    for prim in Usd_traverse(gripper_root):
        if prim.HasAPI(UsdPhysics.CollisionAPI):
            by_body.setdefault(owning_body(prim), []).append(prim.GetName())
    for body, names in by_body.items():
        say(f"  {body}")
        say(f"    {len(names)} collider(s): {', '.join(names[:8])}{' ...' if len(names) > 8 else ''}")

    # ask every link of the robot at once: whichever one owns the touching collider shows up
    say()
    say("=" * 78)
    say(f"[CONTACT BY LINK]  every robot link against {STAFFA_BODY_PATH}")
    say("=" * 78)
    names = list(link_path.keys())
    paths = [link_path[n] for n in names]
    links = RigidPrim(paths, contact_filter_paths=[STAFFA_BODY_PATH], max_contact_count=MAX_CONTACT_COUNT)

    for _ in range(WARMUP_FRAMES):
        await omni.kit.app.get_app().next_update_async()

    best_net = np.zeros(len(paths))
    best_force = np.zeros(len(paths))
    best_count = np.zeros(len(paths), dtype=int)
    mean_point = {}
    for _ in range(SAMPLE_FRAMES):
        net = np.linalg.norm(links.get_net_contact_forces().numpy().reshape(len(paths), 3), axis=1)
        force = np.linalg.norm(links.get_contact_force_matrix().numpy().reshape(len(paths), 1, 3), axis=2)[:, 0]
        _f, points, _n, _d, counts, starts = links.get_contact_force_data()
        points = points.numpy().reshape(-1, 3)
        counts = counts.numpy().reshape(len(paths), 1)[:, 0]
        starts = starts.numpy().reshape(len(paths), 1)[:, 0]
        best_net = np.maximum(best_net, net)
        best_force = np.maximum(best_force, force)
        for i in range(len(paths)):
            if counts[i] > best_count[i]:
                best_count[i] = int(counts[i])
                mean_point[i] = points[int(starts[i]):int(starts[i]) + int(counts[i])].mean(axis=0)
        await omni.kit.app.get_app().next_update_async()

    touching = [i for i in range(len(paths)) if best_net[i] > 0 or best_count[i] > 0]
    if not touching:
        say("  No link of the robot reports any contact, with the workpiece or with anything else.")
        say("  PhysX is not reporting for this articulation: press Stop, then Play, redo the grasp")
        say("  and run this again. If it stays empty, the gripper-workpiece pair is filtered out.")
    for i in touching:
        point = mean_point.get(i)
        where = f"   mean contact point = {point}" if point is not None else ""
        say(f"  {names[i]:<20} net={best_net[i]:.6f}  vs workpiece={best_force[i]:.6f}"
            f"  contacts={best_count[i]}{where}")

    say()
    say("=" * 78)
    say("[READING]")
    say("=" * 78)
    say("  The links listed above are the ones to measure the grasp point from. If they are the")
    say("  claw bodies rather than the finger links, the offsets are still measured the same way,")
    say("  only the prim the contacts are read from changes.")

    with open(OUT, "w") as handle:
        handle.write("\n".join(lines))
    say()
    say(f"written to {OUT}")


def Usd_traverse(root):
    """Depth-first walk of a prim and everything under it, including inactive descendants."""
    stack = [root]
    while stack:
        prim = stack.pop()
        if not prim.IsValid():
            continue
        yield prim
        stack.extend(prim.GetChildren())


asyncio.ensure_future(main())
