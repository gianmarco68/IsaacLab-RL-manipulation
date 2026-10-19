# Copyright (c) 2022-2026, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Description of the gripper mounted on the arm.

The task reasons about "claws": one or more grasp units, each with two fingers and optionally its
own positioning DOF. A plain parallel-jaw gripper is one claw with ``has_positioning_dofs=False``.
List order defines claw order and must be consistent across every field.
"""

from dataclasses import MISSING

from isaaclab.utils.configclass import configclass


@configclass
class GripperDescriptorCfg:
    """Gripper-specific names and geometry consumed by the MDP terms."""

    has_positioning_dofs: bool = False
    """Whether each claw slides along its own guide. False disables the slide action and its reward."""

    slide_joint_names: list[str] = []
    """One prismatic joint per claw, in claw order. Empty when there are no positioning DOFs."""

    finger_joint_names: list[list[str]] = MISSING
    """``[[thumb, forefinger], ...]`` per claw. Driven by the verification close, never by the policy."""

    open_finger_pos: list[list[float]] = MISSING
    """Finger joint positions held during the approach, same shape as :attr:`finger_joint_names`."""

    closed_finger_pos: list[list[float]] = MISSING
    """Finger joint positions commanded by the verification close."""

    grasp_point_frame_names: list[str] = MISSING
    """Per claw, the ee_frame target marking the point between its fingers where they clamp."""

    grasp_target_frame_names: list[str] = MISSING
    """Per claw, the object_frame target marking the point on the workpiece that claw must clamp.

    Listed in claw order, so each claw is bound to one specific target instead of the pairing being
    implied by two separate lists happening to match.
    """

    finger_frame_names: list[list[str]] = MISSING
    """``[[thumb, forefinger], ...]`` ee_frame targets, used to check the workpiece sits between them."""

    contact_sensor_names: list[list[str]] = MISSING
    """``[[thumb, forefinger], ...]`` scene sensor names. Final success check only, nothing else uses them."""

    guide_axis_in_ee: list[tuple[float, float, float]] = MISSING
    """Per claw, the unit vector of its guide expressed in the ee frame, rotating with it at runtime."""

    level_axis_in_ee: tuple[float, float, float] = MISSING
    """Normal to the plane the guides lie in, in the ee frame. The gripper is level when it is vertical.

    Kept as its own field rather than derived from the guides, so a gripper with a single guide or
    with more than two still states what "level" means for it.
    """
