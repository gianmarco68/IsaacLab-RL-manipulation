# Copyright (c) 2022-2026, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Description of the arm the task runs on.

Everything the task needs to know about a 6-rotational-joint arm, so that no MDP term or action
term ever spells out a joint or body name. Swapping arm means writing a new descriptor.
"""

from dataclasses import MISSING

from isaaclab.utils.configclass import configclass


@configclass
class ArmDescriptorCfg:
    """Arm-specific names and geometry consumed by the actions and the startup checks."""

    joint_names_expr: list[str] = MISSING
    """Regex list selecting the arm joints, e.g. ``["J[1-6]"]``. Resolved to indices at manager init."""

    base_link_name: str = MISSING
    """Root link: source frame of the ee FrameTransformer and base of the IK chain."""

    ee_body_name: str = MISSING
    """Body the IK solves for, typically the wrist flange."""

    ee_offset: tuple[float, float, float] = (0.0, 0.0, 0.0)
    """Translation from the flange to the tool centre point, in the flange's local frame."""

    planar_scale: tuple[float, float, float, float, float, float] = MISSING
    """Per-component scale of the relative IK action (dx, dy, dz, droll, dpitch, dyaw).

    Zero on a component makes it uncommandable, which is how the motion is confined to the
    horizontal plane: only dx, dy and dyaw carry a non-zero scale.
    """

    pregrasp_ee_rp: tuple[float, float] = (0.0, 0.0)
    """Roll and pitch the tool frame is expected to hold, in radians. Verified at startup, not enforced."""
