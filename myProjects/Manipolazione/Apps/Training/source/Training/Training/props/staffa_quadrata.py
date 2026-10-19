# Copyright (c) 2022-2026, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Configuration for the "Staffa Quadrata" prop to be grasped"""

import isaaclab.sim as sim_utils
from isaaclab.assets import RigidObjectCfg

##
# Configuration
##

STAFFA_QUADRATA_CFG = RigidObjectCfg(
    spawn=sim_utils.UsdFileCfg(
        usd_path="C:\\Projects\\myProjects\\Manipolazione\\Apps\\Props\\staffa_quadrata.usd",
        # CAD-imported asset carries millimetre point data; the extra 1.5 matches the scale the cell
        # scene applies, which is what every measured offset in the task cfg was taken against
        scale=(0.0015, 0.0015, 0.0015),
        activate_contact_sensors=True,
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            rigid_body_enabled=True,
            max_linear_velocity=None,
            max_angular_velocity=None,
            max_depenetration_velocity=5.0,
            enable_gyroscopic_forces=True,
        ),
    ),
    init_state=RigidObjectCfg.InitialStateCfg(
        pos=(1.75, 0.0, 0.0813)
    ),
)
"""Configuration for the "Staffa Quadrata" prop to be grasped"""
