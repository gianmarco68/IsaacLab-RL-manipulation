# Copyright (c) 2022-2026, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Configuration for the robot Apps --> fanuc robotic arm (m710id_50m) + custom gripper."""

import math

import isaaclab.sim as sim_utils
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets import ArticulationCfg

##
# Configuration
##

APPS_CFG = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(
        # robotApps.usd pulls the arm itself as a payload from the cloud content server; this copy
        # points that payload and the sensor reference at Apps/Props instead
        usd_path="C:/Projects/myProjects/Manipolazione/Apps/Simulation/robotApps_local.usd",
        activate_contact_sensors=True,
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            rigid_body_enabled=True,
            max_linear_velocity=None,
            max_angular_velocity=None,
            max_depenetration_velocity=5.0,
            enable_gyroscopic_forces=True,
        ),
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            # CAD links touch each other: with self collisions PhysX pushes them apart every step and
            # the drives hold them, which reports a constant joint velocity on a motionless arm
            enabled_self_collisions=False,
            solver_position_iteration_count=8,
            solver_velocity_iteration_count=0,
            sleep_threshold=None,
            stabilization_threshold=None,
        ),
    ),
    # pre-grasp pose: arm already facing the staffa, fingers open, gripper close enough that RL
    # only has to learn alignment + grasp + lift, not long-range approach
    init_state=ArticulationCfg.InitialStateCfg(
        pos=(0.0, 0.0, 0.0),
        joint_pos={
            "J1": math.radians(0.0),
            "J2": math.radians(30.0),
            "J3": math.radians(-10.0),
            "J4": math.radians(0.0),
            "J5": math.radians(-50.0),
            "J6": math.radians(0.0),
            "RightClawSlide": 0.0,
            "LeftClawSlide": 0.0,
            "RightThumb": 0.0,
            "RightForefinger": 0.0,
            "LeftThumb": 0.0,
            "LeftForefinger": 0.0,
            "LeftClawRevolution": 0.0,
        },
    ),
    actuators={
        "arm": ImplicitActuatorCfg(
            joint_names_expr=["J1", "J2", "J3", "J4", "J5", "J6"],
            effort_limit_sim={"J1": 8000.0, "J2": 10000.0, "J3": 5000.0, "J4": 2000.0, "J5": 1000.0, "J6": 900.0},
            # sized against the CAD-imported inertias: a held task-space target sags 8 mm instead of
            # 22 cm, and the swing around it is damped out within a tenth of a second
            stiffness=1.0e6,
            damping=4.0e4,
        ),
        "gripper_actuator": ImplicitActuatorCfg(
            joint_names_expr=["RightClawSlide", "LeftClawSlide", "RightThumb", "RightForefinger", "LeftThumb", "LeftForefinger", "LeftClawRevolution"], 
            effort_limit_sim={"RightClawSlide": 700.0, "LeftClawSlide": 700.0, "RightThumb": 500.0, "RightForefinger": 500.0, "LeftThumb": 500.0, "LeftForefinger": 500.0, "LeftClawRevolution": 500.0},
            stiffness=10000.0, 
            damping=350.0,
        ),
    },
)
"""Configuration for the robot Apps --> fanuc robotic arm (m710id_50m) + custom gripper."""
