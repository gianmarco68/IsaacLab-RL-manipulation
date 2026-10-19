# Copyright (c) 2022-2026, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""The move_to_grasp task on the Apps cell: a FANUC m710id_50m carrying a two-claw gripper.

Every number specific to this robot and this workpiece lives here. Running the task on a different
arm means writing another file like this one, leaving the task itself untouched.
"""

from isaaclab_physx.physics import PhysxCfg

from isaaclab.controllers import DifferentialIKControllerCfg
from isaaclab.managers import SceneEntityCfg
from isaaclab.markers.config import FRAME_MARKER_CFG
from isaaclab.sensors import ContactSensorCfg, FrameTransformerCfg, OffsetCfg
from isaaclab.utils.configclass import configclass

# many grasp-related frames sit within centimetres of each other; the default marker (50 cm) makes
# them overlap into a single blob, so every frame in this task uses a shrunk copy instead
_FRAME_MARKER_SMALL_CFG = FRAME_MARKER_CFG.copy()
_FRAME_MARKER_SMALL_CFG.markers["frame"].scale = (0.07, 0.07, 0.07)

from Training.props.staffa_quadrata import STAFFA_QUADRATA_CFG
from Training.robots.apps import APPS_CFG

from ... import mdp
from ...arm_descriptor import ArmDescriptorCfg
from ...gripper_descriptor import GripperDescriptorCfg
from ...move_to_grasp_env_cfg import SLIDE_TOL, STILL_VEL_TOL, MoveToGraspEnvCfg

##
# Prim paths inside the robot USD
##

_ROBOT = "{ENV_REGEX_NS}/Robot/m710id_50m"
_CLAWS = _ROBOT + "/gripper/gripper_body/components"
_RIGHT_CLAW = _CLAWS + "/right_claw"
_LEFT_CLAW = _CLAWS + "/left_claw/rotor"
# the rigid body each slide joint actually moves, measured off the running scene: not the same prim
# as the finger-carrying subtree above, so the two are kept separate
_RIGHT_CLAW_BODY = _CLAWS + "/right_claw/right_fixed"
_LEFT_CLAW_BODY = _CLAWS + "/left_claw/left_fixed"
_OBJECT = "{ENV_REGEX_NS}/staffa_quadrata"
# the rigid body itself: FrameTransformer checks the exact prim it is given and does not walk
# children, unlike RigidObject, so it must point at the child that actually carries RigidBodyAPI
_OBJECT_BODY = _OBJECT + "/structure"

# square ring, 0.45 m per side, 0.1626 m thick, origin at its centre: the top face sits at half the
# thickness and the middle of a side is half the side away from the centre
_HALF_SIDE = 0.225
_TOP_FACE = 0.0813

##
# Robot description
##

APPS_ARM = ArmDescriptorCfg(
    joint_names_expr=["J[1-6]"],
    base_link_name="robot_base",
    ee_body_name="J6_link",
    ee_offset=(0.476290, -0.054219, -0.055634),
    # x, y, z and yaw are commandable; the roll and pitch entries are unused, the action term holds
    # both level itself instead of asking the policy not to change them
    planar_scale=(0.05, 0.05, 0.05, 0.0, 0.0, 0.1),
    # unreliable as a startup check: pitch sits near 90 deg here, a gimbal-lock singularity for
    # this arm; level was confirmed instead by measuring the guides' world tilt directly (< 1 deg)
    pregrasp_ee_rp=(-0.031110, 1.559075),
)

APPS_GRIPPER = GripperDescriptorCfg(
    has_positioning_dofs=True,
    slide_joint_names=["RightClawSlide", "LeftClawSlide"],
    finger_joint_names=[["RightThumb", "RightForefinger"], ["LeftThumb", "LeftForefinger"]],
    open_finger_pos=[[0.0, 0.0], [0.0, 0.0]],
    # commanded past the point of contact, so the fingers stop on the workpiece rather than on a target
    closed_finger_pos=[[0.0389, -0.0389], [0.0389, -0.0389]],
    grasp_point_frame_names=["right_grasp_point", "left_grasp_point"],
    grasp_target_frame_names=["right_grasp_target", "left_grasp_target"],
    finger_frame_names=[["right_thumb", "right_forefinger"], ["left_thumb", "left_forefinger"]],
    contact_sensor_names=[
        ["right_thumb_contact", "right_forefinger_contact"],
        ["left_thumb_contact", "left_forefinger_contact"],
    ],
    # prismatic joint axes, measured in the flange frame; the two are exactly perpendicular
    guide_axis_in_ee=[(0.0, 0.0, -1.0), (0.0, 1.0, 0.0)],
    # normal to those two axes: the flange's own x, which points down at the pre-grasp pose
    level_axis_in_ee=(1.0, 0.0, 0.0),
)


@configclass
class AppsMoveToGraspEnvCfg(MoveToGraspEnvCfg):
    """Apps cell: FANUC arm, two-claw gripper, square ring resting on the ground."""

    arm: ArmDescriptorCfg = APPS_ARM
    gripper: GripperDescriptorCfg = APPS_GRIPPER

    def __post_init__(self) -> None:
        super().__post_init__()

        self.scene.robot = APPS_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")
        self.scene.object = STAFFA_QUADRATA_CFG.replace(prim_path=_OBJECT)

        self.actions.arm_action = mdp.StagedLevelIKActionCfg(
            asset_name="robot",
            joint_names=self.arm.joint_names_expr,
            body_name=self.arm.ee_body_name,
            controller=DifferentialIKControllerCfg(
                command_type="pose", use_relative_mode=True, ik_method="dls"
            ),
            body_offset=mdp.StagedLevelIKActionCfg.OffsetCfg(pos=self.arm.ee_offset),
            scale=self.arm.planar_scale,
            stage_gate=dict(SLIDE_TOL),
            stage_hold_steps=5,  # steps a stage's tolerances must hold before the next one
            stage_vel_tol=STILL_VEL_TOL,
        )
        self.actions.claw_slide_action = mdp.StagedJointPositionActionCfg(
            asset_name="robot",
            joint_names=self.gripper.slide_joint_names,
            use_default_offset=True,
            scale=0.3,
            clip={"RightClawSlide": (0.0, 0.4), "LeftClawSlide": (-0.48, 0.0)},
            stage_gate=dict(SLIDE_TOL),
            stage_hold_steps=5,
            stage_vel_tol=STILL_VEL_TOL,
        )

        self.scene.ee_frame = FrameTransformerCfg(
            prim_path=f"{_ROBOT}/{self.arm.base_link_name}",
            debug_vis=True,
            visualizer_cfg=_FRAME_MARKER_SMALL_CFG.replace(prim_path="/Visuals/EeFrameTransformer"),
            target_frames=[
                FrameTransformerCfg.FrameCfg(
                    prim_path=f"{_ROBOT}/{self.arm.ee_body_name}",
                    name="ee",
                    offset=OffsetCfg(pos=self.arm.ee_offset),
                ),
                # where each claw grips, from the PhysX contact points of a grasp confirmed to lift
                # the workpiece, raised to the height of the ring's top face
                FrameTransformerCfg.FrameCfg(
                    prim_path=_RIGHT_CLAW_BODY,
                    name="right_grasp_point",
                    offset=OffsetCfg(pos=(0.001285, 0.000004, 0.193793)),
                ),
                FrameTransformerCfg.FrameCfg(
                    prim_path=_LEFT_CLAW_BODY,
                    name="left_grasp_point",
                    offset=OffsetCfg(pos=(0.004798, -0.007158, 0.239278)),
                ),
                # fingertips carry no offset: only which side of the target they sit on matters
                FrameTransformerCfg.FrameCfg(prim_path=f"{_RIGHT_CLAW}/right_thumb", name="right_thumb"),
                FrameTransformerCfg.FrameCfg(
                    prim_path=f"{_RIGHT_CLAW}/right_forefinger", name="right_forefinger"
                ),
                FrameTransformerCfg.FrameCfg(prim_path=f"{_LEFT_CLAW}/left_thumb", name="left_thumb"),
                FrameTransformerCfg.FrameCfg(
                    prim_path=f"{_LEFT_CLAW}/left_forefinger", name="left_forefinger"
                ),
            ],
        )

        # the same two world points as the grasp points above, written against the workpiece: at the
        # measured grasp the two frames coincide, which is what makes zero error a reachable grasp
        self.scene.object_frame = FrameTransformerCfg(
            prim_path=_OBJECT_BODY,
            debug_vis=True,
            visualizer_cfg=_FRAME_MARKER_SMALL_CFG.replace(prim_path="/Visuals/ObjectFrameTransformer"),
            target_frames=[
                FrameTransformerCfg.FrameCfg(
                    prim_path=_OBJECT_BODY,
                    name="right_grasp_target",
                    offset=OffsetCfg(
                        # quaternions here are (x, y, z, w), so the identity is (0, 0, 0, 1)
                        pos=(-0.008267, -0.217419, 0.083539),
                        # the -y side runs along the workpiece's own x, so x already points along it
                        rot=(0.0, 0.0, 0.0, 1.0),
                    ),
                ),
                FrameTransformerCfg.FrameCfg(
                    prim_path=_OBJECT_BODY,
                    name="left_grasp_target",
                    offset=OffsetCfg(
                        pos=(-0.216563, -0.006709, 0.085556),
                        # the -x side runs along the workpiece's own y, so turn x onto it: 90 deg about z
                        rot=(0.0, 0.0, 0.70710678, 0.70710678),
                    ),
                ),
            ],
        )

        # one sensor per finger: filtered contact reporting needs the sensor to match a single prim
        for sensor_name, prim_path in {
            "right_thumb_contact": f"{_RIGHT_CLAW}/right_thumb",
            "right_forefinger_contact": f"{_RIGHT_CLAW}/right_forefinger",
            "left_thumb_contact": f"{_LEFT_CLAW}/left_thumb",
            "left_forefinger_contact": f"{_LEFT_CLAW}/left_forefinger",
        }.items():
            setattr(
                self.scene,
                sensor_name,
                ContactSensorCfg(
                    prim_path=prim_path,
                    # forces against the workpiece only, so the ground and the gripper itself are ignored
                    filter_prim_paths_expr=[_OBJECT_BODY],
                    history_length=0,
                ),
            )

        self.rewards.joint_vel.params["asset_cfg"] = SceneEntityCfg(
            "robot", joint_names=self.arm.joint_names_expr + self.gripper.slide_joint_names
        )

        # the CAD assembly meets PhysX with many touching pairs, which overflows the fixed-size GPU
        # buffers long before GPU memory runs out
        self.sim.physx = PhysxCfg(
            bounce_threshold_velocity=0.01,
            gpu_max_rigid_patch_count=4 * 5 * 2**15,
            gpu_found_lost_pairs_capacity=2**26,
        )


@configclass
class AppsMoveToGraspEnvCfg_PLAY(AppsMoveToGraspEnvCfg):
    """Single noiseless environment, for watching a trained policy."""

    def __post_init__(self) -> None:
        super().__post_init__()
        self.scene.num_envs = 1
        self.observations.policy.enable_corruption = False
