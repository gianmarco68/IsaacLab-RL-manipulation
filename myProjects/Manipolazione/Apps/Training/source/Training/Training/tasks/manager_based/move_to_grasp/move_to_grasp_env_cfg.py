# Copyright (c) 2022-2026, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Robot-agnostic configuration of the move_to_grasp task.

Fields left as MISSING describe the robot and are filled by a subclass under config/<robot>/, so the
same task runs on any arm with six rotational joints and any gripper.
"""

import math
from dataclasses import MISSING

import isaaclab.sim as sim_utils
from isaaclab.assets import ArticulationCfg, AssetBaseCfg, RigidObjectCfg
from isaaclab.envs import ManagerBasedRLEnvCfg
from isaaclab.managers import ActionTermCfg
from isaaclab.managers import CurriculumTermCfg as CurrTerm
from isaaclab.managers import EventTermCfg as EventTerm
from isaaclab.managers import ObservationGroupCfg as ObsGroup
from isaaclab.managers import ObservationTermCfg as ObsTerm
from isaaclab.managers import RewardTermCfg as RewTerm
from isaaclab.managers import SceneEntityCfg
from isaaclab.managers import TerminationTermCfg as DoneTerm
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.sensors import FrameTransformerCfg
from isaaclab.utils.configclass import configclass
from isaaclab.utils.noise import UniformNoiseCfg as Unoise

from . import mdp
from .arm_descriptor import ArmDescriptorCfg
from .gripper_descriptor import GripperDescriptorCfg

##
# Scene definition
##


@configclass
class MoveToGraspSceneCfg(InteractiveSceneCfg):
    """Workpiece resting on the ground, and a robot that must bring its claws onto it.

    Contact sensors are not declared here: how many there are depends on the gripper, so the robot
    configuration attaches them to this scene, one per finger.
    """

    robot: ArticulationCfg = MISSING
    object: RigidObjectCfg = MISSING

    ee_frame: FrameTransformerCfg = MISSING
    """Frames carried by the robot: the tool centre, one grasp point per claw, and the fingertips."""

    object_frame: FrameTransformerCfg = MISSING
    """Frames carried by the workpiece: one grasp target per claw, x axis running along its side."""

    ground = AssetBaseCfg(
        prim_path="/World/ground",
        # local copy of the grid: the default path is fetched from the cloud content server, which
        # fails outright often enough on this network to look like a crash at startup
        # usd_path="https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/6.0/Isaac/Environments/Grid/default_environment.usd"
        spawn=sim_utils.GroundPlaneCfg(
            size=(100.0, 100.0),
            usd_path="C:/Projects/myProjects/Manipolazione/Apps/Props/Grid/default_environment.usd",
        ),
    )

    dome_light = AssetBaseCfg(
        prim_path="/World/DomeLight",
        spawn=sim_utils.DomeLightCfg(color=(0.9, 0.9, 0.9), intensity=500.0),
    )


##
# MDP settings
##


@configclass
class ActionsCfg:
    """What the policy may command: the arm in task space, and the claw slides if the gripper has them.

    The fingers are deliberately absent. They are closed by an event once the gripper holds the
    clamping pose, so the policy is only ever asked where to put the gripper.
    """

    arm_action: ActionTermCfg = MISSING
    """Relative end-effector pose, levelled and staged: see mdp.StagedLevelIKActionCfg."""

    claw_slide_action: ActionTermCfg | None = MISSING
    """Joint positions of the slides, writable only in the sliding stage. None if the claws are fixed."""


# Tolerances that open each gate, nested so a later gate implies the earlier ones still hold.
# Shared with the action terms, so a reward pays exactly when its axes become writable.
ALIGN_TOL = {
    "angle_tol": math.radians(2.0),  # guide may sit up to 2 degrees off its side
    "perp_tol": 0.02,  
}
SLIDE_TOL = {**ALIGN_TOL, "along_tol": 0.02}  # offset along the guide, in metres
HEIGHT_TOL = 0.02  # height difference counted as being at the target, in metres
STILL_VEL_TOL = 0.1  # joint speed counted as standing still, in rad/s

# Each std is the error at which its reward has halved: set far below what its stage covers, tanh
# saturates at zero with no gradient. Pre-grasp pose: angle 0.22 rad, perp 0.06/0.13, along 0.40/0.33, height 0.21 m.


@configclass
class ObservationsCfg:
    """What the policy sees."""

    @configclass
    class PolicyCfg(ObsGroup):
        joint_pos_rel = ObsTerm(func=mdp.joint_pos_rel, noise=Unoise(n_min=-0.01, n_max=0.01))
        joint_vel_rel = ObsTerm(func=mdp.joint_vel_rel, noise=Unoise(n_min=-0.01, n_max=0.01))
        task_stage = ObsTerm(func=mdp.task_stage)
        grasp_to_target_error = ObsTerm(func=mdp.grasp_to_target_error)
        gripper_tilt = ObsTerm(func=mdp.gripper_tilt)
        finger_contact_forces = ObsTerm(func=mdp.finger_contact_forces)
        actions = ObsTerm(func=mdp.last_action)

        def __post_init__(self) -> None:
            self.enable_corruption = True
            self.concatenate_terms = True

    policy: PolicyCfg = PolicyCfg()


@configclass
class EventCfg:
    """Resets, and the finger closing that checks whether the pose reached is a grasp."""

    reset_all = EventTerm(func=mdp.reset_scene_to_default, mode="reset")

    close_fingers = EventTerm(
        func=mdp.close_fingers_when_in_position,
        mode="interval",
        # both ends equal so it runs once per environment step instead of at a random moment
        interval_range_s=(0.0, 0.0),
        is_global_time=False,
        params={
            "height_tol": HEIGHT_TOL,
            "vel_tol": STILL_VEL_TOL,
            "hold_steps": 5,  # TODO tune: steps the pose must be held before the fingers close
        },
    )


@configclass
class RewardsCfg:
    """Follows the stages the action terms enforce: line up, slide over the target, come down, clamp.

    Each term is gated on the errors that open its stage, so none of them pays for an axis the policy
    is not allowed to move yet.
    """

    guides_parallel_to_sides = RewTerm(
        func=mdp.guides_parallel_to_sides,
        weight=1.0,
        params={"std": math.radians(15.0)},  # TODO tune: angle at which the reward has clearly dropped
    )
    guides_over_sides = RewTerm(
        func=mdp.guides_over_sides,
        weight=3.0,
        params={"std": 0.08},
    )
    still_when_aligned = RewTerm(
        func=mdp.still_when_aligned,
        weight=1.0,
        params={
            "angle_std": ALIGN_TOL["angle_tol"],
            "perp_std": ALIGN_TOL["perp_tol"],
            "vel_std": STILL_VEL_TOL,
        },
    )
    claw_slide_alignment = RewTerm(
        func=mdp.claw_slide_alignment,
        weight=1.0,
        params={"std": 0.20},
    )
    grasp_points_at_target_height = RewTerm(
        func=mdp.grasp_points_at_target_height,
        weight=2.0,
        params={"std": 0.10},
    )
    align_grasp_around_target = RewTerm(
        func=mdp.align_grasp_around_target,
        weight=0.5,
    )
    hold_still_at_target = RewTerm(
        func=mdp.hold_still_at_target,
        weight=2.0,
        params={"vel_std": 0.1, "height_tol": HEIGHT_TOL},  # TODO tune vel_std
    )
    both_fingers_in_contact = RewTerm(
        func=mdp.both_fingers_in_contact,
        weight=10.0,
        # TODO measure force_threshold, in newtons
        params={"force_threshold": 1.0, "height_tol": HEIGHT_TOL},
    )

    # without these nothing opposes the entropy bonus, so the action noise only grows
    action_rate = RewTerm(func=mdp.action_rate_l2, weight=-1e-4)
    joint_vel = RewTerm(
        func=mdp.joint_vel_l2, weight=-1e-4, params={"asset_cfg": SceneEntityCfg("robot")}
    )


@configclass
class TerminationsCfg:
    """No goal termination: the policy has to hold the pose rather than touch it and leave.

    The two failures below end the episode because nothing can be recovered from them, not because
    the task is over.
    """

    time_out = DoneTerm(func=mdp.time_out, time_out=True)
    descended_before_aligned = DoneTerm(
        func=mdp.descended_before_aligned,
        params={"depth_tol": 0.03},  # TODO tune: how far past the target counts as sunk
    )
    gripper_not_level = DoneTerm(
        func=mdp.gripper_not_level,
        params={"tilt_tol": math.radians(20.0)},  # TODO tune: tilt the action term should never allow
    )


@configclass
class CurriculumCfg:
    """Tightens the motion penalties once the gripper has learned to move at all."""

    action_rate = CurrTerm(
        func=mdp.modify_reward_weight,
        params={"term_name": "action_rate", "weight": -1e-1, "num_steps": 10000},
    )
    joint_vel = CurrTerm(
        func=mdp.modify_reward_weight,
        params={"term_name": "joint_vel", "weight": -1e-1, "num_steps": 10000},
    )


##
# Environment configuration
##


@configclass
class MoveToGraspEnvCfg(ManagerBasedRLEnvCfg):
    """Move a gripper from its pre-grasp pose onto the workpiece and clamp it."""

    scene: MoveToGraspSceneCfg = MoveToGraspSceneCfg(num_envs=64, env_spacing=2.5)
    observations: ObservationsCfg = ObservationsCfg()
    actions: ActionsCfg = ActionsCfg()
    events: EventCfg = EventCfg()
    rewards: RewardsCfg = RewardsCfg()
    terminations: TerminationsCfg = TerminationsCfg()
    curriculum: CurriculumCfg = CurriculumCfg()

    arm: ArmDescriptorCfg = MISSING
    """Names and geometry of the arm, read by the action and reward terms."""

    gripper: GripperDescriptorCfg = MISSING
    """Names and geometry of the gripper, read by the action, reward and event terms."""

    def __post_init__(self) -> None:
        self.decimation = 2
        self.episode_length_s = 5.0
        self.sim.dt = 1 / 120
        self.sim.render_interval = self.decimation
        self.viewer.eye = (10.0, 5.0, 3.0)
        # one interval event per environment step
        step_dt = self.decimation * self.sim.dt
        self.events.close_fingers.interval_range_s = (step_dt, step_dt)
