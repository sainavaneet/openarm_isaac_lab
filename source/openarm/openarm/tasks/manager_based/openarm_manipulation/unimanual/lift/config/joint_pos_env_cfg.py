# Copyright 2025 Enactic, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


from isaaclab.assets import RigidObjectCfg
from isaaclab.sensors import FrameTransformerCfg
from isaaclab.sensors.frame_transformer.frame_transformer_cfg import OffsetCfg
from isaaclab.sim.schemas.schemas_cfg import RigidBodyPropertiesCfg
from isaaclab.sim.spawners.from_files.from_files_cfg import UsdFileCfg
from isaaclab.utils import configclass
from isaaclab.utils.assets import ISAAC_NUCLEUS_DIR
from isaaclab.sim.spawners.materials import PreviewSurfaceCfg

from .. import mdp
from ..lift_env_cfg import (
    LiftEnvCfg,
)

import math
import os

##
# Pre-defined configs
##
from isaaclab.markers.config import FRAME_MARKER_CFG  # isort: skip
from openarm.tasks.manager_based.openarm_manipulation.assets.openarm_unimanual import (
    OPEN_ARM_CFG,
)


@configclass
class OpenArmCubeLiftEnvCfg(LiftEnvCfg):
    def __post_init__(self):
        # post init of parent
        super().__post_init__()

        # Set OpenArm as robot
        self.scene.robot = OPEN_ARM_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")

        # Set actions for the specific robot type (OpenArm)
        self.actions.arm_action = mdp.JointPositionActionCfg(
            asset_name="robot",
            joint_names=[
                "openarm_joint.*",
            ],
            scale=0.5,
            use_default_offset=True,  # False
        )

        self.actions.gripper_action = mdp.BinaryJointPositionActionCfg(
            asset_name="robot",
            joint_names=["openarm_finger_joint.*"],
            open_command_expr={"openarm_finger_joint.*": 0.044},
            close_command_expr={"openarm_finger_joint.*": 0.0},
        )

        # Set the body name for the end effector
        self.commands.object_pose.body_name = "openarm_hand"
        self.commands.object_pose.ranges.pitch = (math.pi / 2, math.pi / 2)

        # Set Cube as object
        self.scene.object = RigidObjectCfg(
            prim_path="{ENV_REGEX_NS}/Object",
            init_state=RigidObjectCfg.InitialStateCfg(
                pos=[0.4, 0, 0.055], rot=[1, 0, 0, 0]
            ),
            spawn=UsdFileCfg(
                usd_path=f"{ISAAC_NUCLEUS_DIR}/Props/Blocks/DexCube/dex_cube_instanceable.usd",
                scale=(0.8, 0.8, 0.8),
                rigid_props=RigidBodyPropertiesCfg(
                    solver_position_iteration_count=16,
                    solver_velocity_iteration_count=1,
                    max_angular_velocity=1000.0,
                    max_linear_velocity=1000.0,
                    max_depenetration_velocity=5.0,
                    disable_gravity=False,
                ),
            ),
        )

        # Listens to the required transforms
        marker_cfg = FRAME_MARKER_CFG.copy()
        marker_cfg.markers["frame"].scale = (0.1, 0.1, 0.1)
        marker_cfg.prim_path = "/Visuals/FrameTransformer"
        self.scene.ee_frame = FrameTransformerCfg(
            prim_path="{ENV_REGEX_NS}/Robot/openarm_link0",
            debug_vis=False,
            visualizer_cfg=marker_cfg,
            target_frames=[
                FrameTransformerCfg.FrameCfg(
                    prim_path="{ENV_REGEX_NS}/Robot/openarm_ee_tcp",
                    name="end_effector",
                ),
            ],
        )





@configclass
class OpenArmCubeLiftEnvCfg_PLAY(OpenArmCubeLiftEnvCfg):
    def __post_init__(self):
        # post init of parent
        super().__post_init__()
        # make a smaller scene for play
        self.scene.num_envs = 50
        self.scene.env_spacing = 2.5
        # disable randomization for play
        self.observations.policy.enable_corruption = False

# ------------------------------------------Navaneet----------------------------------------
@configclass
class OpenArmTwoCubeLiftEnvCfg(LiftEnvCfg):
    def __post_init__(self):
        super().__post_init__()

        # Robot
        self.scene.robot = OPEN_ARM_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")

        self.actions.arm_action = mdp.JointPositionActionCfg(
            asset_name="robot",
            joint_names=["openarm_joint.*"],
            scale=0.5,
            use_default_offset=True,
        )
        self.actions.gripper_action = mdp.BinaryJointPositionActionCfg(
            asset_name="robot",
            joint_names=["openarm_finger_joint.*"],
            open_command_expr={"openarm_finger_joint.*": 0.044},
            close_command_expr={"openarm_finger_joint.*": 0.0},
        )

        self.commands.object_pose.body_name = "openarm_hand"
        self.commands.object_pose.ranges.pitch = (math.pi / 2, math.pi / 2)

        # RED cube
        object_red_cfg = RigidObjectCfg(
            prim_path="{ENV_REGEX_NS}/ObjectRed",
            init_state=RigidObjectCfg.InitialStateCfg(
                pos=[0.40, 0.0, 0.055], rot=[1, 0, 0, 0]
            ),
            spawn=UsdFileCfg(
                usd_path=f"{ISAAC_NUCLEUS_DIR}/Props/Blocks/DexCube/dex_cube_instanceable.usd",
                scale=(0.8, 0.8, 0.8),
                visual_material=PreviewSurfaceCfg(diffuse_color=(1.0, 0.0, 0.0)),
                rigid_props=RigidBodyPropertiesCfg(
                    solver_position_iteration_count=16,
                    solver_velocity_iteration_count=1,
                    max_angular_velocity=1000.0,
                    max_linear_velocity=1000.0,
                    max_depenetration_velocity=5.0,
                    disable_gravity=False,
                ),
            ),
        )

        # GREEN cube
        object_green_cfg = RigidObjectCfg(
            prim_path="{ENV_REGEX_NS}/ObjectGreen",
            init_state=RigidObjectCfg.InitialStateCfg(
                pos=[0.35, -0.20, 0.055], rot=[1, 0, 0, 0]
            ),
            spawn=UsdFileCfg(
                usd_path=f"{ISAAC_NUCLEUS_DIR}/Props/Blocks/DexCube/dex_cube_instanceable.usd",
                scale=(0.8, 0.8, 0.8),
                visual_material=PreviewSurfaceCfg(diffuse_color=(0.0, 1.0, 0.0)),
                rigid_props=RigidBodyPropertiesCfg(
                    solver_position_iteration_count=16,
                    solver_velocity_iteration_count=1,
                    max_angular_velocity=1000.0,
                    max_linear_velocity=1000.0,
                    max_depenetration_velocity=5.0,
                    disable_gravity=False,
                ),
            ),
        )

        # Select target (default: red)
        target_object_name = os.environ.get("OPENARM_TARGET_OBJECT", "object_red")
        if target_object_name == "object_green":
            self.scene.object = object_green_cfg
            self.scene.distractor = object_red_cfg
        else:
            self.scene.object = object_red_cfg
            self.scene.distractor = object_green_cfg

        # Fixed goal for both targets.
        self.commands.object_pose.ranges.pos_x = (0.25, 0.25)
        self.commands.object_pose.ranges.pos_y = (0.0, 0.0)
        self.commands.object_pose.ranges.pos_z = (0.25, 0.25)

        # Frame sensor
        marker_cfg = FRAME_MARKER_CFG.copy()
        marker_cfg.markers["frame"].scale = (0.1, 0.1, 0.1)
        marker_cfg.prim_path = "/Visuals/FrameTransformer"
        self.scene.ee_frame = FrameTransformerCfg(
            prim_path="{ENV_REGEX_NS}/Robot/openarm_link0",
            debug_vis=False,
            visualizer_cfg=marker_cfg,
            target_frames=[
                FrameTransformerCfg.FrameCfg(
                    prim_path="{ENV_REGEX_NS}/Robot/openarm_ee_tcp",
                    name="end_effector",
                ),
            ],
        )


@configclass
class OpenArmTwoCubeLiftEnvCfg_PLAY(OpenArmTwoCubeLiftEnvCfg):
    def __post_init__(self):
        super().__post_init__()
        self.scene.num_envs = 50
        self.scene.env_spacing = 2.5
        self.observations.policy.enable_corruption = False
