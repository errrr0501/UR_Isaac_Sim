# -*- coding: utf-8 -*-
# Copyright (c) 2020-2022, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.

import sys

import carb
import numpy as np
from omni.isaac.kit import SimulationApp
# from omni.isaac.core.articulations import Articulation


TRANSPORTER_PATH = "/World/transporter"
# UR_STAGE_PATH1 = "/World"
TRANSPORTER_USD_PATH = "/isaac_transporter/transporter.usd"
# UR_USD_PATH = "/ur5e_gripper/ur5e_home.usd"


BACKGROUND_STAGE_PATH = "/World"
BACKGROUND_USD_PATH = "/isaac_transporter/full_warehouse.usd"
BACKGROUND_ENV_PATH = "/World/full_warehouse"
CONFIG = {"renderer": "RayTracedLighting", "headless": False}

MALE_USD_PATH = "/isaac_transporter/male_adult_construction_05_new.usd"
MALE_PATH = "/World/male_adult_construction_05_new"
FEMALE_USD_PATH = "/isaac_transporter/F_Business_02.usd"
FEMALE_PATH = "/World/F_Business_02"


UWB_ACTIONGRAPH_PATH = "/isaac_transporter/UWB.usd"
ROBOT_CONTROLLER_ACTIONGRAPH_PATH = "/isaac_transporter/Robot_Controller.usd"
ROBOT_CAMERA_ACTIONGRAPH_PATH = "/isaac_transporter/Robot_Camera.usd"
PEOPLE_ACTIONGRAPH_PATH = "/isaac_transporter/people.usd"

# Example ROS2 bridge sample demonstrating the manual loading of stages
# and creation of ROS components
simulation_app = SimulationApp(CONFIG)
import omni.graph.core as og  # noqa E402
from omni.isaac.core import SimulationContext  # noqa E402
from omni.isaac.core.utils import (  # noqa E402
    extensions,
    prims,
    stage,
    xforms,
    viewports,
)
from omni.isaac.core_nodes.scripts.utils import set_target_prims  # noqa E402
from pxr import Gf  # noqa E402
from omni.isaac.core.prims.xform_prim import XFormPrim

# enable ROS2 bridge extension
extensions.enable_extension("omni.isaac.ros2_bridge")

simulation_context = SimulationContext(stage_units_in_meters=1.0)

# Locate Isaac Sim assets folder to load environment and robot stages
assets_root_path = "omniverse://localhost/"
if assets_root_path is None:
    carb.log_error("Could not find Isaac Sim assets folder")
    simulation_app.close()
    sys.exit()

# Preparing stage
# viewports.set_camera_view(eye=np.array([1.20193, 1.33053, 1.46214]), target=np.array([0, 0.000001, 1]))
viewports.set_camera_view(eye=np.array([-17.36425, -14.21905, 3.88039]), target=np.array([-8.69287, 5.65996, 0]))

# Loading the simple_room environment
stage.add_reference_to_stage(
    assets_root_path + BACKGROUND_USD_PATH, BACKGROUND_STAGE_PATH
)
stage.add_reference_to_stage(
    assets_root_path + TRANSPORTER_USD_PATH, BACKGROUND_STAGE_PATH
)


XFormPrim(
        prim_path=TRANSPORTER_PATH,
        name="transporter",
        position=np.array([0.0, 0.0, 0]),
        orientation=np.array([0.707, 0, 0, 0.707]),
        )  # w,x,y,z

stage.add_reference_to_stage(
    assets_root_path + MALE_USD_PATH, BACKGROUND_STAGE_PATH
)

stage.add_reference_to_stage(
    assets_root_path + FEMALE_USD_PATH, BACKGROUND_STAGE_PATH
)


XFormPrim(
        prim_path=MALE_PATH,
        name="male_adult_construction",
        position=np.array([1.12607, 6.18145, 0.0]),
        orientation=np.array([1.0, 0.0, 0.0, 0.0]),
        )  # w,x,y,z



XFormPrim(
        prim_path=FEMALE_PATH,
        name="F_Business",
        position=np.array([-8.69287, 5.65996, 0.0]),
        orientation=np.array([1.0, 0.0, 0.0, 0.0]),
        )  # w,x,y,z


stage.add_reference_to_stage(
    assets_root_path + UWB_ACTIONGRAPH_PATH, BACKGROUND_STAGE_PATH
)

stage.add_reference_to_stage(
    assets_root_path + ROBOT_CAMERA_ACTIONGRAPH_PATH, BACKGROUND_STAGE_PATH
)

stage.add_reference_to_stage(
    assets_root_path + ROBOT_CONTROLLER_ACTIONGRAPH_PATH, BACKGROUND_STAGE_PATH
)

stage.add_reference_to_stage(
    assets_root_path + PEOPLE_ACTIONGRAPH_PATH, BACKGROUND_STAGE_PATH
)

simulation_app.update()


# Creating a action graph with ROS component nodes
# try:
#     og.Controller.edit(

#         {"graph_path": "/UWB", "evaluator_name": "execution"},
#         {
#             og.Controller.Keys.CREATE_NODES: [
#                 ("on_playback_tick", "omni.graph.action.OnPlayBackTick"),
#                 ("ros2_context", "omni.isaac.ros2_bridge.ROS2Context"),
#                 ("ros2_publish_transform_free", "omni.isaac.ros2_bridge.ROS2PublishTramsfromTree"),
#                 ("isaac_read_simulation_time", "omni.isaac.core_nodes.IsaacReadSimulationTime"),
#             ],
#             og.Controller.Keys.CONNECT: [
#                 ("on_playback_tick.outputs:tick", "ros2_publish_transform_free.inputs:execIn"),
#                 ("ros2_context.outputs:context", "ros2_publish_transform_free.inputs:context"),
#                 (
#                     "isaac_read_simulation_time.outputs:simulationTime",
#                     "ros2_publish_transform_free.inputs:timeStamp",
#                 ),
#             ],
#             og.Controller.Keys.SET_VALUES: [
#                 ("ros2_context.inputs:domain_id", 89),
#                 ("ros2_context.inputs:useDomainIDEnvVar", True),
#                 ("ros2_publish_transform_tree.inputs:parentPrim", BACKGROUND_ENV_PATH),
#                 ("ros2_publish_transform_tree.inputs:targetPrims", TRANSPORTER_PATH + "/lift"),
   
#             ],
#         },
#         )
    # og.Controller.edit(
    #     {"graph_path": "/people", "evaluator_name": "execution"},
    #     {
    #         og.Controller.Keys.CREATE_NODES: [
    #             ("on_playback_tick", "omni.graph.action.OnPlayBackTick"),
    #             ("ros2_context", "omni.isaac.ros2_bridge.ROS2Context"),
    #             ("ros2_publish_transform_free", "omni.isaac.ros2_bridge.ROS2PublishTramsfromTree"),
    #             ("isaac_read_simulation_time", "omni.isaac.core_nodes.IsaacReadSimulationTime"),
    #         ],
    #         og.Controller.Keys.CONNECT: [
    #             ("on_playback_tick.outputs:tick", "ros2_publish_transform_free.inputs:execIn"),
    #             ("ros2_context.outputs:context", "ros2_publish_transform_free.inputs:context"),
    #             (
    #                 "isaac_read_simulation_time.outputs:simulationTime",
    #                 "ros2_publish_transform_free.inputs:timeStamp",
    #             ),
    #         ],
    #         og.Controller.Keys.SET_VALUES: [
    #             ("ros2_context.inputs:domain_id", 89),
    #             ("ros2_context.inputs:useDomainIDEnvVar", True),
    #             ("ros2_publish_transform_tree.inputs:parentPrim", BACKGROUND_ENV_PATH),
    #             ("ros2_publish_transform_tree.inputs:targetPrims", FEMALE_PATH + "/female_adult_business_02"),
   
    #         ],
    #     },
    # )
    # og.Controller.edit(
    #     {"graph_path": "/Robot_Camera", "evaluator_name": "execution"},
    #     {
    #         og.Controller.Keys.CREATE_NODES: [
    #             ("on_playback_tick", "omni.graph.action.OnPlayBackTick"),
    #             ("ros2_context", "omni.isaac.ros2_bridge.ROS2Context"),
    #             ("isaac_create_render_product", "omni.isaac.core_nodes.IsaacCreateRenderProduct"),
    #             ("ros2_camera_helper", "omni.isaac.ros2_bridge.ROS2CameraHelper"),

    #         ],
    #         og.Controller.Keys.CONNECT: [
    #             ("on_playback_tick.outputs:tick", "isaac_create_render_product.inputs:execIn"),
    #             ("ros2_context.outputs:context", "ros2_camera_helper.inputs:context"),
    #             ("isaac_create_render_product.outputs:execOut", "ros2_camera_helper.inputs:execIn"),
    #             ("isaac_create_render_product.outputs:renderProductPath", "ros2_camera_helper.inputs:renderProductPath"),
    #         ],
    #         og.Controller.Keys.SET_VALUES: [
    #             ("isaac_create_render_product.inputs:cameraPrim", "/World/transporter/lift/Camera"),
    #             ("ros2_context.inputs:domain_id", 89),
    #             ("ros2_context.inputs:useDomainIDEnvVar", True),
    #             ("isaac_create_render_product.inputs:enabled", True),
    #             ("ros2_camera_helper.inputs:enabled", True),
    #             ("ros2_camera_helper.inputs:frameId", "sim_camera"),
    #             ("ros2_camera_helper.inputs:semanticLabelsTopicName", "semantic_labels"),
    #             ("ros2_camera_helper.inputs:topicName", "rgb"),
    #         ],
    #     },

    # )
# except Exception as e:
#     print(e)


set_target_prims(
    primPath="/World/Robot_Controller/articulation_controller", targetPrimPaths=[TRANSPORTER_PATH]
)

set_target_prims(
    primPath="/World/UWB/ros2_publish_transform_tree", targetPrimPaths=[BACKGROUND_ENV_PATH], inputName="inputs:parentPrim"
)

set_target_prims(
    primPath="/World/UWB/ros2_publish_transform_tree", targetPrimPaths=[TRANSPORTER_PATH + "/lift"], inputName="inputs:targetPrims"
)

set_target_prims(
    primPath="/World/people/ros2_publish_transform_tree", targetPrimPaths=[BACKGROUND_ENV_PATH], inputName="inputs:parentPrim"
)

set_target_prims(
    primPath="/World/people/ros2_publish_transform_tree", targetPrimPaths=[FEMALE_PATH + "/female_adult_business_02"], inputName="inputs:targetPrims"
)

set_target_prims(
    primPath="/World/Robot_Camera/isaac_create_render_product", targetPrimPaths=["/World/transporter/lift/Camera"], inputName="inputs:cameraPrim"
)


simulation_app.update()


# need to initialize physics getting any articulation..etc
simulation_context.initialize_physics()

simulation_context.play()
# input()

while simulation_app.is_running():

    # Run with a fixed step size
    simulation_context.step(render=True)

    # Tick the Publish/Subscribe JointState, Publish TF and Publish Clock nodes each frame
    # og.Controller.set(
    #     og.Controller.attribute("/ActionGraph/OnImpulseEvent.state:enableImpulse"), True
    # )

simulation_context.stop()
simulation_app.close()
