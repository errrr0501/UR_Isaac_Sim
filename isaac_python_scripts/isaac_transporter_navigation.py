# # -*- coding: utf-8 -*-
# # Copyright (c) 2020-2022, NVIDIA CORPORATION.  All rights reserved.
# #
# # NVIDIA CORPORATION and its licensors retain all intellectual property
# # and proprietary rights in and to this software, related documentation
# # and any modifications thereto.  Any use, reproduction, disclosure or
# # distribution of this software and related documentation without an express
# # license agreement from NVIDIA CORPORATION is strictly prohibited.

# import sys

# import carb
# import numpy as np
# from omni.isaac.kit import SimulationApp
# # from omni.isaac.core.articulations import Articulation


# TRANSPORTER_PATH = "/World/transporter"
# # UR_STAGE_PATH1 = "/World"
# TRANSPORTER_USD_PATH = "/isaac_transporter/transporter.usd"
# # UR_USD_PATH = "/ur5e_gripper/ur5e_home.usd"


# BACKGROUND_STAGE_PATH = "/World"
# BACKGROUND_USD_PATH = "/isaac_transporter/full_warehouse.usd"
# BACKGROUND_ENV_PATH = "/World/full_warehouse"
# CONFIG = {"renderer": "RayTracedLighting", "headless": False}

# MALE_USD_PATH = "/isaac_transporter/male_adult_construction_05_new.usd"
# MALE_PATH = "/World/male_adult_construction_05_new"
# FEMALE_USD_PATH = "/isaac_transporter/F_Business_02.usd"
# FEMALE_PATH = "/World/F_Business_02"


# UWB_ACTIONGRAPH_PATH = "/isaac_transporter/UWB.usd"
# ROBOT_CONTROLLER_ACTIONGRAPH_PATH = "/isaac_transporter/Robot_Controller.usd"
# ROBOT_CAMERA_ACTIONGRAPH_PATH = "/isaac_transporter/Robot_Camera.usd"
# PEOPLE_ACTIONGRAPH_PATH = "/isaac_transporter/people.usd"

# # Example ROS2 bridge sample demonstrating the manual loading of stages
# # and creation of ROS components
# simulation_app = SimulationApp(CONFIG)
# import omni.graph.core as og  # noqa E402
# from omni.isaac.core import SimulationContext  # noqa E402
# from omni.isaac.core.utils import (  # noqa E402
#     extensions,
#     prims,
#     stage,
#     xforms,
#     viewports,
# )
# from omni.isaac.core_nodes.scripts.utils import set_target_prims  # noqa E402
# from pxr import Gf  # noqa E402
# from omni.isaac.core.prims.xform_prim import XFormPrim

# # enable ROS2 bridge extension
# extensions.enable_extension("omni.isaac.ros2_bridge")

# simulation_context = SimulationContext(stage_units_in_meters=1.0)

# # Locate Isaac Sim assets folder to load environment and robot stages
# assets_root_path = "omniverse://localhost/"
# if assets_root_path is None:
#     carb.log_error("Could not find Isaac Sim assets folder")
#     simulation_app.close()
#     sys.exit()

# # Preparing stage
# # viewports.set_camera_view(eye=np.array([1.20193, 1.33053, 1.46214]), target=np.array([0, 0.000001, 1]))
# viewports.set_camera_view(eye=np.array([-17.36425, -14.21905, 3.88039]), target=np.array([-8.69287, 5.65996, 0]))

# # Loading the simple_room environment
# stage.add_reference_to_stage(
#     assets_root_path + BACKGROUND_USD_PATH, BACKGROUND_STAGE_PATH
# )
# stage.add_reference_to_stage(
#     assets_root_path + TRANSPORTER_USD_PATH, BACKGROUND_STAGE_PATH
# )


# XFormPrim(
#         prim_path=TRANSPORTER_PATH,
#         name="transporter",
#         position=np.array([0.0, 0.0, 0]),
#         orientation=np.array([0.707, 0, 0, 0.707]),
#         )  # w,x,y,z

# stage.add_reference_to_stage(
#     assets_root_path + MALE_USD_PATH, BACKGROUND_STAGE_PATH
# )

# stage.add_reference_to_stage(
#     assets_root_path + FEMALE_USD_PATH, BACKGROUND_STAGE_PATH
# )


# XFormPrim(
#         prim_path=MALE_PATH,
#         name="male_adult_construction",
#         position=np.array([1.12607, 6.18145, 0.0]),
#         orientation=np.array([1.0, 0.0, 0.0, 0.0]),
#         )  # w,x,y,z



# XFormPrim(
#         prim_path=FEMALE_PATH,
#         name="F_Business",
#         position=np.array([-8.69287, 5.65996, 0.0]),
#         orientation=np.array([1.0, 0.0, 0.0, 0.0]),
#         )  # w,x,y,z


# stage.add_reference_to_stage(
#     assets_root_path + UWB_ACTIONGRAPH_PATH, BACKGROUND_STAGE_PATH
# )

# stage.add_reference_to_stage(
#     assets_root_path + ROBOT_CAMERA_ACTIONGRAPH_PATH, BACKGROUND_STAGE_PATH
# )

# stage.add_reference_to_stage(
#     assets_root_path + ROBOT_CONTROLLER_ACTIONGRAPH_PATH, BACKGROUND_STAGE_PATH
# )

# stage.add_reference_to_stage(
#     assets_root_path + PEOPLE_ACTIONGRAPH_PATH, BACKGROUND_STAGE_PATH
# )

# simulation_app.update()


# # Creating a action graph with ROS component nodes
# # try:
# #     og.Controller.edit(

# #         {"graph_path": "/UWB", "evaluator_name": "execution"},
# #         {
# #             og.Controller.Keys.CREATE_NODES: [
# #                 ("on_playback_tick", "omni.graph.action.OnPlayBackTick"),
# #                 ("ros2_context", "omni.isaac.ros2_bridge.ROS2Context"),
# #                 ("ros2_publish_transform_free", "omni.isaac.ros2_bridge.ROS2PublishTramsfromTree"),
# #                 ("isaac_read_simulation_time", "omni.isaac.core_nodes.IsaacReadSimulationTime"),
# #             ],
# #             og.Controller.Keys.CONNECT: [
# #                 ("on_playback_tick.outputs:tick", "ros2_publish_transform_free.inputs:execIn"),
# #                 ("ros2_context.outputs:context", "ros2_publish_transform_free.inputs:context"),
# #                 (
# #                     "isaac_read_simulation_time.outputs:simulationTime",
# #                     "ros2_publish_transform_free.inputs:timeStamp",
# #                 ),
# #             ],
# #             og.Controller.Keys.SET_VALUES: [
# #                 ("ros2_context.inputs:domain_id", 89),
# #                 ("ros2_context.inputs:useDomainIDEnvVar", True),
# #                 ("ros2_publish_transform_tree.inputs:parentPrim", BACKGROUND_ENV_PATH),
# #                 ("ros2_publish_transform_tree.inputs:targetPrims", TRANSPORTER_PATH + "/lift"),
   
# #             ],
# #         },
# #         )
#     # og.Controller.edit(
#     #     {"graph_path": "/people", "evaluator_name": "execution"},
#     #     {
#     #         og.Controller.Keys.CREATE_NODES: [
#     #             ("on_playback_tick", "omni.graph.action.OnPlayBackTick"),
#     #             ("ros2_context", "omni.isaac.ros2_bridge.ROS2Context"),
#     #             ("ros2_publish_transform_free", "omni.isaac.ros2_bridge.ROS2PublishTramsfromTree"),
#     #             ("isaac_read_simulation_time", "omni.isaac.core_nodes.IsaacReadSimulationTime"),
#     #         ],
#     #         og.Controller.Keys.CONNECT: [
#     #             ("on_playback_tick.outputs:tick", "ros2_publish_transform_free.inputs:execIn"),
#     #             ("ros2_context.outputs:context", "ros2_publish_transform_free.inputs:context"),
#     #             (
#     #                 "isaac_read_simulation_time.outputs:simulationTime",
#     #                 "ros2_publish_transform_free.inputs:timeStamp",
#     #             ),
#     #         ],
#     #         og.Controller.Keys.SET_VALUES: [
#     #             ("ros2_context.inputs:domain_id", 89),
#     #             ("ros2_context.inputs:useDomainIDEnvVar", True),
#     #             ("ros2_publish_transform_tree.inputs:parentPrim", BACKGROUND_ENV_PATH),
#     #             ("ros2_publish_transform_tree.inputs:targetPrims", FEMALE_PATH + "/female_adult_business_02"),
   
#     #         ],
#     #     },
#     # )
#     # og.Controller.edit(
#     #     {"graph_path": "/Robot_Camera", "evaluator_name": "execution"},
#     #     {
#     #         og.Controller.Keys.CREATE_NODES: [
#     #             ("on_playback_tick", "omni.graph.action.OnPlayBackTick"),
#     #             ("ros2_context", "omni.isaac.ros2_bridge.ROS2Context"),
#     #             ("isaac_create_render_product", "omni.isaac.core_nodes.IsaacCreateRenderProduct"),
#     #             ("ros2_camera_helper", "omni.isaac.ros2_bridge.ROS2CameraHelper"),

#     #         ],
#     #         og.Controller.Keys.CONNECT: [
#     #             ("on_playback_tick.outputs:tick", "isaac_create_render_product.inputs:execIn"),
#     #             ("ros2_context.outputs:context", "ros2_camera_helper.inputs:context"),
#     #             ("isaac_create_render_product.outputs:execOut", "ros2_camera_helper.inputs:execIn"),
#     #             ("isaac_create_render_product.outputs:renderProductPath", "ros2_camera_helper.inputs:renderProductPath"),
#     #         ],
#     #         og.Controller.Keys.SET_VALUES: [
#     #             ("isaac_create_render_product.inputs:cameraPrim", "/World/transporter/lift/Camera"),
#     #             ("ros2_context.inputs:domain_id", 89),
#     #             ("ros2_context.inputs:useDomainIDEnvVar", True),
#     #             ("isaac_create_render_product.inputs:enabled", True),
#     #             ("ros2_camera_helper.inputs:enabled", True),
#     #             ("ros2_camera_helper.inputs:frameId", "sim_camera"),
#     #             ("ros2_camera_helper.inputs:semanticLabelsTopicName", "semantic_labels"),
#     #             ("ros2_camera_helper.inputs:topicName", "rgb"),
#     #         ],
#     #     },

#     # )
# # except Exception as e:
# #     print(e)


# set_target_prims(
#     primPath="/World/Robot_Controller/articulation_controller", targetPrimPaths=[TRANSPORTER_PATH]
# )

# set_target_prims(
#     primPath="/World/UWB/ros2_publish_transform_tree", targetPrimPaths=[BACKGROUND_ENV_PATH], inputName="inputs:parentPrim"
# )

# set_target_prims(
#     primPath="/World/UWB/ros2_publish_transform_tree", targetPrimPaths=[TRANSPORTER_PATH + "/lift"], inputName="inputs:targetPrims"
# )

# set_target_prims(
#     primPath="/World/people/ros2_publish_transform_tree", targetPrimPaths=[BACKGROUND_ENV_PATH], inputName="inputs:parentPrim"
# )

# set_target_prims(
#     primPath="/World/people/ros2_publish_transform_tree", targetPrimPaths=[FEMALE_PATH + "/female_adult_business_02"], inputName="inputs:targetPrims"
# )

# set_target_prims(
#     primPath="/World/Robot_Camera/isaac_create_render_product", targetPrimPaths=["/World/transporter/lift/Camera"], inputName="inputs:cameraPrim"
# )


# simulation_app.update()


# # need to initialize physics getting any articulation..etc
# simulation_context.initialize_physics()

# simulation_context.play()
# # input()

# while simulation_app.is_running():

#     # Run with a fixed step size
#     simulation_context.step(render=True)

#     # Tick the Publish/Subscribe JointState, Publish TF and Publish Clock nodes each frame
#     # og.Controller.set(
#     #     og.Controller.attribute("/ActionGraph/OnImpulseEvent.state:enableImpulse"), True
#     # )

# simulation_context.stop()
# simulation_app.close()


# 導入必要的模組
import sys
import carb
import numpy as np
from omni.isaac.kit import SimulationApp

# 路徑和配置變量的定義
TRANSPORTER_PATH = "/World/transporter1"
TRANSPORTER_USD_PATH = "/isaac_transporter/transporter.usd"
BACKGROUND_STAGE_PATH = "/World"
BACKGROUND_USD_PATH = "/isaac_transporter/World.usd"
BACKGROUND_ENV_PATH = "/World/World"
CONFIG = {"renderer": "RayTracedLighting", "headless": False}
MALE_USD_PATH = "/isaac_transporter/male_adult_construction_05_new.usd"
MALE_PATH = "/World/male_adult_construction_05_new"
FEMALE_USD_PATH = "/isaac_transporter/F_Business_02.usd"
FEMALE_PATH = "/World/F_Business_02"
UWB_ACTIONGRAPH_PATH = "/isaac_transporter/UWB.usd"
ROBOT_CONTROLLER_ACTIONGRAPH_PATH = "/isaac_transporter/Robot_Controller.usd"
ROBOT_CAMERA_ACTIONGRAPH_PATH = "/isaac_transporter/Robot_Camera.usd"
PEOPLE_ACTIONGRAPH_PATH = "/isaac_transporter/people.usd"

# 初始化模擬應用程式
simulation_app = SimulationApp(CONFIG)
import omni.graph.core as og
from omni.isaac.core import SimulationContext
from omni.isaac.core.utils import extensions, prims, stage, xforms, viewports
from omni.isaac.core_nodes.scripts.utils import set_target_prims
from pxr import Gf
from omni.isaac.core.prims.xform_prim import XFormPrim



def create_transporter(name, position, orientation):
    transporter_path = f"/World/{name}"
    stage.add_reference_to_stage(assets_root_path + TRANSPORTER_USD_PATH, transporter_path)
    XFormPrim(prim_path=transporter_path, name=name, position=np.array(position), orientation=np.array(orientation))

    # 加載動作圖
    # stage.add_reference_to_stage(assets_root_path + UWB_ACTIONGRAPH_PATH, transporter_path)
    # stage.add_reference_to_stage(assets_root_path + ROBOT_CAMERA_ACTIONGRAPH_PATH, transporter_path)
    # stage.add_reference_to_stage(assets_root_path + ROBOT_CONTROLLER_ACTIONGRAPH_PATH, transporter_path)

    

def setting_ros_transporter(name):
    transporter_path = f"/World/{name}/transporter"
    # set_target_prims(primPath="/World/" + name + "/Robot_Controller/articulation_controller", targetPrimPaths=[transporter_path])
    # set_target_prims(primPath="/World/" + name + "/UWB/ros2_publish_transform_tree", targetPrimPaths=[BACKGROUND_ENV_PATH], inputName="inputs:parentPrim")
    # set_target_prims(primPath="/World/" + name + "/UWB/ros2_publish_transform_tree", targetPrimPaths=[transporter_path + "/lift"], inputName="inputs:targetPrims")
    # set_target_prims(primPath="/World/" + name + "/Robot_Camera/isaac_create_render_product", targetPrimPaths=[transporter_path + "/lift/Camera"], inputName="inputs:cameraPrim")
    # set_target_prims(primPath="/World/" + name + "/Robot_Camera/ros2_camera_helper", targetPrimPaths=name+"_rgb", inputName="inputs:topicName")

    try:
        og.Controller.edit(
            {"graph_path": "/World/"+name+"/Robot_Camera", "evaluator_name": "execution",},
            {
                og.Controller.Keys.CREATE_NODES: [
                    ("on_playback_tick", "omni.graph.action.OnPlaybackTick"),
                    ("ros2_context", "omni.isaac.ros2_bridge.ROS2Context"),
                    ("isaac_create_render_product", "omni.isaac.core_nodes.IsaacCreateRenderProduct"),
                    ("ros2_camera_helper", "omni.isaac.ros2_bridge.ROS2CameraHelper"),

                ],
                og.Controller.Keys.CONNECT: [
                    ("on_playback_tick.outputs:tick", "isaac_create_render_product.inputs:execIn"),
                    ("ros2_context.outputs:context", "ros2_camera_helper.inputs:context"),
                    ("isaac_create_render_product.outputs:execOut", "ros2_camera_helper.inputs:execIn"),
                    ("isaac_create_render_product.outputs:renderProductPath", "ros2_camera_helper.inputs:renderProductPath"),
                ],
                og.Controller.Keys.SET_VALUES: [
                    ("isaac_create_render_product.inputs:cameraPrim", transporter_path + "/lift/Camera"),
                    ("ros2_context.inputs:domain_id", 89),
                    ("ros2_context.inputs:useDomainIDEnvVar", True),
                    ("isaac_create_render_product.inputs:enabled", True),
                    ("ros2_camera_helper.inputs:enabled", True),
                    ("ros2_camera_helper.inputs:frameId", "sim_camera"),
                    ("ros2_camera_helper.inputs:semanticLabelsTopicName", "semantic_labels"),
                    ("ros2_camera_helper.inputs:renderProductPath", "/Render/RenderProduct_Isaac"),
                    ("ros2_camera_helper.inputs:topicName", name+"/rgb"),
                ],
            },
        )

        og.Controller.edit(
            {"graph_path": "/World/"+name+"/UWB", "evaluator_name": "execution"},
            {
                og.Controller.Keys.CREATE_NODES: [
                    ("on_playback_tick", "omni.graph.action.OnPlaybackTick"),
                    ("ros2_context", "omni.isaac.ros2_bridge.ROS2Context"),
                    ("ros2_publish_transform_free", "omni.isaac.ros2_bridge.ROS2PublishTransformTree"),
                    ("isaac_read_simulation_time", "omni.isaac.core_nodes.IsaacReadSimulationTime"),
                ],
                og.Controller.Keys.CONNECT: [
                    ("on_playback_tick.outputs:tick", "ros2_publish_transform_free.inputs:execIn"),
                    ("ros2_context.outputs:context", "ros2_publish_transform_free.inputs:context"),
                    (
                        "isaac_read_simulation_time.outputs:simulationTime",
                        "ros2_publish_transform_free.inputs:timeStamp",
                    ),
                ],
                og.Controller.Keys.SET_VALUES: [
                    ("ros2_context.inputs:domain_id", 89),
                    ("ros2_context.inputs:useDomainIDEnvVar", True),
                    ("ros2_publish_transform_free.inputs:parentPrim", BACKGROUND_ENV_PATH),
                    ("ros2_publish_transform_free.inputs:targetPrims", transporter_path + "/lift"),
                    ("ros2_publish_transform_free.inputs:topicName", name+"/tf"),
                ],
            },
        )

        og.Controller.edit(
            {"graph_path": "/World/"+name+"/Robot_Controller", "evaluator_name": "execution"},
            {
                og.Controller.Keys.CREATE_NODES: [
                    ("on_playback_tick", "omni.graph.action.OnPlaybackTick"),
                    ("ros2_context", "omni.isaac.ros2_bridge.ROS2Context"),
                    ("ros2_subscribe_twist", "omni.isaac.ros2_bridge.ROS2SubscribeTwist"),
                    ("scale_to_from_stage_units", "omni.isaac.core_nodes.OgnIsaacScaleToFromStageUnit"),
                    ("break_3_vector", "omni.graph.nodes.BreakVector3"),
                    ("break_3_vector_01", "omni.graph.nodes.BreakVector3"),
                    ("constant_token", "omni.graph.nodes.ConstantToken"),
                    ("constant_token_01", "omni.graph.nodes.ConstantToken"),
                    ("articulation_controller", "omni.isaac.core_nodes.IsaacArticulationController"),
                    ("differential_controller", "omni.isaac.wheeled_robots.DifferentialController"),
                    ("make_array", "omni.graph.nodes.ConstructArray"),
                ],
                og.Controller.Keys.CONNECT: [
                    ("on_playback_tick.outputs:tick", "ros2_subscribe_twist.inputs:execIn"),
                    ("on_playback_tick.outputs:tick", "articulation_controller.inputs:execIn"),
                    ("ros2_context.outputs:context", "ros2_subscribe_twist.inputs:context"),
                    ("ros2_subscribe_twist.outputs:execOut", "differential_controller.inputs:execIn"),
                    ("ros2_subscribe_twist.outputs:angularvelocity", "break_3_vector.inputs:vector"),
                    ("ros2_subscribe_twist.outputs:linearvelocity", "scale_to_from_stage_units.inputs:value"),
                    ("scale_to_from_stage_units.outputs:result", "break_3_vector_01.inputs:vector"),
                    ("break_3_vector_01.outputs:x", "differential_controller.inputs:linearVelocity"),
                    ("break_3_vector.outputs:z", "differential_controller.inputs:angularVelocity"),
                    # ("constant_token.inputs:value", "make_array.inputs:input0"),
                    # ("constant_token_01.inputs:value", "make_array.inputs:input1"),
                    ("differential_controller.outputs:velocityCommand", "articulation_controller.inputs:velocityCommand"),
                    ("make_array.outputs:array", "articulation_controller.inputs:jointNames"),
                ],
                og.Controller.Keys.SET_VALUES: [
                    ("ros2_context.inputs:domain_id", 89),
                    ("differential_controller.inputs:maxLinearSpeed", 2.2),
                    ("differential_controller.inputs:wheelDistance", 0.57926),
                    ("differential_controller.inputs:wheelRadius", 0.08),
                    ("constant_token.inputs:value", "left_wheel_joint"),
                    ("constant_token_01.inputs:value", "right_wheel_joint"),
                    ("articulation_controller.inputs:targetPrim", "/World/transporter1/transporters"),
                    ("articulation_controller.inputs:robotPath", transporter_path),
                    ("ros2_subscribe_twist.inputs:topicName", name+"/cmd_vel"),
                    ("make_array.inputs:arraySize", 2),
                ],
            }
        )
        simulation_app.update()
        og.Controller.edit(
            {"graph_path": "/World/"+name+"/Robot_Controller", "evaluator_name": "execution"},
            {
                # og.Controller.Keys.CREATE_NODES: [
                #     ("on_playback_tick", "omni.graph.action.OnPlaybackTick"),
                #     ("ros2_context", "omni.isaac.ros2_bridge.ROS2Context"),
                #     ("ros2_subscribe_twist", "omni.isaac.ros2_bridge.ROS2SubscribeTwist"),
                #     ("scale_to_from_stage_units", "omni.isaac.core_nodes.OgnIsaacScaleToFromStageUnit"),
                #     ("break_3_vector", "omni.graph.nodes.BreakVector3"),
                #     ("break_3_vector_01", "omni.graph.nodes.BreakVector3"),
                #     ("constant_token", "omni.graph.nodes.ConstantToken"),
                #     ("constant_token_01", "omni.graph.nodes.ConstantToken"),
                #     ("articulation_controller", "omni.isaac.core_nodes.IsaacArticulationController"),
                #     ("differential_controller", "omni.isaac.wheeled_robots.DifferentialController"),
                #     ("make_array", "omni.graph.nodes.ConstructArray"),
                # ],
                og.Controller.Keys.CONNECT: [
                    # ("on_playback_tick.outputs:tick", "ros2_subscribe_twist.inputs:execIn"),
                    # ("on_playback_tick.outputs:tick", "articulation_controller.inputs:execIn"),
                    # ("ros2_context.outputs:context", "ros2_subscribe_twist.inputs:context"),
                    # ("ros2_subscribe_twist.outputs:execOut", "differential_controller.inputs:execIn"),
                    # ("ros2_subscribe_twist.outputs:angularvelocity", "break_3_vector.inputs:vector"),
                    # ("ros2_subscribe_twist.outputs:linearvelocity", "scale_to_from_stage_units.inputs:value"),
                    # ("scale_to_from_stage_units.outputs:result", "break_3_vector_01.inputs:vector"),
                    # ("break_3_vector_01.outputs:x", "differential_controller.inputs:linearVelocity"),
                    # ("break_3_vector.outputs:z", "differential_controller.inputs:angularVelocity"),
                    ("constant_token.inputs:Value", "make_array.inputs:input0"),
                    ("constant_token_01.inputs:value", "make_array.inputs:input1"),
                    # ("differential_controller.outputs:velocityCommand", "articulation_controller.inputs:velocityCommand"),
                    # ("make_array.outputs:array", "articulation_controller.inputs:jointNames"),
                ],
                # og.Controller.Keys.SET_VALUES: [
                #     ("ros2_context.inputs:domain_id", 89),
                #     ("differential_controller.inputs:maxLinearSpeed", 2.2),
                #     ("differential_controller.inputs:wheelDistance", 0.57926),
                #     ("differential_controller.inputs:wheelRadius", 0.08),
                #     ("constant_token.inputs:value", "left_wheel_joint"),
                #     ("constant_token_01.inputs:value", "right_wheel_joint"),
                #     ("articulation_controller.inputs:targetPrim", "/World/transporter1/transporters"),
                #     ("articulation_controller.inputs:robotPath", transporter_path),
                #     ("ros2_subscribe_twist.inputs:topicName", name+"/cmd_vel"),
                #     ("make_array.inputs:arraySize", int(2)),
                # ],
            }
        )
    except Exception as error:
        print(error)
    

# 啟用ROS2橋接擴展
extensions.enable_extension("omni.isaac.ros2_bridge")

# 創建模擬上下文並設置場景單位
simulation_context = SimulationContext(stage_units_in_meters=1.0)

# 檢查資產路徑並準備關閉應用程式的條件
assets_root_path = "omniverse://localhost/"
if assets_root_path is None:
    carb.log_error("Could not find Isaac Sim assets folder")
    simulation_app.close()
    sys.exit()

# 設置視角和加載環境場景
viewports.set_camera_view(eye=np.array([-17.36425, -14.21905, 3.88039]), target=np.array([-8.69287, 5.65996, 0]))
stage.add_reference_to_stage(assets_root_path + BACKGROUND_USD_PATH, BACKGROUND_STAGE_PATH)
# stage.add_reference_to_stage(assets_root_path + TRANSPORTER_USD_PATH, BACKGROUND_STAGE_PATH)
stage.add_reference_to_stage(assets_root_path + MALE_USD_PATH, BACKGROUND_STAGE_PATH)
stage.add_reference_to_stage(assets_root_path + FEMALE_USD_PATH, BACKGROUND_STAGE_PATH)
# stage.add_reference_to_stage(assets_root_path + TRANSPORTER_USD_PATH, BACKGROUND_STAGE_PATH)


create_transporter("transporter1", [-36.0, -52.0, 0.0], [1, 0, 0, 0])
create_transporter("transporter2", [-32.0, -52.0, 0.0], [1, 0, 0, 0])

# 創建和配置場景中的物體
# XFormPrim(prim_path=TRANSPORTER_PATH, name="transporter", position=np.array([-36.0, -52.0, 0.0]), orientation=np.array([1.0, 0.0, 0.0, 0.0]))
XFormPrim(prim_path=MALE_PATH, name="male_adult_construction", position=np.array([-23.94327, -51.92124, 0.0]), orientation=np.array([1.0, 0.0, 0.0, 0.0]))
XFormPrim(prim_path=FEMALE_PATH, name="F_Business", position=np.array([-19.59921, -51.71445, 0.0]), orientation=np.array([1.0, 0.0, 0.0, 0.0]))

# 加載動作圖
# stage.add_reference_to_stage(assets_root_path + UWB_ACTIONGRAPH_PATH, BACKGROUND_STAGE_PATH)
# stage.add_reference_to_stage(assets_root_path + ROBOT_CAMERA_ACTIONGRAPH_PATH, BACKGROUND_STAGE_PATH)
# stage.add_reference_to_stage(assets_root_path + ROBOT_CONTROLLER_ACTIONGRAPH_PATH, BACKGROUND_STAGE_PATH)
stage.add_reference_to_stage(assets_root_path + PEOPLE_ACTIONGRAPH_PATH, BACKGROUND_STAGE_PATH)

# 更新應用程式狀態
simulation_app.update()

# 設置目標物體並更新ROS2節點
setting_ros_transporter("transporter1")
setting_ros_transporter("transporter2")
set_target_prims(primPath="/World/people/ros2_publish_transform_tree", targetPrimPaths=[BACKGROUND_ENV_PATH], inputName="inputs:parentPrim")
set_target_prims(primPath="/World/people/ros2_publish_transform_tree", targetPrimPaths=[FEMALE_PATH + "/female_adult_business_02"], inputName="inputs:targetPrims")



# 再次更新應用程式狀態
simulation_app.update()

# 初始化物理引擎並啟動模擬
simulation_context.initialize_physics()
simulation_context.play()

# 模擬運行時的循環
while simulation_app.is_running():
    simulation_context.step(render=True)

# 停止模擬並關閉應用程式
simulation_context.stop()
simulation_app.close()