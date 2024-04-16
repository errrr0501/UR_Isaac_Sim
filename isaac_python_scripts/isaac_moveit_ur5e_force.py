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
from omni.isaac import *

UR_STAGE_PATH = "/World/ur5e"
UR_STAGE_PATH1 = "/World"
UR_USD_PATH = "/ur5e_gripper/ur5e_gripper_fixed.usd"
# UR_USD_PATH = "/ur5e_gripper/ur5e_home.usd"


BACKGROUND_STAGE_PATH = "/background"
BACKGROUND_USD_PATH = "/ur5e_gripper/testbed_table.usd"
CONFIG = {"renderer": "RayTracedLighting", "headless": False}

# Example ROS2 bridge sample demonstrating the manual loading of stages
# and creation of ROS components
simulation_app = SimulationApp(CONFIG)
import omni.graph.core as og  # noqa E402
from omni.isaac.core import SimulationContext  # noqa E402
from omni.isaac.core.utils import (  # noqa E402
    extensions,
    prims,
    viewports,
)

from omni.isaac.core_nodes.scripts.utils import set_target_prims  # noqa E402
from pxr import Gf  # noqa E402

from omni.isaac.core.articulations import Articulation
from omni.isaac.core import World
import asyncio
from omni.isaac.core.utils.stage import (
    add_reference_to_stage,
    create_new_stage_async,
    get_current_stage,
)
async def joint_force():
    print("====")
    # my_world = World(stage_units_in_meters=1.0)
    # await my_world.initialize_simulation_context_async()
    # # await omni.kit.app.get_app().next_update_async()
    # arti_view = Articulation("/World/ur5e/base_link")
    # my_world.scene.add(arti_view)
    # stage = get_current_stage()
    # print("====================")

    # sensor_joint_forces = arti_view.get_measured_joint_forces()
    # sensor_actuation_efforts = arti_view.get_measured_joint_efforts()
    # # Iterates through the joint names in the articulation, retrieves information about the joints and their associated links,
    # # and creates a mapping between joint names and their corresponding link indices.
    # joint_link_id = dict()
    # joint_names_list = []
    # for joint_name in arti_view._articulation_view.joint_names:
    #     joint_names_list.append(joint_name)
    #     # joint_path = "/World/ur5e/wrist_2_link/" + joint_name
    #     # joint = UsdPhysics.Joint.Get(stage, joint_path)
    # #     body_1_path = joint.GetBody1Rel().GetTargets()[0]
    # #     body_1_name = stage.GetPrimAtPath(body_1_path).GetName()
    # #     child_link_index = arti_view._articulation_view.get_link_index(body_1_name)
    #     # joint_link_id[joint_name] = child_link_index
    # links_list=['base_link','base_link_inertia','shoulder_link','upper_arm_link','forearm_link',
    # 'wrist_1_link','wrist_2_link','wrist_3_link','flange','tool0','robotiq_coupler','robotiq_85_base_link',
    # 'robotiq_85_left_finger_link','robotiq_85_left_finger_tip_link','robotiq_85_left_inner_knuckle_link',
    # 'robotiq_85_left_knuckle_link','robotiq_85_right_finger_link','robotiq_85_right_finger_tip_link',
    # 'robotiq_85_right_inner_knuckle_link','robotiq_85_right_knuckle_link']


    # joint_path = "/World/ur5e/base_link/" + joint_names_list[0]
    # joint = UsdPhysics.Joint.Get(stage, joint_path)
    # body_1_path = joint.GetBody1Rel().GetTargets()[0]
    # body_1_name = stage.GetPrimAtPath(body_1_path).GetName()
    # child_link_index = arti_view._articulation_view.get_link_index(body_1_name)
    # joint_link_id[joint_names_list[0]] = child_link_index

    # joint_path = "/World/ur5e/base_link/" + joint_names_list[1]
    # joint = UsdPhysics.Joint.Get(stage, joint_path)
    # body_1_path = joint.GetBody1Rel().GetTargets()[0]
    # body_1_name = stage.GetPrimAtPath(body_1_path).GetName()
    # child_link_index = arti_view._articulation_view.get_link_index(body_1_name)
    # joint_link_id[joint_names_list[1]] = child_link_index

    # joint_path = "/World/ur5e/base_link_inertia/" + joint_names_list[2]
    # joint = UsdPhysics.Joint.Get(stage, joint_path)
    # body_1_path = joint.GetBody1Rel().GetTargets()[0]
    # body_1_name = stage.GetPrimAtPath(body_1_path).GetName()
    # child_link_index = arti_view._articulation_view.get_link_index(body_1_name)
    # joint_link_id[joint_names_list[2]] = child_link_index

    # joint_path = "/World/ur5e/shoulder_link/" + joint_names_list[3]
    # joint = UsdPhysics.Joint.Get(stage, joint_path)
    # body_1_path = joint.GetBody1Rel().GetTargets()[0]
    # body_1_name = stage.GetPrimAtPath(body_1_path).GetName()
    # child_link_index = arti_view._articulation_view.get_link_index(body_1_name)
    # joint_link_id[joint_names_list[3]] = child_link_index

    # joint_path = "/World/ur5e/upper_arm_link/" + joint_names_list[4]
    # joint = UsdPhysics.Joint.Get(stage, joint_path)
    # body_1_path = joint.GetBody1Rel().GetTargets()[0]
    # body_1_name = stage.GetPrimAtPath(body_1_path).GetName()
    # child_link_index = arti_view._articulation_view.get_link_index(body_1_name)
    # joint_link_id[joint_names_list[4]] = child_link_index

    # joint_path = "/World/ur5e/forearm_link/" + joint_names_list[5]
    # joint = UsdPhysics.Joint.Get(stage, joint_path)
    # body_1_path = joint.GetBody1Rel().GetTargets()[0]
    # body_1_name = stage.GetPrimAtPath(body_1_path).GetName()
    # child_link_index = arti_view._articulation_view.get_link_index(body_1_name)
    # joint_link_id[joint_names_list[5]] = child_link_index

    # joint_path = "/World/ur5e/wrist_1_link/" + joint_names_list[6]
    # joint = UsdPhysics.Joint.Get(stage, joint_path)
    # body_1_path = joint.GetBody1Rel().GetTargets()[0]
    # body_1_name = stage.GetPrimAtPath(body_1_path).GetName()
    # child_link_index = arti_view._articulation_view.get_link_index(body_1_name)
    # joint_link_id[joint_names_list[6]] = child_link_index

    # joint_path = "/World/ur5e/wrist_2_link/" + joint_names_list[7]
    # joint = UsdPhysics.Joint.Get(stage, joint_path)
    # body_1_path = joint.GetBody1Rel().GetTargets()[0]
    # body_1_name = stage.GetPrimAtPath(body_1_path).GetName()
    # child_link_index = arti_view._articulation_view.get_link_index(body_1_name)
    # joint_link_id[joint_names_list[7]] = child_link_index

    # joint_path = "/World/ur5e/wrist_3_link/" + joint_names_list[8]
    # joint = UsdPhysics.Joint.Get(stage, joint_path)
    # body_1_path = joint.GetBody1Rel().GetTargets()[0]
    # body_1_name = stage.GetPrimAtPath(body_1_path).GetName()
    # child_link_index = arti_view._articulation_view.get_link_index(body_1_name)
    # joint_link_id[joint_names_list[8]] = child_link_index

    # joint_path = "/World/ur5e/flange/" + joint_names_list[9]
    # joint = UsdPhysics.Joint.Get(stage, joint_path)
    # body_1_path = joint.GetBody1Rel().GetTargets()[0]
    # body_1_name = stage.GetPrimAtPath(body_1_path).GetName()
    # child_link_index = arti_view._articulation_view.get_link_index(body_1_name)
    # joint_link_id[joint_names_list[9]] = child_link_index

    # joint_path = "/World/ur5e/tool0/" + joint_names_list[10]
    # joint = UsdPhysics.Joint.Get(stage, joint_path)
    # body_1_path = joint.GetBody1Rel().GetTargets()[0]
    # body_1_name = stage.GetPrimAtPath(body_1_path).GetName()
    # child_link_index = arti_view._articulation_view.get_link_index(body_1_name)
    # joint_link_id[joint_names_list[10]] = child_link_index

    # joint_path = "/World/ur5e/robotiq_coupler/" + joint_names_list[11]
    # joint = UsdPhysics.Joint.Get(stage, joint_path)
    # body_1_path = joint.GetBody1Rel().GetTargets()[0]
    # body_1_name = stage.GetPrimAtPath(body_1_path).GetName()
    # child_link_index = arti_view._articulation_view.get_link_index(body_1_name)
    # joint_link_id[joint_names_list[11]] = child_link_index

    # joint_path = "/World/ur5e/robotiq_85_base_link/" + joint_names_list[12]
    # joint = UsdPhysics.Joint.Get(stage, joint_path)
    # body_1_path = joint.GetBody1Rel().GetTargets()[0]
    # body_1_name = stage.GetPrimAtPath(body_1_path).GetName()
    # child_link_index = arti_view._articulation_view.get_link_index(body_1_name)
    # joint_link_id[joint_names_list[12]] = child_link_index

    # joint_path = "/World/ur5e/robotiq_85_base_link/" + joint_names_list[13]
    # joint = UsdPhysics.Joint.Get(stage, joint_path)
    # body_1_path = joint.GetBody1Rel().GetTargets()[0]
    # body_1_name = stage.GetPrimAtPath(body_1_path).GetName()
    # child_link_index = arti_view._articulation_view.get_link_index(body_1_name)
    # joint_link_id[joint_names_list[13]] = child_link_index

    # joint_path = "/World/ur5e/robotiq_85_base_link/" + joint_names_list[14]
    # joint = UsdPhysics.Joint.Get(stage, joint_path)
    # body_1_path = joint.GetBody1Rel().GetTargets()[0]
    # body_1_name = stage.GetPrimAtPath(body_1_path).GetName()
    # child_link_index = arti_view._articulation_view.get_link_index(body_1_name)
    # joint_link_id[joint_names_list[14]] = child_link_index

    # joint_path = "/World/ur5e/robotiq_85_base_link/" + joint_names_list[15]
    # joint = UsdPhysics.Joint.Get(stage, joint_path)
    # body_1_path = joint.GetBody1Rel().GetTargets()[0]
    # body_1_name = stage.GetPrimAtPath(body_1_path).GetName()
    # child_link_index = arti_view._articulation_view.get_link_index(body_1_name)
    # joint_link_id[joint_names_list[15]] = child_link_index

    # joint_path = "/World/ur5e/robotiq_85_left_inner_knuckle_link/" + joint_names_list[16]
    # joint = UsdPhysics.Joint.Get(stage, joint_path)
    # body_1_path = joint.GetBody1Rel().GetTargets()[0]
    # body_1_name = stage.GetPrimAtPath(body_1_path).GetName()
    # child_link_index = arti_view._articulation_view.get_link_index(body_1_name)
    # joint_link_id[joint_names_list[16]] = child_link_index

    # joint_path = "/World/ur5e/robotiq_85_left_knuckle_link/" + joint_names_list[17]
    # joint = UsdPhysics.Joint.Get(stage, joint_path)
    # body_1_path = joint.GetBody1Rel().GetTargets()[0]
    # body_1_name = stage.GetPrimAtPath(body_1_path).GetName()
    # child_link_index = arti_view._articulation_view.get_link_index(body_1_name)
    # joint_link_id[joint_names_list[17]] = child_link_index

    # joint_path = "/World/ur5e/robotiq_85_right_inner_knuckle_link/" + joint_names_list[18]
    # joint = UsdPhysics.Joint.Get(stage, joint_path)
    # body_1_path = joint.GetBody1Rel().GetTargets()[0]
    # body_1_name = stage.GetPrimAtPath(body_1_path).GetName()
    # child_link_index = arti_view._articulation_view.get_link_index(body_1_name)
    # joint_link_id[joint_names_list[16]] = child_link_index

    # joint_path = "/World/ur5e/robotiq_85_right_knuckle_link/" + joint_names_list[19]
    # joint = UsdPhysics.Joint.Get(stage, joint_path)
    # body_1_path = joint.GetBody1Rel().GetTargets()[0]
    # body_1_name = stage.GetPrimAtPath(body_1_path).GetName()
    # child_link_index = arti_view._articulation_view.get_link_index(body_1_name)
    # joint_link_id[joint_names_list[17]] = child_link_index

    # print("joint link IDs", joint_link_id)
    # print(sensor_joint_forces[joint_link_id["wrist_3_joint"]])
    # print(sensor_actuation_efforts[joint_link_id["wrist_3_joint"]])


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
viewports.set_camera_view(eye=np.array([1.20193, 1.33053, 1.46214]), target=np.array([0, 0.000001, 1]))

# Loading the simple_room environment
add_reference_to_stage(
    assets_root_path + BACKGROUND_USD_PATH, BACKGROUND_STAGE_PATH
)

# Loading the UR robot USD
prims.create_prim(
    UR_STAGE_PATH1,
    "Xform",
    position=np.array([0.06, 0.06, 0.826]),
    orientation=([1.0, 0.0, 0.0, 0.0]),
    usd_path=assets_root_path + UR_USD_PATH,
)


simulation_app.update()



# Creating a action graph with ROS component nodes
try:
    og.Controller.edit(
        {"graph_path": "/ActionGraph", "evaluator_name": "execution"},
        {
            og.Controller.Keys.CREATE_NODES: [
                ("OnImpulseEvent", "omni.graph.action.OnImpulseEvent"),
                ("ReadSimTime", "omni.isaac.core_nodes.IsaacReadSimulationTime"),
                ("Context", "omni.isaac.ros2_bridge.ROS2Context"),
                ("PublishJointState", "omni.isaac.ros2_bridge.ROS2PublishJointState"),
                (
                    "SubscribeJointState",
                    "omni.isaac.ros2_bridge.ROS2SubscribeJointState",
                ),
                (
                    "ArticulationController",
                    "omni.isaac.core_nodes.IsaacArticulationController",
                ),
                ("PublishClock", "omni.isaac.ros2_bridge.ROS2PublishClock"),
            ],
            og.Controller.Keys.CONNECT: [
                ("OnImpulseEvent.outputs:execOut", "PublishJointState.inputs:execIn"),
                ("OnImpulseEvent.outputs:execOut", "SubscribeJointState.inputs:execIn"),
                ("OnImpulseEvent.outputs:execOut", "PublishClock.inputs:execIn"),
                (
                    "OnImpulseEvent.outputs:execOut",
                    "ArticulationController.inputs:execIn",
                ),
                ("Context.outputs:context", "PublishJointState.inputs:context"),
                ("Context.outputs:context", "SubscribeJointState.inputs:context"),
                ("Context.outputs:context", "PublishClock.inputs:context"),
                (
                    "ReadSimTime.outputs:simulationTime",
                    "PublishJointState.inputs:timeStamp",
                ),
                ("ReadSimTime.outputs:simulationTime", "PublishClock.inputs:timeStamp"),
                (
                    "SubscribeJointState.outputs:jointNames",
                    "ArticulationController.inputs:jointNames",
                ),
                (
                    "SubscribeJointState.outputs:positionCommand",
                    "ArticulationController.inputs:positionCommand",
                ),
                (
                    "SubscribeJointState.outputs:velocityCommand",
                    "ArticulationController.inputs:velocityCommand",
                ),
                (
                    "SubscribeJointState.outputs:effortCommand",
                    "ArticulationController.inputs:effortCommand",
                ),
            ],
            og.Controller.Keys.SET_VALUES: [
                ("Context.inputs:useDomainIDEnvVar", 1),
                # Setting the /UR target prim to Articulation Controller node
                ("ArticulationController.inputs:usePath", True),
                ("ArticulationController.inputs:robotPath", UR_STAGE_PATH),
                ("PublishJointState.inputs:topicName", "isaac_joint_states"),
                ("SubscribeJointState.inputs:topicName", "isaac_joint_commands"),
            ],
        },
    )

except Exception as e:
    print(e)

# Setting the /UR target prim to Publish JointState node
set_target_prims(
    primPath="/ActionGraph/PublishJointState", targetPrimPaths=[UR_STAGE_PATH]
)

simulation_app.update()


# need to initialize physics getting any articulation..etc
simulation_context.initialize_physics()

simulation_context.play()
# input()
i = 0
while simulation_app.is_running():

    # Run with a fixed step size
    simulation_context.step(render=True)

    # Tick the Publish/Subscribe JointState, Publish TF and Publish Clock nodes each frame
    og.Controller.set(
        og.Controller.attribute("/ActionGraph/OnImpulseEvent.state:enableImpulse"), True
    )
    
    i = i +1
    if i == 1:
        asyncio.ensure_future(joint_force())

simulation_context.stop()
simulation_app.close()


