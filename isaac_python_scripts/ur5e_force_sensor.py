from omni.isaac.core.articulations import Articulation
import asyncio
from omni.isaac.core import World
from omni.isaac.core.utils.stage import (
    add_reference_to_stage,
    create_new_stage_async,
    get_current_stage,
)
from omni.isaac.core.utils.nucleus import get_assets_root_path
from pxr import UsdPhysics

async def joint_force():
    World.clear_instance()
    await create_new_stage_async()
    my_world = World(stage_units_in_meters=1.0, backend="torch", device="cpu")
    await my_world.initialize_simulation_context_async()
    await omni.kit.app.get_app().next_update_async()
    assets_root_path = get_assets_root_path()
    assets_root_path = "omniverse://localhost/"
    if assets_root_path is None:
        carb.log_error("Could not find Isaac Sim assets folder")
        simulation_app.close()
        sys.exit()
    asset_path = assets_root_path + "/ur5e_gripper/ur5e_gripper_force.usd"
    add_reference_to_stage(usd_path=asset_path, prim_path="/World")
    await omni.kit.app.get_app().next_update_async()
    my_world.scene.add_default_ground_plane()
    arti_view = Articulation("/World/ur5e/base_link")
    my_world.scene.add(arti_view)
    await my_world.reset_async(soft=False)
    stage = get_current_stage()

    sensor_joint_forces = arti_view.get_measured_joint_forces()
    sensor_actuation_efforts = arti_view.get_measured_joint_efforts()
    # Iterates through the joint names in the articulation, retrieves information about the joints and their associated links,
    # and creates a mapping between joint names and their corresponding link indices.
    joint_link_id = dict()
    joint_names_list = []
    for joint_name in arti_view._articulation_view.joint_names:
        joint_names_list.append(joint_name)
        # joint_path = "/World/ur5e/wrist_2_link/" + joint_name
        # joint = UsdPhysics.Joint.Get(stage, joint_path)
    #     body_1_path = joint.GetBody1Rel().GetTargets()[0]
    #     body_1_name = stage.GetPrimAtPath(body_1_path).GetName()
    #     child_link_index = arti_view._articulation_view.get_link_index(body_1_name)
        # joint_link_id[joint_name] = child_link_index
    links_list=['base_link','base_link_inertia','shoulder_link','upper_arm_link','forearm_link',
    'wrist_1_link','wrist_2_link','wrist_3_link','flange','tool0','robotiq_coupler','robotiq_85_base_link',
    'robotiq_85_left_finger_link','robotiq_85_left_finger_tip_link','robotiq_85_left_inner_knuckle_link',
    'robotiq_85_left_knuckle_link','robotiq_85_right_finger_link','robotiq_85_right_finger_tip_link',
    'robotiq_85_right_inner_knuckle_link','robotiq_85_right_knuckle_link']


    joint_path = "/World/ur5e/base_link/" + joint_names_list[0]
    joint = UsdPhysics.Joint.Get(stage, joint_path)
    body_1_path = joint.GetBody1Rel().GetTargets()[0]
    body_1_name = stage.GetPrimAtPath(body_1_path).GetName()
    child_link_index = arti_view._articulation_view.get_link_index(body_1_name)
    joint_link_id[joint_names_list[0]] = child_link_index

    joint_path = "/World/ur5e/base_link/" + joint_names_list[1]
    joint = UsdPhysics.Joint.Get(stage, joint_path)
    body_1_path = joint.GetBody1Rel().GetTargets()[0]
    body_1_name = stage.GetPrimAtPath(body_1_path).GetName()
    child_link_index = arti_view._articulation_view.get_link_index(body_1_name)
    joint_link_id[joint_names_list[1]] = child_link_index

    joint_path = "/World/ur5e/base_link_inertia/" + joint_names_list[2]
    joint = UsdPhysics.Joint.Get(stage, joint_path)
    body_1_path = joint.GetBody1Rel().GetTargets()[0]
    body_1_name = stage.GetPrimAtPath(body_1_path).GetName()
    child_link_index = arti_view._articulation_view.get_link_index(body_1_name)
    joint_link_id[joint_names_list[2]] = child_link_index

    joint_path = "/World/ur5e/shoulder_link/" + joint_names_list[3]
    joint = UsdPhysics.Joint.Get(stage, joint_path)
    body_1_path = joint.GetBody1Rel().GetTargets()[0]
    body_1_name = stage.GetPrimAtPath(body_1_path).GetName()
    child_link_index = arti_view._articulation_view.get_link_index(body_1_name)
    joint_link_id[joint_names_list[3]] = child_link_index

    joint_path = "/World/ur5e/upper_arm_link/" + joint_names_list[4]
    joint = UsdPhysics.Joint.Get(stage, joint_path)
    body_1_path = joint.GetBody1Rel().GetTargets()[0]
    body_1_name = stage.GetPrimAtPath(body_1_path).GetName()
    child_link_index = arti_view._articulation_view.get_link_index(body_1_name)
    joint_link_id[joint_names_list[4]] = child_link_index

    joint_path = "/World/ur5e/forearm_link/" + joint_names_list[5]
    joint = UsdPhysics.Joint.Get(stage, joint_path)
    body_1_path = joint.GetBody1Rel().GetTargets()[0]
    body_1_name = stage.GetPrimAtPath(body_1_path).GetName()
    child_link_index = arti_view._articulation_view.get_link_index(body_1_name)
    joint_link_id[joint_names_list[5]] = child_link_index

    joint_path = "/World/ur5e/wrist_1_link/" + joint_names_list[6]
    joint = UsdPhysics.Joint.Get(stage, joint_path)
    body_1_path = joint.GetBody1Rel().GetTargets()[0]
    body_1_name = stage.GetPrimAtPath(body_1_path).GetName()
    child_link_index = arti_view._articulation_view.get_link_index(body_1_name)
    joint_link_id[joint_names_list[6]] = child_link_index

    joint_path = "/World/ur5e/wrist_2_link/" + joint_names_list[7]
    joint = UsdPhysics.Joint.Get(stage, joint_path)
    body_1_path = joint.GetBody1Rel().GetTargets()[0]
    body_1_name = stage.GetPrimAtPath(body_1_path).GetName()
    child_link_index = arti_view._articulation_view.get_link_index(body_1_name)
    joint_link_id[joint_names_list[7]] = child_link_index

    joint_path = "/World/ur5e/wrist_3_link/" + joint_names_list[8]
    joint = UsdPhysics.Joint.Get(stage, joint_path)
    body_1_path = joint.GetBody1Rel().GetTargets()[0]
    body_1_name = stage.GetPrimAtPath(body_1_path).GetName()
    child_link_index = arti_view._articulation_view.get_link_index(body_1_name)
    joint_link_id[joint_names_list[8]] = child_link_index

    joint_path = "/World/ur5e/flange/" + joint_names_list[9]
    joint = UsdPhysics.Joint.Get(stage, joint_path)
    body_1_path = joint.GetBody1Rel().GetTargets()[0]
    body_1_name = stage.GetPrimAtPath(body_1_path).GetName()
    child_link_index = arti_view._articulation_view.get_link_index(body_1_name)
    joint_link_id[joint_names_list[9]] = child_link_index

    joint_path = "/World/ur5e/tool0/" + joint_names_list[10]
    joint = UsdPhysics.Joint.Get(stage, joint_path)
    body_1_path = joint.GetBody1Rel().GetTargets()[0]
    body_1_name = stage.GetPrimAtPath(body_1_path).GetName()
    child_link_index = arti_view._articulation_view.get_link_index(body_1_name)
    joint_link_id[joint_names_list[10]] = child_link_index

    joint_path = "/World/ur5e/robotiq_coupler/" + joint_names_list[11]
    joint = UsdPhysics.Joint.Get(stage, joint_path)
    body_1_path = joint.GetBody1Rel().GetTargets()[0]
    body_1_name = stage.GetPrimAtPath(body_1_path).GetName()
    child_link_index = arti_view._articulation_view.get_link_index(body_1_name)
    joint_link_id[joint_names_list[11]] = child_link_index

    joint_path = "/World/ur5e/robotiq_85_base_link/" + joint_names_list[12]
    joint = UsdPhysics.Joint.Get(stage, joint_path)
    body_1_path = joint.GetBody1Rel().GetTargets()[0]
    body_1_name = stage.GetPrimAtPath(body_1_path).GetName()
    child_link_index = arti_view._articulation_view.get_link_index(body_1_name)
    joint_link_id[joint_names_list[12]] = child_link_index

    joint_path = "/World/ur5e/robotiq_85_base_link/" + joint_names_list[13]
    joint = UsdPhysics.Joint.Get(stage, joint_path)
    body_1_path = joint.GetBody1Rel().GetTargets()[0]
    body_1_name = stage.GetPrimAtPath(body_1_path).GetName()
    child_link_index = arti_view._articulation_view.get_link_index(body_1_name)
    joint_link_id[joint_names_list[13]] = child_link_index

    joint_path = "/World/ur5e/robotiq_85_base_link/" + joint_names_list[14]
    joint = UsdPhysics.Joint.Get(stage, joint_path)
    body_1_path = joint.GetBody1Rel().GetTargets()[0]
    body_1_name = stage.GetPrimAtPath(body_1_path).GetName()
    child_link_index = arti_view._articulation_view.get_link_index(body_1_name)
    joint_link_id[joint_names_list[14]] = child_link_index

    joint_path = "/World/ur5e/robotiq_85_base_link/" + joint_names_list[15]
    joint = UsdPhysics.Joint.Get(stage, joint_path)
    body_1_path = joint.GetBody1Rel().GetTargets()[0]
    body_1_name = stage.GetPrimAtPath(body_1_path).GetName()
    child_link_index = arti_view._articulation_view.get_link_index(body_1_name)
    joint_link_id[joint_names_list[15]] = child_link_index

    joint_path = "/World/ur5e/robotiq_85_left_inner_knuckle_link/" + joint_names_list[16]
    joint = UsdPhysics.Joint.Get(stage, joint_path)
    body_1_path = joint.GetBody1Rel().GetTargets()[0]
    body_1_name = stage.GetPrimAtPath(body_1_path).GetName()
    child_link_index = arti_view._articulation_view.get_link_index(body_1_name)
    joint_link_id[joint_names_list[16]] = child_link_index

    joint_path = "/World/ur5e/robotiq_85_left_knuckle_link/" + joint_names_list[17]
    joint = UsdPhysics.Joint.Get(stage, joint_path)
    body_1_path = joint.GetBody1Rel().GetTargets()[0]
    body_1_name = stage.GetPrimAtPath(body_1_path).GetName()
    child_link_index = arti_view._articulation_view.get_link_index(body_1_name)
    joint_link_id[joint_names_list[17]] = child_link_index

    joint_path = "/World/ur5e/robotiq_85_right_inner_knuckle_link/" + joint_names_list[18]
    joint = UsdPhysics.Joint.Get(stage, joint_path)
    body_1_path = joint.GetBody1Rel().GetTargets()[0]
    body_1_name = stage.GetPrimAtPath(body_1_path).GetName()
    child_link_index = arti_view._articulation_view.get_link_index(body_1_name)
    joint_link_id[joint_names_list[16]] = child_link_index

    joint_path = "/World/ur5e/robotiq_85_right_knuckle_link/" + joint_names_list[19]
    joint = UsdPhysics.Joint.Get(stage, joint_path)
    body_1_path = joint.GetBody1Rel().GetTargets()[0]
    body_1_name = stage.GetPrimAtPath(body_1_path).GetName()
    child_link_index = arti_view._articulation_view.get_link_index(body_1_name)
    joint_link_id[joint_names_list[17]] = child_link_index

    print("joint link IDs", joint_link_id)
    print(sensor_joint_forces[joint_link_id["wrist_3_joint"]])
    print(sensor_actuation_efforts[joint_link_id["wrist_3_joint"]])

asyncio.ensure_future(joint_force())