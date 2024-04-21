import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():

    moveit_config = (
        MoveItConfigsBuilder(
            robot_name="ur5e", package_name="ur5e_gripper_source_moveit_config"
        )
        .robot_description(file_path="config/ur5e.urdf.xacro")
        .robot_description_semantic(file_path="config/ur5e.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .moveit_cpp(
            file_path=get_package_share_directory("ur_moveit_python_api")
            + "/config/motion_planning.yaml"
        )
        .to_moveit_configs()
    )

    python_file = DeclareLaunchArgument(
        "python_file",
        default_value="ur5e_gripper_joint_goal.py",
        description="Python API tutorial file name",
    )


    moveit_py_node = Node(
        name="moveit_py",
        package="ur_moveit_python_api",
        executable=LaunchConfiguration("python_file"),
        output="both",
        parameters=[moveit_config.to_dict()],
    )




    return LaunchDescription(
        [
            python_file,
            moveit_py_node,
        ]
    )
