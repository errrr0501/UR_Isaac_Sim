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

    # Start the actual move_group node/action server
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict()],
    )

    moveit_py_node = Node(
        name="moveit_py",
        package="ur_moveit_python_api",
        executable=LaunchConfiguration("python_file"),
        output="both",
        parameters=[moveit_config.to_dict()],
    )
    # RViz
    rviz_base = os.path.join(
        get_package_share_directory("ur5e_gripper_source_moveit_config"), "launch"
    )
    rviz_full_config = os.path.join(rviz_base, "moveit.rviz")


    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_full_config],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            # moveit_config.planning_pipelines,
            # moveit_config.robot_description_kinematics,
        ],
    )

    # Static TF
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "world", "base_link"],
    )

    # Publish TF
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[moveit_config.robot_description],
    )

    # ros2_control using FakeSystem as hardware
    ros2_controllers_path = os.path.join(
        get_package_share_directory("ur5e_gripper_source_moveit_config"),
        "config/",
        "ros2_controllers.yaml",
    )
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[ros2_controllers_path],
        remappings=[
            ("/controller_manager/robot_description", "/robot_description"),
        ],
        output="both",
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
    )


    ur5e_arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "ur5e_arm_controller",
            "gripper_controller",
            "--controller-manager",
            "/controller_manager",
        ],
    )




    return LaunchDescription(
        [
            python_file,
            rviz_node,
            static_tf,
            robot_state_publisher,
            move_group_node,
            ros2_control_node,
            joint_state_broadcaster_spawner,
            ur5e_arm_controller_spawner,
            moveit_py_node,
        ]
    )
