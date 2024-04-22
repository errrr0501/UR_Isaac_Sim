#!/usr/bin/env python3

# Generic ROS libraries
import rclpy
from rclpy.logging import get_logger


# RobotState is used to set joint values
from moveit.core.robot_state import RobotState

# MoveIt python library
from moveit.planning import (
    MoveItPy,
)
import time

def plan_and_execute(
    robot,
    planning_component,
    logger,
    single_plan_parameters=None,
    multi_plan_parameters=None,
    sleep_time=0.0,
):
    """Helper function to plan and execute a motion."""
    # plan to goal
    logger.info("Planning trajectory")
    if multi_plan_parameters is not None:
        plan_result = planning_component.plan(
            multi_plan_parameters=multi_plan_parameters
        )
    elif single_plan_parameters is not None:
        plan_result = planning_component.plan(
            single_plan_parameters=single_plan_parameters
        )
    else:
        plan_result = planning_component.plan()

    # execute the plan
    if plan_result:
        logger.info("Executing plan")
        robot_trajectory = plan_result.trajectory
        robot.execute(robot_trajectory, controllers=[])
    else:
        logger.error("Planning failed")

    time.sleep(sleep_time)

def main():

    # Initialize rclpy and ROS logger
    rclpy.init()
    logger = get_logger("moveit_py.pose_goal")

    # Instantiate MoveItPy and get planning component
    ur5e = MoveItPy(node_name="moveit_py")
    ur5e_arm = ur5e.get_planning_component("ur5e_arm")

    # Create a robot state to set the joint values
    robot_model = ur5e.get_robot_model()
    robot_state = RobotState(robot_model)

    # Set plan start state to current state
    ur5e_arm.set_start_state_to_current_state()

    # Set target joint values
    joint_values = {
        "shoulder_pan_joint": 0.0,
        "shoulder_lift_joint": -1.5,
        "elbow_joint": 0.0,
        "wrist_1_joint": 0.0,
        "wrist_2_joint": 0.0,
        "wrist_3_joint": 0.0,
    }

    robot_state.joint_positions = joint_values

    # Set the joint values as the goal state
    ur5e_arm.set_goal_state(robot_state=robot_state)


    plan_and_execute(ur5e, ur5e_arm, logger, sleep_time=0.02)

    ur5e_arm.set_start_state_to_current_state()


    ########gripper plan###

    gripper = ur5e.get_planning_component("gripper")

    # Set plan start state to current state
    gripper.set_start_state_to_current_state()

    # Set target joint values
    joint_values = {
        "robotiq_85_left_knuckle_joint": 0.8,
    }

    robot_state.joint_positions = joint_values

    # Set the joint values as the goal state
    gripper.set_goal_state(robot_state=robot_state)


    plan_and_execute(ur5e, gripper, logger, sleep_time=0.02)

    gripper.set_start_state_to_current_state()

    # rclpy.shutdown()

if __name__ == "__main__":
    main()