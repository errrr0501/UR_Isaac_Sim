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

    # set plan start state to current state
    ur5e_arm.set_start_state_to_current_state()

    # Create PoseStamped message that will hold the target Pose
    from geometry_msgs.msg import PoseStamped
    pose_goal = PoseStamped()
    pose_goal.header.frame_id = "base_link"

    # Describe the target pose for the end-effector
    pose_goal.pose.orientation.w = 1.0
    pose_goal.pose.position.x = 0.83
    pose_goal.pose.position.y = 0.27
    pose_goal.pose.position.z = 0.35

    # Set the target pose
    ur5e_arm.set_goal_state(pose_stamped_msg=pose_goal, pose_link="tool0")

    plan_and_execute(ur5e, ur5e_arm, logger, sleep_time=0.02)

    ur5e_arm.set_start_state_to_current_state()

    rclpy.shutdown()

if __name__ == "__main__":
    main()