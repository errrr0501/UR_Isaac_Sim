#include <memory>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
// #include <moveit_visual_tools/moveit_visual_tools.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>


/**
 * Pilz Example -- MoveGroup Interface
 *
 * To run this example, first run this launch file:
 * ros2 launch moveit2_tutorials pilz_moveit.launch.py
 *
 * For best results, hide the "MotionPlanning" widget in RViz.
 *
 * Then, run this file:
 * ros2 run moveit2_tutorials pilz_move_group
 */

int main(int argc, char* argv[])
{
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
      "pilz_move_group_node", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  // We spin up a SingleThreadedExecutor so MoveItVisualTools interact with ROS
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  auto spinner = std::thread([&executor]() { executor.spin(); });

  // Create a ROS logger
  auto const logger = rclcpp::get_logger("pilz_move_group_node");

  // Create the MoveIt MoveGroup Interface
  using moveit::planning_interface::MoveGroupInterface;
  auto move_group_interface = MoveGroupInterface(node, "ur5e_arm");

  // Construct and initialize MoveItVisualTools
  // auto moveit_visual_tools =
  //     moveit_visual_tools::MoveItVisualTools{ node, "base_link", rviz_visual_tools::RVIZ_MARKER_TOPIC,
  //                                             move_group_interface.getRobotModel() };
  // moveit_visual_tools.deleteAllMarkers();
  // moveit_visual_tools.loadRemoteControl();

  // // Create closures for visualization
  // auto const draw_title = [&moveit_visual_tools](const auto& text) {
  //   auto const text_pose = [] {
  //     auto msg = Eigen::Isometry3d::Identity();
  //     msg.translation().z() = 1.0;
  //     return msg;
  //   }();
  //   moveit_visual_tools.publishText(text_pose, text, rviz_visual_tools::WHITE, rviz_visual_tools::XLARGE);
  // };
  // auto const prompt = [&moveit_visual_tools](const auto& text) { moveit_visual_tools.prompt(text); };
  // auto const draw_trajectory_tool_path =
  //     [&moveit_visual_tools, jmg = move_group_interface.getRobotModel()->getJointModelGroup("panda_arm")](
  //         auto const trajectory) { moveit_visual_tools.publishTrajectoryLine(trajectory, jmg); };

  // Helper to plan and execute motion
  auto const plan_and_execute = [&](const std::string& title) {
    // prompt("Press 'Next' in the RVizVisualToolsGui window to plan");
    // draw_title("Planning " + title);
    // moveit_visual_tools.trigger();
    auto const [success, plan] = [&move_group_interface] {
      moveit::planning_interface::MoveGroupInterface::Plan msg;
      auto const ok = static_cast<bool>(move_group_interface.plan(msg));
      return std::make_pair(ok, msg);
    }();

    // Execute the plan
    if (success)
    {
      // draw_trajectory_tool_path(plan.trajectory);
      // moveit_visual_tools.trigger();
      // prompt("Press 'Next' in the RVizVisualToolsGui window to execute");
      // draw_title("Executing " + title);
      // moveit_visual_tools.trigger();
      move_group_interface.execute(plan);
    }
    else
    {
      RCLCPP_ERROR(logger, "Planning failed!");
      // draw_title("Planning Failed!");
      // moveit_visual_tools.trigger();
    }
  };

  // Plan and execute a multi-step sequence using Pilz
  move_group_interface.setPlanningPipelineId("pilz_industrial_motion_planner");
  // Current pose
  // geometry_msgs::msg::Pose start_pose = move_group_interface.getCurrentPose().pose;
  geometry_msgs::msg::PoseStamped current_pose = move_group_interface.getCurrentPose();

  tf2::Quaternion orientation;
  tf2::fromMsg(current_pose.pose.orientation, orientation);

  tf2::Vector3 translation_in_tool_frame(0.0, 0.0, 0.1); // Move z 10 cm forward
  tf2::Vector3 translation_in_world_frame = tf2::quatRotate(orientation, translation_in_tool_frame);

  // Create a target Pose for the end-effector
  geometry_msgs::msg::Pose target_pose = current_pose.pose;
  // target_pose.position.x += translation_in_world_frame.x();
  // target_pose.position.y += translation_in_world_frame.y();
  // target_pose.position.z += translation_in_world_frame.z();
  {
    
    // Move to a pre-grasp pose
    move_group_interface.setPlannerId("LIN");
    auto const pre_grasp_pose = [target_pose, translation_in_world_frame] {

      geometry_msgs::msg::PoseStamped msg;
      // msg.pose = start_pose;
      // msg.header.frame_id = "world";
      // msg.pose.orientation.x = start_pose.orientation.x;
      // msg.pose.orientation.y = start_pose.orientation.y;
      // msg.pose.orientation.z = start_pose.orientation.z;
      // msg.pose.orientation.w = start_pose.orientation.w;
      // msg.pose.position.x = start_pose.position.x;
      // msg.pose.position.y = start_pose.position.y;
      // msg.pose.position.z = start_pose.position.z+0.2;
      msg.header.frame_id = "world";
      msg.pose.orientation.x = target_pose.orientation.x;
      msg.pose.orientation.y = target_pose.orientation.y;
      msg.pose.orientation.z = target_pose.orientation.z;
      msg.pose.orientation.w = target_pose.orientation.w;
      msg.pose.position.x = target_pose.position.x+translation_in_world_frame.x();
      msg.pose.position.y = target_pose.position.y+translation_in_world_frame.y();
      msg.pose.position.z = target_pose.position.z+translation_in_world_frame.z();
      return msg;
    }();
    // auto const pre_grasp_pose = [start_pose] {

    //   geometry_msgs::msg::Pose msg;
    //   // msg.pose = start_pose;
    //   msg.pose.orientation.x = start_pose.orientation.x;
    //   msg.pose.orientation.y = start_pose.orientation.y;
    //   msg.pose.orientation.z = start_pose.orientation.z;
    //   msg.pose.orientation.w = start_pose.orientation.w;
    //   msg.pose.position.x = start_pose.position.x;
    //   msg.pose.position.y = start_pose.position.y;
    //   msg.pose.position.z = start_pose.position.z+0.2;
    //   // msg.pose.orientation.x = 1.0;
    //   // msg.pose.orientation.y = 0.0;
    //   // msg.pose.orientation.z = 0.0;
    //   // msg.pose.orientation.w = 0.0;
    //   // msg.pose.position.x = 0.6;
    //   // msg.pose.position.y = -0.2;
    //   // msg.pose.position.z = 0.6;
    //   return msg;
    // }();
    // move_group_interface.setPoseReferenceFrame("flange");
    move_group_interface.setPoseTarget(pre_grasp_pose, "tool0");
    // move_group_interface.setRPYTarget()
    plan_and_execute("[PTP] Approach");
  }

  // {
  //   // Move in a linear trajectory to a grasp pose using the LIN planner.
  //   move_group_interface.setPlannerId("LIN");
  //   auto const grasp_pose = [] {
  //     geometry_msgs::msg::PoseStamped msg;
  //     msg.header.frame_id = "world";
  //     msg.pose.orientation.x = 1.0;
  //     msg.pose.orientation.y = 0.0;
  //     msg.pose.orientation.z = 0.0;
  //     msg.pose.orientation.w = 0.0;
  //     msg.pose.position.x = 0.6;
  //     msg.pose.position.y = -0.2;
  //     msg.pose.position.z = 0.4;
  //     return msg;
  //   }();
  //   move_group_interface.setPoseTarget(grasp_pose, "ur5e_arm");
  //   plan_and_execute("[LIN] Grasp");
  // }

  // {
  //   // Move in a circular arc motion using the CIRC planner.
  //   move_group_interface.setPlannerId("CIRC");
  //   auto const goal_pose = [] {
  //     geometry_msgs::msg::PoseStamped msg;
  //     msg.header.frame_id = "world";
  //     msg.pose.orientation.x = 0.7071;
  //     msg.pose.orientation.y = 0.0;
  //     msg.pose.orientation.z = 0.0;
  //     msg.pose.orientation.w = 0.7071;
  //     msg.pose.position.x = 0.6;
  //     msg.pose.position.y = 0.0;
  //     msg.pose.position.z = 0.6;
  //     return msg;
  //   }();
  //   move_group_interface.setPoseTarget(goal_pose, "ur5e_arm");

  //   // Set a constraint pose. This is the center of the arc.
  //   auto const center_pose = [] {
  //     geometry_msgs::msg::PoseStamped msg;
  //     msg.header.frame_id = "world";
  //     msg.pose.orientation.x = 1.0;
  //     msg.pose.orientation.y = 0.0;
  //     msg.pose.orientation.z = 0.0;
  //     msg.pose.orientation.w = 0.0;
  //     msg.pose.position.x = 0.6;
  //     msg.pose.position.y = 0.0;
  //     msg.pose.position.z = 0.4;
  //     return msg;
  //   }();
    // moveit_msgs::msg::Constraints constraints;
    // constraints.name = "center";  // Change to "interim" to use an intermediate point on arc instead.
    // moveit_msgs::msg::PositionConstraint pos_constraint;
    // pos_constraint.header.frame_id = center_pose.header.frame_id;
    // pos_constraint.link_name = "flange";
    // pos_constraint.constraint_region.primitive_poses.push_back(center_pose.pose);
    // pos_constraint.weight = 1.0;
    // constraints.position_constraints.push_back(pos_constraint);
    // move_group_interface.setPathConstraints(constraints);

    // plan_and_execute("[CIRC] Turn");
  // }

  {
    // Move back home using the PTP planner.
    move_group_interface.setPlannerId("PTP");
    // move_group_interface.setNamedTarget("ready");
    plan_and_execute("[PTP] Return");
  }

  // Shutdown ROS
  spinner.join();
  rclcpp::shutdown();
  return 0;
}
// int main(int argc, char * argv[])
// {
//   // Initialize ROS and create the Node
//   rclcpp::init(argc, argv);
//   auto const node = std::make_shared<rclcpp::Node>("pose_goal");

//   // Create a ROS logger
//   auto const logger = rclcpp::get_logger("pose_goal");

//   // Create the MoveIt Move Group Interface for panda arm
//   using moveit::planning_interface::MoveGroupInterface;
//   auto move_group_interface = MoveGroupInterface(node, "ur5e_arm");

//   // Create a target Pose for the end-effector
//   geometry_msgs::msg::Pose target_pose;
//   target_pose.orientation.w = 1.0;
//   target_pose.position.x = 0.83;
//   target_pose.position.y = 0.27;
//   target_pose.position.z = 0.35;

//   // Set the target pose
//   move_group_interface.setPoseTarget(target_pose);

//   // Create a plan to that target pose and check if that plan is successful
//   moveit::planning_interface::MoveGroupInterface::Plan my_plan;
//   bool success = (move_group_interface.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);

//   // If the plan is successful, execute the plan
//   if(success) {
//     move_group_interface.execute(my_plan);
//   } else {
//     RCLCPP_ERROR(logger, "Planing failed!");
//   }

//   // Shutdown
//   rclcpp::shutdown();
//   return 0;
// }
