#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

int main(int argc, char * argv[])
{
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>("tool_coordinate_move");

  // Create a ROS logger
  auto const logger = rclcpp::get_logger("tool_coordinate_move");

  // Create the MoveIt Move Group Interface for panda arm
  using moveit::planning_interface::MoveGroupInterface;
  auto move_group_interface = MoveGroupInterface(node, "ur5e_arm");


  // We spin up a SingleThreadedExecutor to get current pose of the robot later
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  auto spinner = std::thread([&executor]() { executor.spin(); });

  geometry_msgs::msg::PoseStamped current_pose = move_group_interface.getCurrentPose();

  tf2::Quaternion orientation;
  tf2::fromMsg(current_pose.pose.orientation, orientation);

  tf2::Vector3 translation_in_tool_frame(0.0, 0.0, 0.1); // Move z 10 cm forward
  tf2::Vector3 translation_in_world_frame = tf2::quatRotate(orientation, translation_in_tool_frame);

  // Create a target Pose for the end-effector
  geometry_msgs::msg::Pose target_pose = current_pose.pose;
  target_pose.position.x += translation_in_world_frame.x();
  target_pose.position.y += translation_in_world_frame.y();
  target_pose.position.z += translation_in_world_frame.z();


  std::vector<geometry_msgs::msg::Pose> waypoints;
  
  // Add target pose to waypoints
  waypoints.push_back(target_pose); 

  // Variable for next target pose
  geometry_msgs::msg::Pose target_pose2 = target_pose;

  // // Move only along one axis
  // target_pose2.position.y -= 0.1; //Right

  // // Add next target pose to waypoints
  // waypoints.push_back(target_pose2);


  // We want the Cartesian path to be interpolated at a resolution of 1 cm which is why we will specify 0.01 as the max step in Cartesian translation
  // We will specify the jump threshold as 0.0, effectively disabling it
  moveit_msgs::msg::RobotTrajectory trajectory;
  const double jump_threshold = 0.0;
  const double eef_step = 0.01;
  
  // Computing the Cartesian path, which is stored in trajectory
  double fraction = move_group_interface.computeCartesianPath(waypoints, eef_step,   jump_threshold, trajectory);
  RCLCPP_INFO(logger, "Visualizing Cartesian path plan (%.2f%% achieved)", fraction * 100.0);

  // Check if complete path is possible and execute the trajectory
  if(fraction == 1){
    move_group_interface.execute(trajectory);
  } else {
    RCLCPP_ERROR(logger, "Planing failed!");
  }


  // Shutdown
  rclcpp::shutdown();
  spinner.join();
  return 0;
}
