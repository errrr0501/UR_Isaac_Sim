# UR_Isaac_Sim

This repository provides a moveit_config to control a UR5e robot using Moveit2. Currently for ROS2 Humble.

## Getting Started

To get started with this repository, follow these steps:

1. Build your own workspace and clone Universal_Robots_ROS2_Description repository to your local (edited for isaac sim).
````
mkdir -p <your workspace>/src
cd <your workspace>/src
git clone https://github.com/errrr0501/Universal_Robots_ROS2_Description.git -b humble
````

2. Clone robotiq_85_gripper repository to your local.
````
https://github.com/errrr0501/robotiq_85_gripper.git
````

3. Clone this repository to your local.
````
https://github.com/errrr0501/UR_Isaac_Sim.git
````

4. Install ROS2 Humble and Moveit2 if you haven't already.

5. Open a terminal and run the following command

````
cd <your workspace>
sudo apt update
rosdep update
rosdep install --ignore-src --from-paths src -y
sudo apt install ros-humble-*-controllers
sudo apt install ros-humble-joint-state-broadcaster
sudo apt install ros-humble-topic-based-ros2-control
colcon build --symlink-install
````

6. Open a terminal to isaac sim directory, usual be "~/.local/share/ov/pkg/isaac_sim-<version>/"

7. Locate ````ur5e_gripper_fixed.usd```` in your localhost Nucleus environment under the folder name ````ur5e_gripper````, if you haven't install Nucleus, please check ````https://docs.omniverse.nvidia.com/isaacsim/latest/installation/install_faq.html````.

8. Enable your ROS2_bridge in isaac_sim ````https://docs.omniverse.nvidia.com/isaacsim/latest/installation/install_ros.html#enabling-the-ros-bridge-extension````.

9. Export fastdds.xml for ROS2 Bridge and launch Isaac-sim with ur5e_gripper inside the launch directory
 ````
 export FASTRTPS_DEFAULT_PROFILES_FILE=~/.ros/fastdds.xml
 ./python.sh <your_workspace>/src/ur_isaac_sim/isaac_moveit_ur5e.py 
 ````

10. Open another terminal to your workspace
````
cd <your workspace>
. install/setup.bash
ros2 launch ur5e_gripper_moveit_config isaac_demo.launch.py ros2_control_hardware_type:=isaac

````

11. Then your ur5e in isaac sim should connect to ur5e in Rviz, now you can plan ur5e and make it move in isaac sim

12. If you want to build your own robot, you can use urdf importer in isaac sim with your robot model to export USD file, but probably need to add articulation root on it and check it can move or not after press start button. 


