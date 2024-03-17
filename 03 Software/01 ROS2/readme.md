# ROS2 
This folder contains all custom ROS2 packages. 
*Note: MARC is currently developed for ROS2 Humble*

## Usage
For detailed descriptions of how to run the ROS2 project, check the [marc_bringup](marc/marc_bringup) package.

For launching the default simulation use

    ros2 launch marc_bringup MARC.launch.py

after building an sourcing the workspace.

Use 

    ros2 launch marc_bringup MARC.launch.py simulation:=False

for launching the project on the physical robot.

## Dependencies
The ROS2 project depends on
*   General
    *   [xacro](https://index.ros.org/p/xacro/)  
    *   [rviz2](https://index.ros.org/p/rviz2/)
    *   [robot_localization](https://index.ros.org/p/robot_localization/)
    *   [navigation2](https://navigation.ros.org/)
    *   [robot_state_publisher](https://index.ros.org/p/robot_state_publisher/)
    *   [joint_state_publisher](https://index.ros.org/p/joint_state_publisher/)
*   Simulation only
    *   [gazebo_ros](https://index.ros.org/p/gazebo_ros/)
    *   [gazebo_ros_pkgs](https://index.ros.org/p/gazebo_ros_pkgs/)
*   Physical robot
    *   [micro_ros_agent](https://github.com/micro-ROS/)
    *   [image_tools](https://index.ros.org/p/image_tools/)