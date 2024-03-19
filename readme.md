# Project MARC
This repository containes the documentation of the Modular Autonomous Research Companion (MARC).

<img src="04%20Documentation/img/MARC_rendering_3D.png" alt="MARC" width="600"/>

## Purpose and goal
MARC is a small mobile robot built for at-home testing and experimentation with autonomous robots and all related tasks.
The initial project was started in 2023 with the goal of having a compact and cost-efficient plattform, which can be easily adapted for a multitude of different tasks. For this MARC is setup in different layered modules. Each having a clear functionality as well as providing a defined interface for controlling and for sensory feedback.

## Current status
Up until now, the development mainly focused on setting up the first prototype. Thus, most time was spent on designing a robust implementation of the drive layer, responsible for moving the robot and providing basic sensory feedback for autonomous navigation.

### Hardware
<img src="04%20Documentation/img/MARC_img_front.jpeg" alt="MARC" width="600"/>
At this stage, prototypes for the 4 most crucial modules of MARC are completed.

| Module name              | Functionality                                     | Interface              |
|--------------------------|---------------------------------------------------|------------------------|
| Differential Drive Layer | Locomotion, odometry and basic obstacle detection | micro-ROS via UART     |
| Battery Layer            | Houses the main battery                           | n.a.                   |
| Computation Layer        | Containes the main computer and a RGB camera      | Joins all other layers |
| LiDAR Layer              | Main 2D LiDAR for localization                    | UART                   |

#### Differential Drive Layer
This layer is a implementation of a Differential Drive robot. Computations are done on a [Raspberry Pi Pico](https://www.raspberrypi.com/products/raspberry-pi-pico/) mounted on a custom designed PCB. 2 DC motors together with PWM-based motor drivers are used for locomotion. 
The odometry data is currently only computed from wheel encoder signals. Additionally an on-board IMU provides linear acceleration and angular velocity feedback. 
Basic obstacle avoidance can be implemented using 8 ultrasonic distance sensors placed on the circumference of the chassis.

#### Battery Layer
MARC is planned to be powerd either via on-board battery or using a stationary power supply. For now, this layer houses a step-down power supply for converting the input voltage to 12V and 5V in order to supply the other layers.

#### Computation Layer
The main computation unit, a [Raspberry Pi 4B](https://www.raspberrypi.com/products/raspberry-pi-4-model-b/) with 4GB of RAM, is used for all high-level tasks. This includes handling the ROS2 network, processing all data from other layers and providing WIFI connectivity for remote control. Additionally, the main camera is also directly connected to the computer. 

#### LiDAR
For localization, this module contains a 2D LiDAR sensor. The range measurements are streamed over a serial interface and processed directly on the main computer.


### Software
The current version of MARC is based on [ROS2 Humble]( https://docs.ros.org/en/humble/index.html) and [micro-ROS]( https://micro.ros.org/). For now, [Nav2]( https://navigation.ros.org/) is used for all navigation related tasks.

Due to the current focus on hardware prototypes and low-level locomotion control, the ROS2 setup is still under development and may not be completely stable.

Simulation with Gazebo 11          |  Visualization in RVIZ2
:-------------------------:|:-------------------------:
<img src="04%20Documentation/img/gazebo_sim.png" alt="Simulation with Gazebo 11" width="400"/>  |  <img src="04%20Documentation/img/rviz2_viz.png" alt="Visualization with RVIZ2" width="400"/>

## Roadmap and planned features

Current development

*   Finishing the intial setup of the ROS2 project for basic SLAM tasks
*   Stabilizing the ROS2 network to improve performance
    *   Revisit the implementation of the camera driver
    *   Improve performance of the custom LiDAR driver
  
Planned features
*   Differential Drive Layer
    *   Implement on-board odometry fusion with an EKF
    *   Change control setup from decoupled PIDs to a model based controller
*   Motion planning and control
    * Implement a trajectory tracking control based on input-output linearization
*   Object detection and classification
    *    Utilize a [YOLO](https://github.com/ultralytics/ultralytics) model for basic on-board detection tasks  
*   Environment interaction
    *   Design and implement a prototype layer for interacting with the environment (e.g. a 3-DOF robotic arm, gripper or fork-lift module)
