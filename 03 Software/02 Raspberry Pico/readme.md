# Raspberry Pico
This folder contains code for the Raspberry Pico microcontrollers used to control peripherals of MARC. 

## Pico driver
These directories include the C/C++ code for controlling the Differential Drive module. 

Version 1.0 is only included for completness. Use V1.1, as it includes many stability and performance improvements.

### Setup and tasks
The microcontroller utilizes micro-ROS for integration into the bigger ROS2 project. The implementation is based on timers for executing sampling and control routines. V1.1 fully utilizes both cores of the Raspberry Pico. Core 0 is used for the micro-ROS execution and general senor sampling including IMU data and ultrasonic range measurements. Core 2 is solely used for kinematic transformations of velocity commands, decoupled DC motor control via PWM and calculation of the wheel odometry.

The implementation is mainly done header-only to stay independet of the hardware platform.

The code project includes the following files:
*   [diff_drive_utils.h](pico_driver_v1.1/diff_drive_utils.h): Methods and calculations Differential Drive kinematics
*   [driver_config.h](pico_driver_v1.1/driver_config.h): Parameter configuration of the driver including dimensions, ROS2 config and control settings
*   [pico_driver.c](pico_driver_v1.1/pico_driver.c): Main project file
*   [pico_interface.h](pico_driver_v1.1/pico_interface.h): Methods for interacting with the Raspberry Pico including reading and writing I/O signals and accessing timers
*   [quadrature_encoder.pio](pico_driver_v1.1/quadrature_encoder.pio): Register-based quadrature encoder reading [source](https://github.com/raspberrypi/pico-examples/blob/master/pio/quadrature_encoder/quadrature_encoder.pio)
*   [ros_interface.h](pico_driver_v1.1/ros_interface.h): Methods for interacting with the ROS2 network e.g. publishing and listening to topics