/*  Includes all configuration settings for the differential drive plattform.
    Author:     Alexander Raab
    E-Mail:     alex.raab@liwest.at
    Date:       17.06.2023
    Version:    1.1
    Licence:    TODO

    This header is used to store all configuaration settings, global variables and macros relevant for the differential drive plattform in one file.
*/

#pragma once

// ROS2 settings
// --- Main settings
#define DOMAIN_ID (0)
#define NAMESPACE_DRIVER ""                         // Node namespace 
#define NAMESPACE_SENSOR ""                         // Namespace for sensor data

// --- Frames
#define BASE_LINK "base_link"                       // Robot base link
#define IMU_FRAME "imu"                             // IMU sensor link
#define ODOM_FRAME "odom"                           // Odometry link
#define ULTRASONIC_1_FRAME "ultrasonic_1_link"      // Ultrasonic sensor 1 link
#define ULTRASONIC_2_FRAME "ultrasonic_2_link"      // Ultrasonic sensor 2 link
#define ULTRASONIC_3_FRAME "ultrasonic_3_link"      // Ultrasonic sensor 3 link
#define ULTRASONIC_4_FRAME "ultrasonic_4_link"      // Ultrasonic sensor 4 link
#define ULTRASONIC_5_FRAME "ultrasonic_5_link"      // Ultrasonic sensor 5 link
#define ULTRASONIC_6_FRAME "ultrasonic_6_link"      // Ultrasonic sensor 6 link
#define ULTRASONIC_7_FRAME "ultrasonic_7_link"      // Ultrasonic sensor 7 link
#define ULTRASONIC_8_FRAME "ultrasonic_8_link"      // Ultrasonic sensor 8 link

// --- Topics
#define IMU_TOPIC "imu"                             // IMU data topic
#define ODOM_TOPIC "odom"                           // Odometry data topic
#define SONAR_TOPIC "ultrasonic/range"              // Ultrasonic range data topic
#define CMD_VEL_TOPIC "/cmd_vel"                    // Veclocity command topic

// Misc.
#define CONTEXT_CNT (3)                             // Context count - must match number of timers and subscribers

// Robot settings
// --- Hardware specifications
#define ENCODER_CPR         (12.)                   // Pulses per revoultion of motor shaft
#define GEAR_RATIO          (75.)                   // Gear ratio x:1
#define WHEEL_CPR           (900.)                  // Pulses per revoultion of wheel
#define WHEEL_RADIUS        (0.016)                 // Radius of a wheel
#define WHEEL_TRACK         (0.0625)                // Distance from center of the robot to one wheel

#define SONAR_FOV           (0.2618)                // Measurement angle of the HC-SR04 in radians (15°)
#define SONAR_MIN_RANGE     (0.002)                 // Shortes measureable distance of the HC-SR04
#define SONAR_MAX_RANGE     (4.000)                 // Farthest measureable distance of the HC-SR04

// --- Sample and controller settings
// --- --- Sensors enable
#define ULTRASONIC_5_ON     (true)                  // Enable ultrasonic sensor 5
#define ULTRASONIC_8_ON     (true)                  // Enable ultrasonic sensor 8

// --- --- Timers
#define TIMER_SAMPLE_INTERVAL   (50)                // Publisher and sensor sample timer in ms
#define TIMER_CONTROL_INTERVAL  (25)                // Control timer in ms

// --- --- Motor velocity control
#define MAX_MOT_PWM         (65025)                 // Maximum PWM value for motor control
#define MIN_MOT_PWM         (5500)                  // Minimum PWM value for motor control
#define MAX_WHEEL_VEL_ANG   (50.)                   // Maximum angular wheel velocity in rad/s
#define MIN_WHEEL_VEL_ANG   (0.625)                 // Minimum angular wheel velocity in rad/s

#define MOT_PID_P           (.25)                   // Motor PID controller P-gain
#define MOT_PID_I           (7.0)                   // Motor PID controller I-gain
#define MOT_PID_D           (.01)                   // Motor PID controller D-gain
#define MOT_PID_N           (50.)                   // Motor PID controller filter coefficient
#define MOT_CTR_FF          (1300.5)                // Motor control feed-forward constant MAX_MOT_PWM/MAX_WHEEL_VEL_ANG




// Raspberry Pico
// --- Hardware config
// --- --- GPIO Pins
#define PIN_I2C0_SDA    (0)                         // I2C SDA
#define PIN_I2C0_SCL    (1)                         // I2C SCL

#define PIN_TRIG_1      (2)                         // Trigger 1 for ultrsonic sensors 1,3,5,7
#define PIN_TRIG_2      (3)                         // Trigger 2 for ultrsonic sensors 2,4,6,8
#define PIN_ECHO_1      (4)                         // Echo for ultrsonic sensors 1
#define PIN_ECHO_2      (5)                         // Echo for ultrsonic sensors 2
#define PIN_ECHO_3      (6)                         // Echo for ultrsonic sensors 3
#define PIN_ECHO_4      (7)                         // Echo for ultrsonic sensors 4
#define PIN_ECHO_5      (8)                         // Echo for ultrsonic sensors 5
#define PIN_ECHO_6      (9)                         // Echo for ultrsonic sensors 6
#define PIN_ECHO_7      (10)                        // Echo for ultrsonic sensors 7
#define PIN_ECHO_8      (11)                        // Echo for ultrsonic sensors 8

#define PIN_MOT_ON      (15)                        // Motor driver enable

#define PIN_ENC_R_A     (18)                        // Right motor encoder output A
#define PIN_ENC_R_B     (19)                        // Right motor encoder output B
#define PIN_MOT_R_PWM   (20)                        // Right motor PWM input 
#define PIN_MOT_R_FOR   (21)                        // Right motor direction input forward
#define PIN_MOT_R_BAC   (22)                        // Right motor direction input backward

#define PIN_ENC_L_A     (12)                        // Left motor encoder output A
#define PIN_ENC_L_B     (13)                        // Left motor encoder output B
#define PIN_MOT_L_PWM   (14)                        // Left motor PWM input 
#define PIN_MOT_L_FOR   (16)                        // Left motor direction input forward
#define PIN_MOT_L_BAC   (17)                        // Left motor direction input backward

#define LED_BUILTIN     (25)                        // Internal LED

// --- --- PIO
#define PIO_ENC_L_SM    (0)                        // PIO statemachine left encoder
#define PIO_ENC_R_SM    (1)                        // PIO statemachine right encoder


// -- Sensor config
// --- --- I2C Sensors and addresses
#define I2C_BAUDRATE    (400e3)                     // I2C baudrate: 400kHz
#define I2C_INSTANCE    (i2c0)                      // I2C instance
#define I2C_TIMEOUT_US  (100)                       // Timeout for read and write functions

// --- --- --- 6-DOF IMU LSM6DS33
#define LSM_ADDR        (0x6A)                      // Device address
#define LSM_CTRL1_XL    (0x10)                      // Control register acceleration rate & range
#define LSM_CTRL2_G     (0x11)                      // Control register gyro rate & range
#define LSM_CTRL3_C     (0x12)                      // Common settings
#define LSM_CTRL6_C     (0x15)                      // Common settings 
#define LSM_OUT_TEMP_L  (0x20)                      // LSB of temperature data
#define LSM_OUTX_L_G    (0x22)                      // LSB of gyro x-data
#define LSM_OUTY_L_G    (0x24)                      // LSB of gyro y-data
#define LSM_OUTZ_L_G    (0x26)                      // LSB of gyro z-data
#define LSM_OUTX_L_XL   (0x28)                      // LSB of acceleration x-data
#define LSM_OUTY_L_XL   (0x30)                      // LSB of acceleration y-data
#define LSM_OUTZ_L_XL   (0x32)                      // LSB of acceleration z-data

#define LSM_CONFIG_CTRL1 (0x48)                     // Acceleration config: 4g@104Hz b01001000
#define LSM_CONFIG_CTRL2 (0x44)                     // Gyro config: 500dps@104Hz b01000100
#define LSM_CONFIG_CTRL3 (0x04)                     // Common config: automatically increment register address b00000100

#define LSM_GYRO_SCALE   (0.0175)                   // Scaling factor gyro @ 500 DPS - 0.14@4000, 0.07@2000, 0.035@1000...
#define LSM_ACCE_SCALE   (0.000122)                 // Scaling factor for acceleration @ 4g - 0.488@16g, 0.244@8g, ...
#define LSM_TEMP_SCALE   (16)                       // Scaling factor for temperature


// Misc. constants and conversion factors
#define CONV_DPS_TO_RPS  (0.01745329251)            // 1°/s = pi/180 rad/s
#define CONV_G_TO_MPS2   (9.80665)                  // 1g = 9.8... m/s2
#define SOUND_SPEED_2    (0.0001716)                // 1/2 * Speed of sound in air in m/us
const double IMU_ANG_VEL_COV[9] = {0, 0, 0, 0, 0, 0, 0, 0, 0};                              // IMU angular velocity covariance matrix
const double IMU_LIN_ACC_COV[9] = {0, 0, 0, 0, 0, 0, 0, 0, 0};                              // IMU linear acceleration covariance matrix
const double ODOM_POS_COV[36]{1e-9, 1e-9, 1e-9, 1e-9, 1e-9, 1e-9, 1e-9, 1e-9, 1e-9,
                                1e-9, 1e-9, 1e-9, 1e-9, 1e-9, 1e-9, 1e-9, 1e-9, 1e-9,
                                1e-9, 1e-9, 1e-9, 1e-9, 1e-9, 1e-9, 1e-9, 1e-9, 1e-9,
                                1e-9, 1e-9, 1e-9, 1e-9, 1e-9, 1e-9, 1e-9, 1e-9, 1e-9};      // Odometry position covariance matrix
const double ODOM_VEL_COV[36]{1e-9, 1e-9, 1e-9, 1e-9, 1e-9, 1e-9, 1e-9, 1e-9, 1e-9,
                                1e-9, 1e-9, 1e-9, 1e-9, 1e-9, 1e-9, 1e-9, 1e-9, 1e-9,
                                1e-9, 1e-9, 1e-9, 1e-9, 1e-9, 1e-9, 1e-9, 1e-9, 1e-9,
                                1e-9, 1e-9, 1e-9, 1e-9, 1e-9, 1e-9, 1e-9, 1e-9, 1e-9};      // Odometry velocity covariance matrix


// Global sensor data buffers
// --- IMU LSM6DS33
float imu_lin_acc[3];                   // IMU linear acceleration [x,y,z] in m/s^2
float imu_ang_vel[3];                   // IMU angular velocity [x,y,z] in rad/s

// --- Ultrasonic range sensors
float ultrasonic_range_results[8] = {0};    // Stores the latest range measurement of each sensor (sensors 1-8) in m

// --- Encoder data
int enc_ticks[2] = {0};                 // Encoder ticks [left,right]

// --- Motor data
// double motor_angle[2] = {0};          // Motor angle in rad - unused
double motor_spd[2] = {0};              // Motor speed in rad/s

// --- Odometry data
float odom_pos[3] = {0};                // Odometry positions and orientation [x,y,rot_z] in m or rad
float odom_vel[2] = {0};                // Odometry velocities [vx,wz] in m/s or rad/s

// --- Control data
double cmd_vel_body[2] = {0};           // Velocity command for body velcoity [vx,wz] in m/s or rad/s
int pwm_out_mot[2] = {0};               // PWM outputs of the velocity controller for the left and right motors 

