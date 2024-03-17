/*  Includes the setup and ROS2 specific methods.
    Author:     Alexander Raab
    E-Mail:     alex.raab@liwest.at
    Date:       17.06.2022
    Version:    1.1
    Licence:    TODO

    This header is used to setup and interface the ROS via micro-ROS.
*/

#pragma once



#include <std_msgs/msg/string.h>
#include <nav_msgs/msg/odometry.h>
#include <sensor_msgs/msg/range.h>
#include <sensor_msgs/msg/imu.h>
#include <geometry_msgs/msg/twist.h>

#include "driver_config.h"
#include "pico_interface.h"


// Macros
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc); return 1;}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}

// ROS variables
rcl_node_t node_driver;

//rcl_timer_t timer_control;
rcl_allocator_t allocator;
rclc_support_t support;
rclc_executor_t executor;

// --- Publishers
rcl_publisher_t odom_pub;
rcl_publisher_t imu_pub;
rcl_publisher_t sonar_pub;

// --- Subscribers
rcl_subscription_t cmd_vel_sub;

// --- Timers 
rcl_timer_t timer_sample;

// --- Messages
geometry_msgs__msg__Twist msg_cmd_vel;
nav_msgs__msg__Odometry msg_odom;
sensor_msgs__msg__Imu msg_imu;
sensor_msgs__msg__Range msg_sonar_range;

// --- System health
bool cmd_vel_watchdog = false;
uint cmd_vel_timeouts = 0;
uint max_cmd_vel_timeouts = 2;

// Flags
bool sample_range_sensors_even = false;
volatile bool timer_update_sample = false;
volatile bool timer_update_ctrl = false;

// Callback of the sample timer -used for updating the odometry data
void timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{  
    publish_odometry();
    update_wheel_control();
    update_and_publish_imu();
    timer_update_sample = true;
}

// Callback for control timer on core 1
bool timer_ctrl_callback(repeating_timer_t *rt)
{
    timer_update_ctrl = true;
}

// Callback of the cmd_vel subscriber used to update the target velocities
void cmd_vel_callback(const void *msgin)
{
    cmd_vel_watchdog = true;
}

// Publishes odometry data
void publish_odometry()
{
    // Generate message
    fill_msg_stamp(&msg_odom.header.stamp);

    mutex_enter_blocking(&mtx_odom_data);
    msg_odom.pose.pose.position.x = odom_pos[0];
    msg_odom.pose.pose.position.y = odom_pos[1];

    msg_odom.pose.pose.orientation.z = sin(odom_pos[2] / 2);
    msg_odom.pose.pose.orientation.w = cos(odom_pos[2] / 2);


    msg_odom.twist.twist.linear.x = odom_vel[0];
    msg_odom.twist.twist.angular.z = odom_vel[1];
    mutex_exit(&mtx_odom_data);

    // Publish odometry data
    RCSOFTCHECK(rcl_publish(&odom_pub, &msg_odom, NULL));
}

// Update velocity control with current command
void update_wheel_control()
{  
    // Ensure a connection to the main device is given via cmd_vel
    if(!cmd_vel_watchdog)
    {
        cmd_vel_timeouts++;

        if(cmd_vel_timeouts >= max_cmd_vel_timeouts)
        {
            msg_cmd_vel.linear.x = 0;
            msg_cmd_vel.linear.z = 0;
        } 
    }
    else
    {
        cmd_vel_timeouts = 0;
    }

    // Update velocity command
    mutex_enter_blocking(&mtx_cmd_vel);
    cmd_vel_body[2] = {msg_cmd_vel.linear.x, msg_cmd_vel.angular.z};
    mutex_exit(&mtx_cmd_vel);

    cmd_vel_watchdog = false;
}

// Updates and publishes imu data
void update_and_publish_imu()
{
    read_LSM6DS33(&imu_lin_acc, &imu_ang_vel);

    fill_msg_stamp(&msg_imu.header.stamp);

    msg_imu.angular_velocity.x = imu_ang_vel[0];
    msg_imu.angular_velocity.y = imu_ang_vel[1];
    msg_imu.angular_velocity.z = imu_ang_vel[2];

    msg_imu.linear_acceleration.x = imu_lin_acc[0];
    msg_imu.linear_acceleration.y = imu_lin_acc[1];
    msg_imu.linear_acceleration.z = imu_lin_acc[2];
    
    // Publish odometry data
    RCSOFTCHECK(rcl_publish(&imu_pub, &msg_imu, NULL));
}

// Updates and publishes sonar data
void update_sonar()
{  
    fill_msg_stamp(&msg_sonar_range.header.stamp);

    if(sample_range_sensors_even)
    {
        msg_sonar_range.header.frame_id.data = ULTRASONIC_2_FRAME;
        msg_sonar_range.range = ultrasonic_range_results[1];
        RCSOFTCHECK(rcl_publish(&sonar_pub, &msg_sonar_range, NULL));

        msg_sonar_range.header.frame_id.data = ULTRASONIC_4_FRAME;
        msg_sonar_range.range = ultrasonic_range_results[3];
        RCSOFTCHECK(rcl_publish(&sonar_pub, &msg_sonar_range, NULL));

        msg_sonar_range.header.frame_id.data = ULTRASONIC_6_FRAME;
        msg_sonar_range.range = ultrasonic_range_results[5];
        RCSOFTCHECK(rcl_publish(&sonar_pub, &msg_sonar_range, NULL));

        #if ULTRASONIC_8_ON 
            msg_sonar_range.header.frame_id.data = ULTRASONIC_8_FRAME;
            msg_sonar_range.range = ultrasonic_range_results[7];
            RCSOFTCHECK(rcl_publish(&sonar_pub, &msg_sonar_range, NULL));
        #endif

        range_sensor_int_sample(PIN_TRIG_1);
    }
    else
    {
        msg_sonar_range.header.frame_id.data = ULTRASONIC_1_FRAME;
        msg_sonar_range.range = ultrasonic_range_results[0];
        RCSOFTCHECK(rcl_publish(&sonar_pub, &msg_sonar_range, NULL));

        msg_sonar_range.header.frame_id.data = ULTRASONIC_3_FRAME;
        msg_sonar_range.range = ultrasonic_range_results[2];
        RCSOFTCHECK(rcl_publish(&sonar_pub, &msg_sonar_range, NULL));

        #if  ULTRASONIC_5_ON 
        msg_sonar_range.header.frame_id.data = ULTRASONIC_5_FRAME;
        msg_sonar_range.range = ultrasonic_range_results[4];
        RCSOFTCHECK(rcl_publish(&sonar_pub, &msg_sonar_range, NULL));
        #endif
        
        msg_sonar_range.header.frame_id.data = ULTRASONIC_7_FRAME;
        msg_sonar_range.range = ultrasonic_range_results[6];
        RCSOFTCHECK(rcl_publish(&sonar_pub, &msg_sonar_range, NULL));
        
        range_sensor_int_sample(PIN_TRIG_2);
    }
    
    sample_range_sensors_even = !sample_range_sensors_even;
}

// Initializes all publishers
void setup_publishers()
{
    RCCHECK(rclc_publisher_init_default(
        &odom_pub,
        &node_driver,
        ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
        ODOM_TOPIC));

    RCCHECK(rclc_publisher_init_default(
        &imu_pub,
        &node_driver,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
        IMU_TOPIC));

    RCCHECK(rclc_publisher_init_default(
        &sonar_pub,
        &node_driver,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Range),
        SONAR_TOPIC));
}

// Initializes all publishers
void setup_subscribers()
{
    RCCHECK(rclc_subscription_init_default(
        &cmd_vel_sub,
        &node_driver,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        CMD_VEL_TOPIC));
}

// Initializes all timers
void setup_timers()
{
    RCCHECK(rclc_timer_init_default(
        &timer_sample,
        &support,
        RCL_MS_TO_NS(TIMER_SAMPLE_INTERVAL),
        timer_callback));
}

// Setup messages
void setup_msg()
{   
    // Fill frames of odometry msg
    msg_odom.header.frame_id.data = ODOM_FRAME;
    msg_odom.child_frame_id.data = BASE_LINK;
    memcpy(msg_odom.pose.covariance, ODOM_POS_COV, sizeof(ODOM_POS_COV));
    memcpy(msg_odom.twist.covariance, ODOM_VEL_COV, sizeof(ODOM_VEL_COV));

    // Fill frames of imu msg
    msg_odom.header.frame_id.data = IMU_FRAME;
    msg_odom.child_frame_id.data = BASE_LINK;
    msg_imu.orientation.x = -1; // Set to -1 to be interpreted as unknown
    memcpy(msg_imu.angular_velocity_covariance, IMU_ANG_VEL_COV, sizeof(IMU_ANG_VEL_COV));
    memcpy(msg_imu.linear_acceleration_covariance, IMU_LIN_ACC_COV, sizeof(IMU_LIN_ACC_COV));

    // Fill sonar message
    msg_sonar_range.radiation_type = 0;
    msg_sonar_range.field_of_view = SONAR_FOV;
    msg_sonar_range.min_range = SONAR_MIN_RANGE;
    msg_sonar_range.max_range = SONAR_MAX_RANGE;
}

// Fill stamp of message header
void fill_msg_stamp(builtin_interfaces__msg__Time *stamp)
{
    stamp->sec = time_us_32()/1000000;
    stamp->nanosec = (time_us_32()%1000000)*1000;
}