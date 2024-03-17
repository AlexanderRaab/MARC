/* microROS Differantial Driver Robot
    Author:     Alexander Raab
    E-Mail:     alex.raab@liwest.at
    Date:       12.18.2022
    Version:    1.2
    Licence:    TODO

    An application for the Raspberry Pi Pico to work as a ROS2-driver for a differential drive robot.

    TODO: Additional sensors, tf2 publisher odom -> base_link (optional), parameters for relevant variables, 
    optimize for higher sample rates
            

*/


#include <stdio.h>
#include <math.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/string.h>
#include <nav_msgs/msg/odometry.h>
#include <sensor_msgs/msg/range.h>
#include <sensor_msgs/msg/imu.h>
#include <geometry_msgs/msg/twist.h>

#include <rmw_microros/rmw_microros.h>

#include "diff_drive_utils.h"
#include "pico_interface.h"

// Macros
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc); return 1;}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}

// --- Configuaration

// ROS2
#define NAMESPACE_SENSOR ""
#define NAMESPACE_DRIVER ""
#define BASE_LINK "base_link"
#define IMU_FRAME "imu"
#define ODOM_FRAME "odom"
#define IMU_TOPIC "imu"
#define ODOM_TOPIC "odom"
#define SONAR_TOPIC "ultrasonic/range"
#define CMD_VEL_TOPIC "/cmd_vel"
#define DOMAIN_ID (0)
#define CONTEXT_CNT (3)

// Misc. Constants
#define MAX_WHEEL_VEL_ANG   (47.5)                      // Maximum angular wheel velocity in rad/s
#define MIN_WHEEL_VEL_ANG   (0.625)                     // Maximum angular wheel velocity in rad/s
#define ENCODER_CPR         (12.)                       // Pulses per revoultion of motor shaft
#define GEAR_RATIO          (75.)                       // Gear ratio x:1
#define WHEEL_CPR           (900.)                      // Pulses per revoultion of wheel
#define WHEEL_RADIUS        (0.016)                     // Radius of a wheel
#define WHEEL_TRACK         (0.0625)                    // Distance from center of the robot to one wheel
#define SONAR_FOV           (0.2618)                    // Measurement angle of the HC-SR04 in radians (15°)
#define SONAR_MIN_RANGE     (0.002)                     // Shortes measureable distance of the HC-SR04
#define SONAR_MAX_RANGE     (4.000)                     // Farthest measureable distance of the HC-SR04

// Timer config
#define TIMER_SAMPLE_INTERVAL   (50)                // Publisher and sample timer in ms

// Driver
#define FEEDFORWARD_KONST (1.)                     // Tuning factor for feedforward control
PIDController mot_l_vel_ctrl = {(double)TIMER_SAMPLE_INTERVAL*1e-3, {.25, 7., .01, 50.}, {0,0,0,0,0}, {0,0}, {0,0}};

PIDController mot_r_vel_ctrl = {(double)TIMER_SAMPLE_INTERVAL*1e-3, {.25, 7., .01, 50.}, {0,0,0,0,0}, {0,0}, {0,0}};

DCMotor mot_L = {0., 0., 0., MAX_WHEEL_VEL_ANG, MIN_WHEEL_VEL_ANG, TIMER_SAMPLE_INTERVAL*1e-3, 
                    ENCODER_CPR, GEAR_RATIO, &mot_l_vel_ctrl};
DCMotor mot_R = {0., 0., 0., MAX_WHEEL_VEL_ANG, MIN_WHEEL_VEL_ANG, TIMER_SAMPLE_INTERVAL*1e-3, 
                    ENCODER_CPR, GEAR_RATIO, &mot_r_vel_ctrl};

DiffDrive driver = {{0,0}, {0,0}, {0,0}, WHEEL_RADIUS, WHEEL_TRACK, 0, {0, 0}, 0, {0, 0},
                        {1e-9, 1e-9, 1e-9, 1e-9, 1e-9, 1e-9, 1e-9, 1e-9, 1e-9,
                                1e-9, 1e-9, 1e-9, 1e-9, 1e-9, 1e-9, 1e-9, 1e-9, 1e-9,
                                1e-9, 1e-9, 1e-9, 1e-9, 1e-9, 1e-9, 1e-9, 1e-9, 1e-9,
                                1e-9, 1e-9, 1e-9, 1e-9, 1e-9, 1e-9, 1e-9, 1e-9, 1e-9},
                        {1e-9, 1e-9, 1e-9, 1e-9, 1e-9, 1e-9, 1e-9, 1e-9, 1e-9,
                                1e-9, 1e-9, 1e-9, 1e-9, 1e-9, 1e-9, 1e-9, 1e-9, 1e-9,
                                1e-9, 1e-9, 1e-9, 1e-9, 1e-9, 1e-9, 1e-9, 1e-9, 1e-9,
                                1e-9, 1e-9, 1e-9, 1e-9, 1e-9, 1e-9, 1e-9, 1e-9, 1e-9},
                        &mot_L, &mot_R};
// Additional Sensors
// IMU LSM6DS33
float imu_lin_acc[3];
float imu_ang_vel[3];
const double imu_lin_acc_cov[9] = {0, 0, 0, 0, 0, 0, 0, 0, 0};
const double imu_ang_vel_cov[9] = {0, 0, 0, 0, 0, 0, 0, 0, 0};


// ROS variables
rcl_node_t node_driver;
rcl_timer_t timer_sample;
//rcl_timer_t timer_control;
rcl_allocator_t allocator;
rclc_support_t support;
rclc_executor_t executor;

rcl_publisher_t odom_pub;
rcl_publisher_t imu_pub;
rcl_publisher_t sonar_pub;
rcl_subscription_t cmd_vel_sub;

geometry_msgs__msg__Twist msg_cmd_vel;
nav_msgs__msg__Odometry msg_odom;
sensor_msgs__msg__Imu msg_imu;
sensor_msgs__msg__Range msg_sonar_range;

// System health
bool cmd_vel_watchdog = false;
uint cmd_vel_timeouts = 0;
uint max_cmd_vel_timeouts = 2;

// Flags
bool sample_range_sensors_even = false;
volatile bool timer_update_sample = false;
volatile bool timer_update_ctrl = false;

// Methods
void timer_callback(rcl_timer_t *timer, int64_t last_call_time);
bool timer_ctrl_callback(repeating_timer_t *rt);
void cmd_vel_callback(const void *msgin);
void update_odometry();
void update_wheel_control();
void update_imu();
void update_sonar();
void setup_publishers();
void setup_subscribers();
void setup_timers();
void setup_msg();
void fill_msg_stamp(builtin_interfaces__msg__Time *stamp);
void shutdown_node();


// Callback of the sample timer -used for updating the odometry data
void timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{  
    update_odometry();
    update_wheel_control();
    update_imu();
    timer_update_sample = true;
}

// Callback for control timer on core 1
bool timer_ctrl_callback(repeating_timer_t *rt)
{
    timer_update_ctrl = true;
}

// Updates and publishes odometry data
void update_odometry()
{

    // Generate message
    fill_msg_stamp(&msg_odom.header.stamp);

    mutex_enter_blocking(&mtx_odom_data);
    msg_odom.pose.pose.position.x = driver.odom_pos[0];
    msg_odom.pose.pose.position.y = driver.odom_pos[1];

    msg_odom.pose.pose.orientation.z = sin(driver.odom_rot / 2);
    msg_odom.pose.pose.orientation.w = cos(driver.odom_rot / 2);


    msg_odom.twist.twist.linear.x = driver.odom_vel[0];
    msg_odom.twist.twist.angular.z = driver.odom_vel[1];
    mutex_exit(&mtx_odom_data);

    // Publish odometry data
    RCSOFTCHECK(rcl_publish(&odom_pub, &msg_odom, NULL));
}

// Update velocity control and actuate motors
void update_wheel_control()
{  

    double cmd_vel[2] = {msg_cmd_vel.linear.x, msg_cmd_vel.angular.z};
 
    // Ensure a connection to the main device is given via cmd_vel
    if(!cmd_vel_watchdog)
    {
        cmd_vel_timeouts++;

        if(cmd_vel_timeouts >= max_cmd_vel_timeouts)
        {
            cmd_vel[0] = 0; 
            cmd_vel[1] = 0; 
        } 
    }
    else
    {
        cmd_vel_timeouts = 0;
    }

    // Update velocity command
    mutex_enter_blocking(&mtx_cmd_vel);
    set_cmd_vel_body(&driver, &cmd_vel);
    mutex_exit(&mtx_cmd_vel);

    cmd_vel_watchdog = false;
    
    // control_velocity(&driver);           // Cascaded velocity control
    //set_motor_pwm((driver.motor_left->velocity_controller->outputs[0] + driver.vel_tar_wheels[0]/driver.motor_left->max_speed)*MAX_MOT_PWM,
    //            (driver.motor_right->velocity_controller->outputs[0] + driver.vel_tar_wheels[1]/driver.motor_right->max_speed)*MAX_MOT_PWM);
    
}

// Updates and publishes imu data
void update_imu()
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
        msg_sonar_range.header.frame_id.data = "ultrasonic_2_link";
        msg_sonar_range.range = range_results[1];
        RCSOFTCHECK(rcl_publish(&sonar_pub, &msg_sonar_range, NULL));

        msg_sonar_range.header.frame_id.data = "ultrasonic_4_link";
        msg_sonar_range.range = range_results[3];
        RCSOFTCHECK(rcl_publish(&sonar_pub, &msg_sonar_range, NULL));

        msg_sonar_range.header.frame_id.data = "ultrasonic_6_link";
        msg_sonar_range.range = range_results[5];
        RCSOFTCHECK(rcl_publish(&sonar_pub, &msg_sonar_range, NULL));

        /*
        msg_sonar_range.header.frame_id.data = "ultrasonic_8_link";
        msg_sonar_range.range = range_results[7];
        RCSOFTCHECK(rcl_publish(&sonar_pub, &msg_sonar_range, NULL));
        */

        range_sensor_int_sample(PIN_TRIG_1);
    }
    else
    {
        msg_sonar_range.header.frame_id.data = "ultrasonic_1_link";
        msg_sonar_range.range = range_results[0];
        RCSOFTCHECK(rcl_publish(&sonar_pub, &msg_sonar_range, NULL));

        msg_sonar_range.header.frame_id.data = "ultrasonic_3_link";
        msg_sonar_range.range = range_results[2];
        RCSOFTCHECK(rcl_publish(&sonar_pub, &msg_sonar_range, NULL));

        /*
        msg_sonar_range.header.frame_id.data = "ultrasonic_5_link";
        msg_sonar_range.range = range_results[4];
        RCSOFTCHECK(rcl_publish(&sonar_pub, &msg_sonar_range, NULL));
        */
        
        msg_sonar_range.header.frame_id.data = "ultrasonic_7_link";
        msg_sonar_range.range = range_results[6];
        RCSOFTCHECK(rcl_publish(&sonar_pub, &msg_sonar_range, NULL));
        
        range_sensor_int_sample(PIN_TRIG_2);
    }
    
    sample_range_sensors_even = !sample_range_sensors_even;
}

// Callback of the cmd_vel subscriber used to update the target velocities
void cmd_vel_callback(const void *msgin)
{
    cmd_vel_watchdog = true;
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
    memcpy(msg_odom.pose.covariance, driver.odom_pos_cov, sizeof(driver.odom_pos_cov));
    memcpy(msg_odom.twist.covariance, driver.odom_vel_cov, sizeof(driver.odom_vel_cov));

    // Fill frames of imu msg
    msg_odom.header.frame_id.data = IMU_FRAME;
    msg_odom.child_frame_id.data = BASE_LINK;
    msg_imu.orientation.x = -1; // Set to -1 to be interpreted as unknown
    memcpy(msg_imu.angular_velocity_covariance, imu_ang_vel_cov, sizeof(imu_ang_vel_cov));
    memcpy(msg_imu.linear_acceleration_covariance, imu_lin_acc_cov, sizeof(imu_lin_acc_cov));

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

// Shutsdown the node
void shutdown_node()
{
    RCCHECK(rcl_subscription_fini(&cmd_vel_sub, &node_driver));
	RCCHECK(rcl_publisher_fini(&odom_pub, &node_driver));
    RCCHECK(rcl_publisher_fini(&imu_pub, &node_driver));
	RCCHECK(rcl_node_fini(&node_driver));
}

int main()
{
    // Setup transport system for RPi Pico
    rmw_uros_set_custom_transport(
		true,
		NULL,
		pico_serial_transport_open,
		pico_serial_transport_close,
		pico_serial_transport_write,
		pico_serial_transport_read
	);

    // Initilize Pico GPIO etc.
    pico_setup();

    // Initialize controller parameters
    PID_update_parameters(driver.motor_left->velocity_controller);
    PID_update_parameters(driver.motor_right->velocity_controller);

    allocator = rcl_get_default_allocator();

    // Wait for agent successful ping for 2 minutes.
    const int timeout_ms = 1000; 
    const uint8_t attempts = 120;
    RCCHECK(rmw_uros_ping_agent(timeout_ms, attempts));

    // Init support
    rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
    RCCHECK(rcl_init_options_init(&init_options, allocator));
    RCCHECK(rcl_init_options_set_domain_id(&init_options, DOMAIN_ID));
    RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));

    // Node
    RCCHECK(rclc_node_init_default(&node_driver, "driver_node", NAMESPACE_DRIVER, &support));

    // Setup node functionalities
    setup_publishers();
    setup_subscribers();
    setup_timers();
    setup_msg();

    // Executor
    RCCHECK(rclc_executor_init(&executor, &support.context, CONTEXT_CNT, &allocator));
    RCCHECK(rclc_executor_add_timer(&executor, &timer_sample));
    RCCHECK(rclc_executor_add_subscription(&executor, &cmd_vel_sub, &msg_cmd_vel, &cmd_vel_callback, ON_NEW_DATA));

    // Loop
    gpio_put(PIN_MOT_ON, 1);

    rcl_ret_t ret = RCL_RET_OK; 
    while (ret == RCL_RET_OK)
    {        
        if(timer_update_sample)
        {
            update_sonar();
            timer_update_sample = false;
        }
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1));
    }
        
    //rclc_executor_spin(&executor);
    gpio_put(PIN_MOT_ON, 0);

    shutdown_node();

    return 0;
}

void core1_main()
{
    repeating_timer_t ctrl_timer;
    add_repeating_timer_ms(TIMER_SAMPLE_INTERVAL, timer_ctrl_callback, NULL, &ctrl_timer);

    while(1)
    {   
        if(timer_update_ctrl)
        {
            // Read data from PIO and update measurement data
            mutex_enter_blocking(&mtx_odom_data);
            // update_odometry_data(&driver, quadrature_encoder_get_count(pio_enc_L, PIO_ENC_L_SM), quadrature_encoder_get_count(pio_enc_R, PIO_ENC_R_SM));
            update_odometry_data_approximate(&driver, quadrature_encoder_get_count(pio_enc_L, PIO_ENC_L_SM), -quadrature_encoder_get_count(pio_enc_R, PIO_ENC_R_SM));
            mutex_exit(&mtx_odom_data);

            // Cascaded velocity control
            mutex_enter_blocking(&mtx_cmd_vel);
            control_velocity(&driver);      
            set_motor_pwm((driver.motor_left->velocity_controller->outputs[0] + driver.vel_tar_wheels[0]/driver.motor_left->max_speed)*MAX_MOT_PWM,
                (driver.motor_right->velocity_controller->outputs[0] + driver.vel_tar_wheels[1]/driver.motor_right->max_speed)*MAX_MOT_PWM);  
            mutex_exit(&mtx_cmd_vel);   

            timer_update_ctrl = false;
        }
    }

    cancel_repeating_timer(&ctrl_timer);
}