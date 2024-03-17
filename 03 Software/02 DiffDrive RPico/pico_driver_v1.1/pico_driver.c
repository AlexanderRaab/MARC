/* microROS Differantial Driver Robot
    Author:     Alexander Raab
    E-Mail:     alex.raab@liwest.at
    Date:       17.06.2023
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

#include <rmw_microros/rmw_microros.h>

#include "driver_config.h"
#include "diff_drive_utils.h"
#include "pico_interface.h"
#include "ros_interface.h"

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
        // Perform non-critical tasks 
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

// Main loop on core 1 - used for velocity control
void core1_main()
{
    repeating_timer_t ctrl_timer;
    add_repeating_timer_ms(TIMER_CONTROL_INTERVAL, timer_ctrl_callback, NULL, &ctrl_timer);

    while(1)
    {   
        if(timer_update_ctrl)
        {

            // Store current velcoity command for use on core 1
            mutex_enter_blocking(&mtx_cmd_vel);
            set_target_vel_body();
            mutex_exit(&mtx_cmd_vel); 

            // Read data from PIO and update measurement data
            mutex_enter_blocking(&mtx_odom_data);
            update_odometry_data(quadrature_encoder_get_count(pio_enc_L, PIO_ENC_L_SM), -quadrature_encoder_get_count(pio_enc_R, PIO_ENC_R_SM));
            mutex_exit(&mtx_odom_data);

            // Apply velocity control
            control_velocity();      
            set_motor_pwm(pwm_out_mot[0], pwm_out_mot[1]);  
             
            timer_update_ctrl = false;
        }
    }

    cancel_repeating_timer(&ctrl_timer);
}