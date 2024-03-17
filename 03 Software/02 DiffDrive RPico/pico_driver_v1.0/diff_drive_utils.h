
/*  Utilities for a Differantial Driver Robot
    Author:     Alexander Raab
    E-Mail:     alex.raab@liwest.at
    Date:       12.18.2022
    Version:    1.1
    Licence:    TODO

    This header provides functionalities used for interfacing an DiffDrive Robot equipped with DC motors.

    TODO: optimize for higher sample rates
*/

#pragma once
#define _USE_MATH_DEFINES
#include <math.h>

// ------------- PID Controller

// Data structure of the PID controller
struct pid_controller
{
    double Ts;                                  // Sample time
    double parameters[4];                       // Parameters P, I, D and N         
    double coeffs[5];                           // Computed coefficients k_u1, k_u2, k_e0, k_e1, k_e2
    double errors[2];                           // Past error inputs
    double outputs[2];                          // Past error outputs
} default_pid_controller = {1e-2, {1,0,0,0}, {0,0,0,0,0}, {0,0}, {0,0}};
typedef struct pid_controller PIDController;
/* Set parameters of the controllerto input values and update filter coefficients.

    @param ctrl pointer to the controller data structure
*/
void PID_update_parameters(PIDController *ctrl)
{
    double a0 = 1+ctrl->parameters[3]*ctrl->Ts;
    double a1 = -(2+ctrl->parameters[3]*ctrl->Ts);
    double b0 = ctrl->parameters[0]*(1+ctrl->parameters[3]*ctrl->Ts) + ctrl->parameters[1]*ctrl->Ts*(1+ctrl->parameters[3]*ctrl->Ts) + ctrl->parameters[2]*ctrl->parameters[3];
    double b1 = -(ctrl->parameters[0]*(2+ctrl->parameters[3]*ctrl->Ts) + ctrl->parameters[1]*ctrl->Ts + 2*ctrl->parameters[2]*ctrl->parameters[3]);
    double b2 = ctrl->parameters[0] + ctrl->parameters[2]*ctrl->parameters[3];

    ctrl->coeffs[0] = a1/a0;
    ctrl->coeffs[1] = 1/a0;
    ctrl->coeffs[2] = b0/a0;
    ctrl->coeffs[3] = b1/a0;
    ctrl->coeffs[4] = b2/a0;
}

 /* Updates the control loop.
    Must be called with set sample time!

    @param ctrl pointer to the controller data structure
    @param error difference between control target and current value
    @returns unscaled controller output between [-1, 1]
*/
double PID_update(PIDController *ctrl, double error)
{
    double output = -ctrl->coeffs[0]*ctrl->outputs[0] - ctrl->coeffs[1]*ctrl->outputs[1] + ctrl->coeffs[2]*error + ctrl->coeffs[3]*ctrl->errors[0] + ctrl->coeffs[4]*ctrl->errors[1];

    if(output > 1) output = 1;
    else if(output < -1) output = -1;

    ctrl->errors[1] = ctrl->errors[0];
    ctrl->errors[0] = error;
    ctrl->outputs[1] = ctrl->outputs[0];
    ctrl->outputs[0] = output;

    return output;
}

// ------------- DC Motor 

// Data structure of a DC Motor
struct dc_motor
{
    double speed;                           // Speed data for odometry and control samples
    double angle;                           // Angle data for odometry and control samples
    int enc_ticks;                          // Ticks of odometry and control samples

    double max_speed;                       // Maximum angular velocity
    double min_speed;                       // Maximum angular velocity
    double sample_time;                     // Sample rate of encoder in s
    int enc_cpr;                            // Encoder ticks per revolution
    double gear_ratio;                      // Gear ratio of motor

    PIDController *velocity_controller;     // Velocity controller
} default_dc_motor = {0, 0, 0, 0, 0, 0, 0, 0, NULL};
typedef struct dc_motor DCMotor;

/* Updates encoder data.
    Must be called regularily within set sample time!

    @param motor pointer to the motor data structure
    @param ticks absolute ticks measured by encoder
    @returns angular difference to the previous measurement in rad      
*/
double update_encoder(DCMotor *motor, int ticks)
{
    double delta_angle = 2 * M_PI * (double)(motor->enc_ticks - ticks) / (motor->enc_cpr * motor->gear_ratio);

    motor->angle += delta_angle;

    if(motor->angle > 2*M_PI) motor->angle -= 2*M_PI;
    else if (motor->angle < -2*M_PI) motor->angle += 2*M_PI;

    motor->speed = delta_angle / motor->sample_time;

    motor->enc_ticks = ticks;
    return delta_angle;
}

/*  Applies the velocity control loop.
    Must be called within the sample time set for the velocity controller.

    @param motor pointer to the motor data structure
    @param target_velocity target value for the controller in rad/s     
*/
void apply_velocity_control(DCMotor *motor, double target_velocity)
{
    if(target_velocity > motor->max_speed) target_velocity = motor->max_speed;
    else if(target_velocity < -motor->max_speed) target_velocity = -motor->max_speed;  
    else if(target_velocity < motor->min_speed && target_velocity > -motor->min_speed) target_velocity = 0;  
    PID_update(motor->velocity_controller, (target_velocity - motor->speed) / (2 * motor->max_speed));         
}

// ------------- Differential Drive 

// Data structure of a differential drive robot
struct diff_drive
{
    double vel_cmd_body[2];             // Velocity target from command for body [vx, wz]
    double vel_cmd_wheels[2];           // Velocity target from command for wheels [wl, wr]

    double vel_tar_wheels[2];           // Velocity target from body controller for wheels [wl, wr]

    double wheel_radius;                // Wheel radius in m
    double wheel_track;                 // Wheel track in m, distance of a wheel from the center of the robot
    double max_turning_radius;          // Maximum turning radius for driving in a straight line

    double odom_pos[2];                 // Global position from odometry data [x, y] in m
    double odom_rot;                    // Global rotation from odometry data z in rad
    double odom_vel[2];                 // Local velocity from odometry data [vx, wz] in m/s and rad/s           

    double odom_pos_cov[36];            // Position covariance of odometry data
    double odom_vel_cov[36];            // Velocity covariance of odometry data
    
    DCMotor *motor_left, *motor_right;

} default_diff_drive = {{0,0}, {0,0}, {0,0}, 0, 0, 0, {0, 0}, 0, {0, 0},
                        {1e-9, 1e-9, 1e-9, 1e-9, 1e-9, 1e-9, 1e-9, 1e-9, 1e-9,
                                1e-9, 1e-9, 1e-9, 1e-9, 1e-9, 1e-9, 1e-9, 1e-9, 1e-9,
                                1e-9, 1e-9, 1e-9, 1e-9, 1e-9, 1e-9, 1e-9, 1e-9, 1e-9,
                                1e-9, 1e-9, 1e-9, 1e-9, 1e-9, 1e-9, 1e-9, 1e-9, 1e-9},
                        {1e-9, 1e-9, 1e-9, 1e-9, 1e-9, 1e-9, 1e-9, 1e-9, 1e-9,
                                1e-9, 1e-9, 1e-9, 1e-9, 1e-9, 1e-9, 1e-9, 1e-9, 1e-9,
                                1e-9, 1e-9, 1e-9, 1e-9, 1e-9, 1e-9, 1e-9, 1e-9, 1e-9,
                                1e-9, 1e-9, 1e-9, 1e-9, 1e-9, 1e-9, 1e-9, 1e-9, 1e-9},
                        NULL, NULL};
typedef struct diff_drive DiffDrive;

/* Applies wheel transformation to transform data of the body to the wheels.
    Used for distance and velocity transformations.

    @param driver pointer to the differential drive data structure
    @param body body data [lin_x, rot_z]
    @param wheel wheel data [rot_l, rot_r]
*/
void convert_to_wheels(DiffDrive *driver, double* body, double* wheel)
{
    // rot_l = (lin_x - rot_z * b) / r
    wheel[0] = (body[0] - body[1] * driver->wheel_track) / driver->wheel_radius;
    // rot_r = (lin_x + lin_z * b) / r
    wheel[1] = (body[0] + body[1] * driver->wheel_track) / driver->wheel_radius;
}

/* Applies inverse wheel transformation to transform data of the wheels to the body
    Used for distance and velocity transformations.

    @param driver pointer to the differential drive data structure
    @param wheel wheel data [rot_l, rot_r]
    @param body body data [lin_x, rot_z]
*/
void convert_to_body(DiffDrive *driver, double* wheel, double* body)
{
    // lin_x = (rot_l + rot_r) * r / 2 
    body[0] = (wheel[0] + wheel[1]) * driver->wheel_radius * 0.5;
    // rot_z = (rot_r - rot_l) * r / (2 * b) 
    body[1] = (wheel[1] - wheel[0]) * driver->wheel_radius / (2 * driver->wheel_track);
}

/* Sets velocity command using body velocities.

    @param driver pointer to the differential drive data structure
    @param cmd_vel [vx, wz]
*/
void set_cmd_vel_body(DiffDrive *driver, double* cmd_vel)
{
    driver->vel_cmd_body[0] = cmd_vel[0];
    driver->vel_cmd_body[1] = cmd_vel[1];
    convert_to_wheels(driver, driver->vel_cmd_body, driver->vel_cmd_wheels);
}

/* Sets velocity command using wheel velocities.

    @param driver pointer to the differential drive data structure
    @param cmd_vel [wl, wr]
*/
void set_cmd_vel_wheels(DiffDrive *driver, double* cmd_vel)
{
    driver->vel_cmd_wheels[0] = cmd_vel[0];
    driver->vel_cmd_wheels[1] = cmd_vel[1];
    convert_to_body(driver, driver->vel_cmd_wheels, driver->vel_cmd_body);
}

/* Calls the velocity controller of body velocities and updates the target velocities of the wheels.
    @param driver pointer to the differential drive data structure
*/
void control_vel_body(DiffDrive *driver)
{
    driver->vel_tar_wheels[0] = driver->vel_cmd_wheels[0];
    driver->vel_tar_wheels[1] = driver->vel_cmd_wheels[1];
}

/* Calls the velocity controller for the wheels.

    @param driver pointer to the differential drive data structure
*/
void control_vel_wheels(DiffDrive *driver)
{
    apply_velocity_control(driver->motor_left, driver->vel_tar_wheels[0]);
    apply_velocity_control(driver->motor_right, driver->vel_tar_wheels[1]);
}

/* Calls the cascaded velocity controller.
    TODO: No outer controller implemented.

    @param driver pointer to the differential drive data structure
*/
void control_velocity(DiffDrive *driver)
{
    control_vel_body(driver);              // Outer velocity control
    control_vel_wheels(driver);            // Inner velocity control
}

/* Updates motor encoders and odometry data.
    TODO: Improve handling of low velocities, currently done by |R_IRC| < 1e-3

    @param driver pointer to the differential drive data structure    
    @param ticks_left Absolute tick value of left encoder
    @param ticks_right Absolute tick value of right encoder
*/
void update_odometry_data(DiffDrive *driver, int ticks_left, int ticks_right)
{
    double rot = (update_encoder(driver->motor_right, ticks_right) - update_encoder(driver->motor_left, ticks_left)) * driver->wheel_radius / (2 * driver->wheel_track);
                
    double wheel_vel[2] = {driver->motor_left->speed, driver->motor_right->speed};
    convert_to_body(driver, wheel_vel, driver->odom_vel);

    double R_IRC;

    if (driver->odom_vel[1] == 0)
        R_IRC = driver->max_turning_radius;             
    else
        R_IRC = driver->odom_vel[0] / driver->odom_vel[1];

    double pos_x = 0, pos_y = 0;

    if(R_IRC >= driver->max_turning_radius || R_IRC <= -driver->max_turning_radius)
    {
        pos_x = driver->odom_vel[0] * driver->motor_left->sample_time;
    }
    else if(R_IRC > 1e-3 || R_IRC < -1e-3)
    {
        pos_x = R_IRC*sin(rot);
        pos_y = R_IRC*(1-cos(rot));
    }
    
    double cosine = cos(driver->odom_rot);
    double sine = sin(driver->odom_rot);

    driver->odom_pos[0] += pos_x * cosine - pos_y * sine;
    driver->odom_pos[1] += pos_x * sine + pos_y * cosine;   

    driver->odom_rot += rot; 
}

/*  Updates motor encoders and odometry data using an approximation for the positional data.
    Valid for high sample rates and/or low velocities.

    @param driver pointer to the differential drive data structure    
    @param ticks_left Absolute tick value of left encoder
    @param ticks_right Absolute tick value of right encoder
*/
void update_odometry_data_approximate(DiffDrive *driver, int ticks_left, int ticks_right)
{
    double rot = (update_encoder(driver->motor_right, ticks_right) - update_encoder(driver->motor_left, ticks_left)) * driver->wheel_radius / (2 * driver->wheel_track);
                
    double wheel_vel[2] = {driver->motor_left->speed, driver->motor_right->speed};
    convert_to_body(driver, wheel_vel, driver->odom_vel);

    double dist = driver->odom_vel[0] * driver->motor_left->sample_time;

    driver->odom_pos[0] += dist * cos(driver->odom_rot + rot/2);
    driver->odom_pos[1] += dist * sin(driver->odom_rot + rot/2);  

    driver->odom_rot += rot; 
}