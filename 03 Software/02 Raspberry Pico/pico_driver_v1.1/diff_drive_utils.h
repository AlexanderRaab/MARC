/*  Utilities for a Differantial Driver Robot
    Author:     Alexander Raab
    E-Mail:     alex.raab@liwest.at
    Date:       17.06.2023
    Version:    1.1
    Licence:    TODO

    This header provides functionalities used for interfacing an DiffDrive Robot equipped with DC motors.

    TODO: optimize for higher sample rates
*/

#pragma once
#define _USE_MATH_DEFINES

#include <math.h>
#include "driver_config.h"

// Velocities
double  target_vel_body[2] = {0};           // Target velocities, copied from cmd_vel_body for multi core use

// Singel axis PID velocity controller
double PID_coeffs_mot[5] = {0};             // Computed coefficients k_u1, k_u2, k_e0, k_e1, k_e2
double PID_vals_mot_l[4] = {0};             // Past error inputs and controller outputs left motor [ek-1, ek-2, ok-1, ok-2]
double PID_vals_mot_r[4] = {0};             // Past error inputs and controller outputs right motor [ek-1, ek-2, ok-1, ok-2]


// --- PID Controller
/* Set parameters of the controllerto input values and calculates the filter coefficients.

    @param Kp propotional gain
    @param Ki integral gain
    @param Kd derivative gain
    @param N filter constant
    @param Ts sample time
    @param coeffs destination for the coefficients [k_u1, k_u2, k_e0, k_e1, k_e2]
*/
void init_PID_parameters(double Kp, double Ki, double, Kd, double N, double Ts, double *coeffs)
{
    double a0 = 1+N*Ts;
    double a1 = -(2+N*Ts);
    double b0 = Kp*(1+N*Ts) + Ki*Ts*(1+N*Ts) + Kd*N;
    double b1 = -(Kp*(2+N*Ts) + Ki*Ts + 2*Kd*N);
    double b2 = Kp + Kd*N;

    coeffs[0] = a1/a0;
    coeffs[1] = 1/a0;
    coeffs[2] = b0/a0;
    coeffs[3] = b1/a0;
    coeffs[4] = b2/a0;
}

 /* Updates the PID control loop.
    Must be called with set sample time!

    @param coeffs array of PID coefficient [k_u1, k_u2, k_e0, k_e1, k_e2]
    @param past_vals array of past errors and output values of the controller [ek-1, ek-2, ok, ok-1]
    @param error difference between control target and current value
    @returns unscaled controller output between [-1, 1]
*/
double update_PID(double *coeffs, double *past_vals, double error)
{
    double output = -coeffs[0]*past_vals[2] - coeffs[1]*past_vals[3] + coeffs[2]*error + coeffs[3]*past_vals[0] + coeffs[4]*past_vals[1];

    if(output > 1) output = 1;
    else if(output < -1) output = -1;

    past_vals[1] = past_vals[0];
    past_vals[0] = error;
    past_vals[3] = past_vals[2];
    past_vals[2] = output;

    return output;
}


/* Applies wheel transformation to transform data of the body to the wheels.
    Used for distance and velocity transformations.

    @param body body data [lin_x, rot_z]
    @param wheel wheel data [rot_l, rot_r]
*/
void convert_to_wheels(double* body, double* wheel)
{
    // rot_l = (lin_x - rot_z * b) / r
    wheel[0] = (body[0] - body[1] * WHEEL_TRACK) / WHEEL_RADIUS;
    // rot_r = (lin_x + lin_z * b) / r
    wheel[1] = (body[0] + body[1] * WHEEL_TRACK) / WHEEL_RADIUS;
}

/* Applies inverse wheel transformation to transform data of the wheels to the body
    Used for distance and velocity transformations.

    @param wheel wheel data [rot_l, rot_r]
    @param body body data [lin_x, rot_z]
*/
void convert_to_body(double* wheel, double* body)
{
    // lin_x = (rot_l + rot_r) * r / 2 
    body[0] = (wheel[0] + wheel[1]) * WHEEL_RADIUS * 0.5;
    // rot_z = (rot_r - rot_l) * r / (2 * b) 
    body[1] = (wheel[1] - wheel[0]) * WHEEL_RADIUS / (2 * WHEEL_TRACK);
}

/* Stores the velocity command cmd_vel_body to a local variable.
     
*/
void set_target_vel_body()
{
    target_vel_body[0] = cmd_vel_body[0];
    target_vel_body[1] = cmd_vel_body[1];
}


/* Calls the velocity controller for the whole plattform.

*/
void control_velocity()
{
    double target_vel_wheels[2] = {0};
    convert_to_wheels(target_vel_body, target_vel_wheels);
    pwm_out_mot[0] = update_PID(PID_coeffs_mot, PID_vals_mot_l,  target_vel_wheels[0] - target_vel_body[0]) * MAX_MOT_PWM + MOT_CTR_FF*target_vel_wheels[0];
    pwm_out_mot[1] = update_PID(PID_coeffs_mot, PID_vals_mot_r,  target_vel_wheels[1] - target_vel_body[1]) * MAX_MOT_PWM + MOT_CTR_FF*target_vel_wheels[1];
}

/*  Updates motor encoders and odometry data.
    Valid for high sample rates and/or low velocities.
  
    @param ticks_left Absolute tick value of left encoder
    @param ticks_right Absolute tick value of right encoder
*/
void update_odometry_data(int ticks_left, int ticks_right)
{
    // Update encoder data
    delta_angle[2] = {0};
    delta_angle[0] = 2 * M_PI * (double)(enc_ticks[0] - ticks_left) / WHEEL_CPR;
    delta_angle[1] = 2 * M_PI * (double)(enc_ticks[1] - ticks_right) / WHEEL_CPR;

    // Get motor angel - unused
    /*
    motor_angle[0] += delta_angle[0];
    if(motor_angle[0] > 2*M_PI) motor_angle[0] -= 2*M_PI;
    else if (motor_angle[0] < -2*M_PI) motor_angle[0] += 2*M_PI;

    motor_angle[1] += delta_angle[1];
    if(motor_angle[1] > 2*M_PI) motor_angle[1] -= 2*M_PI;
    else if (motor_angle[1] < -2*M_PI) motor_angle[1] += 2*M_PI;
    
    */
 
    // Store encoder ticks
    enc_ticks[0] = ticks_left;
    enc_ticks[1] = ticks_right;

    // Store body velocity
    motor_speed[0] = 1000 * delta_angle[0] / TIMER_CONTROL_INTERVAL;
    motor_speed[1] = 1000 * delta_angle[1] / TIMER_CONTROL_INTERVAL;
    convert_to_body(motor_speed, odom_vel);

    // Convert delta angles to delta position and rotation
    double delta_pos[2] = {0};
    convert_to_body(delta_angle, delta_pos);
    odom_pos[0] += delta_pos[0] * cos(odom_pos[2] + delta_pos[1]/2);
    odom_pos[1] += delta_pos[0] * sin(odom_pos[2] + delta_pos[1]/2);  
    odom_pos[2] += delta_pos[1]; 
}