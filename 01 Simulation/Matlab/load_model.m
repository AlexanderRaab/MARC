clear; close all; clc;

% Simulation
Ts = 1e-3;

% Robot dimensions
wheel_track = 0.125/2;
wheel_radius = 0.01625;
max_turning_radius = 100;

% Motor data
motor_tf = load('..\data\platform\TF_motor.mat').TF_MOT;
motor_vel_max = 47.5;
motor_Ts = 50e-3;
motor_vcc = 5.93;
motor_gear = 1/75;

% Encoder data
encoder_cpr = 12;

% Velocity controller motor
motor_FF_factor = 1.0;
PID_mot_vel = [.02, .0000001,.000001 100] % [0.25, 7, 0.01, 50] % P, I, D, N
PID_mot_vel_coeff = get_PID_coeff(PID_mot_vel, motor_Ts);

%% Helper Functions 
function coeffs = get_PID_coeff(params, Ts)
    % Generates the coefficients for the dicrete PID implementation
    % using the parameters P, I, D, N and the sample time Ts

    a0 = 1 + params(4)*Ts;
    a1 = -(1 + a0);
    b0 = params(1)*a0 + params(2)*Ts*a0 + params(3)*params(4);
    b1 = -(params(1)*(1+a0) + params(2)*Ts + 2*params(3)*params(4));
    b2 = params(1) + params(3)*params(4);
    coeffs = [a1/a0, 1/a0, b0/a0, b1/a0, b2/a0]
end