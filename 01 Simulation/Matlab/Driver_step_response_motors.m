close all; 
clear;

%% Load measurement data
step_50 = readtable('..\data\platform_ident\motor_step_data_50');
step_30 = readtable('..\data\platform_ident\motor_step_data_30');
step_25 = readtable('..\data\platform_ident\motor_step_data_25');
step_10 = readtable('..\data\platform_ident\motor_step_data_10');

% Normalize data
step_50  = step_50(step_50.u > 0,:);
step_30  = step_30(step_30.u > 0,:);
step_25  = step_25(step_25.u > 0,:);
step_10  = step_10(step_10.u > 0,:);

step_50.t  = step_50.t - step_50.t(1);
step_30.t  = step_30.t - step_30.t(1);
step_25.t  = step_25.t- step_25.t(1);
step_10.t  = step_10.t- step_10.t(1);

N = min([length(step_50.u),length(step_30.u),length(step_25.u),length(step_10.u)]);

step_50  = step_50(1:N,:);
step_30  = step_30(1:N,:);
step_25  = step_25(1:N,:);
step_10  = step_10(1:N,:);

%% Plot step response
figure;
subplot(2,2,1);
plot(step_10.t, [step_10.u, step_10.wl, step_10.wr]);
subplot(2,2,2);
plot(step_25.t, [step_25.u, step_25.wl, step_25.wr]);
subplot(2,2,3);
plot(step_30.t, [step_30.u, step_30.wl, step_30.wr]);
subplot(2,2,4);
plot(step_50.t, [step_50.u, step_50.wl, step_50.wr]);

% Compare motors
figure;
subplot(2,1,1);
plot(step_10.t, [step_10.wl, step_25.wl, step_30.wl, step_50.wl]);
subplot(2,1,2);
plot(step_50.t, [step_10.wr, step_25.wr, step_30.wr, step_50.wr]);

%% Selected Ts = 50ms 
data = step_50;
w_max = [mean(data.wl(15:N)), mean(data.wr(15:N))];
u_norm = 8;

[K_l, T_l, mse] = IdentLag1(data.t, data.u, data.wl);
[K_r, T_r, mse] = IdentLag1(data.t, data.u, data.wr);
K = (K_l + K_r)/2
T = (T_l + T_r)/2
TF_MOT = tf(K,[T,1])

%% Plot caluclated step responses
figure;
step(TF_MOT)


