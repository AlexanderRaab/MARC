close all; 
clear;

%% Load measurement data
data_path = "..\data\platform\motor\";
file_names = ["prbs_Ts10_T50.txt", "prbs_Ts10_T100.txt", "prbs_Ts10_T500.txt", "prbs_Ts10_T1000.txt";
                "prbs_Ts25_T50.txt", "prbs_Ts25_T100.txt", "prbs_Ts25_T500.txt", "prbs_Ts25_T1000.txt";
                "prbs_Ts50_T50.txt", "prbs_Ts50_T100.txt", "prbs_Ts50_T500.txt", "prbs_Ts50_T1000.txt";]

Ts_idx = 4;
outlier_threshold = 50;

K_ident = [];
T_ident = [];
for i = 1:3
    data = readtable(data_path + file_names(i,Ts_idx));
    % Preprocess
    % Remove outliers and first follwing sample
    outs = isoutlier(data.wl, "mean") + isoutlier(data.wr, "mean");
    outs = outs + [0; outs(1:end-1)];
    outs = outs > 0;
    data(outs,:) = [];
    
    idx_s = 0;
    idx_e = 0;
    idx = 1;
    while(idx < length(data.t))
        idx_s = find(data.u(idx:end) > 0, 1,'first') - 1 + idx;
        idx_e = find(data.u(idx_s + 1:end) == 0, 1,'first') + idx_s;

        if (isempty(idx_s) || isempty(idx_e))
            break;
        end

        slice = idx_s:idx_e;

        [K1, T1, MSE] = IdentLag1(data.t(slice), data.u(slice), data.wl(slice), 0.8);
        [K2, T2, MSE] = IdentLag1(data.t(slice), data.u(slice), data.wl(slice), 0.8);
        K_ident = [K_ident K1, K2];
        T_ident = [T_ident T1, T2];
        idx = idx_e + 1;
    end
end

%% Derive parameters from mean

K = mean(K_ident)
T = mean(T_ident)

TF_MOT = tf(K,[T,1])

%% Plot results
data = readtable(data_path + file_names(2,4));
V = 6.21;
range = 473:605;
figure;
hold on;
plot(data.t(range)-data.t(range(1)), data.wl(range)/V);
plot(data.t(range)-data.t(range(1)), data.wr(range)/V);
step(TF_MOT/2)

%%
data = readtable(data_path + file_names(2, 4));
u = data.u;
y = data.wl;

[A, B] = IdentARX(u, y, 2, 0, 1);
TF_MOT = tf(B,A, 0.025, 'Variable', 'q^-1');

data = readtable(data_path + file_names(2,4));
V = 6.21/2;
range = 473:605;
figure;
hold on;
plot(data.t(range)-data.t(range(1)), data.wl(range)/V);
plot(data.t(range)-data.t(range(1)), data.wr(range)/V);
step(TF_MOT)