clc
clear
close all

%% input 1 (from moodle simulink)
% vars = load("sim_out_vars.mat");
% 
% time = vars.out_struct.time_vector;
% pos_noise = vars.out_struct.pos_with_noise;
% %clean_data = [vars.out_struct.pos, vars.out_struct.vel, vars.out_struct.acc]; % clean data from simulink
% vel = vars.out_struct.vel;
% acc = vars.out_struct.acc;
% 
% Ts = time(10, 1) - time(9, 1);

%% input 2
% data = load('master_slave_1kHz.txt');
% pos_noise = data(400:end, 2); % second column is the master position
% vel = data(400:end, 3); % third column is the master velocity
% acc = zeros(size(vel));
% Ts = 0.001;

%% input 3 (from tasnk based simulink)
vars = load("noisy_master_position.mat");
time_vector = vars.out.xm.Time;
pos_noise = vars.out.xm.Data;
vel = vars.out.dot_xm.Data;
acc = vars.out.dot_dot_xm.Data;
Ts = 0.001;

%% euler derivatives
pos_dot = diff(pos_noise) / Ts; % position first derivative
pos_dot = lowpass(pos_dot, 5, 1/Ts);
pos_dot_dot = diff(pos_dot) / Ts; % position second derivative
pos_dot_dot = lowpass(pos_dot_dot, 5, 1/Ts);
derivative_data = [pos_noise, [0; pos_dot], [0; 0; pos_dot_dot]];

%% plot data
% figure();
% hold on;
% plot(time, pos_noise);
% plot(time, pos);
% plot(time, vel);
% plot(time, acc);
% legend("pos_noise", "Position", "vel", "acc")

%% kalman filter/predictor
% estimate position, velocity and acceleration starting from noisy data

% dynamic system discrete time matrices
A = [1 Ts (Ts^2)/2; 0 1 Ts; 0 0 1];
B = [Ts^3/6; Ts^2/2; Ts];
C = [1 0 0];

% initial conditions
p_0 = 0.1*eye(3);
x_0 = zeros(3, 1);
R = 0.1;
q_low_case=100000;
Q = q_low_case * (B * B'); % mostrare cosa cambia modificando questo parametro (0, 1, 100)

filter_data = kalman_filter(x_0, p_0, Q, R, A, C, pos_noise);
filter_ss_data = kalman_filter_ss(x_0, p_0, Q, R, A, C, pos_noise);
predictor_data = kalman_predictor(x_0, p_0, Q, R, A, C, pos_noise);
predictor_ss_data = kalman_predictor_ss(x_0, p_0, Q, R, A, C, pos_noise);
smoother_data = kalman_smoother(x_0, p_0, Q, R, A, C, pos_noise);
smoother_data = smoother_data';

rmse_vel_euler = rmse(derivative_data(:, 2), vel);
rmse_vel_filter = rmse(filter_data(:, 2), vel);
rmse_vel_filter_ss = rmse(filter_ss_data(:, 2), vel);
rmse_vel_predictor = rmse(predictor_data(:, 2), vel);
rmse_vel_predictor_ss = rmse(predictor_ss_data(:, 2), vel);
rmse_vel_smoother = rmse(smoother_data(:, 2), vel);

rmse_acc_euler = rmse(derivative_data(:, 3), acc);
rmse_acc_filter = rmse(filter_data(:, 3), acc);
rmse_acc_filter_ss = rmse(filter_ss_data(:, 3), acc);
rmse_acc_predictor = rmse(predictor_data(:, 3), acc);
rmse_acc_predictor_ss = rmse(predictor_ss_data(:, 3), acc);
rmse_acc_smoother = rmse(smoother_data(:, 3), acc);
%%
% figure;
% subplot(2,1,1);
% hold on;
% plot(time_vector, pos_noise);
% legend('simulink noisy position');
% xlabel("seconds [s]");
% ylabel("position");
% 
% subplot(2,1,2);
% hold on;
% plot(time_vector, vel);
% plot(time_vector, filter_data(:, 2));
% plot(time_vector, predictor_data(:, 2));
% plot(time_vector, smoother_data(:, 2));
% legend('simulink velocity', strcat("estimated vel kalman filter ", num2str(rmse_vel_filter)), strcat("estimated vel kalman predictor ", num2str(rmse_vel_predictor)), strcat("estimated vel kalman smoother ", num2str(rmse_vel_smoother)));
% xlabel("seconds [s]");
% ylabel("velocity");
% 
% set(gcf, 'Position', get(0, 'Screensize'));
% export_fig(strcat('C:\Users\emanuele\Desktop\phri_report_images\', datestr(now,'dd_mm_yyyy_HH_MM_SS_FFF')), '-pdf');
% close;

%% plot all
figure;
subplot(2,1,1);
hold on;
plot(vel);
plot(derivative_data(:, 2));
plot(filter_data(:, 2));
plot(predictor_data(:, 2));
plot(smoother_data(:, 2));
legend('simulink velocity', strcat("euler position derivative ", num2str(rmse_vel_euler)), strcat("estimated vel kalman filter ", num2str(rmse_vel_filter)), strcat("estimated vel kalman predictor ", num2str(rmse_vel_predictor)), strcat("estimated vel kalman smoother ", num2str(rmse_vel_smoother)))

subplot(2,1,2);
hold on;
plot(acc);
plot(derivative_data(:, 3));
plot(filter_data(:, 3));
plot(predictor_data(:, 3));
plot(smoother_data(:, 3));
legend("simulink acceleration", strcat("euler position 2nd derivative ", num2str(rmse_acc_euler)), strcat("estimated acc kalman filter ", num2str(rmse_acc_filter)), strcat("estimated acc kalman predictor ", num2str(rmse_acc_predictor)), strcat("estimated acc kalman smoother ", num2str(rmse_acc_smoother)))

set(gcf, 'Position', get(0, 'Screensize'));
% export_fig(strcat('C:\Users\emanuele\Desktop\phri_report_images\', datestr(now,'dd_mm_yyyy_HH_MM_SS_FFF')), '-pdf');
% close;

%% plot all acceleration but without euler second derivative
figure();
hold on;
plot(acc);
plot(filter_data(:, 3));
plot(predictor_data(:, 3));
plot(smoother_data(:, 3));
legend("simulink acceleration", strcat("estimated acc kalman filter ", num2str(rmse_acc_filter)), strcat("estimated acc kalman predictor ", num2str(rmse_acc_predictor)), strcat("estimated acc kalman smoother ", num2str(rmse_acc_smoother)))

set(gcf, 'Position', get(0, 'Screensize'));
% export_fig(strcat('C:\Users\emanuele\Desktop\phri_report_images\', datestr(now,'dd_mm_yyyy_HH_MM_SS_FFF')), '-pdf');
% close;

%% Kalman filter vs Kalman filter ss
figure;
subplot(2,1,1);
hold on;
plot(vel)
plot(filter_data(:,2));
plot(filter_ss_data(:,2));
legend('simulink velocity', strcat('estimated vel kalman filter:', " ", num2str(rmse_vel_filter)), strcat('estimated vel kalman filter ss: ', " ", num2str(rmse_vel_filter_ss)));

subplot(2,1,2);
hold on;
plot(acc);
plot(filter_data(:, 3));
plot(filter_ss_data(:, 3));
legend('simulink acceleration', strcat("acc kalman filter ", num2str(rmse_acc_filter)), strcat("acc kalman filter ss ", num2str(rmse_acc_filter_ss)));

set(gcf, 'Position', get(0, 'Screensize'));
export_fig(strcat('C:\Users\emanuele\Desktop\phri_report_images\', datestr(now,'dd_mm_yyyy_HH_MM_SS_FFF')), '-pdf');
close;

%% Kalman predictor vs Kalman predictor ss
figure;
subplot(2,1,1);
hold on;
plot(vel);
plot(predictor_data(:, 2));
plot(predictor_ss_data(:, 2));
legend('simulink velocity', strcat('vel kalman predictor:', " ", num2str(rmse_vel_predictor)), strcat('vel kalman predictor ss:', " ", num2str(rmse_vel_predictor_ss)));

subplot(2,1,2);
hold on;
plot(acc);
plot(predictor_data(:, 3));
plot(predictor_ss_data(:, 3));
legend('simulink acceleration', strcat("acc kalman predictor ", num2str(rmse_acc_predictor)), strcat("acc kalman predictor ss ", num2str(rmse_acc_predictor_ss)));

set(gcf, 'Position', get(0, 'Screensize'));
export_fig(strcat('C:\Users\emanuele\Desktop\phri_report_images\', datestr(now,'dd_mm_yyyy_HH_MM_SS_FFF')), '-pdf');
close;

%% Kalman filter vs Kalman smoother
figure;
subplot(2,1,1);
hold on;
plot(vel)
plot(filter_data(:, 2));
plot(smoother_data(:, 2));
legend('simulink velocity', strcat('vel kalman filter:', " ", num2str(rmse_vel_filter)), strcat('vel kalman smoother:', " ", num2str(rmse_vel_smoother)));

subplot(2,1,2);
hold on;
plot(acc);
plot(filter_data(:, 3));
plot(smoother_data(:, 3));
legend('simulink acceleration', strcat("acc kalman filter ", num2str(rmse_acc_filter)), strcat("acc kalman smoother ", num2str(rmse_acc_smoother)));

set(gcf, 'Position', get(0, 'Screensize'));
export_fig(strcat('C:\Users\emanuele\Desktop\phri_report_images\', datestr(now,'dd_mm_yyyy_HH_MM_SS_FFF')), '-pdf');
close;
