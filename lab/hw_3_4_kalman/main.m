clc
clear
close all

%%
vars = load("sim_out_vars.mat");

time = vars.out_struct.time_vector;
pos_noise = vars.out_struct.pos_with_noise;
clean_data = [vars.out_struct.pos, vars.out_struct.vel, vars.out_struct.acc]; % clean data from simulink

Ts = time(10, 1) - time(9, 1);

%%
data = load('master_slave_1kHz.txt');
pos_noise = data(400:end, 2); % second column is the master position
vel = data(400:end, 3); % third column is the master velocity
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
R = 1;
q_low_case=1000;
Q = q_low_case * B * B'; % mostrare cosa cambia modificando questo parametro (0, 1, 100)

filter_data = kalman_filter(x_0, p_0, Q, R, A, C, pos_noise);
filter_ss_data = kalman_filter_ss(x_0, p_0, Q, R, A, C, pos_noise);
predictor_data = kalman_predictor(x_0, p_0, Q, R, A, C, pos_noise);
predictor_ss_data = kalman_predictor_ss(x_0, p_0, Q, R, A, C, pos_noise);
smoother_data = kalman_smoother(x_0, p_0, Q, R, A, C, pos_noise);
smoother_data = smoother_data';

compute_rmse(filter_data(:, 2), vel)
compute_rmse(filter_ss_data(:, 2), vel)
compute_rmse(predictor_data(:, 2), vel)
compute_rmse(predictor_ss_data(:, 2), vel)
compute_rmse(smoother_data(:, 2), vel)

%% plot result data
figure;
subplot(1,2,1);
hold on;
plot(derivative_data(:, 2));
plot(vel)
plot(filter_data(:, 2));
plot(predictor_data(:, 2));
plot(smoother_data(:, 2));
legend('euler position derivative', 'vel encoder', 'vel kalman filter', 'vel kalman predictor', 'vel kalman smoother')

subplot(1,2,2);
hold on;
plot(derivative_data(:, 3));
plot(filter_data(:, 3));
plot(predictor_data(:, 3));
plot(smoother_data(:, 3));
legend('euler position 2nd derivative', 'acc kalman filter', 'acc kalman predictor', 'acc kalman smoother')
%% steady state comparison - filter
figure;
subplot(1,2,1);
hold on;
plot(vel)
plot(filter_data(:,2));
plot(filter_ss_data(:,2));
legend('vel encoder', 'vel kalman filter', 'vel kalman filter ss')

subplot(1,2,2);
hold on;
plot(filter_data(:, 3));
plot(filter_ss_data(:, 3));
legend('acc kalman filter', 'acc kalman filter ss')
%% steady state comparison - predictor
figure;
subplot(1,2,1);
hold on;
plot(vel)
plot(predictor_data(:, 2));
plot(predictor_ss_data(:, 2));
legend('vel encoder', 'vel kalman predictor', 'vel kalman predictor ss')

subplot(1,2,2);
hold on;
plot(predictor_data(:, 3));
plot(predictor_ss_data(:, 3));
legend('acc kalman predictor', 'acc kalman predictor ss')