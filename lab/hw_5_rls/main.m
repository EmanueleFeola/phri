clc;
clear;

motor_pos_volt = load("dc_pos_volt_2.mat");
time_vector = motor_pos_volt.out_struct.time;
pos = motor_pos_volt.out_struct.pos;
% vel = motor_pos_volt.out_struct.vel;
voltage = motor_pos_volt.out_struct.voltages;
Ts = 0.001;

figure();
hold on;
plot(pos);
% plot(vel);
plot(voltage);
legend("pos", "voltage");

%% kalman speed estimation (prev hw)
A = [1 Ts (Ts^2)/2; 0 1 Ts; 0 0 1];
B = [Ts^3/6; Ts^2/2; Ts];
C = [1 0 0];

% initial conditions
p_0 = 0.1*eye(3);
x_0 = zeros(3, 1);
R = var(pos); %1;
q_low_case = 100000000;
Q = q_low_case * (B * B'); % mostrare cosa cambia modificando questo parametro (0, 1, 100)

filter_data = kalman_filter(x_0, p_0, Q, R, A, C, pos);
kalman_vel = filter_data(:, 2); % speed estimated with kalman filter
kalman_acc = filter_data(:, 3); % acc estimated with kalman filter

% figure();
% hold on;
% plot(kalman_vel);
% plot(kalman_acc);
% plot(vel);
% legend("kalman esimated speed", "kalman estimated acc", "'real' velocity");

%% least squares
X = [kalman_vel, kalman_acc];
Y = voltage;
Y = lowpass(Y, 5, 1/Ts);

beta_hat_ls = least_squares(X, Y);
y_hat_ls = X * beta_hat_ls; % assess param estimation: y_hat = x * beta_hat_ls

motor_k_ls = 1 / beta_hat_ls(2);
motor_tau_ls = motor_k_ls * beta_hat_ls(1);

figure();
hold on;
plot(Y);
plot(y_hat_ls);
legend("'real' voltage", "estimated voltage (least squares)");

%% recursive least squares
lambda = 1;
beta_hat_rls = recursive_least_squares(X, Y, lambda);

% size(b) = 1xN
y_hat_rls = zeros(size(X, 1), 1);
motor_k_rls = zeros(size(X, 1), 1);
motor_tau_rls = zeros(size(X, 1), 1);

for i=1:size(X, 1)
    y_hat_rls(i, 1) = X(i, :) * beta_hat_rls(:, i);

    motor_k_rls(i, 1) = 1 / beta_hat_rls(2, i);
    motor_tau_rls(i, 1) = motor_k_rls(i, 1) * beta_hat_rls(1, i);
end

figure();
hold on;
plot(Y);
plot(y_hat_rls);
legend("'real' voltage", "estimated voltage (recursive least squares)");

%% tau param estimation comparison
figure();
hold on;
plot(motor_k_ls * ones(size(X, 1), 1));
plot(motor_k_rls);
% plot(motor_tau_ls * ones(size(X, 1), 1));
% plot(motor_tau_rls);
legend("motor tau (ls)", "motor tau (rls)");