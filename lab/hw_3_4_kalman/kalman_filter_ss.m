function [filter_data] = kalman_filter_ss(x_0, p_0, Q, R, A, C, y)
    % params:
    % initial conditions: x_0, p_0
    % variance of model and measurement Q, R
    % state matrices A, C
    % list of measurements y
    
    time_len = size(y, 1);
    filter_data = zeros(time_len, 3);
    filter_data(1, :) = x_0;

    % set initial conditions
    x_k_k = x_0;
    p_k_k = p_0;

    % solution of the algebraic riccati equation
    p_inf = idare(A', C', Q, R);
    % kalman gain - constant over time
    K_inf = p_inf * C' * inv(C * p_inf * C' + R); 
    
    for k=2:time_len
        y_k1 = y(k); % measurement at time k
    
        % prediction step
        x_k1_k = A * x_k_k;
        % estimation step
        x_k1_k1 = x_k1_k + K_inf * (y_k1 - C * x_k1_k);
    
        filter_data(k, :) = x_k1_k1; % update current instant state
        x_k_k = x_k1_k1; % next iteration
    end
end

