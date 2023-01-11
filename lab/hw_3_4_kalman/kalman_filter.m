function [filter_data] = kalman_filter(x_0, p_0, Q, R, A, C, y)
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

    for k=2:time_len
        y_k1 = y(k); % measurement at time k
    
        % prediction step
        x_k1_k = A * x_k_k;
        p_k1_k = A * p_k_k * A'+ Q;
    
        % kalman gain
        % i can't use 'idare' because this is not an algebraic equation
        % it changes over time -> no idare
        K_gain = p_k1_k * C' * inv(C * p_k1_k * C' + R);
    
        % estimation step
        x_k1_k1 = x_k1_k + K_gain * (y_k1 - C * x_k1_k);
        p_k1_k1 = p_k1_k - p_k1_k * C' * (inv(C * p_k1_k * C' + R) * C * p_k1_k);
    
        filter_data(k, :) = x_k1_k1; % update current instant state
        
        x_k_k = x_k1_k1; % next iteration
        p_k_k = p_k1_k1; % next iteration
    end
end

