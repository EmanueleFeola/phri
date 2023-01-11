function [smoother_data] = kalman_smoother(x_0, p_0, Q, R, A, C, y)
    % params:
    % initial conditions: x_0, p_0
    % variance of model and measurement Q, R
    % state matrices A, C
    % list of measurements y
    
    time_len = size(y, 1);
    filter_x_k1_k1{1} = x_0;
    filter_x_k1_k{1} = x_0;
    filter_p_k1_k1{1} = p_0;
    filter_p_k1_k{1} = p_0;

    % set initial conditions
    x_k_k = x_0;
    p_k_k = p_0;

    % forward step
    for k=2:time_len
        y_k1 = y(k); % measurement at time k
    
        % prediction step
        x_k1_k = A * x_k_k;
        p_k1_k = A * p_k_k * A'+ Q;
    
        % kalman gain - time varying
        K_gain = p_k1_k * C' * inv(C * p_k1_k * C' + R);
    
        % estimation step
        x_k1_k1 = x_k1_k + K_gain * (y_k1 - C * x_k1_k);
        p_k1_k1 = p_k1_k - p_k1_k * C' * (inv(C * p_k1_k * C' + R) * C * p_k1_k);
    
        filter_x_k1_k1{k} = x_k1_k1; % update current instant state
        filter_x_k1_k{k} = x_k1_k; % update current instant state
        filter_p_k1_k1{k} = p_k1_k1;
        filter_p_k1_k{k} = p_k1_k;

        x_k_k = x_k1_k1; % next iteration
        p_k_k = p_k1_k1; % next iteration
    end

    smoother_data{time_len} = filter_x_k1_k1{time_len}; % caso base ricorsione

    % backward step
    for k=time_len-1:-1:1
        % get forward step kalman filter state value
        x_k_k = filter_x_k1_k1{k};
        x_k1_k = filter_x_k1_k{k+1};
        
        % smoother state value at time k+1
        x_k1_N = smoother_data{k+1};
        
        % innovation
        innovation = x_k1_N - x_k1_k;
        
        % kalman gain
        p_k_k = filter_p_k1_k1{k};
        p_k1_k = filter_p_k1_k{k+1};
        K_gain_smoother = p_k_k * A' * inv(p_k1_k);

        x_k_N = x_k_k + K_gain_smoother * innovation;
        
        smoother_data{k} = x_k_N;
    end

    smoother_data = cell2mat(smoother_data);
end

