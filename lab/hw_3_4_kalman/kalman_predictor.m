function [predictor_data] = kalman_predictor(x_0, p_0, Q, R, A, C, y)
    % params:
    % initial conditions: x_0, p_0
    % variance of model and measurement Q, R
    % state matrix C
    % list of measurements y
    
    time_len = size(y, 1);
    predictor_data = zeros(time_len, 3);

    % kalman predictor
    x_k_km1 = x_0;
    p_k_km1 = p_0;
    
    for k=2:time_len
        y_k = y(k); % measurement at time k
    
        % kalman gain
        % i can't use 'idare' because this is not an algebraic equation
        % it changes over time -> no idare
        K_gain_bar = A * p_k_km1 * C' * inv(C * p_k_km1 * C' + R);

        x_k1_k = A * x_k_km1 + K_gain_bar * (y_k - C * x_k_km1);
        p_k1_k = A * p_k_km1 * A' - A * p_k_km1 * C' * inv(C * p_k_km1 * C' + R) * C * p_k_km1 * A' + Q;
    
        predictor_data(k, :) = x_k1_k; % update current instant state
        x_k_km1 = x_k1_k; % next iteration
        p_k_km1 = p_k1_k; % next iteration
    end

end

