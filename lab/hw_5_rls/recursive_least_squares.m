function [beta_hat_rls] = recursive_least_squares(x, y, lambda)
    % output size: m x N
    % where m = # of cols in x, N = # time instants
    % the params are fixed but their estimation change over time
    % because this is an 'online' algorithm

    % lambda: forgetting factor: 0 < lambda <= 1
    % if lambda = 1, same importance to all samples (from time 0 to now)
    % if lambda < 1: recent samples are more important

    b{1} = zeros(2, 1); % b is col vector
    K{1} = zeros(2, 2);
    P{1} = eye(2);
    e{1} = y(1);
     
    for k = 2:size(x, 1)
        P_num = P{k - 1} * x(k, :)' * x(k, :) * P{k - 1};
        P_den = lambda + x(k, :) * P{k - 1} * x(k, :)';
    
        P{k} = P{k - 1} - P_num / P_den;
        P{k} = P{k} / lambda;
        K{k} = P{k} * x(k, :)';
        e{k} = y(k, :) - x(k, :) * b{k - 1};
        b{k} = b{k - 1} + K{k} * e{k};
    end 

    beta_hat_rls = b;
    beta_hat_rls = cell2mat(beta_hat_rls);
end

