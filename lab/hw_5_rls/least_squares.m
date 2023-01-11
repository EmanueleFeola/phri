function [beta_hat_ls] = least_squares(X, Y)
    % Y = X * theta
    % predict y_hat with new x: y_hat = x * beta_hat_ls

    % params:
    % - X: [velocity, acceleration] (column vectors)
    % - Y: [voltage] (column vector)
    
    % output:
    % - beta_hat_ls: approximated theta params

    % output size: m x 1
    % where m = number of cols in X
    % the number of estimated params in beta depends on the number of columns of X

    % estimation of theta params
    beta_hat_ls = inv(X' * X) * X' * Y;
end

