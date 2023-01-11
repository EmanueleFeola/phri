function [rmse] = compute_rmse(signal, ref_signal)
    rmse = signal - ref_signal;
    rmse = sqrt(mean(rmse.^2));
end

