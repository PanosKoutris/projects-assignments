function f_x = objective_function(x)
    % Capacity of each road
    c_i = [54.13, 21.56, 34.08, 49.19, 33.0, 21.84, 29.96, 24.87, ...
         47.24, 33.97, 26.89, 32.76, 39.98, 37.12, 53.83, 61.65, 59.73];
    
    % Congestion coefficients
    a_i = [1.25, 1.25, 1.25, 1.25, 1.25, 1.5, 1.5, 1.5, 1.5, 1.5, ...
          1, 1, 1, 1, 1, 1, 1];
    
    % Base travel time (assumed constant for now)
    t_i = ones([1 17]) * 5;

    % Initialize f_x
    f_x = 0;

    % Compute total travel time
    for k = 1:17
        if x(k) >= c_i(k)
            f_x = inf;  % Avoid exceeding capacity
            return;
        end
        f_x = f_x + x(k) * (t_i(k) + a_i(k) * x(k) / (1 - x(k)/c_i(k)));
    end
end
