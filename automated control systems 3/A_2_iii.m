% System parameters
K = 5;
T = 0.2;
e0 = 0.2;
alpha = 0.05;

% Reference inputs
inputs = {
    struct('name', 'Step', 'r', @(t) 0.5, 'r_prime', @(t) 0, 'r_double_prime', @(t) 0),
    struct('name', 'Ramp1', 'r', @(t) 1.2 * t, 'r_prime', @(t) 1.2, 'r_double_prime', @(t) 0),
    struct('name', 'Ramp2', 'r', @(t) 0.04 * t, 'r_prime', @(t) 0.04, 'r_double_prime', @(t) 0),
    struct('name', 'Ramp3', 'r', @(t) 0.5 * t, 'r_prime', @(t) 0.5, 'r_double_prime', @(t) 0)
};

% Initial conditions 
initial_conds = [
    -2, 0; % Simulation 1
    1, 0;  % Simulation 2
    0, 0.5; % Simulation 3
    2, 2; % Simulation 4
    2.5, -1; % Simulation 5
    1.1, 2  % Simulation 6
];

% Simulation time span
t_span = [0, 12];

% State-space equations with embedded nonlinear gain
system_dynamics = @(t, x, r_func, r_prime, r_double_prime, K, T, e0, alpha) [
    x(2); % x1' = x2
    (T * r_double_prime(t) + r_prime(t) - x(1) * ...
    (1 + K * ((abs(x(1)) <= e0) .* (alpha * x(1)) + (abs(x(1)) > e0) .* sign(x(1))))) / T % x2'
];

% Generate plots for each input and set of initial conditions
for i = 1:length(inputs)
    input = inputs{i};
    for j = 1:size(initial_conds, 1)
        x0 = initial_conds(j, :); % Initial conditions

        % Solve the differential equations
        [t, x] = ode45(@(t, x) system_dynamics(t, x, ...
            input.r, input.r_prime, input.r_double_prime, K, T, e0, alpha), ...
            t_span, x0);

        % Calculate output y, reference input r, and error e
        y = x(:, 1); % Output
        r = arrayfun(input.r, t); % Reference input
        e = r - y; % Error

        % Create a new figure
        figure;
        tiledlayout(3, 1);

        % State response over time
        nexttile;
        plot(t, x(:, 1), 'm', 'DisplayName', 'x_1 (Position)');
        hold on;
        plot(t, x(:, 2), 'c', 'DisplayName', 'x_2 (Velocity)');
        xlabel('Time (s)');
        ylabel('States');
        legend;
        title(sprintf('State Response Over Time - Input: %s, Simulation: %d', input.name, j));
        grid on;

        % Phase diagram
        nexttile;
        plot(x(:, 1), x(:, 2), 'r');
        xlabel('x_1 (Position)');
        ylabel('x_2 (Velocity)');
        title('Phase Diagram');
        grid on;

        % Output, reference input, and error
        nexttile;
        plot(t, y, 'b', 'DisplayName', 'y (Output)');
        hold on;
        plot(t, r, 'g--', 'DisplayName', 'r (Reference Input)');
        plot(t, e, 'k:', 'DisplayName', 'e (Error)');
        xlabel('Time (s)');
        ylabel('Output/Error');
        legend;
        title('Output, Reference Input, and Error');
        grid on;

        
    end
end
