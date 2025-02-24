% Definition of the function for the closed-loop system
function dx = closedLoopSystem(t, x, control_case)
    % x(1) = x1, x(2) = x2
    x1 = x(1);
    x2 = x(2);
    
    % Nonlinear function g(x) (fixed for θ = 1)
    g_x = x1 * x2 + x2^2;  % Fixed g(x) with θ = 1
    
    % Select control case
    if control_case == 1
        % Controller for case 1
        u_x = -(x1 * x2 + x2^2) - 3 * x1 - 5 * x2;
    elseif control_case == 2
        % Controller for case 2
        u_x = -(x1 * x2 + 0.5 * x2^2) - 3 * x1 - 5 * x2;
    else
        error('Invalid value for control_case');
    end
    
    % Closed-loop system equations
    dx1 = -x1 + x2;
    dx2 = -x1 + g_x + u_x; 
    
    % Return rate vector
    dx = [dx1; dx2];
end

% Initial conditions
initial_conditions = [
    9, 11;
    11, 1;
];

% Simulation time
t_span = [0, 10];  % From 0 to 10 seconds
t_eval = linspace(t_span(1), t_span(2), 5000);  % Evaluate at 5000 points for higher resolution

% Define colors for better visualization
colors = lines(size(initial_conditions, 1));

% Save Plot 1: System response over time for control_case = 1
figure('Position', [100, 100, 1200, 800]);
hold on;
for i = 1:size(initial_conditions, 1)
    x0 = initial_conditions(i, :);
    [T, X] = ode45(@(t, x) closedLoopSystem(t, x, 1), t_eval, x0);
    plot(T, X(:, 1), 'Color', colors(i, :), 'LineWidth', 1.5, ...
        'DisplayName', ['x1(t), Initial x1 = ', num2str(x0(1))]);
    plot(T, X(:, 2), '--', 'Color', colors(i, :) * 0.7, 'LineWidth', 1.5, ...
        'DisplayName', ['x2(t), Initial x2 = ', num2str(x0(2))]);
end
title('System Response Over Time for Controller 1, θ=1');
xlabel('Time (s)', 'FontSize', 12);
ylabel('States (x1, x2)', 'FontSize', 12);
legend('Location', 'best');
grid on;
set(gca, 'FontSize', 12);


% Save Plot 2: System response over time for control_case = 2
figure('Position', [100, 100, 1200, 800]);
hold on;
for i = 1:size(initial_conditions, 1)
    x0 = initial_conditions(i, :);
    [T, X] = ode45(@(t, x) closedLoopSystem(t, x, 2), t_eval, x0);
    plot(T, X(:, 1), 'Color', colors(i, :), 'LineWidth', 1.5, ...
        'DisplayName', ['x1(t), Initial x1 = ', num2str(x0(1))]);
    plot(T, X(:, 2), '--', 'Color', colors(i, :) * 0.7, 'LineWidth', 1.5, ...
        'DisplayName', ['x2(t), Initial x2 = ', num2str(x0(2))]);
end
title('System Response Over Time for Controller 2, θ=1/2');
xlabel('Time (s)', 'FontSize', 12);
ylabel('States (x1, x2)', 'FontSize', 12);
legend('Location', 'best');
grid on;
set(gca, 'FontSize', 12);


% Save Plot 3: State-space response for control_case = 1
figure('Position', [100, 100, 1200, 800]);
hold on;
for i = 1:size(initial_conditions, 1)
    x0 = initial_conditions(i, :);
    [T, X] = ode45(@(t, x) closedLoopSystem(t, x, 1), t_eval, x0);
    plot(X(:, 1), X(:, 2), 'Color', colors(i, :), 'LineWidth', 1.5, ...
        'DisplayName', ['Initial: ', num2str(x0(1)), ', ', num2str(x0(2))]);
end
title('State-Space Response for Controller 1, θ=1');
xlabel('x1', 'FontSize', 12);
ylabel('x2', 'FontSize', 12);
legend('Location', 'best');
grid on;
set(gca, 'FontSize', 12);

% Save Plot 4: State-space response for control_case = 2
figure('Position', [100, 100, 1200, 800]);
hold on;
for i = 1:size(initial_conditions, 1)
    x0 = initial_conditions(i, :);
    [T, X] = ode45(@(t, x) closedLoopSystem(t, x, 2), t_eval, x0);
    plot(X(:, 1), X(:, 2), 'Color', colors(i, :), 'LineWidth', 1.5, ...
        'DisplayName', ['Initial: ', num2str(x0(1)), ', ', num2str(x0(2))]);
end
title('State-Space Response for Controller 2, θ=1/2');
xlabel('x1', 'FontSize', 12);
ylabel('x2', 'FontSize', 12);
legend('Location', 'best');
grid on;
set(gca, 'FontSize', 12);

