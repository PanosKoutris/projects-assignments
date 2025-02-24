% Definition of the function for the system
function dx = system(t,x)
    % x(1) = x1, x(2) = x2
    x1 = x(1);
    x2 = x(2);
    
    % Nonlinear function g(x)
    g_x = x1 * x2 + 0.5 * x2^2;
    
    % System equations
    dx1 = -x1 + x2;
    dx2 = -x1 + x1 * x2 + 0.5 * x2^2; % u(x) = 0 (no control input)
    
    % Return the rate vector
    dx = [dx1; dx2];
end

% Initial conditions
initial_conditions = [
    0.1, 0.1;
    0.4, 0.4;
    0.8, 0.8;
    -0.4, 1
];

% Simulation time
t_span = [0, 10];  % From 0 to 10 seconds
t_eval = linspace(t_span(1), t_span(2), 5000);  % Evaluation at 5000 points for higher resolution

% Simulation of the system for each set of initial conditions
% Plot 1: System response over time
figure('Position', [100, 100, 1200, 800]); % Larger figure size for clarity
hold on;
colors = lines(size(initial_conditions, 1)); % Use distinct colors for each initial condition
for i = 1:size(initial_conditions, 1)
    % Set initial conditions
    x0 = initial_conditions(i, :);
    
    % Solve the system with ODE solver
    [T, X] = ode45(@system, t_eval, x0);
    
    % Time response
    plot(T, X(:, 1), 'Color', colors(i, :), 'LineWidth', 1.5, ...
        'DisplayName', ['x1(t), initial x1 = ', num2str(x0(1))]);
    plot(T, X(:, 2), '--', 'Color', colors(i, :) * 0.7, 'LineWidth', 1.5, ...
        'DisplayName', ['x2(t), initial x2 = ', num2str(x0(2))]);
end
title('System Response Over Time');
xlabel('Time (s)', 'FontSize', 12);
ylabel('States (x1, x2)', 'FontSize', 12);
legend('Location', 'best');
grid on;
set(gca, 'FontSize', 12); % Larger font for readability
ylim([-20, 20]); % Set y-axis range for better visibility



% Plot 2: State-space response
figure('Position', [100, 100, 1200, 800]); % New figure
hold on;
for i = 1:size(initial_conditions, 1)
    % Set initial conditions
    x0 = initial_conditions(i, :);
    
    % Solve the system with ODE solver
    [T, X] = ode45(@system, t_eval, x0);
    
    % State-space response
    plot(X(:, 1), X(:, 2), 'Color', colors(i, :), 'LineWidth', 1.5, ...
        'DisplayName', ['Initial: ', num2str(x0(1)), ', ', num2str(x0(2))]);
end
title('State-Space Response');
xlabel('x1', 'FontSize', 12);
ylabel('x2', 'FontSize', 12);
legend('Location', 'best');
grid on;
set(gca, 'FontSize', 12); % Larger font for readability



% Plot 3: Flow field and attraction basin
figure('Position', [100, 100, 1200, 800]); % New figure
[x1, x2] = meshgrid(-2:0.1:2, -2:0.1:2); % Grid of values for x1 and x2
dx1 = -x1 + x2;
dx2 = -x1 + x1 .* x2 + 0.5 * x2.^2; % No control (u(x) = 0)

% Normalize the flows
norms = sqrt(dx1.^2 + dx2.^2);
dx1 = dx1 ./ norms;
dx2 = dx2 ./ norms;

% Flow diagram
quiver(x1, x2, dx1, dx2, 'AutoScale', 'on', 'Color', [0, 0.5, 0.8]);
title('Flow Field and Attraction Basin');
xlabel('x1', 'FontSize', 12);
ylabel('x2', 'FontSize', 12);
axis tight;
grid on;
set(gca, 'FontSize', 12); % Larger font for readability



