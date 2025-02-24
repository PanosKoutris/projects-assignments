% Parameters
theta = 1/2;          % Actual value of θ (known for the simulation)
P = [3, -2; -2, 5];   % Symmetric positive definite matrix P
x0 = [11; 1];         % Initial conditions for x (x1, x2)
theta_hat_0 = 1;      % Initial estimate of θ
T = 10;               % Total simulation time
dt = 0.01;            % Time step
time = 0:dt:T;        % Time vector

% Initialization
x = x0;                        % State variables (x1, x2)
theta_hat = theta_hat_0;       % Initial estimate of θ
theta_tilde = theta - theta_hat; % Difference between θ and its estimate
V = zeros(length(time), 1);    % Lyapunov function values

% Storage for plotting
x1_vals = zeros(1, length(time)); % Storage for x1
x2_vals = zeros(1, length(time)); % Storage for x2
theta_hat_vals = zeros(1, length(time)); % Storage for θ̂

% Simulation loop
for i = 1:length(time)-1
    % Calculate g(x) and u(x)
    g_x = x(1) * x(2) + theta * x(2)^2;         % g(x) = x1 * x2 + θ * x2^2
    u_x = -x(1) * x(2) - theta_hat * x(2)^2;   % u(x) = -x1 * x2 - θ̂ * x2^2
    
    % Compute derivatives of x
    dx1 = -x(1) + x(2);                        % dx1/dt = -x1 + x2
    dx2 = -x(1) + g_x + u_x;                   % dx2/dt = -x1 + g(x) + u(x)
    
    % Update states using Euler's method
    x(1) = x(1) + dx1 * dt;
    x(2) = x(2) + dx2 * dt;
    
    % Update the estimate of θ (adaptive strategy)
    theta_hat = theta_hat + 0.01 * (theta_tilde * x(2)^2); 
    
    % Compute the difference between θ and θ̂
    theta_tilde = theta - theta_hat;
    
    % Compute the Lyapunov function
    V(i) = x' * P * x + theta_tilde^2;
    
    % Store values for plotting
    x1_vals(i) = x(1);
    x2_vals(i) = x(2);
    theta_hat_vals(i) = theta_hat;
end

% Plot the results
figure;

% Plot x1 response over time
subplot(3, 1, 1);
plot(time, x1_vals, 'Color', [0.85, 0.33, 0.1], 'LineWidth', 2); % Orange color
title('Time Response of x1');
xlabel('Time (s)');
ylabel('x1');
grid on;

% Plot x2 response over time
subplot(3, 1, 2);
plot(time, x2_vals, 'Color', [0.1, 0.45, 0.9], 'LineWidth', 2); % Blue color
title('Time Response of x2');
xlabel('Time (s)');
ylabel('x2');
grid on;

% Plot theta estimate over time
subplot(3, 1, 3);
plot(time, theta_hat_vals, 'Color', [0.47, 0.67, 0.19], 'LineWidth', 2); % Green color
title('Time Response of Theta Estimate');
xlabel('Time (s)');
ylabel('θ̂');
grid on;


