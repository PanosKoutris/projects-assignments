% Functions
g = @(x) x(1)*x(2) + 0.5*x(2)^2;  % Function g(x)
u = @(x) -1*(x(2) + 2.3*norm(x)^2); % Function u(x)


% Definition of the system equations
dxdt = @(t, x) [
    -x(1) + x(2);
    -x(1) + g(x) + u(x)
];

% Initial conditions
x1_0 = [0.8; 0.8];  % First initial state
x2_0 = [-0.4; 1];   % Second initial state

% Simulation time
tspan = [0 10];
t_eval = linspace(tspan(1), tspan(2), 1000);

% Solve the system for both initial conditions
[t1, x1] = ode15s(dxdt, t_eval, x1_0);
[t2, x2] = ode15s(dxdt, t_eval, x2_0);

% Plot 1: Time response of x1
figure;
plot(t1, x1(:, 1), 'LineWidth', 1.5, 'Color', [0.85, 0.33, 0.1], ...
    'DisplayName', 'x1 - Initial [0.8, 0.8]');
hold on;
plot(t2, x2(:, 1), 'LineWidth', 1.5, 'Color', [0, 0.45, 0.74], ...
    'DisplayName', 'x1 - Initial [-0.4, 1]');
xlabel('t (s)');
ylabel('x1');
legend('Location', 'best');
title('Response of x1 Over Time');
grid on;

% Plot 2: Time response of x2
figure;
plot(t1, x1(:, 2), 'LineWidth', 1.5, 'Color', [0.85, 0.33, 0.1], ...
    'DisplayName', 'x2 - Initial [0.8, 0.8]');
hold on;
plot(t2, x2(:, 2), 'LineWidth', 1.5, 'Color', [0, 0.45, 0.74], ...
    'DisplayName', 'x2 - Initial [-0.4, 1]');
xlabel('t (s)');
ylabel('x2');
legend('Location', 'best');
title('Response of x2 Over Time');
grid on;

% Plot 3: State-space response
figure;
plot(x1(:, 1), x1(:, 2), 'LineWidth', 1.5, 'Color', [0.85, 0.33, 0.1], ...
    'DisplayName', 'Initial [0.8, 0.8]');
hold on;
plot(x2(:, 1), x2(:, 2), 'LineWidth', 1.5, 'Color', [0, 0.45, 0.74], ...
    'DisplayName', 'Initial [-0.4, 1]');
xlabel('x1');
ylabel('x2');
legend('Location', 'best');
title('State-Space Response');
grid on;
