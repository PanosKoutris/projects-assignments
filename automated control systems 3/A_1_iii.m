% System parameters
K = 5;
T = 0.2;

% State-space matrix definition
A = [0, 1; -K/T, -1/T];
B = [0; K/T];
C = [-1, 0];
D = 1;

% Inputs
step_input = 0.5; % Step input
ramp_input = @(t) 1.2 * t; % Ramp input

% Simulation time
tspan = [0, 3];

% Initial conditions 
initial_conditions = [
    -2, 0;
    1, 0;
    0, 0.5;
    2, 2;
    2.5, -1;
    1.1, 2
];

% Simulation for each initial condition
for i = 1:size(initial_conditions, 1)
    % Initial conditions
    x0 = initial_conditions(i, :)';

    % Simulation for step input
    [t1, x1] = ode45(@(t, x) A*x + B*step_input, tspan, x0);
    y1 = C*x1' + D*step_input; % Output calculation
    e1 = step_input - y1; % Error calculation

    % Simulation for ramp input
    [t2, x2] = ode45(@(t, x) A*x + B*ramp_input(t), tspan, x0);
    y2 = C*x2' + D*ramp_input(t2); % Output calculation
    e2 = ramp_input(t2) - y2; % Error calculation

    % Plot for step input
    figure;
    subplot(3, 1, 1);
    plot(t1, x1(:, 1), 'g-', t1, x1(:, 2), 'm--'); 
    title(['State Responses Over Time (Step Input, Initial: y(0)=' num2str(x0(1)) ', dy/dt(0)=' num2str(x0(2)) ')']);
    legend('x_1(t)', 'x_2(t)');
    xlabel('Time (s)');
    ylabel('States');

    subplot(3, 1, 2);
    plot(x1(:, 1), x1(:, 2), 'c-'); 
    title('Phase Diagram (Step Input)');
    xlabel('x_1');
    ylabel('x_2');

    subplot(3, 1, 3);
    plot(t1, y1, 'b-', t1, step_input * ones(size(t1)), 'r--', t1, e1, 'k:'); 
    title('Output Response and Error (Step Input)');
    legend('y(t)', 'r(t)', 'e(t)');
    xlabel('Time (s)');
    ylabel('Response');

    

    % Plot for ramp input
    figure;
    subplot(3, 1, 1);
    plot(t2, x2(:, 1), 'g-', t2, x2(:, 2), 'm--'); 
    title(['State Responses Over Time (Ramp Input, Initial: y(0)=' num2str(x0(1)) ', dy/dt(0)=' num2str(x0(2)) ')']);
    legend('x_1(t)', 'x_2(t)');
    xlabel('Time (s)');
    ylabel('States');

    subplot(3, 1, 2);
    plot(x2(:, 1), x2(:, 2), 'c-'); 
    title('Phase Diagram (Ramp Input)');
    xlabel('x_1');
    ylabel('x_2');

    subplot(3, 1, 3);
    plot(t2, y2, 'b-', t2, ramp_input(t2), 'r--', t2, e2, 'k:'); % Changed colors
    title('Output Response and Error (Ramp Input)');
    legend('y(t)', 'r(t)', 'e(t)');
    xlabel('Time (s)');
    ylabel('Response');

    
end
