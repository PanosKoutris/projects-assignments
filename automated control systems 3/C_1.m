clearvars;
clc;
close all;

% Parameters 
m1 = 6; 
m2 = 4;
l1 = 0.5;
l2 = 0.4;
g = 9.81;
lc1_est = 0.25;
lc2_est = 0.175;
I1_est = 0.26;
I2_est = 0.08;
ml_est = 1;

% Desired position
des_pos = [pi/2; -pi/3];

% Sliding mode parameters
lambda = 5; 
k = 10; 

% Simulation settings
tspan = [0 2]; 
q0 = [pi/3; pi/3];
qdot0 = [0; 0];
initial_conditions = [q0; qdot0];

% Solver options
options = odeset('RelTol',1e-5,'AbsTol',1e-7);

% Solve using ode15s
[t, states] = ode15s(@(t, y) dynamics(t, y, des_pos, m1, m2, l1, l2, lc1_est, lc2_est, I1_est, I2_est, g, ml_est, lambda, k), tspan, initial_conditions, options);

% Extract states
q = states(:, 1:2);
qdot = states(:, 3:4);

% Define new colors for plots
joint1_color = [0.494, 0.184, 0.556];  % Purple
joint2_color = [0.850, 0.325, 0.098];  % Orange
desired_joint1_color = [0.301, 0.745, 0.933];  % Light blue
desired_joint2_color = [0.466, 0.674, 0.188];  % Green

% Plot for Joint Positions
figure;
plot(t, q(:,1), 'Color', joint1_color, 'LineWidth', 1.5); hold on;
plot(t, q(:,2), 'Color', joint2_color, 'LineWidth', 1.5);
plot(t, des_pos(1)*ones(size(t)), '--', 'Color', desired_joint1_color, 'LineWidth', 1.2);
plot(t, des_pos(2)*ones(size(t)), '--', 'Color', desired_joint2_color, 'LineWidth', 1.2);
xlabel('Time (s)'); 
ylabel('Position (rad)');
legend('Joint 1 Position', 'Joint 2 Position', 'Desired Joint 1', 'Desired Joint 2');
title('Joint Position Response'); 
grid on;



% Plot for Joint Velocities
figure;
plot(t, qdot(:,1), 'Color', joint1_color, 'LineWidth', 1.5); hold on;
plot(t, qdot(:,2), 'Color', joint2_color, 'LineWidth', 1.5);
xlabel('Time (s)'); 
ylabel('Velocity (rad/s)');
legend('Joint 1 Velocity', 'Joint 2 Velocity');
title('Joint Velocity Response'); 
grid on;



% Dynamics function
function dydt = dynamics(~, y, qd, m1, m2, l1, l2, lc1_est, lc2_est, I1_est, I2_est, g, ml_est, lambda, k)
    % Extract states
    q = y(1:2);
    qdot = y(3:4);

    % Desired position
    e = q - qd;
    edot = qdot;
    s = edot + lambda * e;

    % Inertia matrix H(q) (using estimated parameters)
    h11 = m1*lc1_est^2 + m2*(lc2_est^2 + l1^2 + 2*l1*lc2_est*cos(q(2))) + ml_est*(l2^2 + l1^2 + 2*l1*l2*cos(q(2))) + I1_est + I2_est;
    h12 = m2*lc2_est*(lc2_est + l1*cos(q(2))) + ml_est*l2*(l2 + l1*cos(q(2))) + I2_est;
    h22 = m2*lc2_est^2 + ml_est*l2^2 + I2_est;
    H = [h11, h12; h12, h22];

    % Coriolis and centripetal matrix C(q, qdot) (using estimated parameters)
    c11 = -l1*(m2*lc2_est + ml_est*l2)*sin(q(2))*qdot(2);
    c12 = -l1*(m2*lc2_est + ml_est*l2)*sin(q(2))*(qdot(2) + qdot(1));
    c21 = l1*(m2*lc2_est + ml_est*l2)*sin(q(2))*qdot(1);
    c22 = 0;
    C = [c11, c12; c21, c22];

    % Gravity vector g(q) (using estimated parameters)
    g1 = (m2*lc2_est + ml_est*l2)*g*cos(q(1) + q(2)) + (m2*l1 + ml_est*l1 + m1*lc1_est)*g*cos(q(1));
    g2 = (m2*lc2_est + ml_est*l2)*g*cos(q(1) + q(2));
    G = [g1; g2];

    % Control input
    u = H*(-lambda*edot - k*sat(s)) + C*qdot + G;

    % State derivatives
    qddot = H \ (u - C*qdot - G); % Solve H(q)*qddot = u - C(q, qdot)*qdot - G(q)
    dydt = [qdot; qddot];
end

% Saturation function
function s_out = sat(s)
    s_out = max(-1, min(1, s));
end

