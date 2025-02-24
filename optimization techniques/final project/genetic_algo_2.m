clear; clc;

% Define upper and lower bounds for x (flow on each road)
lower_bounds = zeros(1, 17);
upper_bounds = [54.13, 21.56, 34.08, 49.19, 33.03, 21.84, 29.96, 24.87, ...
      47.24, 33.97, 26.89, 32.76, 39.98, 37.12, 53.83, 61.65, 59.73] * 0.999;

%% **Define Equality Constraints (Aeq * x = beq)**
% Conservation of flow at each node (9 nodes)
Aeq = [
    -1, -1, -1, -1, 0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0;   % Node 1 (entry)
     0,  1,  0,  0,  0,  0, -1, -1,  0,  0,  0,  0,  0,  0,  0,  0,  0;  % Node 2
     0,  0,  0,  1,  0,  0,  0,  0, -1, -1,  0,  0,  0,  0,  0,  0,  0;  % Node 3
     1,  0,  0,  0, -1, -1,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0;  % Node 4
     0,  0,  1,  0,  0,  0,  0,  1,  1,  0, -1, -1, -1,  0,  0,  0,  0;  % Node 5
     0,  0,  0,  0,  0,  1,  1,  0,  0,  0,  0,  0,  1, -1, -1,  0,  0;  % Node 6
     0,  0,  0,  0,  0,  0,  0,  0,  0,  1,  1,  0,  0,  0,  0,  0, -1;  % Node 7
     0,  0,  0,  0,  1,  0,  0,  0,  0,  0,  0,  0,  0,  1,  0, -1,  0;  % Node 8
     0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  1,  0,  0,  1,  1,  1   % Node 9 (exit)
];

% Define beq based on conservation of flow
beq = [-100; 0; 0; 0; 0; 0; 0; 0; 100];

%% **Run Genetic Algorithm (GA)**
obj_fun = @(x) objective_function(x);
options = optimoptions('ga', ...
    'ConstraintTolerance', 1e-5, ...
    'PlotFcn', @(options, state, flag) gaplotbestf(options, state, flag));

% Run the GA optimization
[x, fval, exitflag, output, population, scores] = ...
    ga(obj_fun, 17, [], [], Aeq, beq, lower_bounds, upper_bounds, [], options);

% Zoom in
ylim([min(scores(:)), max(scores(:)) * 1.1]); 

%% **Display Results**
disp('Optimal Traffic Flow x_i:');
disp(x);
disp(['Minimum Total Travel Time: ', num2str(fval), ' min']);
