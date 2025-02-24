clear; clc;

% Define specific values for V
V = [85, 95, 105, 115]; % Only these four values

% Define upper and lower bounds for x
lower_bounds = zeros(1, 17);
upper_bounds = [54.13, 21.56, 34.08, 49.19, 33.03, 21.84, 29.96, 24.87, ...
      47.24, 33.97, 26.89, 32.76, 39.98, 37.12, 53.83, 61.65, 59.73] * 0.999; % Adjusted upper bounds

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

% Initialize storage for results
x = zeros(length(V), 17);
fval = zeros(length(V), 1);
generations = zeros(length(V), 1);

% Run GA for each V value
for i = 1:length(V)
    Vi = V(i); % Get the current V value
    fprintf("V = %d\n", Vi);
    
    % Update equality constraints based on Vi
    beq = [-Vi; 0; 0; 0; 0; 0; 0; 0; Vi];

    % Define the objective function
    obj_fun = @(x) objective_function(x);
    
    % GA Options: Keep visualization (best & mean fitness)
    options = optimoptions('ga', ...
        'ConstraintTolerance', 1e-5, ...
        'PlotFcn', @gaplotbestf);  

    % Solve using GA
    [xt, fvalt, exitflag, output, population, scores] = ...
        ga(obj_fun, 17, [], [], Aeq, beq, lower_bounds, upper_bounds, [], options);
    
    % Store results
    x(i, :) = xt(:);
    fval(i) = fvalt;
    generations(i) = output.generations;

    % Zoom in
    ylim([min(scores(:)), max(scores(:)) * 1.1]); % Now applied after GA execution

    
    plot_filename = sprintf('GA_V%d.eps', Vi); % Generate filename based on V value
    saveas(gcf, plot_filename, 'epsc'); % Save the current figure in EPS format
    close(gcf); % Close the figure to avoid excessive figures opening
end

%% **Display Final Results**
fprintf("\nOptimization Completed!\n");
disp("Final Results:");
disp(x);
disp(['Minimum Travel Time Values: ', num2str(fval')]);
disp(['Number of Generations: ', num2str(generations')]);
