clearvars;
clc;
close all;

f = @(x) (1/3)*x(1)^2 + 3*x(2)^2;
grad_f = @(x) [(2/3)*x(1); 6*x(2)];

% Problem parameters
x0 = [5; -5];        
s = 5;                
gamma = 0.5;        
tol = 0.01;           
max_iter = 100;       
a = [-10; -8];        
b = [5; 12];          

% Run the constrained steepest descent method
[x_history, f_history] = deepest_descent_with_constraints(f, grad_f, x0, gamma, s, a, b, max_iter, tol);

% Display the final results
disp('Final point:');
disp(x_history(:, end));
disp('Final function value:');
disp(f_history(end));
disp('Number of iterations:');
disp(length(f_history));

% Plot the convergence
figure;
plot(1:length(f_history), f_history, '-o', 'LineWidth', 1.5, 'MarkerSize', 4, 'Color', 'b');
title('Convergence of Function Value ');
xlabel('Number of Iterations');
ylabel('Function Value');
grid on;
