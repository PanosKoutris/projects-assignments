% Initializing Variables
c0 = 3*10^8;  
eps_r_eff = 3.329;
a = 0.2198;
l = 0.03199; % Calculated at (c) 

% Function
equation = @(fr) tan((2 * pi * fr * l) / (c0 / sqrt(eps_r_eff))) + sqrt(a * c0 / (2 * fr * sqrt(eps_r_eff)));

% Starting Estimation
initial_guess = 2.5*10^9;  

% Solving the equation
fr = fzero(equation, initial_guess);

% Printing the solution
fprintf('The solution for the frequency is: %.10e Hz\n', fr);
