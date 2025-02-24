% Initializing variables 
d = 0.0015; 
L = 0.06; 
l_1 = L - d; 
Zin_1_measured = 4.9678 + 1j*43.9439; 

beta_0 = 156.33; 
Z_0 = 419.2; 

% Function to solve 
fun = @(x) solve_impedance(x, Zin_1_measured, d, l_1, beta_0, Z_0);

% Initial guess 
initial_guess = [2.5, 0.01]; % Adjust as necessary!

% Solving the equation
solution = fsolve(fun, initial_guess);
e_r_solution = solution(1);
tan_delta_solution = solution(2);

% Display the results
fprintf('e_r = %.6f\n', e_r_solution);
fprintf('tan_delta = %.6f\n', tan_delta_solution);

function F = solve_impedance(x, Zin_measured, d, l_1, beta_0, Z_0)
    e_r = x(1);
    tan_delta = x(2);
    
    beta_1 = sqrt(44413.22 * e_r - 18886.32);
    a_d1 = ((44413.22 * e_r)^2 * tan_delta) / (2 * sqrt(44413.22 * e_r - 18886.32));
    gamma_1 = a_d1 + 1j * beta_1;
    
    Z_1 = 377 / sqrt(e_r - 0.1913);
    
    Z_A1 = Z_1 * tanh(gamma_1 * d);
    
    Zin_1 = Z_0 * (Z_A1 + 1j * Z_0 * tan(beta_0 * l_1)) / (Z_0 + 1j * Z_A1 * tan(beta_0 * l_1));
    
    % Define the real and imaginary parts to be zero
    F(1) = real(Zin_1) - real(Zin_measured);
    F(2) = imag(Zin_1) - imag(Zin_measured);
end




