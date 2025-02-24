% Intitializing variables
c = 3*10^8;
f = 10^9;
I = 1;
N = 8;
Z0 = 377; 
l = c / f; 
d_array = [l/4, l/2, 3*l/4];

% Angles
theta = linspace(0, pi, 181); 
phi = linspace(0, 2*pi, 361); 
[Theta, Phi] = meshgrid(theta, phi);

dTheta = pi / 180; 
dPhi = pi / 180; 

for d = d_array

    % Calculation of power density
    E_3D = zeros(size(Theta));
    
    for n = 0:N-1
        E_3D = E_3D + I * exp(1j * 2 * pi / l * n * d * sin(Theta) .* cos(Phi));
    end
    E_3D = abs(E_3D);

    S = (E_3D.^2) / (2 * Z0); 

    % Max Power Density
    S_max = max(S(:));
    
    % Total Power
    P_tot = sum(sum(S .* sin(Theta) * dTheta * dPhi));
    
    % Calculation of Directivity
    D = 4 * pi * S_max / P_tot;
    
    % Theoretical Directivity
    D_theoretical = 2 * N * d / l;
    
    % Display Results
    fprintf('d = %.2fλ:\n', d/l);
    fprintf('Calculated Directicity: %.2f\n', D);
    fprintf('Theoretical Directicity: %.2f\n\n', D_theoretical);
end
