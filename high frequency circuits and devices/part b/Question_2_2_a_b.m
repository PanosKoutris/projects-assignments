% Intitializing variables
c = 3*10^8; 
f = 10^9; 
I = 1; 
N = 8;
l = c / f; 
d_array = [l/4, l/2, 3*l/4]; 

% Angles
theta = linspace(0, pi, 1000);
phi = linspace(0, 2*pi, 1000); 
[Theta, Phi] = meshgrid(theta, phi);

for d = d_array

    % Initialization of E's
    E_horizontal = zeros(size(theta));
    E_vertical = zeros(size(phi));
    E_3D = zeros(size(Theta));
    
    % Calculation of Horizontal Radiation 
    for n = 0:N-1
        E_horizontal = E_horizontal + I * exp(1i * 2 * pi * (n * d / l) * cos(theta));
    end
    E_horizontal = abs(E_horizontal); % We take the magnitude
    
    % Calculation of Vertical Radiation
    for n = 0:N-1
        E_vertical = E_vertical + I * exp(1i * 2 * pi * (n * d / l) * cos(phi));
    end
    E_vertical = abs(E_vertical); % We take the magnitude
    
    % Normalization 
    E_horizontal = E_horizontal / max(E_horizontal);
    E_vertical = E_vertical / max(E_vertical);
    
    % Plot of horizontal radiation 
    figure;
    polarplot(theta, E_horizontal);
    title(['Horizontal Radiation for d = ', num2str(d / l), '\lambda']);
    ax = gca;
    ax.RLim = [0 1];
    
    % Plot of vertical radiation
    figure;
    polarplot(phi, E_vertical);
    title(['Vertical Radiation for d = ', num2str(d / l), '\lambda']);
    ax = gca;
    ax.RLim = [0 1];
    
    % Calculation of 3D radiation 
    for n = 0:N-1
        E_3D = E_3D + I * exp(1i * 2 * pi * (n * d / l) * (sin(Phi) .* cos(Theta)));
    end
    E_3D = abs(E_3D); % We take the magnitude
    
    % Normalize 
    E_3D = E_3D / max(E_3D(:));
    
    % Convert spherical to Cartesian coordinates for plotting
    X = E_3D .* sin(Phi) .* cos(Theta);
    Y = E_3D .* sin(Phi) .* sin(Theta);
    Z = E_3D .* cos(Phi);
    
    % Plot of 3D radiation 
    figure;
    surf(X, Y, Z, E_3D, 'EdgeColor', 'none');
    colorbar;
    title(['3D Radiation Pattern for d = ', num2str(d / l), '\lambda']);
    xlabel('X');
    ylabel('Y');
    zlabel('Z');
    axis equal;
end