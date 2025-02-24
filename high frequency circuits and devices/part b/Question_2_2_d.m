% Initializing Variables
f = 10^9;  
c = 3*10^8;  
l = c / f;  

% Distances 
hx = [l/4, l/4, 3*l/4];
hy = [l/4, 3*l/4, 3*l/4];

% Angles
theta = linspace(0, 2*pi, 360);
phi = linspace(0, pi, 180);

% Radiation function
radiation = @(hx, hy, theta, phi) abs(1 + ...
    exp(1i * 2 * pi * hx * cos(theta) .* sin(phi) / l) + ...
    exp(1i * 2 * pi * hy * sin(theta) .* sin(phi) / l) + ...
    exp(1i * 2 * pi * hx * cos(theta + pi) .* sin(phi) / l) + ...
    exp(1i * 2 * pi * hy * sin(theta + pi) .* sin(phi) / l));

% Plotting Radiation
for i = 1:3

    E_2D = radiation(hx(i), hy(i), theta, pi/2);
    
    figure;
    polarplot(theta, E_2D);
    title(sprintf('2D Case %d: hx = %.2f, hy = %.2f', i, hx(i), hy(i)));

    % 3D radiation 
    [Theta, Phi] = meshgrid(theta, phi);
    E_3D = radiation(hx(i), hy(i), Theta, Phi);

    % spherical to Cartesian coordinates 
    X = E_3D .* sin(Phi) .* cos(Theta);
    Y = E_3D .* sin(Phi) .* sin(Theta);
    Z = E_3D .* cos(Phi);

    figure;
    surf(X, Y, Z, 'EdgeColor', 'none');
    title(sprintf('3D Case %d: hx = %.2f, hy = %.2f', i, hx(i), hy(i)));
    xlabel('X');
    ylabel('Y');
    zlabel('Z');
    axis equal;
    colorbar;
end

