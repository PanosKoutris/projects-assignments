% Initializing Variables
Z0 = 50; 
ZL = 20 - 1i*30; 
f0 = 5*10^9; 
N = 101;
l1_values = [0.132, 0.632];
l2_values = [0.181, 0.681];

% f vector
f = 0:(2*f0/N):(2*f0);

Y0 = 1/Z0;
YL = 1/ZL;

for i = 1:length(l1_values)
    l1 = l1_values(i);
    l2 = l2_values(i);
    
    Yopen1 = 1i*Y0*tan(l1*2*pi*f/f0);

    Yin1 = YL + Yopen1;
    Zin1 = 1./Yin1;

    ZL1 = Z0*((Zin1 + 1i*Z0*tan(0.125*2*pi*f/f0))./(Z0 + 1i*Zin1.*tan(0.125*2*pi*f/f0)));
    YL1 = 1./ZL1;

    Yopen2 = 1i*Y0*tan(l2*2*pi*f/f0); 

    Yin = Yopen2 + YL1;
    Zin = 1./Yin;

    S11 = abs((Zin - Z0)./(Zin + Z0));

    % Plot 
    figure;
    plot(f, S11);
    xlabel('Frequency');
    ylabel('Magnitude of Reflection Coefficient');
    title(['Magnitude of Reflection Coefficient for l1 = ', num2str(l1), 'λ, l2 = ', num2str(l2), 'λ']);
    grid on;
end

