% Intitailization of variables
Zo=50;
Yo= 1/Zo;
fo=10^9;
N=201;
ZL=100;
C=2*10^(-12);

% f vector
f = 0:(4*fo/N):(4*fo);

% β(f)*l
bl1 = 0.64*pi*f/fo;
bl2 = 0.48*pi*f/fo;
bl3 = 0.2*pi*f/fo;

% Calculation of Reflection Coefficient Magnitude 
Zin3 = Zo*(ZL + 1i * Zo * tan(bl1))./(Zo + 1i * ZL * tan(bl1));

Zc= Zin3 - 1i./(2*pi*f*C);

Zin2 = Zo*(Zc + 1i * Zo * tan(bl2))./(Zo + 1i * Zc .* tan(bl2));
Yin2 = 1./Zin2;

Yopen = 1i * Yo * tan(bl3);
 
Yin = Yopen + Yin2;
Zin = 1./Yin;

%Finally
S11 = abs((Zin - Zo)./(Zin + Zo));
S11_dB = 20*log(S11);

% Plots
figure
plot(f,S11);
title("Reflection Coefficient Magnitude")
hold on
xlabel("Frequency");
ylabel("|Γ|","Rotation",0);
hold off

S11_dB(S11_dB < -40) = -40;

figure
plot(f,S11_dB);
title("Reflection Coefficient Magnitude(dB)")
hold on
xlabel("Frequency");
ylabel("|Γ|(dB)","Rotation",0);
hold off


