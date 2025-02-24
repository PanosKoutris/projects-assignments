% Intitializing variables
fo = 10^9;
N = 201;
Zo = 50;
ZH = 150;
ZL = 20;

% f vector
f = 0:(2*fo/N):(2*fo);

% β(f)*l
bl1 = (pi/180)*21.903*f/fo;
bl2 = (pi/180)*31.426*f/fo;
bl3 = (pi/180)*37.72*f/fo;
bl4 = bl2;
bl5 = bl1;

% Calculation of Reflection Coefficient Magnitude and SWR
Zin1 = Zo;

Zin2 = ZH*(Zin1 + 1i*ZH*tan(bl5))./(ZH + 1i*Zin1*tan(bl5));

Zin3 = ZL*(Zin2 + 1i*ZL*tan(bl4))./(ZL + 1i*Zin2.*tan(bl4));

Zin4 = ZH*(Zin3 + 1i*ZH*tan(bl3))./(ZH + 1i*Zin3.*tan(bl3));

Zin5 = ZL*(Zin4 + 1i*ZL*tan(bl2))./(ZL + 1i*Zin4.*tan(bl2));

Zin = ZH*(Zin5 + 1i*ZH*tan(bl1))./(ZH + 1i*Zin5.*tan(bl1));

%Finally
S11 = abs((Zin - Zo)./(Zin + Zo));
S11_dB = 20*log(S11);
SWR = (1 + S11)./(1 - S11);

%Plots
S11_dB(S11_dB < -60) = -60;

figure
plot(f,S11_dB);
title('Reflection Coefficient(dB)')
hold on
xlabel("Frequency");
ylabel("|Γ|(dB)","Rotation",0);
hold off

SWR(SWR > 10) = 10;

figure
plot(f,SWR);
title("SWR")
hold on
xlabel("Frequency");
ylabel("SWR","Rotation",0);
hold off










