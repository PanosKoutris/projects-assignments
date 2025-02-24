% Initialization of variables
fo = 10^9;
N = 201;
Zo1 = 101.6;
Zo2 = 98.45;    Yo2 = 1/Zo2;
Zo3 = 43.6;     Yo3 = 1/Zo3;
Zo4 = 50;
ZL = 50;

% f vector
f = 0:(3*fo/N):(3*fo); 

% β(f)*l
bl = pi*f/(4*fo);

% Calculation of Reflection Coefficient Magnitude and SWR
Zin3 = Zo4;
Yin3 = 1./Zin3;

Yopen3 = 1i*Yo2*tan(bl);

YL2 = Yin3 + Yopen3;
ZL2 = 1./YL2;

Zin2 = Zo1*(ZL2 + 1i*Zo1*tan(bl))./(Zo1 + 1i*ZL2.*tan(bl));
Yin2 = 1./Zin2;

Yopen2 = 1i*Yo3*tan(bl);

YL1 = Yin2 + Yopen2;
ZL1 = 1./YL1;

Zin1 = Zo1*(ZL1 + 1i*Zo1*tan(bl))./(Zo1 + 1i*ZL1.*tan(bl));
Yin1 = 1./Zin1;

Yopen1 = Yopen3;

Yin = Yin1 +Yopen1;
Zin = 1./Yin;

% Finally
S11 = abs((Zin - Zo4)./(Zin + Zo4));
S11_dB = 20*log(S11);
SWR = (1 + S11)./(1 - S11);

% Plots
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















