function [] = plot_S11(p)
p=[0.27685  0.10287 0.11955 0.12833 0.13405 0.090377]; %Optimal Values
d1 = p(1);
d2 = p(2);
d3 = p(3);
l1 = p(4);
l2 = p(5);
l3 = p(6);
Zo = 50;   Yo = 1/Zo;
ZL = 120 +1i*60;

% Normalized f vector
normf = 0.01:0.01:2;

% β(f)*l
bd1 = 2*pi*d1*normf;
bd2 = 2*pi*d2*normf;
bd3 = 2*pi*d3*normf;
bl1 = 2*pi*l1*normf;
bl2 = 2*pi*l2*normf;
bl3 = 2*pi*l3*normf;

% Calculation of Reflection Coefficient Magnitude
Zin1 = Zo*(ZL + 1i*Zo*tan(bd1))./(Zo + 1i*ZL*tan(bd1));
Yin1 = 1./Zin1;

Yopen1 = 1i*Yo*tan(bl1);

YL1 = Yin1 + Yopen1;
ZL1 = 1./YL1;

Zin2 = Zo*(ZL1 + 1i*Zo*tan(bd2))./(Zo + 1i*ZL1.*tan(bd2));
Yin2 = 1./Zin2;

Yopen2 = 1i*Yo*tan(bl2);

YL2 = Yin2 + Yopen2;
ZL2 = 1./YL2;

Zin3 = Zo*(ZL2 + 1i*Zo*tan(bd3))./(Zo + 1i*ZL2.*tan(bd3));
Yin3 = 1./Zin3;

Yopen3 = 1i*Yo*tan(bl3);

Yin = Yin3 + Yopen3;
Zin = 1./Yin;

% Finally
S11 = abs((Zin - Zo)./(Zin + Zo)); 

% Plots
figure
plot(normf,S11);
title("Reflection Coefficient Magnitude")
hold on
xlabel("Frequency");
ylabel("|Γ|","Rotation",0);
hold off

end