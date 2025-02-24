% Intialization of variables
fo = 10^9;
N = 201;
Vs = 1;
Zo = 50;
ZL = 10 + 15i;
Zg = 50 - 40i;  Rg = real(Zg);  Xg = imag(Zg);
C3 = 3.02*10^(-12);
C4 = 2.65*10^(-12);

% f vector
f = 0:(2*fo/N):(2*fo);

% β(f)*l
bl3 = 0.088*pi*f/fo;
bl4 = 0.28*pi*f/fo;

% Choose 0 for parallel and 1 for in series
option = 1;

if option == 0 % Capacitor parallel to the entrance
    Za = Zo*(ZL + 1i*Zo*tan(bl3))./(Zo + 1i*ZL*tan(bl3));
    Ya = 1./Za;

    Zc = -1i./(2*pi*f*C3);
    Yc = 1./Zc;

    Yin = Ya + Yc;
    Zin = 1./Yin;
    Rin = real(Zin);
    Xin = imag(Zin);
elseif option == 1 % Capacitor in series to the entrance 
    Za = Zo*(ZL + 1i*Zo*tan(bl4))./(Zo + 1i*ZL*tan(bl4));

    Zc = -1i./(2*pi*f*C4);

    Zin = Za + Zc;
    Rin = real(Zin);
    Xin = imag(Zin);
end

P = (abs(Vs)^2/2)*(Rin./((Rin+Rg).^2+(Xin+Xg).^2));

% Plot
figure
plot(f,P);
title("Power")
hold on
xlabel("Frequency");
ylabel("P","Rotation",0);
hold off




