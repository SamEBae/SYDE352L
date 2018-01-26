% [data_vector_dot, time_vector_dot] = differ(data_vector, time_vector, freq)
% anything in milli, should mutiply by 10^-3
s = tf('s');

Kt = 24.1 * 10^(-3); % 39.3 mNm/A
Ke = 0.02411438531; % 243 rpm/V = 25.446900494 rads/s

%La =  0.416 * 10^(-3);  % 1.54 mH
La = 0;
Ra =  3.02;  % 7.94 Ω
b = 2.8816206319953345 * 10^(-6);
Jt = J_avg;
%Jt = Jr + Jc + Je;

Ps2 = Kt/( (Jt * La)* s^3 + (Jt*Ra+b*La)*s^2  + (b*Ra+Kt*Ke)*s ); %
PsFinal1 = 0.0241/( 0.001691*s^2 + 0.0005899*s);
PsFinal2 = 37.81/(s*(2.77*s+1));