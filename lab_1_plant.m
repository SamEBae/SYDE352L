% [data_vector_dot, time_vector_dot] = differ(data_vector, time_vector, freq)
% anything in milli, should mutiply by 10^-3
s = tf('s');

Kt = 39.3 * 10^(-3); % 39.3 mNm/A
Ke = 1/25.446900494; % 243 rpm/V = 25.446900494 rads/s
Jr = 28;
% !!!!
Jc = -1;    % needs to be computed empirically
% !!!!

Je = 1.7;
La = 1.54 * 10^(-3);  % 1.54 mH
Ra = 7.94;  % 7.94 Ω
b = 1.38 * 10^(-6); % 1.38 x 10 ^(-6) Nms /rad
Jt = Jr + Jc + Je;

Ps = Kt/( (Jt * La)* s^3 + (Jt*Ra+b*La)*s^2  + (b*Ra+Kt*Ke)*s ); %