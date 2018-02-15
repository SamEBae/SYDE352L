% Y(s): 
s = tf('s');
Kt= 24.1 * 10^(-3); % 39.3 mNm/A
Ke= 0.02411438531; % 243 rpm/V = 25.446900494 rads/s
b = 2.8816206319953345 * 10^(-6);
Ra= 3.02;  % TODO: CHANGE ME
m = 0.0135;
g = 9.81;
ia= 30.9 * 10^(-3);
l = 0.162;  % TODO: CHANGE ME
Jt = 4.44*10^(-4); % TODO: CHANGE ME
Y_s = (Kt*ia)/(Jt*s^3+b*s^2+m*g*l*s);
V_s = (Y_s*s*Ke) + ((Ra*ia)/s);
%((-Ke*Kt*ia*s)-Ra*ia*(Jt*s^2+b*s+m*g*l))/ (s*(Jt*s^2+b*s+m*g*l));
T_s = Y_s/V_s;
T_s_2 = Kt/(Ra*Jt*s^2+(Ra*b+Ke*Kt)*s+Ra*m*g*l);
T_s_2