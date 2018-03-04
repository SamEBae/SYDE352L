s = tf('s');
I_s = 651.9;
H_s = 651.9;
Ps = 17.59/(s^2+2.2*s-42.31);

Kp = 0.37034;
% rlocus(Ps);
% PD
% calculating  Kd:
% t3= (-42.31+651.9*(0.37034)*(17.59));
% t2= 2.2 + 651.9*(17.59);
T_PD1 = s/(s^2+2.20*s+4204);
% rlocus(T_PD1);
Kd_PD1 = 46.7/4204;
Cs_PD1 = Kp+Kd_PD1*s;
Tcl_PD1 = (I_s*Cs_PD1*Ps)/(1+H_s*Cs_PD1*Ps);
bandwidth_PD1 = bandwidth(Tcl_PD1)/(2*pi);
period_PD1 = 1/(bandwidth(Tcl_PD1)/(2*pi))/10;
period_PD1