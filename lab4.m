s = tf('s');
I_s = 651.9;
H_s = 651.9;
Ps = 17.59/(s^2+2.2*s-42.31);


Kp = 0.37034;
Kp2 =3.91*(10-3);
Cs_P1 = Kp;
Cs_P2 = Kp2;

Tcl_P2 = (I_s*Cs_P2*Ps)/(1+H_s*Cs_P2*Ps);
period_P2 = 1/(bandwidth(Tcl_P2)/(2*pi))/10;
Tcl_P1 = (I_s*Cs_P1*Ps)/(1+H_s*Cs_P1*Ps);
period_P1 = 1/(bandwidth(Tcl_P1)/(2*pi))/10;

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
period_PD1 = 1/(bandwidth(Tcl_PD1)/(2*pi))/10;

Kd_PD2 = 45/4204;
Cs_PD2 = Kp+Kd_PD2*s;
Tcl_PD2 = (I_s*Cs_PD2*Ps)/(1+H_s*Cs_PD2*Ps);
period_PD2 = 1/(bandwidth(Tcl_PD2)/(2*pi))/10;

% period_PD1


PID1_t3 =  -42.31+651.9*(17.59)*(0.37034);
PID1_t4 = (651.9)*(17.59);
T_PID1 = s^2/(s^3+2.2*s^2+4204*s+11467);
% rlocus(T_PID1);
% Tcl_PID1 = (I_s*Cs_PD1*Ps)/(1+H_s*Cs_PD1*Ps);

T_PID2 = 1/(s^3+116.8692*s^2+4204*s);
% rlocus(T_PID2);
Cs_PID1 = 0.37034 + 0.0305/s + 0.0100*s;
Tcl_PD1 = (I_s*Cs_PID1*Ps)/(1+H_s*Cs_PID1*Ps);

period_PID2 = 1/(bandwidth(Tcl_PD1)/(2*pi))/10;


T_PID3 = 1/(s^3+116.8692*s^2+4204*s);
Cs_PID3 = 0.37034 + 0.0043/s + 0.0157*s;
Tcl_PID3 = (I_s*Cs_PID3*Ps)/(1+H_s*Cs_PID3*Ps);
period_PID3 = 1/(bandwidth(Tcl_PID3)/(2*pi))/10;

% rlocus(T_PD1);
% hold on;
% plot(-23, 23.4647, 'kx', 'Markersize', 10);
% plot(-23, -23.4647, 'kx', 'Markersize', 10);
% x = -400:0.01:0
% y1 = (23.4647/-23)*x;
% y2 = (-23.4647/-23)*x;
% plot(x, y1,'k');
% plot(x, y2, 'k');
% hold off;
% period_PD2

Cs_PID_final = 0.37034 + 0.1/s + 0.0157*s;
Tcl_PID_final = (I_s*Cs_PID_final*Ps)/(1+H_s*Cs_PID_final*Ps);

bandwidth_final_Rads = bandwidth(Tcl_PID_final);
bandwidth_final_Hz = bandwidth(Tcl_PID_final)/(2*pi);
bandwidth_final_Hz_rule = bandwidth_final_Hz*10;
period_PID_final = 1/(bandwidth(Tcl_PID_final)/(2*pi))/10;

% open loop:
margin(Cs_PID_final*Ps*I_s);
% bode(Cs_PID_final*Ps*I_s);