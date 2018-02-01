zetta = 0.6901;
t_s_desired = 0.2;
sigma = 4.6/t_s_desired;
w_n = sigma/zetta;

K_p_PD_trial6 = 0.06339105587;
K_d_PD_trial6 = 0.003643317527;

Cs_PD_trial6 = K_p_PD_trial6 + (K_d_PD_trial6)*s;
Tcl_PD_trial6=(I_s*Cs_PD_trial6*Ps)/(1+Cs_PD_trial6*Ps*H_s);

% my bandwidth: 50.3850 (rad/s
% bandwidthin Hertz:  8.0190
bandwidth_PD = bandwidth(Tcl_PD_trial6)/(2*pi);
period_trial6 = 1/(bandwidth(Tcl_PD_trial6)/(2*pi));
period_trial6

% N_PD_for_arduino = bandwidth_PD * 20;
% N_PD_for_arduino
% Chris said N of 1000 for all tests with derivative term
