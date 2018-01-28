zetta = 0.6901;
t_s_desired = 0.2;
sigma = 4.6/t_s_desired;
w_n = sigma/zetta;

K_p_PD_trial4 = 0.07023939708;
K_d_PD_trial4 = 0.003837206358;

Cs_PD_trial4 = K_p_PD_trial4 + (K_d_PD_trial4)*s;
Tcl_PD_trial4=(I_s*Cs_PD_trial4*Ps)/(1+Cs_PD_trial4*Ps*H_s);

% my bandwidth: 50.3850 (rad/s
% bandwidthin Hertz:  8.0190
bandwidth_PD = bandwidth(Tcl_PD_trial4)/(2*pi);
period_trial4 = 1/(bandwidth(Tcl_PD_trial4)/(2*pi));
period_trial4

N_PD_for_arduino = bandwidth_PD * 20;
N_PD_for_arduino
% Chris said N of 1000 for all tests with derivative term
