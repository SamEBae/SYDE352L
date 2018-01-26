% -----------------------
% part A: modelling
% not sure how to calc t_r_PD, t_s_PD 
t_r_PD = 1; % TODO change me
t_s_PD = 1; % TODO change me

zetta = 2.55555556*(t_r_PD/t_s_PD);

d_1_PD = 0.365;
d_0_PD = 0;

Ps_PD = K/(s^2 + d_1_PD*s+ d_0_PD);
pole_Ps_PD = pole(Ps_PD);

H1 = H_s;

% case 1: 0 <= zetta <= 1l
a_PD = real(pole_Ps_PD);
b_PD = imag(pole_Ps_PD);
% case 2: zetta > 1
    % pole = -a +- b

Kd_PD = (2*a_PD-d_1_PD)/(H1*K);
Kp_PD = (a_PD^2+b_PD^2-d_0_PD)/(H1*K);

Cs_PD = Kd_PD*Kp_PD*s;

% PART B: --------------
% calculate desired poles

% rise
w_n_PD = 25; % TODO change me, adjust to liking
zetta_fixed = 0.7;
sigma = zetta_fixed * w_n_PD;
poles_desired_PD = [1 1] * zetta*w_n_PD + [-1 1] * w_n*(1-zetta_fixed^2)^0.5;

a_desired_PD = 1; % TODO change me
b_desired_PD = 1; % TODO change me

K_d_desired_PD = (2*a_desired_PD-d_1)/(H1*K);
K_p_desired_PD = (a_desired_PD^2+b_desired_PD^2-d_0)/(H1*K);

% I think we need to add C(s) in here
Tcl_PD_desired = (I_s*K_p_desired_PD*Ps)/(1+K_p_desired_PD*Ps*H_s);
% rootlocus