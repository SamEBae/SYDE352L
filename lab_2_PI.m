% part A: modelling
% not sure how to calc t_r_PD, t_s_PD 
t_r_PI = 1; % TODO change me
t_s_PI = 1; % TODO change me

% NOT SURE IF THIS IS RIGHT?? d_0_PD = 0;

d_0_PI = 0.365;
P_s_PI = K / (s+d_0_PI);

% case 1: 0 <= zetta <= 1l
a_PI = real(pole_Ps_PI);
b_PI = imag(pole_Ps_PI);
% case 2: zetta > 1

Kp_PI = (2*a_PI - d_0)/(Hs*K);
Ki_PI = (a_PI^2-b_PI^2)/(Hs*K);
Cs_PI = Kp_PI + K_I/s;


a_desired_PI = 1; %TODO change me
b_desired_PI = 1; %TODO change me



% part B: desired poles
zetta_fixed = 0.7;