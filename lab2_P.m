% plant function from previous lab:
Ps = 13.65/((s+0.361)*s);
I_s = 651.8784;
H_s = 651.8784;
K = 13.65;
sigma_1 = 0.361;
sigma_2 = 0;

% -----------------------
% part A: modelling
K_p_trial = 0.001;
Tcl_P_trial = (I_s*K_p_trial*Ps)/(1+K_p_trial*Ps*H_s); % [eqn 3-1]

% do stuff with [Fig 3-4] and [Fig 3-5]
% this lets us compute
t_r_P = 1; % TODO
t_s_P = 1; % TODO

zetta = 2.55555556*(t_r_P/t_s_P);

% calculate rads for the pole [eqn 3-23]
w_n_P = 1.8/t_r_P;

% calculate K_p_P [eqn 3-24]
K_p_P = ((w_n_P)^2-sigma_1*sigma_2)/(K*H_s);

% [Fig 3-10]

% part B: desired poles for 
% ------------------ 5% overshoot
zetta_fixed = 0.7;

w_n_P_fixed_zetta = (sigma_1+sigma_2)/(2*zetta_fixed);
K_p_fixed_zetta   = ((w_n_P)^2-sigma_1*sigma_2)/(K*H_s);

Tcl_P_desired = (I_s*K_p_fixed_zetta*Ps)/(1+K_p_fixed_zetta*Ps*H_s);

% [Figure 3-12]
% plug into simulink to test:
    % this K_p meets 5% overshoot but not target risem
desired_root_1_P = -zetta_fixed*w_n_P + j*w_n_P*(1-zetta_fixed^2)^0.5;
desired_root_2_P = -zetta_fixed*w_n_P + j*w_n_P*(1-zetta_fixed^2)^0.5;

% probably will see that desired_roots are not even close to sigma_1 and
% sigma_2 , our real roots. So reject P controller as a possibility

% ------------------ 0.2 ts
