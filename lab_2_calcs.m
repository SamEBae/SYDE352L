

% run after lab_2_calcs
t_r = 1; % TODO: change
w_n = 1.8/t_r;

K_p = (w_n)^2-sigma_1*sigma_2/(K*H_s);

% ----- next part
% overshoot results in damping ratio:
zetta_fixed = 0.7;

w_n_fixed_zetta = (sigma_1+sigma_2)/(2*zetta_fixed);
K_p_fixed_zetta = ((w_n)^2-sigma_1*sigma_2)/(K*H_s);

% this K_p meets 5% overshoot but not target rise
% plug into simulink to test

desired_root_1 = -zetta_fixed*w_n + j*w_n*(1-zetta_fixed^2)^0.5;
desired_root_2 = -zetta_fixed*w_n + j*w_n*(1-zetta_fixed^2)^0.5;

% probably will see that desired_roots are not even close to sigma_1 and
% sigma_2 , our real roots
