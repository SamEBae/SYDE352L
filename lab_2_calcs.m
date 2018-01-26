% run after lab_2_calcs
t_r = 1; % TODO: input
w_n = 1.8/t_r;

K_p = (w_n)^2-sigma_1*sigma_2/(K*H_s);

% ----- next part
% overshoot results in damping ratio:
zetta = 0.7;

w_n_fixed_zetta = (sigma_1+sigma_2)/(2*zetta);
K_p_fixed_zetta = (w_n)^2-sigma_1*sigma_2/(K*H_s);

% this K_p meets 5% overshoot but not target rise
% plug into simulink to test