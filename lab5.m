s = tf('s');

%d_n = ; -- measure and enter
% tube_diameter = ;
% d_t = ; -- measure and enter
% tube locations / arc length
% r = ; 
%theta = acos( ((d_t/2-d_n/2)^2-2*r^2) / (-2*r^2));
% rise time:

step_size = pi/6; % assuming 30 deg
t_travel = 0.03;
w_n 	 = t_travel*step_size;
t_r 	 = 1.8/w_n;

%
    % plant stuff:
    % TODO: fill this in from collect.m results
    Ps = 13.65/((s+0.361)*s);
     
    % controller stuff
    % TODO: change me
    Kp = 0.03641137847;
    Kd_PD1 = 0.0033427;
    
    Cs_PD1 = Kp+Kd_PD1*s;
    I_s = 651.9;
    H_s = I_s;
    
    % transfer function
    Tcl_PD1 = (I_s*Cs_PD1*Ps)/(1+H_s*Cs_PD1*Ps);
    filter_coeff =  bandwidth(Tcl_PD1)*20;
    period_PD1 = (1/(bandwidth(Tcl_PD1)/(2*pi)))/10;
    [period_PD1, filter_coeff]
    