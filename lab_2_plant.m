s = tf('s');
Ps = 37.81/(s*(2.77*s + 1));
Ps = 13.65/((s+0.361)*s);
I_s = 651.8784;
H_s = 651.8784;
K = 13.65;
K_p1 = 0.001;
sigma_1 = 0.361;
sigma_2= 0;


Transfer_s = (I_s*K_p1*Ps)/ (1+K_p1*Ps*H_s);
% Transfer_s
% Transfer_normalized = 
Tcl = (K_p1 * K * I_s)/(s^2 + (sigma_1 + sigma_2)*s + (sigma_1*sigma_2 + K_p1*K*H_s));
Tcl