s = tf('s');
K = 38.1924;
Tau = 2.132;
Ps = K/(s*(Tau*s+1));
%       38.19
%   -------------
%   2.132 s^2 + s

Ps_norm = (17.9128)/(s^2+0.4690*s);

% controller stuff
% TODO: change me
Kp = 0.02773998321;
Kd_PD1 = 0.0025374;
Kd_PD1 = 0.0027374;

Cs_PD1 = Kp+Kd_PD1*s;
I_s = 651.9;
H_s = I_s;

% transfer function
Tcl_PD1 = (I_s*Cs_PD1*Ps_norm)/(1+H_s*Cs_PD1*Ps_norm);
filter_coeff =  bandwidth(Tcl_PD1)*20;
period_PD1 = (1/(bandwidth(Tcl_PD1)/(2*pi)))/10;
[period_PD1, filter_coeff]

%%
hold on;
rlocus(Tcl_PD1);
x = -400:0.01:0;
y1 = (1/1.553176461)*x;
y2 = -y1;
plot(x, y1,'k');
plot(x, y2, 'k');
hold off;

%% 
Kp2 = 0.6934995803;
Kd_PD2 = 0.0128477;

Cs_PD2 = Kp2+Kd_PD2*s;
I_s = 651.9;
H_s = I_s;

% transfer function
Tcl_PD2 = (I_s*Cs_PD2*Ps_norm)/(1+H_s*Cs_PD2*Ps_norm);
filter_coeff2 =  bandwidth(Tcl_PD2)*20;
period_PD2 = (1/(bandwidth(Tcl_PD2)/(2*pi)))/10;
disp([period_PD2, filter_coeff2])

%%
hold on;
rlocus(Tcl_PD2);
x = -400:0.01:0;
y1 = (1/1.553176461)*x;
y2 = -y1;
plot(x, y1,'k');
plot(x, y2, 'k');
hold off;


%% 
Kp3 = 0.04334372377;
Kd_PD3 = 0.0031818;

Cs_PD3 = Kp3+Kd_PD3*s;
I_s = 651.9;
H_s = I_s;

% transfer function
Tcl_PD3 = (I_s*Cs_PD3*Ps_norm)/(1+H_s*Cs_PD3*Ps_norm);
filter_coeff3 =  bandwidth(Tcl_PD3)*20;
period_PD3 = (1/(bandwidth(Tcl_PD3)/(2*pi)))/10;

%%
hold on;
rlocus(Tcl_PD3);
x = -400:0.01:0;
y1 = (1/1.553176461)*x;
y2 = -y1;
plot(x, y1,'k');
plot(x, y2, 'k');
hold off;