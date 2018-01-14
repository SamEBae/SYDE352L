% This script calculates the average inertia (J) in kg*m^2 from Start_Time  
% in seconds to End_Time in seconds.

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% User defined specifications. Add yours now.
% Torque constant in N*m/A
Kt = 39.3 * 10^(-3);

% Back EMF coefficient in V*s/rad
Ke = 1/243;

% Viscous friction coefficient in N*m*s/rad    
b = 1.38 * 10^(-6); % 1.38 x 10 ^(-6) Nms /rad

% Total resistance in Ohms
RT = 9.97;

% Voltage input in Volts
V_in = 12;                

% Time at which J average calculation starts in seconds,
% calculate by looking at steady-state part of the angular Velocity
Start_Time = 0;          

% Time at which J average calculation ends in seconds
End_Time = 0;            
Freq = Freq;             % Data capture frequency

% Because you might have used different vector names, add yours now.
Ydot = data_vector_dot;                % User enters their Angular Velocity data vector
Ydoubledot = data_vector_dot_dot;      % User enters their Angular Acceleration data vector
Tdoubledot = time_vector_dot_dot;      % User enters their Angular Acceleration time vector    

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

L = length(Ydoubledot);

% Calculate the combined inertia (J)
for i=1:L
    
    Va(i) = R(i)*V_in/230;               % Calculate the Applied Voltage in V
    I(i) = (Va(i) - Ke*Ydot(i))/RT;      % Calculate the Average Current in A
    J(i) = (Kt*I(i) - b*Ydot(i))/Ydoubledot(i);   % Calculate the Inertia is kg*m^2
    
end

% Calculate Start and End data points
Start_Point = Start_Time*Freq;
End_Point = End_Time*Freq;
if(End_Point > L)
    End_Point = L;
end

% Calculate average over specified time interval
J_total = 0;

for i=Start_Point:End_Point
    
    J_total = J_total + J(i);
    
end

J_avg = J_total/(End_Point-Start_Point)

% Figure 2-21
% Plot J versus time
figure;
plot(Tdoubledot,J)
title('Combined Inertia - J vs. Time');
ylabel('Combined Inertia - J (kgm^2)');
xlabel('Time (seconds)');
grid on;