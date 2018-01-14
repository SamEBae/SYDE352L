% This script collects the data sent from the Arduino Mega.  It collects 
% the Loop Frequency (Freq) in Hz, the Duration of the Test (Time) 
% in seconds, the Reference Input vector (R) in units dependent on the type
% of test and the Output vector (Y) in radians. 

% Setup comport
comport = input('Which COM port is the Electronic Hardware Module attached to? [i.e. COM1, COM2, ... etc.]: ','s');
arduino = serial(comport,'BaudRate',115200); 
fopen(arduino);
 
disp('Press button to start data capture.')

% Read in inital values
Freq = fscanf(arduino,'%f');      % Read in Loop Frequency in Hz
Time = fscanf(arduino,'%f');      % Read in Duration of Test in seconds
I_Gain = fscanf(arduino,'%f');    % Read in Input Filter gain
cnt_max = round(Freq*Time);       % Calculate total # of time steps
T = linspace(0,Time,cnt_max);     % Setup Time Vector

% Read in loop values
for i=1:cnt_max
   
  R(i)=fscanf(arduino,'%f');      % Read in reference input
  Y(i)=fscanf(arduino,'%f');      % Read in output in encoder counts
  
end
   
% Close comport
fclose(arduino);

disp('Data capture complete.')

% Convert Y from encoder counts to radians where I_Gain is the Input Filter 
% gain in encoder counts per radian
Y = Y/I_Gain;

% Plot captured data
figure;
plot(T,R)
grid on
title('Reference Input vs. Time');
ylabel('Reference Input');
xlabel('Time (seconds)');

figure;
plot(T,Y)
grid on
title('Output vs. Time');
ylabel('Angular Position (radians)');
xlabel('Time (seconds)');

disp('Plotting complete.')


% custom code starts here
my_freq = Freq;
time_vector = T;
data_vector = Y;

[data_vector_dot, time_vector_dot] = differ(data_vector, time_vector, my_freq)
[data_vector_dot_dot, time_vector_dot_dot] = differ(data_vector_dot, time_vector, my_freq)

% figure 2-14 
figure;
plot(time_vector_dot, data_vector_dot)
grid on
title('Output vs. Time');
ylabel('Angular Velocity (radians)');
xlabel('Time (seconds)');

% figure 2-15
figure;
plot(time_vector_dot_dot, data_vector_dot_dot)
grid on
title('Output vs. Time');
ylabel('Angular Acceleration (radians)');
xlabel('Time (seconds)');
