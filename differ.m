% This function takes in a vector (data_vector) and numerically 
% differentiates the vector data based on a simple two-point estimation.  
% To do so it requires the frequency (Freq) of the data sampling to 
% determine the period or delta time.  Because this calculation uses a data
% point one delta time in the future to calculate the present slope, the 
% size of the output vector (data_vector_dot) has been reduced by 1 sample.  
% This reduction in vector size means we need to create a new vector 
% (time_vector_dot) based on the original (time_vector) that has also been
% reduced in size by one sample so that we can plot and perform 
% calculations with vectors of the same size.

function [data_vector_dot, time_vector_dot] = differ(data_vector, time_vector, freq)

L = length(data_vector);

% Calculate differentiated output 
for i=1:(L-1)
  data_vector_dot(i) = (data_vector(i+1) - data_vector(i))*freq;
  time_vector_dot(i) = time_vector(i);
end   

% Plot differeniated data versus time
figure;
plot(time_vector_dot,data_vector_dot)
title('Differentiated Data vs. Time');
ylabel('Differentiated Data');
xlabel('Time (seconds)');
grid on;



