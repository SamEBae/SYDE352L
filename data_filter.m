% This function uses three user defined values (f1_n, f2_n, f3_n) that
% represent the normalized corner frequency of the low-pass filter to
% filter vector_in.  The output of this function is 3 vectors 
% (v_f1, v_f2, v_f3) of filtered data.  A plot is created that compares 
% these new vectors with the origial vector_in.

function [v_f1, v_f2, v_f3] = data_filter(f1_n, f2_n, f3_n, vector_in)

% Apply low pass filter with cutoff frequency of f1 Hz
[b, a]= butter(2, f1_n);
v_f1 = filter(b, a, vector_in);

% Apply low pass filter with cutoff frequency of f2 Hz
[b, a]= butter(2, f2_n);
v_f2 = filter(b, a, vector_in);

% Apply low pass filter with cutoff frequency of f3 Hz
[b, a]= butter(2, f3_n);
v_f3 = filter(b, a, vector_in);

% Create X axis vector for plotting purposes
L = length(vector_in);       % Calculate length of vector_in
X = linspace(1, L, L);       % Calculate X vector 

% Plot orignial vector plus the 3 filtered vectors for comparison purposes
figure;
hold on;
grid on;
plot(X, vector_in);
plot(X, v_f1);
plot(X, v_f2);
plot(X, v_f3);
legend('Unfiltered','Filter 1','Filter 2','Filter 3');
ylabel('Input Data');
xlabel('Samples');
hold off;

