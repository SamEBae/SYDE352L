% custom code starts here
% normalized cutoff frequencies of 0.2, 0.1 and 0.02
%{
f1_n = 0.2;
f2_n = 0.1;
f3_n = 0.02;
vector_in = data_vector_dot;

[v_f1, v_f2, v_f3] = data_filter(f1_n, f2_n, f3_n, vector_in);
%}

% Figure 2-17
% filter the vector_in
% pick the closest filter 1 or filter 2, CHRIS SAID 2 because who knows
% diffrentiate filter 1 or 2 to get acceleration
% test different frequencies
% then put back into filter: [a_f1, a_f2, a_f3] = data_filter(f1_n, f2_n, f3_n, v_f2);

%[a_f1, time_vector] = differ(v_f1, time_vector, my_freq)
%{
figure;
plot(time_vector, a_f2)
grid on
title('Differentiated Data vs Time');
ylabel('Differentiated Data');
xlabel('Time (seconds)');
%}

f1_n_1 = 0.08;
f1_n_2 = 0.04;
f1_n_3 = 0.02;

[a_f1_filtered_1, a_f1_filtered_2, a_f1_filtered_3] = data_filter(f1_n_1, f1_n_2, f1_n_3, a_f1);
%}
%{
[vector_in_dot, time_vector] = differ(vector_in, time_vector, my_freq)
[v_f1_dot, time_vector] = differ(v_f1, time_vector, my_freq)
[v_f2_dot, time_vector] = differ(v_f2, time_vector, my_freq)
[v_f3_dot, time_vector] = differ(v_f3, time_vector, my_freq)

sample_vector = linspace(1, length(vector_in), length(vector_in)); % Calculate sample vector 

% Figure 2-18
figure;
plot(time_vector,vector_in_dot)
title('Combined Inertia - J vs. Time');
ylabel('Combined Inertia - J (kgm^2)');
xlabel('Time (seconds)');
grid on;


% Figure 2-19/2-20
figure;
plot(sample_vector, vector_in);
plot(sample_vector, v_f1_dot);
plot(sample_vector, v_f2_dot);
plot(sample_vector, v_f3_dot);
legend('Unfiltered differentiated','Filter 1 differentiated','Filter 2 differentiated','Filter 3 differentiated');
ylabel('Input Data');
xlabel('Samples');
grid on;


% try multiple frequencies until one looks smooth and not shifted
%}