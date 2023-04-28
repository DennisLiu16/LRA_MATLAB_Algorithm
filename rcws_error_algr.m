clc;clear;close all;
%%%%%%%%%%%%%%%%%
%%% parameter %%%
%%%%%%%%%%%%%%%%%
Fs = 4000;
f_base = 170;
A_base = 0;

x_desired_mod = [];
x_base_adjusted = [];

% TODO: increase this term to 1000 to make sure controller is stable
desired_wave_num = 10;

snr_db = 30;
snr = 10^(snr_db/10);

% Kp = 1.01;
% Ki = 0.0025;
% Kd = 0.001;
Kp = 1.5;
Ki = 0.005;
Kd = 0.001;
integral = 0;
previous_error = 0;

%%%%%%%%%%%%%%%%%
%%%    init   %%%
%%%%%%%%%%%%%%%%%
% init desired wave with n different waves, n = desired_wave_num
for i = 1:desired_wave_num
    % Generate random frequency and amplitude for x_desired_mod
    [rand_freq, rand_amp] = generate_random_freq_amp();

    % Generate x_desired_mod using random frequency and amplitude
    % /2 for full wave rectified
    x_desired_mod_temp = generate_one_period_signal(Fs, rand_amp, rand_freq / 2);
    half_len = floor(length(x_desired_mod_temp)/2);
    x_desired_mod_temp = abs(x_desired_mod_temp(1:half_len));

    % Transmit to 
    
    % Append the generated waveform to x_desired_mod
    x_desired_mod = [x_desired_mod, x_desired_mod_temp];
end

%%%%%%%%%%%%%%%%%
%%% parameter %%%
%%%%%%%%%%%%%%%%%
num_periods = floor(length(x_desired_mod)/(Fs/f_base));
base_period_length = floor(Fs/f_base);
i = 1;

%%%%%%%%%%%%%%%%%
%%%    main   %%%
%%%%%%%%%%%%%%%%%
while true
    start_index = (i-1)*base_period_length + 1;
    end_index = i*base_period_length;
    
    % Check if there is enough space in x_desired_mod to fit another base period
    if end_index > length(x_desired_mod)
        break;
    end

    x_base_temp = generate_one_period_signal(Fs, A_base, f_base);
    x_base_temp = add_noise(x_base_temp, snr_db);

    [~, max_index] = max(x_base_temp);

    error = x_base_temp(max_index) - x_desired_mod(start_index + max_index - 1);

    derivative_errors(i) = error - previous_error;

    [A_base_adjustment, integral, previous_error] = pid_controller(error, integral, previous_error, Kp, Ki, Kd);

    A_base = A_base - A_base_adjustment;

    x_base_adjusted(start_index:end_index) = x_base_temp;

    errors(i) = error;
    integral_errors(i) = integral;
    

    i = i + 1;
end

figure;
Ts = 1 / Fs;
t_desired = linspace(0, (length(x_desired_mod)-1)*Ts, length(x_desired_mod));
t_adjusted = linspace(0, (length(x_base_adjusted)-1)*Ts, length(x_base_adjusted));
% TODO: modified t_error seperation, it's not linear
t_error = linspace(0, (length(errors)-1) / f_base, length(errors));

subplot(2,1,1);
hold on;
plot(t_desired, x_desired_mod, 'r', 'LineWidth',2);
plot(t_adjusted, x_base_adjusted, 'b');
plot(t_desired, -x_desired_mod, 'r', 'LineWidth',2);
legend("Target", "LRA signals")
title("Signal")

hold off;
subplot(2,1,2);
hold on;
plot(t_error, errors, 'LineWidth',2);
plot(t_error, integral_errors, 'LineWidth',2);
plot(t_error, derivative_errors, 'LineWidth',2);
legend("Error", "Integral Error", "Derivative Error")
title("Error Curve")
hold off;


%%%%%%%%%%%%%%%%%
%%% functions %%%
%%%%%%%%%%%%%%%%%

% TODO: fix the hard-coding part
function [rand_freq, rand_amp] = generate_random_freq_amp()
    rand_freq = 0.5 + 19.5 * rand(); % Random frequency in the range of 0.5 to 10 Hz
    rand_amp = 0.5 + 1.5 * rand(); % Random amplitude in the range of 0.5 to 2
end

function output = generate_one_period_signal(Fs, A, f)
    T = 1 / Fs;
    num = floor(Fs / f);
    t = (0:num-1) * T;
    output = A * sin(2 * pi * f * t);
end

function x_noisy = add_noise(x_original, snr_db)
    snr = 10^(snr_db/10);
    x_original_power = var(x_original);
    noise_power = x_original_power / snr;
    noise_power_dB = 10 * log10(noise_power);
    noise = wgn(1, length(x_original), noise_power_dB, 'dBW');
    x_noisy = x_original + noise;
end

function [A_base_adjustment, integral, previous_error] = pid_controller(error, integral, previous_error, Kp, Ki, Kd)
    integral = integral + error;
    derivative = error - previous_error;
   
    % limit intergral
    integral_max = 2;
    integral_min = -2;
    if integral > integral_max
        integral = integral_max;
    elseif integral < integral_min
         integral = integral_min;
    end

    % limit derivative
    derivative_max = 2;
    derivative_min = -2;
    if derivative > derivative_max
       derivative = derivative_max;
    elseif derivative < derivative_min
       derivative = derivative_min;
    end
    

    A_base_adjustment = Kp * error + Ki * integral + Kd * derivative;
    previous_error = error;
end