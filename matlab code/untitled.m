%% Vehicle Dynamics Simulation under Constant Tractive Effort (F_TE)
clear; clc; close all;

%% 1. Vehicle & Environmental Parameters
m = 1200;               % Vehicle mass (kg)
G = 10;                 % Gear reduction ratio
r = 0.2;                % Wheel radius (m)
eta_G = 0.9;            % Gearbox efficiency
C_d = 0.26;             % Aerodynamic drag coefficient
A = 2.2;                % Frontal area (m^2)
mu_rr = 0.01;           % Rolling resistance coefficient
rho = 1.25;             % Air density (kg/m^3)
g = 9.81;               % Acceleration due to gravity (m/s^2)
theta = 0;              % Incline angle (rad)
T_max = 40;             % Constant motor input torque (Nm)

% Equivalent mass accounting for rotational inertia (5% of vehicle mass)
m1 = 0.05 * m;
m_eq = m + m1;

%% 2. Force & Constant Calculations
% Constant Tractive Force
F_TE = (G / r) * T_max * eta_G;

% Resistive Forces
F_rr = mu_rr * m * g * cos(theta);
F_g  = m * g * sin(theta);

% Constant Factors K1 and K2
K1 = (0.5 * rho * C_d * A) / m_eq;
K2 = (F_TE - F_rr - F_g) / m_eq;

% Analytical Performance Benchmarks
v_T = sqrt(K2 / K1);                   % Terminal velocity (m/s)
v_T_kmh = v_T * 3.6;                   % Terminal velocity (km/h)
t_f = 2.3 / (K1 * v_T);                % Time to reach 98% of v_T (s)
P_T = (F_TE * v_T) / 1000;             % Terminal Power (kW)

fprintf('--- Analytical Benchmark Values ---\n');
fprintf('Terminal Velocity (v_T): %.2f m/s (%.2f km/h)\n', v_T, v_T_kmh);
fprintf('Time to 98%% of v_T (t_f): %.2f seconds\n', t_f);
fprintf('Peak Tractive Power: %.2f kW\n\n', P_T);

%% 3. Numerical Simulation (Euler Integration)
dt = 0.1;                              % Time step (seconds)
t_total = 150;                         % Total simulation duration (seconds)
time = 0:dt:t_total;
N = length(time);

% Pre-allocating arrays
v = zeros(1, N);                       % Velocity (m/s)
d = zeros(1, N);                       % Distance (m)
P = zeros(1, N);                       % Power (kW)

for n = 1:(N - 1)
    % Derivative: dv/dt = -K1*v^2 + K2
    dv_dt = -K1 * (v(n)^2) + K2;
    
    % Update velocity and distance
    v(n+1) = v(n) + dv_dt * dt;
    d(n+1) = d(n) + v(n) * dt;
    
    % Instantaneous Power
    P(n) = (F_TE * v(n)) / 1000;       % kW
end
P(N) = (F_TE * v(N)) / 1000;

%% 4. Graphical Plots
figure('Color', [1 1 1], 'Position', [100, 100, 1000, 700]);

% Velocity Plot
subplot(3, 1, 1);
plot(time, v * 3.6, 'b', 'LineWidth', 1.8);
grid on;
xlabel('Time (s)');
ylabel('Velocity (km/h)');
title('Vehicle Velocity vs. Time');

yline(v_T_kmh, '--r');
ylim([0 270]);

text(105, v_T_kmh + 5, sprintf('v_T ≈ %.1f km/h', v_T_kmh), ...
    'Color', 'r', 'FontSize', 11, 'FontWeight', 'bold');

% Distance Plot
subplot(3, 1, 2);
plot(time, d / 1000, 'g', 'LineWidth', 1.8);
grid on;
xlabel('Time (s)');
ylabel('Distance (km)');
title('Distance Traveled vs. Time');

% Power Plot
subplot(3, 1, 3);
plot(time, P, 'm', 'LineWidth', 1.8);
grid on;
xlabel('Time (s)');
ylabel('Power (kW)');
title('Instantaneous Tractive Power vs. Time');
yline(P_T, '--r', sprintf('P ≈ %.1f kW', P_T));