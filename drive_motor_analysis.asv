clear; clc
%% Max Acceleration driving
rw = 48.0/2000.0;

J = [0, 1 / rw, 0.1417 / rw; ...
    sqrt(3) / (2 * rw), -0.5 / rw, 0.1565 / rw; 
    sqrt(3) / (2 * rw), 0.5 / rw, -0.1565 / rw];
Ts = 14; %kg-cm
operating_voltage = 10;
voltage_ratio = operating_voltage/12;
Ts_operating = Ts*voltage_ratio;
mass = 3.023; %kg
Ts_real = Ts_operating / 100; %maps to kg-m

Fx_max = [1 0 0] * J' * [0; Ts_real; Ts_real];
Fy_max = [0 1 0] * J' * [Ts_real; -Ts_real/2; Ts_real/2];
tau_max = [0 0 1] * J' * [Ts_real; Ts_real; Ts_real];

ax_max = Fx_max/mass * 100 % cm/s^2
ay_max = Fy_max/mass * 100 % cm/s^2


%% Max RPM driving
clc;
omega_nl = 55; %RPM
omega_nl = omega_nl / 60 * 2 * pi; % rad/s
w_real = omega_nl*voltage_ratio;

vx_max = [1 0 0]*inv(J)*[0; w_real; w_real] * 100
vy_max = [0 1 0]*inv(J)*[w_real; -w_real/2; w_real/2] * 100
wz_max = [0 0 1]*inv(J)*[w_real;w_real;w_real]

%% Conveyor math
clc;
omega_nl = 41.25; % RPM
omega_nl = omega_nl /60 * 2 * pi; % rad/s
T_s = 10.5; % kg-cm
T_s = T_s / 100; % kg-m
Ts_operating = T_s * voltage_ratio;
r = 7.68256 / 1000; % m
vmax = r*omega_nl * 100 % cm/s
fmax = T_s/r

%% Rack & Pinion
rack_radius = 16 % mm