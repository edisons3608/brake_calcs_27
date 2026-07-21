

%n_w: wheel speed in rpm

syms v n_w

% v: vehicle speed in mph
% tire_radius: inches (matches brakes_27.m)

tire_radius = 8;
wheel_diameter = 2 * tire_radius;

n_w = (v * 63360/60) / (pi * wheel_diameter);

% apply planet gearbox reduction


% n_m: motor speed in rpm
rat = 11;

n_m = rat * n_w;

% approx 31.6 Nm until 12000 rpm

v_at_12000rpm = double(solve(n_m == 12000, v)); % mph

