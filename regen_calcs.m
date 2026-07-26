

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

% regen braking force - 4x DTI F-MOT hub motors (AWD), constant torque region

motor_torque = 31.6;   % Nm, constant per motor below 12000 rpm corner
n_motors = 4;           % one per wheel

wheel_torque = motor_torque * rat;             % Nm, torque at wheel after planetary gearbox
total_regen_torque = n_motors * wheel_torque;  % Nm, summed across all 4 wheels

tire_radius_m = tire_radius * 0.0254;          % in -> m

regen_force_total = total_regen_torque / tire_radius_m;   % N, total longitudinal regen force
regen_force_total_lbf = regen_force_total * 0.224809;     % lbf, panic braking (all 4 motors at max constant torque)

