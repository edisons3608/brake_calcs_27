% Edison Sun
% NFR27

% Init constants
close all;clc;clear all;

mass = 19.954;                 % slugs
f_mass_dist = 0.5;         % percentage of mass on front axle at rest
cog_height = 10.6;         % inches
wheelbase = 60.25;             % inches
tire_radius = 8;            % inches
pedal_ratio = 4.9;          % mechanical advantage of pedal over master cylinder - worst case can change. 
f_pad_cof = 0.47;           % est. front pad frictional coefficient on stainless steel while warm-ish (EBC)
r_pad_cof = 0.47;           % est. rear pad frictional coefficient on stainless steel while warm-ish (Wilwood)
panic_force = 450;          % lbs
f_rotor_radius = 3.4625;      % inches, measuring to center of rotor swept height, pad height is ~1.335, so to edge of rotor is f_rotor_radius+(0.5*1.335)=3.8365
r_rotor_radius = 3.403;     % inches, measuring to center of rotor swept height, og value was 2.955
pedal_efficiency = 0.85;    % percentage efficiency of brake pedal [very arbitrary]
downforce_f_dist = 0.45;    % percentage of downforce on front axle
f_bias = 0.521;              % percentage of driver force that goes to front axle - up to .65 in either direction. try to keep between .45 - .55.
downforce_coef = 0.08589;   % downforce coefficient
max_speed = 60;             % mph (not top speed of car, but fastest speed at which data is being analyzed)

% Regen specs - 4x DTI F-SiC hub motors, one per wheel (AWD), so regen is
% available at both axles. Simple model: constant continuous torque at all
% speeds, no corner-speed taper and no regen power cap.
regen_torque_motor = 5.1;  % Nm, continuous regen torque per motor
gear_ratio = 11;            % planetary reduction, motor -> wheel
n_motors = 4;               % total motors, one per wheel
% NOTE: 0.26/0.74 front/rear is the DRIVE (acceleration) torque split, not a
% braking split - the 4 independent hub motors don't have to command the same
% ratio in regen. Braking transfers weight forward (front axle carries more
% load and more available traction than rear), so regen here is assumed
% front-biased - mirrored from the drive split as a placeholder assumption
% until an actual braking-specific split is defined.
regen_f_dist_braking = 0.74;   % fraction of regen torque at the front axle while braking

% Constant total regen force at the contact patches, speed-independent
regen_force_total = n_motors * regen_torque_motor * gear_ratio ...
    / (tire_radius * 0.0254) * 0.224809;    % lbf

f_fx = 0;                   % preallocation
r_fx = 0;                   % preallocation

% Hydraulic specs

% piston areas are per caliper
f_piston_area = 2.454;      % square inches, for Wilwood GP200 [front calipers] (changed?)
r_piston_area = 2.454;      % square inches, for Wilwood GP200 [rear calipers]
f_master_cyl_area = 0.38485;   % square inches, Tilton 78-series value used (NFR24), possible from Tilton 78-series is 5/8"=.3068,7/10"=.38485,3/4"=.4418,13/16"=.5185,7/8"=.6013,15/16"=.6903,1"=.7854
r_master_cyl_area = 0.7854;   % square inches, Tilton 78-series value used (NFR24)


%%

step = 0.1;
v = 0:step:max_speed;
N = size(v,2);


% All arrays go from 0-max speed mph in 0.1mph increments
fn_f_axle = zeros(1,N);              % front axle normal force at different speeds
fn_r_axle = zeros(1,N);              % rear axle normal force at different speeds
decels = zeros(1,N);                 % theoretical max. decel. at different speeds (limited by traction)
fpsi = zeros(1,N);                   % psi required in front circuit for max decel. at different speeds
rpsi = zeros(1,N);                   % psi required in rear circuit for max decel. at different speeds
fpsi_pedal = zeros(1,N);             % psi actually in front circuit in the pedal-limited case
rpsi_pedal = zeros(1,N);             % psi actually in rear circuit in the pedal-limited case
regen_avail = zeros(1,N);            % total regen force available (torque- and power-capped) at different speeds
regen_f = zeros(1,N);                % regen force at front axle after traction scaling
regen_r = zeros(1,N);                % regen force at rear axle after traction scaling
fx_f_fric = zeros(1,N);              % front axle force the friction brakes must supply with regen active
fx_r_fric = zeros(1,N);              % rear axle force the friction brakes must supply with regen active
fpsi_regen = zeros(1,N);             % front circuit psi required at max decel. with regen active
rpsi_regen = zeros(1,N);             % rear circuit psi required at max decel. with regen active
f_pedal_force_regen = zeros(1,N);    % front pedal force required at max decel. with regen active
r_pedal_force_regen = zeros(1,N);    % rear pedal force required at max decel. with regen active
lowest_pedal_force_regen = zeros(1,N); % applied pedal force at max decel. with regen active
wt = zeros(1,N);                     % weight transfer forwards under max decel. at different speeds
fx_f_braking = zeros(1,N);           % front axle longitudinal force under max decel. at different speeds
fx_r_braking = zeros(1,N);           % rear axle longitudinal force under max decel. at different speeds
f_pedal_force = zeros(1,N);          % pedal force required for front brakes at max decel. at different speeds
r_pedal_force = zeros(1,N);          % pedal force required for rear brakes at max decel. at different speeds
f_pedal_force_init = zeros(1,N);     % front pedal force after initial traction-limited calculation
r_pedal_force_init = zeros(1,N);     % rear pedal force after initial traction-limited calculation
decels_actual = zeros(1,N);          % "actual" max. decel. at different speeds given both traction and pedal force limitations
decels_init = zeros(1,N);            % stores initial traction-limited decelerations
pedal_force_delta = zeros(1,N);      % stores delta between front and rear pedal forces at max deceleration
lowest_pedal_force = zeros(1,N);
actual_front_brake_bias = zeros(1,N); % actual delivered front-axle share of total braking force (pedal-limited case), at different speeds

a = 1.7;
b = 0.0008;

%% TRACTION-LIMITED BRAKING CASE

% Calculating axle normal forces with no braking
for i = 1:N
    temp = aero_forces((i-1)/10);
    %disp((i-1)/10)
    fn_f_axle(i) = (mass * f_mass_dist) * 32.2 + temp(1);   % lbf
    fn_r_axle(i) = (mass * (1 - f_mass_dist)) * 32.2 + temp(2); % lbf
end
fprintf("\nfront axle weight 30mph = %.2f lbs\nfront axle weight 60mph = %.2f lbs\n", fn_f_axle(301), fn_f_axle(601));
fprintf("rear axle weight 30mph = %.2f lbs\nrear axle weight 60mph = %.2f lbs\n", fn_r_axle(301), fn_r_axle(601));



% Calculating maximum deceleration at different speeds via nonlinear system
% of equations using fsolve
fprintf("\ncalculating traction-limited max. decels...\n");

% Preallocate decels array
decels = zeros(1, N);

% Set options for fsolve
options = optimoptions('fsolve', 'Display', 'off');

for i = 1:N
    % Extract parameters for current time step
    fn_f_axle_i = fn_f_axle(i);
    fn_r_axle_i = fn_r_axle(i);

    % Initial guess for variables [a, fzf, fzr, fxf, fxr]
    x0 = [0; fn_f_axle_i; fn_r_axle_i; 0; 0];

    % Define the system function with current parameters
    systemFunc = @(x) mySystem(x, mass, cog_height, wheelbase, fn_f_axle_i, fn_r_axle_i,a,b);

    % Solve the system of equations
    [x_sol, fval, exitflag] = fsolve(systemFunc, x0, options);

    % Check if fsolve converged
    if exitflag <= 0
        warning('fsolve did not converge at iteration %d.', i);
        decels(i) = NaN; % Assign NaN or handle as needed
    else
        decels(i) = x_sol(1); % Extract deceleration (a)
    end
end

fprintf("\ntraction-limited max decel from 30mph = %.4f ft/s^2\ntraction-limited max decel from 60mph = %.4f ft/s^2\n", decels(301), decels(601));
decels_init = decels;

%% PEDALS-LIMITED BRAKING CASE

j = 1;  % number of loops executed
while abs(decels(N) - decels_actual(N)) > 0.0001   % will converge quickly
    if j > 1    % every loop after the first
        decels = decels_actual; % Update decels for next iteration
    end
    for i = 1:N  % all decels
        % Calculating axle weights and corresponding max. axle longitudinal forces
        wt(i) = mass * decels(i) * (cog_height / wheelbase);
        fx_f_braking(i) = a*(fn_f_axle(i) + wt(i)) - (b*(fn_f_axle(i) + wt(i))^2);
        fx_r_braking(i) = a*(fn_r_axle(i) - wt(i)) - (b*(fn_r_axle(i) - wt(i))^2); 

        % Calculating PSIs required for max decels at different speeds
        fpsi(i) = ((fx_f_braking(i) / 2) * (tire_radius / f_rotor_radius)) / (2*f_piston_area * f_pad_cof); % psi
        rpsi(i) = ((fx_r_braking(i) / 2) * (tire_radius / r_rotor_radius)) / (2*r_piston_area * r_pad_cof); % psi

        % Calculating pedal forces required for max decels
        f_pedal_force(i) = (((fpsi(i) * f_master_cyl_area) / pedal_ratio) / pedal_efficiency) / f_bias; % lbf
        r_pedal_force(i) = (((rpsi(i) * r_master_cyl_area) / pedal_ratio) / pedal_efficiency) / (1 - f_bias);

        if j == 1
            f_pedal_force_init(i) = f_pedal_force(i);
            r_pedal_force_init(i) = r_pedal_force(i);
        end

        % REGEN-BLENDED CASE. Regen cannot raise the traction ceiling, so the
        % target axle forces are still fx_f_braking/fx_r_braking. What regen
        % changes is how much of that the friction brakes have to supply, and
        % therefore the pedal force needed to reach the traction limit.
        regen_avail(i) = regen_force_total;

        % Split front-biased for braking (regen_f_dist_braking), then scale the
        % total down so neither axle is asked for more than its traction allows
        % (a fixed split means the grip-limited axle governs the whole regen command).
        regen_scale = 1;
        if regen_avail(i) * regen_f_dist_braking > fx_f_braking(i)
            regen_scale = min(regen_scale, fx_f_braking(i) / (regen_avail(i) * regen_f_dist_braking));
        end
        if regen_avail(i) * (1 - regen_f_dist_braking) > fx_r_braking(i)
            regen_scale = min(regen_scale, fx_r_braking(i) / (regen_avail(i) * (1 - regen_f_dist_braking)));
        end
        regen_f(i) = regen_avail(i) * regen_f_dist_braking * regen_scale;
        regen_r(i) = regen_avail(i) * (1 - regen_f_dist_braking) * regen_scale;

        % Friction brakes cover the remainder, and the pedal forces follow
        fx_f_fric(i) = max(0, fx_f_braking(i) - regen_f(i));
        fx_r_fric(i) = max(0, fx_r_braking(i) - regen_r(i));
        fpsi_regen(i) = ((fx_f_fric(i) / 2) * (tire_radius / f_rotor_radius)) / (2*f_piston_area * f_pad_cof);
        rpsi_regen(i) = ((fx_r_fric(i) / 2) * (tire_radius / r_rotor_radius)) / (2*r_piston_area * r_pad_cof);
        f_pedal_force_regen(i) = (((fpsi_regen(i) * f_master_cyl_area) / pedal_ratio) / pedal_efficiency) / f_bias;
        r_pedal_force_regen(i) = (((rpsi_regen(i) * r_master_cyl_area) / pedal_ratio) / pedal_efficiency) / (1 - f_bias);
        lowest_pedal_force_regen(i) = min(f_pedal_force_regen(i), r_pedal_force_regen(i));

        % Identifying lowest pedal force required to lock either axle
        if f_pedal_force(i) < r_pedal_force(i)
            r_pedal_force(i) = f_pedal_force(i);
            f_fx = 4 * (f_pedal_force(i) * pedal_efficiency * pedal_ratio * (f_bias) * f_pad_cof * f_piston_area)/(f_master_cyl_area * (tire_radius / f_rotor_radius));
            r_fx = 4 * (r_pedal_force(i) * pedal_efficiency * pedal_ratio * (1 - f_bias) * r_pad_cof * r_piston_area)/(r_master_cyl_area * (tire_radius / r_rotor_radius));
            %decels_actual(i) = (fx_f_braking(i) + f_fx + r_fx) / mass;
            %fprintf("first loop\n");
        else
            f_pedal_force(i) = r_pedal_force(i);
            f_fx = 4 * (f_pedal_force(i) * pedal_efficiency * pedal_ratio * (f_bias) * f_pad_cof * f_piston_area)/(f_master_cyl_area * (tire_radius / f_rotor_radius));
            r_fx = 4 * (r_pedal_force(i) * pedal_efficiency * pedal_ratio * (1 - f_bias) * r_pad_cof * r_piston_area)/(r_master_cyl_area * (tire_radius / r_rotor_radius));
            %decels_actual(i) = (f_fx + r_fx + fx_r_braking(i)) / mass;
            %fprintf("second loop\n");
        end
        % Circuit pressures actually reached in the pedal-limited case. The
        % applied pedal force is the lower of the two axle-locking forces
        % (both entries are equal to it after the min above), and that single
        % force feeds both circuits through the bias bar.
        applied_pedal_force = f_pedal_force(i);
        fpsi_pedal(i) = applied_pedal_force * pedal_efficiency * pedal_ratio * f_bias / f_master_cyl_area;
        rpsi_pedal(i) = applied_pedal_force * pedal_efficiency * pedal_ratio * (1 - f_bias) / r_master_cyl_area;

        decels_actual(i) = (f_fx + r_fx) / mass;

        % Actual delivered front-axle share of total braking force in the
        % pedal-limited case, i.e. what fraction of real longitudinal force
        % lands at the front tires once the applied pedal force (limited by
        % whichever axle locks first) is fed through both circuits.
        actual_front_brake_bias(i) = f_fx / (f_fx + r_fx);
    end
    fprintf("\nloop complete. number of loops so far: %d", j);
    fprintf("\ninitial 60mph decel: %.4f ft/s^2\nnew 60mph decel: %.4f ft/s^2\n", decels(N), decels_actual(N));

    j = j + 1;
end

for i = 1:N
    lowest_pedal_force(i) = min(f_pedal_force_init(i), r_pedal_force_init(i));
end
decels = decels_init;

fprintf("\nnominal (pedal/hydraulic) front bias = %.1f%%\n", f_bias * 100);
fprintf("actual delivered front brake bias at 30mph = %.1f%%\nactual delivered front brake bias at 60mph = %.1f%%\n", ...
    actual_front_brake_bias(301) * 100, actual_front_brake_bias(N) * 100);

%% TIME-DOMAIN CALCULATIONS

% Building a time vector t, where t(i) is the elapsed time (s) since the
% start of a threshold-braking event from max_speed until the car has
% slowed to the speed corresponding to index i, i.e. v(i) = (i-1)/10 mph.
% t(N) = 0 (braking just started at max_speed); t(1) = total time to stop.
t = zeros(1, N);
mph_to_fps = 1.466667;              % conversion factor, mph -> ft/s
dv_fps = step * mph_to_fps;         % constant speed increment in ft/s

for i = N-1:-1:1
    a_avg = (decels_actual(i) + decels_actual(i+1)) / 2;   % ft/s^2
    if a_avg <= 0
        t(i) = t(i+1);
    else
        t(i) = t(i+1) + dv_fps / a_avg;
    end
end

fprintf("\ntotal time to decelerate from %.0f mph to 0 mph: %.4f s\n", max_speed, t(1));

%% PLOTS

%%
% figure
% plot((0:N-1)/10, lowest_pedal_force(1:N))
% legend("Pedal Force")
% xlabel("Speed (mph)")
% ylabel("Pedal Force (lbs)")
% title("Max Pedal Force vs. Speed")

figure
plot((0:N-1)/10, lowest_pedal_force(1:N))
legend("Pedal Force")
xlabel("Speed (mph)")
ylabel("Pedal Force (lbs)")
title("Max Pedal Force vs. Speed")

% Plotting pedal force at threshold braking vs. deceleration at different speeds
figure
plot(decels(1:N), f_pedal_force(1:N))
legend("Pedal Force")
xlabel("deceleration ft/ss")
ylabel("Pedal Force (lbs)")
title("Threshold Pedal Force vs. Deceleration")

% Plotting actual maximum deceleration at different speeds
figure
plot((0:N-1)/10, decels)
legend("Deceleration")
xlabel("Speed (mph)")
ylabel("Deceleration (ft/s^2)")
title("Max Pedal-Limited Deceleration at Different Speeds")

% Calculates panic force hydraulic pressures
f_panic_pressure = panic_force * pedal_efficiency * pedal_ratio * f_bias / f_master_cyl_area;
r_panic_pressure = panic_force * pedal_efficiency * pedal_ratio * (1 - f_bias) / r_master_cyl_area;
fprintf("\nfront panic pressure = %.2f psi\nrear panic pressure = %.2f psi\n", f_panic_pressure, r_panic_pressure);

% Determining difference between front and rear pedal forces under threshold braking
if r_pedal_force_init(1) > f_pedal_force_init(1)
    fprintf("Required rear pedal force is always higher than front\n");
elseif r_pedal_force_init(N) <= f_pedal_force_init(N)
    fprintf("WARNING: Rear pedal force lower than fronts for all speeds\n");
else
    for i = 2:N
        if r_pedal_force_init(i) > f_pedal_force_init(i)
            fprintf("Required rear pedal force higher than front starting at %.1f mph\n", (i-1)/10);
            break; 
        end
    end
end

% Plotting front and rear pedal force at maximum deceleration vs. speed.
% Whichever axle needs the LOWER force locks first and is what actually
% governs the achievable braking event (solid); the higher-force axle
% never actually reaches its own traction limit at that speed, since the
% pedal can't be pushed past lockup on the other end (dotted). Shaded
% background bands make the "who locks first" regions obvious at a glance.
speed_vec = (0:N-1)/10;
front_color = [0      0.4470 0.7410];
rear_color  = [0.8500 0.3250 0.0980];

front_governs = f_pedal_force_init(1:N) <= r_pedal_force_init(1:N);
rear_governs  = ~front_governs;

f_governing = f_pedal_force_init(1:N); f_governing(~front_governs) = NaN;
f_excess    = f_pedal_force_init(1:N); f_excess(front_governs)     = NaN;
r_governing = r_pedal_force_init(1:N); r_governing(~rear_governs)  = NaN;
r_excess    = r_pedal_force_init(1:N); r_excess(rear_governs)      = NaN;

% Convergence (crossover) speed and force, interpolated between grid points
delta = f_pedal_force_init(1:N) - r_pedal_force_init(1:N);
crossover_idx = find(diff(sign(delta)) ~= 0, 1);
has_crossover = ~isempty(crossover_idx);
if has_crossover
    x1 = speed_vec(crossover_idx); x2 = speed_vec(crossover_idx+1);
    y1 = delta(crossover_idx);     y2 = delta(crossover_idx+1);
    crossover_speed = x1 - y1 * (x2 - x1) / (y2 - y1);
    f1 = f_pedal_force_init(crossover_idx); f2 = f_pedal_force_init(crossover_idx+1);
    crossover_force = f1 + (f2 - f1) * (crossover_speed - x1) / (x2 - x1);
end

ymax = max([f_pedal_force_init(1:N), r_pedal_force_init(1:N)]) * 1.12;

figure('Position', [100 100 900 520], 'Color', 'w')
hold on

% Shaded bands showing which axle governs (locks first) over each speed range
if has_crossover
    if front_governs(1)
        band1_color = front_color; band2_color = rear_color;
        band1_label = "Front locks first"; band2_label = "Rear locks first";
    else
        band1_color = rear_color; band2_color = front_color;
        band1_label = "Rear locks first"; band2_label = "Front locks first";
    end
    patch([0 crossover_speed crossover_speed 0], [0 0 ymax ymax], band1_color, ...
        'FaceAlpha', 0.07, 'EdgeColor', 'none', 'HandleVisibility', 'off');
    patch([crossover_speed max_speed max_speed crossover_speed], [0 0 ymax ymax], band2_color, ...
        'FaceAlpha', 0.07, 'EdgeColor', 'none', 'HandleVisibility', 'off');
    text(crossover_speed/2, ymax*0.96, band1_label, 'HorizontalAlignment', 'center', ...
        'FontSize', 9, 'Color', band1_color*0.7, 'FontWeight', 'bold');
    text(crossover_speed + (max_speed - crossover_speed)/2, ymax*0.96, band2_label, ...
        'HorizontalAlignment', 'center', 'FontSize', 9, 'Color', band2_color*0.7, 'FontWeight', 'bold');
end

plot(speed_vec, f_governing, '-', 'Color', front_color, 'LineWidth', 2.0, 'DisplayName', 'Front Pedal Force')
plot(speed_vec, f_excess, ':', 'Color', front_color, 'LineWidth', 1.5, 'HandleVisibility', 'off')
plot(speed_vec, r_governing, '-', 'Color', rear_color, 'LineWidth', 2.0, 'DisplayName', 'Rear Pedal Force')
plot(speed_vec, r_excess, ':', 'Color', rear_color, 'LineWidth', 1.5, 'HandleVisibility', 'off')

if has_crossover
    xline(crossover_speed, 'k--', 'LineWidth', 1, 'HandleVisibility', 'off');
    plot(crossover_speed, crossover_force, 'ko', 'MarkerFaceColor', 'k', 'MarkerSize', 6, 'HandleVisibility', 'off');
    text(crossover_speed, crossover_force - ymax*0.06, ...
        sprintf('  Convergence: %.1f mph, %.0f lbf', crossover_speed, crossover_force), ...
        'FontSize', 9, 'FontWeight', 'bold', 'VerticalAlignment', 'top');
end

grid on
box on
set(gca, 'FontSize', 11, 'LineWidth', 1, 'GridAlpha', 0.15)
xlim([0 max_speed])
ylim([0 ymax])
xlabel("Speed (mph)", 'FontSize', 12, 'FontWeight', 'bold')
ylabel("Pedal Force (lbf)", 'FontSize', 12, 'FontWeight', 'bold')
title("Front/Rear Pedal Force Delta at Different Speeds", 'FontSize', 13, 'FontWeight', 'bold')
subtitle("Solid = governs actual lockup at that speed  |  Dotted = other axle not yet reached", 'FontSize', 9.5)
legend('Location', 'best', 'FontSize', 10, 'Box', 'off')
hold off

% Plotting the actual delivered front-axle brake bias (% of total real
% longitudinal force landing at the front tires) vs. speed, alongside the
% fixed nominal/hydraulic bias set by the bias bar, to show how much the
% delivered split drifts from the nominal setting due to weight transfer
% and whichever axle locks first at a given speed.
figure('Position', [100 100 900 520], 'Color', 'w')
hold on
plot(speed_vec, actual_front_brake_bias(1:N) * 100, '-', 'Color', front_color, ...
    'LineWidth', 2.0, 'DisplayName', 'Actual Front Brake Bias')
yline(f_bias * 100, 'k--', 'LineWidth', 1, 'DisplayName', sprintf('Nominal Bias (%.1f%%)', f_bias * 100))
grid on
box on
set(gca, 'FontSize', 11, 'LineWidth', 1, 'GridAlpha', 0.15)
xlim([0 max_speed])
xlabel("Speed (mph)", 'FontSize', 12, 'FontWeight', 'bold')
ylabel("Front Brake Bias (%)", 'FontSize', 12, 'FontWeight', 'bold')
title("Actual Delivered Front Brake Bias vs. Speed", 'FontSize', 13, 'FontWeight', 'bold')
subtitle("Share of real longitudinal force landing at the front axle, pedal-limited case", 'FontSize', 9.5)
legend('Location', 'best', 'FontSize', 10, 'Box', 'off')
hold off

% Plotting applied pedal force to reach the traction limit, with and without
% regen, to show how much pedal effort the regen blend removes
figure
plot((0:N-1)/10, lowest_pedal_force(1:N), (0:N-1)/10, lowest_pedal_force_regen(1:N))
legend("Friction Only","With Regen")
xlabel("Speed (mph)")
ylabel("Pedal Force (lbs)")
title("Pedal Force to Reach Traction Limit - Regen vs. Friction Only")

% Plotting regen force actually usable at each axle after traction capping
figure
plot((0:N-1)/10, regen_avail(1:N), (0:N-1)/10, regen_f(1:N), (0:N-1)/10, regen_r(1:N))
legend("Regen Available (uncapped)","Front Axle Regen","Rear Axle Regen")
xlabel("Speed (mph)")
ylabel("Longitudinal Force (lbs)")
title("Regen Force Available vs. Usable After Traction Limit")

% Plotting front and rear circuit pressure vs. speed during a
% pedal-limited braking event from max_speed to 0
figure
plot((0:N-1)/10, fpsi_pedal(1:N), (0:N-1)/10, rpsi_pedal(1:N))
legend("Front PSI","Rear PSI")
xlabel("Speed (mph)")
ylabel("Pressure (psi)")
title("Front/Rear Circuit Pressure vs. Speed (Pedal-Limited)")

%% TIME CALCULATIONS FOR THERMAL SIMULATIONS

% Calculating the amount of time it takes to decelerate from maximum speed
time = 0;
decels_actual_metric = decels_actual * 0.3048;    % Convert decelerations to m/s^2
vnew = max_speed * 0.44704;   % Convert mph to m/s
tdelt = 0.0001;    % Time step 
while vnew > 0  % Continues as long as the speed is greater than 0
    vold = vnew;

    % Finding index of corresponding acceleration value at that given speed
    accel_ind = round(vold / 0.44704 * 10) + 1;
    if accel_ind < 1 || accel_ind > N
        break;
    end

    % Determining change in velocity by calling corresponding value from decels vector
    dv = decels_actual_metric(accel_ind) * tdelt;
    vnew = vold - dv;
    time = time + tdelt;

end
fprintf("\nFastest possible time to decelerate from %.0f mph to 0 mph: %.4f s\n", max_speed, time);


% System of equations function for fsolve
function F = mySystem(x, mass, cog_height, wheelbase, fn_f_axle_i, fn_r_axle_i,a,b)
    % Unpack variables
    decel = x(1);  % Deceleration
    fzf = x(2);  % Front normal force
    fzr = x(3);  % Rear normal force
    fxf = x(4);  % Front longitudinal force
    fxr = x(5);  % Rear longitudinal force

    % Equations
    F = zeros(5,1);
    F(1) = (fxf + fxr) / mass - decel;    % Equation 1: Newton's second law
    F(2) = a * fzf - b * fzf^2 - fxf;  % Equation 2: Front tire force
    F(3) = a * fzr - b * fzr^2 - fxr;  % Equation 3: Rear tire force
    F(4) = fn_f_axle_i + (mass * decel * (cog_height / wheelbase)) - fzf;  % Equation 4: Front normal force
    F(5) = fn_r_axle_i - (mass * decel * (cog_height / wheelbase)) - fzr;  % Equation 5: Rear normal force
end

% Aerodynamic forces function
function [df] = aero_forces(v)
    % aero_forces: calculates the front and rear downforce on the car axles 
    % given the velocity of the car (mph)

    % Parameters
    rho = 1.225; % Air density (kg/m^3)
    A = 1.2; % Reference area (m^2)
    C_L = 3.39; % Lift coefficient
    wheelbase_m = 60.25 * 0.0254; % Convert wheelbase to meters
    vel = v * 0.44704; % Convert velocity to m/s
    CP_long = (wheelbase_m / 2) + 0.01905; % Center of pressure from front axle (m)

    % Aerodynamic forces
    F_L = 0.5 * rho * vel^2 * C_L * A; % Downforce (N)

    % Distribute downforce between front and rear axles
    Fz_front = F_L * (wheelbase_m - CP_long) / wheelbase_m;
    Fz_rear = F_L * CP_long / wheelbase_m;

    front_df = Fz_front / 4.44822; % Convert to lbf
    rear_df = Fz_rear / 4.44822; % Convert to lbf

    df = [front_df, rear_df];
end

