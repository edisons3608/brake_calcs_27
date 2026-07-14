% Edison Sun
% NFR27

% Init constants

% Constants
mass = 21.135;                 % slugs
f_mass_dist = 0.48;         % percentage of mass on front axle at rest
cog_height = 12.8;         % inches
wheelbase = 60.25;             % inches
tire_radius = 8;            % inches
pedal_ratio = 8;          % mechanical advantage of pedal over master cylinder - worst case can change. 
f_pad_cof = 0.42;           % est. front pad frictional coefficient on stainless steel while warm-ish (EBC)
r_pad_cof = 0.42;           % est. rear pad frictional coefficient on stainless steel while warm-ish (Wilwood)
panic_force = 450;          % lbs
f_rotor_radius = 3.0825;      % inches, measuring to center of rotor swept height, pad height is ~1.335, so to edge of rotor is f_rotor_radius+(0.5*1.335)=3.8365
r_rotor_radius = 2.9635;     % inches, measuring to center of rotor swept height, og value was 2.955
pedal_efficiency = 0.85;    % percentage efficiency of brake pedal [very arbitrary]
downforce_f_dist = 0.45;    % percentage of downforce on front axle
f_bias = 0.4824;              % percentage of driver force that goes to front axle - up to .65 in either direction. try to keep between .45 - .55.
downforce_coef = 0.08589;   % downforce coefficient
max_speed = 60;             % mph (not top speed of car, but fastest speed at which data is being analyzed)

% regen_torque = 70;          % lbs-ft; assumed constant regen torque at motor
% axle_regen_long_force = (75 / (tire_radius / 12)) / 2;  % lbs

f_fx = 0;                   % preallocation
r_fx = 0;                   % preallocation

% Hydraulic specs
f_piston_area = 2.454;      % square inches, for Wilwood GP200 [front calipers] (changed?)
r_piston_area = 2.454;      % square inches, for Wilwood GP200 [rear calipers]
f_master_cyl_area = 0.3068;   % square inches, Tilton 78-series value used (NFR24), possible from Tilton 78-series is 5/8"=.3068,7/10"=.38485,3/4"=.4418,13/16"=.5185,7/8"=.6013,15/16"=.6903,1"=.7854
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
    systemFunc = @(x) mySystem(x, mass, cog_height, wheelbase, fn_f_axle_i, fn_r_axle_i);

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
        fx_f_braking(i) = 1.7*(fn_f_axle(i) + wt(i)) - (0.0008*(fn_f_axle(i) + wt(i))^2);
        fx_r_braking(i) = 1.7*(fn_r_axle(i) - wt(i)) - (0.0008*(fn_r_axle(i) - wt(i))^2); 

        % Calculating PSIs required for max decels at different speeds
        fpsi(i) = ((fx_f_braking(i) / 2) * (tire_radius / f_rotor_radius)) / (2*f_piston_area * f_pad_cof); % psi %%%%%%fpsi(i) = ((fx_f_braking(i) / 2) * (tire_radius / f_rotor_radius)) / (2*f_piston_area * f_pad_cof); % psi
        rpsi(i) = ((fx_r_braking(i) / 2) * (tire_radius / r_rotor_radius)) / (2*r_piston_area * r_pad_cof); % psi %%%%%%rpsi(i) = ((fx_r_braking(i) / 2) * (tire_radius / r_rotor_radius)) / (2*r_piston_area * r_pad_cof); % psi

        % Calculating pedal forces required for max decels
        f_pedal_force(i) = (((fpsi(i) * f_master_cyl_area) / pedal_ratio) / pedal_efficiency) / f_bias; % lbf
        r_pedal_force(i) = (((rpsi(i) * r_master_cyl_area) / pedal_ratio) / pedal_efficiency) / (1 - f_bias);

        if j == 1
            f_pedal_force_init(i) = f_pedal_force(i);
            r_pedal_force_init(i) = r_pedal_force(i);
        end

        % Identifying lowest pedal force required to lock either axle
        if f_pedal_force(i) < r_pedal_force(i)
            r_pedal_force(i) = f_pedal_force(i);
            f_fx = 4 * (f_pedal_force(i) * pedal_efficiency * pedal_ratio * (f_bias) * f_pad_cof * f_piston_area)/(f_master_cyl_area * (tire_radius / f_rotor_radius));
            r_fx = 4 * (r_pedal_force(i) * pedal_efficiency * pedal_ratio * (1 - f_bias) * r_pad_cof * r_piston_area)/(r_master_cyl_area * (tire_radius / r_rotor_radius));
            % decels_actual(i) = (fx_f_braking(i) + f_fx + r_fx) / mass;
            %fprintf("first loop\n");
        else
            f_pedal_force(i) = r_pedal_force(i);
            f_fx = 4 * (f_pedal_force(i) * pedal_efficiency * pedal_ratio * (f_bias) * f_pad_cof * f_piston_area)/(f_master_cyl_area * (tire_radius / f_rotor_radius));
            r_fx = 4 * (r_pedal_force(i) * pedal_efficiency * pedal_ratio * (1 - f_bias) * r_pad_cof * r_piston_area)/(r_master_cyl_area * (tire_radius / r_rotor_radius));
            % decels_actual(i) = (f_fx + r_fx + fx_r_braking(i)) / mass;
            %fprintf("second loop\n");
        end
        decels_actual(i) = (f_fx + r_fx) / mass;
    end
    fprintf("\nloop complete. number of loops so far: %d", j);
    fprintf("\ninitial 60mph decel: %.4f ft/s^2\nnew 60mph decel: %.4f ft/s^2\n", decels(N), decels_actual(N));

    j = j + 1;
end

for i = 1:N
    lowest_pedal_force(i) = min(f_pedal_force_init(i), r_pedal_force_init(i));
end
decels = decels_init;

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

% Plotting front and rear pedal force at maximum deceleration vs. speed
figure
plot((0:N-1)/10, f_pedal_force_init(1:N), (0:N-1)/10, r_pedal_force_init(1:N))
legend("Front Pedal Force","Rear Pedal Force")
xlabel("Speed (mph)")
ylabel("Pedal Force (lbs)")  
title("Front/Rear Pedal Force Delta at Different Speeds")

% Plotting front and rear circuit pressure vs. time elapsed during a
% threshold-braking event from max_speed to 0
figure
plot(t(1:N), fpsi(1:N), t(1:N), rpsi(1:N))
legend("Front PSI","Rear PSI")
xlabel("Time (s)")
ylabel("Pressure (psi)")
title("Front/Rear Circuit Pressure vs. Time")

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
    a = decels_actual_metric(accel_ind) * tdelt;
    vnew = vold - a;
    time = time + tdelt;

end
fprintf("\nFastest possible time to decelerate from %.0f mph to 0 mph: %.4f s\n", max_speed, time);


% System of equations function for fsolve
function F = mySystem(x, mass, cog_height, wheelbase, fn_f_axle_i, fn_r_axle_i)
    % Unpack variables
    a   = x(1);  % Deceleration
    fzf = x(2);  % Front normal force
    fzr = x(3);  % Rear normal force
    fxf = x(4);  % Front longitudinal force
    fxr = x(5);  % Rear longitudinal force

    % Equations
    F = zeros(5,1);
    F(1) = (fxf + fxr) / mass - a;    % Equation 1: Newton's second law
    F(2) = 1.7 * fzf - 0.0008 * fzf^2 - fxf;  % Equation 2: Front tire force
    F(3) = 1.7 * fzr - 0.0008 * fzr^2 - fxr;  % Equation 3: Rear tire force
    F(4) = fn_f_axle_i + (mass * a * (cog_height / wheelbase)) - fzf;  % Equation 4: Front normal force
    F(5) = fn_r_axle_i - (mass * a * (cog_height / wheelbase)) - fzr;  % Equation 5: Rear normal force
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

