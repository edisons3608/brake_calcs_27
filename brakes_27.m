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