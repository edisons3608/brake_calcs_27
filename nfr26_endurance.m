% Edison Sun
% Reads LOG_*.csv (long-format CAN log) and plots front/rear brake
% pressure vs. time.

%% READ LOG CSV

log_file = "LOG_0006.csv";

% Log is long-format (one row per signal sample)

% use datastore to save my fucking ram
ds = tabularTextDatastore(log_file, "TextType", "string");
ds.SelectedVariableNames = ["signal_name", "timestamp_ms", "value"];

tt = tall(ds);

is_front = tt.signal_name == "Front_Brake_Pressure";
is_rear  = tt.signal_name == "Rear_Brake_Pressure";

front_tbl = gather(tt(is_front, ["timestamp_ms", "value"]));
rear_tbl  = gather(tt(is_rear,  ["timestamp_ms", "value"]));

% Ensure chronological order and convert timestamps to seconds
front_tbl = sortrows(front_tbl, "timestamp_ms");
rear_tbl  = sortrows(rear_tbl, "timestamp_ms");

front_t = front_tbl.timestamp_ms / 1000;   % s
front_p = front_tbl.value;                 % psi

rear_t = rear_tbl.timestamp_ms / 1000;     % s
rear_p = rear_tbl.value;                   % psi

%% BIN INTO MAX-ENVELOPE FOR PLOTTING

% Raw samples are dense enough (~100Hz) that a plain line plot is just a
% wall of spikes. Bin into windows and keep the max per bin instead, so
% every braking event's peak pressure still shows up but the trace is
% actually readable.
bin_width_s = 5;   % envelope bin width (s); tune for detail vs. clutter

[front_t_env, front_p_env] = max_envelope(front_t, front_p, bin_width_s);
[rear_t_env, rear_p_env]   = max_envelope(rear_t, rear_p, bin_width_s);

%% PLOT (RAW)

front_max = max(front_p);
rear_max  = max(rear_p);

figure
plot(front_t, front_p, rear_t, rear_p)
yline(front_max, "--", sprintf("Front Max = %.1f psi", front_max))
yline(rear_max, "--", sprintf("Rear Max = %.1f psi", rear_max))
legend("Front Brake Pressure", "Rear Brake Pressure")
xlabel("Time (s)")
ylabel("Brake Pressure (psi)")
title("Front/Rear Brake Pressure vs. Time (Raw)")
grid on

figure
plot(front_t, front_p)
xlabel("Time (s)")
ylabel("Brake Pressure (psi)")
title("Front Brake Pressure vs. Time (Raw)")
grid on

figure
plot(rear_t, rear_p)
xlabel("Time (s)")
ylabel("Brake Pressure (psi)")
title("Rear Brake Pressure vs. Time (Raw)")
grid on

%% PLOT (MAX ENVELOPE)

figure
plot(front_t_env, front_p_env, rear_t_env, rear_p_env)
legend("Front Brake Pressure", "Rear Brake Pressure")
xlabel("Time (s)")
ylabel("Brake Pressure (psi)")
title("Front/Rear Brake Pressure vs. Time (Max Envelope)")
grid on

%%


figure
plot(front_t_env, front_p_env)
xlabel("Time (s)")
ylabel("Brake Pressure (psi)")
title("Front Brake Pressure vs. Time (Max Envelope)")
grid on

figure
plot(rear_t_env, rear_p_env)
xlabel("Time (s)")
ylabel("Brake Pressure (psi)")
title("Rear Brake Pressure vs. Time (Max Envelope)")
grid on

%% FUNCTIONS

function [t_env, p_env] = max_envelope(t, p, bin_width)
    % max_envelope: downsamples (t, p) by taking the max of p within
    % fixed-width time bins, returning the bin center as the time value.
    edges = 0:bin_width:(max(t) + bin_width);
    bin_idx = discretize(t, edges);
    n_bins = numel(edges) - 1;

    p_env = accumarray(bin_idx, p, [n_bins, 1], @max, NaN);
    t_env = edges(1:end-1)' + bin_width/2;

    valid = ~isnan(p_env);   % drop bins with no samples
    t_env = t_env(valid);
    p_env = p_env(valid);
end


