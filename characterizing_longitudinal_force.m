% a lazy copy of "tire_comparison_2020.m" except for longitudinal forces

% make sure matlab is clean
clear;
clc;
close all;

% begin by picking the data we want to import
% matched tires to filenames based on tire testing run summary spreadsheets
% the files below are all warm drive/brake runs
Hoosier_18x6x10_LC0_drivebrake = {'round5/A1464run33.mat', 'round5/A1464run34.mat'};
Hoosier_18x6x10_R25B_drivebrake = {'round5/A1464run39.mat', 'round5/A1464run40.mat'};
filepath_list = [Hoosier_18x6x10_LC0_drivebrake];

% store the data as a cell array of structs. For struct fields, see "import_tire_data.m"
run_data = cell(length(filepath_list),1);
for file_index = 1:length(filepath_list)
	run_data{file_index} = import_tire_data(filepath_list{file_index});
end

% combine each pair of runs so we have one dataset per tire
fields = fieldnames(run_data{1});
for tire_ind = 1:1
	for field_ind = 1:length(fields)
		% concantenate arrays and copy over strings to new cell array
		if (~strcmp(fields{field_ind}, 'tireid') && ~strcmp(fields{field_ind}, 'testid'))
			tire_data.(fields{field_ind}) = ...
				[run_data{tire_ind*2 - 1}.(fields{field_ind}); run_data{tire_ind*2}.(fields{field_ind})];
        else
            tire_data.(fields{field_ind}) = run_data{tire_ind*2}.(fields{field_ind});
        end
	end
end

% we want to sort the data based on nominal conditions
% nominal conditions are intended in tire testing procedure
nominal_P = [8, 10, 12, 14];
P_tolerance = .4;
nominal_IA = [0, 2, 4];
IA_tolerance = .4;
nominal_FZ = [50, 150, 200, 250];
FZ_tolerance = 10;
nominal_SA = [0, -3 -6];
SA_tolerance = 1;

% create logical arrays to mask our data based on closest nominal value
% store masks in cell array, where column is nominal value, and row is the tire
% we can combine masks later to get specific subsets of data
P_masks = cell(1, length(nominal_P));
IA_masks = cell(1, length(nominal_IA));
FZ_masks = cell(1, length(nominal_FZ));
SA_masks = cell(1, length(nominal_SA));

for nom_P_index = 1:length(nominal_P)
    P_masks{nom_P_index} = abs(tire_data.P - nominal_P(nom_P_index)) < P_tolerance;
end
for nom_IA_index = 1:length(nominal_IA)
	IA_masks{nom_IA_index} = abs(tire_data.IA - nominal_IA(nom_IA_index)) < IA_tolerance;
end
for nom_FZ_index = 1:length(nominal_FZ)
	FZ_masks{nom_FZ_index} = abs(tire_data.FZ - nominal_FZ(nom_FZ_index)) < FZ_tolerance;
end
for nom_SA_index = 1:length(nominal_SA)
	SA_masks{nom_SA_index} = abs(tire_data.SA - nominal_SA(nom_SA_index)) < SA_tolerance;
end

% more masks may be useful if we want to look at other conditions/variables

% went through all that trouble with cell arrays so that we can now plot data in loops using:
% plot(tire_data{ii}.SA(FZ_masks{1} & P_masks{2}), tire_data{ii}.NFY(FZ_masks{1} & P_masks{2}), '.');
% etc.


%{
% raw data is scatterplot
% compare using max NFY @ each SA, and also using best fit line using
% simplified 4 coefficient Pacjeka tire model provided by TTC forum member billCobb
% store sets of 4 coefficients in array, in case we want to look through it later
magic_coeff = zeros(length(nominal_FZ), 4);
SL_sweep = (-.26:.001:.21)';		% for plotting @Pacejka4_Model
figure('WindowState', 'maximized');
for FZ_ind = 1:length(nominal_FZ)
	subplot(2,2,FZ_ind);
    hold on;

	% if a conditions_mask returns no data for a tire, we want to ignore it in the legend
    legend_entries = cell(1, 2*length(nominal_SA));

	for SA_ind = 1:length(nominal_SA)
		% assume symmetric tire, so only look at SA>0 @ each FZ
		conditions_mask = FZ_masks{FZ_ind} & P_masks{2} & IA_masks{1} & SA_masks{SA_ind};
        
        if(any(conditions_mask))		% if there are valid datapoints
            % for debugging, plot full scatterplot to visually see fit (currently breaks legend)
            %plot(tire_data.SL(conditions_mask), tire_data.NFX(conditions_mask), '.');
            
            % find and plot best fit line. Make initial coefficient guesses to not fuck up the trig functions into local minimum
            [magic_coeff(FZ_ind, :), ~, ~] = lsqcurvefit('Pacejka4_Model', [1.33 .1 6.337 1.50], ...
                [tire_data.SL(conditions_mask), tire_data.FZ(conditions_mask)], ...
                tire_data.NFX(conditions_mask));
            plot(SL_sweep, Pacejka4_Model(magic_coeff(FZ_ind, :), ...
            	[SL_sweep, nominal_FZ(FZ_ind) .* ones(size(SL_sweep))]));
            
            % find and plot maximum line
            % assume we're using the same SL_sweep for both graphs
            [max_NFX_SL_edges, max_NFX] = find_max_per_SL(tire_data.SL(conditions_mask), tire_data.NFX(conditions_mask));
            plot(max_NFX_SL_edges, max_NFX);
            
            % add entry to legend based on tire
            legend_entries{SA_ind*2} = sprintf('SA = %d, max', nominal_SA(SA_ind));
            legend_entries{SA_ind*2-1} = sprintf('SA = %d, avg', nominal_SA(SA_ind));
        end
	end

	title(sprintf('FZ = %d', nominal_FZ(FZ_ind)));
	xlabel('SA (deg)');
	ylabel('NFX (FX/FZ)');
    legend_entries = legend_entries(~cellfun(@isempty,legend_entries));
	legend(legend_entries, 'Location', 'northeast');
end

% if confident, save figures into folder
%saveas(gcf, 'old vs new', 'png');
%}




drive_magic_coeff = zeros(1, 4);
optimal_conditions_mask = P_masks{2} & IA_masks{1} & SA_masks{1};
options = optimset('TolFun', 1e-15, 'MaxFunEvals', 1e4, 'MaxIter', 1e4);
[drive_magic_coeff, ~, ~] = lsqcurvefit('Pacejka4_Model', [2 .1 .2 3], ...
	[tire_data.SL(optimal_conditions_mask), tire_data.FZ(optimal_conditions_mask)], ...
	-tire_data.FX(optimal_conditions_mask), [], [], options);
figure('WindowState', 'maximized');
hold on;
grid on;
grid minor;
legend_entries = cell(length(nominal_FZ), 1);
peak_NFX = zeros(length(nominal_FZ), 2);
for FZ_ind = 1:length(nominal_FZ) 
	conditions_mask = FZ_masks{FZ_ind} & P_masks{2} & IA_masks{1} & SA_masks{1};
    
	%find and plot best fit line
    SL_sweep = (min(tire_data.SL(conditions_mask)):.001:max(tire_data.SL(conditions_mask)))';
    best_fit_curve = Pacejka4_Model(drive_magic_coeff, ...
		[SL_sweep, nominal_FZ(FZ_ind) .* ones(size(SL_sweep))]);
	plot(SL_sweep, best_fit_curve);
    
    
    [peak_NFX(FZ_ind, 1), peak_NFX(FZ_ind, 2)] = max(best_fit_curve);
    peak_NFX(FZ_ind, 2) = SL_sweep(peak_NFX(FZ_ind, 2));

	legend_entries{FZ_ind*2} = [sprintf('FZ = %d', nominal_FZ(FZ_ind))];
    
    % debug with scatterplot
    %plot(tire_data.SL(conditions_mask), -tire_data.FX(conditions_mask), '.');
end
plot(peak_NFX(:, 2), peak_NFX(:, 1), '*');
%plot(SL_sweep(trough_NFX(:, 2)), trough_NFX(:, 1), '*');
legend_entries{100} = 'peaks';  % 100 = infinity, right?
xlabel('SA (deg)');
ylabel('FX (lbs)');
legend_entries = legend_entries(~cellfun(@isempty,legend_entries));
legend(legend_entries, 'Location', 'south');
title('FX vs SL @P=10, IA = 0, SA = 0');

% plot peaks and linear fit them
figure('WindowState', 'maximized');
hold on;
grid on;
grid minor;
plot(nominal_FZ, peak_NFX(:,1)', '*');
FZ_sweep = (0:1:300)';
SA_at_peak = peak_NFX(1,2);
best_fit_line = Pacejka4_Model(drive_magic_coeff, ...
		[SA_at_peak .* ones(size(FZ_sweep)), FZ_sweep]);
	plot(FZ_sweep, best_fit_line);
xlabel('tire weight (lbs)');
ylabel('FX (lbs)');
legend(sprintf('peaks @ nominal FZ, SA = %6.6f', SA_at_peak), ...
    sprintf('best fit is FY = (%6.6f FZ + %6.8f FZ^2) * sin(%6.6f * atan(%6.6f SA))', ...
            drive_magic_coeff(1), drive_magic_coeff(2)/1000, drive_magic_coeff(4), drive_magic_coeff(3)));
title('peak FX vs FZ @P=10, IA=0, SA=0');

save("magic_coefficients_longitudinal", "drive_magic_coeff")
