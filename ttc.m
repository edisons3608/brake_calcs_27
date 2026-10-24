% Fx-vs-Fz load-sensitivity fit for the Hoosier 16.0x7.5-10 R20 (Round 9).
%
% TTC has no drive/brake data for the 16x7.5 (Round 9, Round 8 and Round 6
% drive/brake sets do not include it), so:
%   1. fit peak lateral Fy vs Fz for the 16x7.5 from its cornering runs
%      (this gives the load-sensitivity shape for the actual tire)
%   2. on the Hoosier 18.0x6.0-10 R20, which has both cornering and
%      drive/brake data, find the average ratio of peak Fx to peak Fy
%   3. scale the 16x7.5 Fy fit by that ratio to get Fx = a*Fz + b*Fz^2
%
% Caveat: the Fx/Fy ratio on the 18x6 R20 changes with Fz, so a single scale
% factor is an average over 50-250 lb and does not capture that.

dirCorner = "C:\Users\uux4685\Documents\GitHub\NFR26-CAD\VI\Suspension MATLAB\Tire Data\Round 9 Testing Data\RunData_Cornering_ASCII_USCS_Round9";
dirDrive  = "C:\Users\uux4685\Downloads\RawData_DriveBrake_Matlab_SI_Round9";

% Round 9 cornering run numbers, Hoosier 16.0x7.5-10 R20 sweeps at ~12 psi.
% Runs 2 and 7 are a different test (SA -1..6 deg) and run 5 was at 14 psi.
runs16 = [4 6 8 9];

% Hoosier 18.0x6.0-10 R20, matched by rim width.
% cornering runs 28 29 / 31 32, drive/brake raw runs 69 70 / 72 73
rims18 = struct('name',   {'6 in rim', '7 in rim'}, ...
                'corner', {[28 29], [31 32]}, ...
                'drive',  {[69 70], [72 73]});

fzCheck = 50:50:250;   % lb, range the fit is valid over and the ratio averaged over

% 1. 16x7.5 lateral fit
[fz16, fy16] = fyPeaks(dirCorner, runs16);
c16 = [fz16, fz16.^2] \ fy16;

% 2. Fx/Fy ratio on the 18x6 R20
ratio = zeros(numel(rims18), numel(fzCheck));
for i = 1:numel(rims18)
    [fzy, fy] = fyPeaks(dirCorner, rims18(i).corner);
    [fzx, fx] = fxPeaks(dirDrive,  rims18(i).drive);
    cy = [fzy, fzy.^2] \ fy;
    cx = [fzx, fzx.^2] \ fx;
    ratio(i,:) = (cx(1)*fzCheck + cx(2)*fzCheck.^2) ./ (cy(1)*fzCheck + cy(2)*fzCheck.^2);
    fprintf('18x6 R20, %s: Fy = %.3f*Fz %+.6f*Fz^2, Fx = %.3f*Fz %+.6f*Fz^2\n', ...
        rims18(i).name, cy(1), cy(2), cx(1), cx(2));
    fprintf('   Fx/Fy at Fz = %s lb: %s\n', mat2str(fzCheck), mat2str(ratio(i,:), 3));
end
k = mean(ratio(:));

% 3. scale the 16x7.5 curve
a = k * c16(1);
b = k * c16(2);

fprintf('\n16x7.5 R20 Fy fit:  Fy = %.4f*Fz %+.6f*Fz^2\n', c16(1), c16(2));
fprintf('Fx/Fy scale factor k = %.3f (range %.2f to %.2f)\n', k, min(ratio(:)), max(ratio(:)));
fprintf('16x7.5 R20 Fx = %.4f*Fz %+.6f*Fz^2\n', a, b);
fprintf('brakes_27.m form: a = %.4f, b = %.6f  (Fx = a*Fz - b*Fz^2)\n', a, -b);
fprintf('mu = %.3f at Fz = %d lb, %.3f at Fz = %d lb\n', ...
    a + b*fzCheck(1), fzCheck(1), a + b*fzCheck(end), fzCheck(end));
fprintf('Valid for per-tire Fz of %d to %d lb\n', fzCheck(1), fzCheck(end));

fzFit = linspace(0, fzCheck(end), 100);
figure;
scatter(fz16, fy16, 30, [0.7 0.7 0.7], 'filled'); hold on;
plot(fzFit, c16(1)*fzFit + c16(2)*fzFit.^2, 'b--', 'LineWidth', 1.5);
plot(fzFit, a*fzFit + b*fzFit.^2, 'r-', 'LineWidth', 2);
xlabel('F_z (lb)'); ylabel('peak force (lb)');
legend('16x7.5 peak F_y per run','16x7.5 F_y fit','F_x estimate (F_y fit \times k)','Location','best');
title('Hoosier 16.0x7.5-10 R20, Round 9 - estimated F_x vs F_z');
grid on;

function [fz, fy] = fyPeaks(dirCorner, runs)
% Peak of the median-filtered |Fy| over each full slip-angle sweep, at IA = 0,
% for each settled Fz step. Cornering data, USCS units (lb).
    fzStep = 25; fzTol = 5; minPts = 200; win = 51;
    fz = []; fy = [];
    for r = runs
        data = readmatrix(fullfile(dirCorner, sprintf('A2356run%d.dat', r)), ...
            'FileType', 'text', 'NumHeaderLines', 3);
        SA = data(:,4); IA = data(:,5); FY = data(:,10); FZ = -data(:,11);
        step = round(FZ/fzStep)*fzStep;
        mask = abs(IA) < 0.5 & abs(FZ - step) < fzTol & step > 0;
        for s = unique(step(mask))'
            sel = mask & step == s;
            if sum(sel) < minPts, continue; end
            % sweep must reach |SA| >= 10 deg both sides or it misses the peak
            if max(SA(sel)) < 10 || min(SA(sel)) > -10, continue; end
            fz(end+1,1) = mean(FZ(sel));
            fy(end+1,1) = max(movmedian(abs(FY(sel)), win));
        end
    end
end

function [fz, fx] = fxPeaks(dirDrive, runs)
% Peak of a Magic Formula fit to Fx vs slip ratio at each Fz level, at
% SA = 0 and IA = 0. Drive/brake data, SI file converted to lb.
%   Fx = D*sin(C*atan(B*s - E*(B*s - atan(B*s))))
% Only the slip sweeps are fitted (|SL| > 0.02). Most samples in the raw
% file are dwell at SL ~ 0 and would swamp the fit. Residuals above 200 lb
% are clipped so a few transient spikes cannot drag the curve.
    N2lb = 0.224809; fzStep = 25; fzTol = 8; minPts = 150;
    mf = @(p, s) p(1)*sin(p(2)*atan(p(3)*s - p(4)*(p(3)*s - atan(p(3)*s))));
    sGrid = linspace(-0.3, 0.3, 3001);
    opts = optimset('MaxFunEvals', 6000, 'MaxIter', 6000, 'TolX', 1e-7, 'TolFun', 1e-7);
    fz = []; fx = [];
    for r = runs
        d = load(fullfile(dirDrive, sprintf('B2356raw%d.mat', r)));
        z = -d.FZ * N2lb; x = d.FX * N2lb; sl = d.SL;
        mask = abs(d.IA) < 0.5 & abs(d.SA) < 0.3 & z > 25 & abs(sl) > 0.02 & abs(sl) < 0.3;
        bin = round(z/fzStep)*fzStep;
        for k = unique(bin(mask))'
            if k < 50 || k > 250, continue; end
            sel = mask & bin == k & abs(z - k) < fzTol;
            if sum(sel) < minPts, continue; end
            s = sl(sel); f = x(sel);
            % the sweep has to reach |SL| >= 0.12 on both sides to see the peak
            if min(s) > -0.12 || max(s) < 0.12, continue; end
            best = inf;
            for B0 = [8 15 25], for C0 = [1.2 1.6 2], for E0 = [0 -1 0.5]
                cost = @(p) sum(min((mf(p,s) - f).^2, 200^2)) ...
                    + 1e6*(p(2) < 1 || p(2) > 2.5 || p(1) < 0 || p(3) < 0);
                [p, c] = fminsearch(cost, [max(abs(f)) C0 B0 E0], opts);
                if c < best, best = c; pBest = p; end
            end, end, end
            fz(end+1,1) = mean(z(sel));
            fx(end+1,1) = max(abs(mf(pBest, sGrid)));   % peak of the fitted curve
        end
    end
end
