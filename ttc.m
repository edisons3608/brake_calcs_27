pth = "C:\Users\esun3\Downloads\Data-20260715T141503Z-1-001\Data\A2356raw9.dat";

data = readmatrix(pth, 'FileType', 'text', 'NumHeaderLines', 3);

% Columns per the file header:
% ET V N SA IA RL RE P FX FY FZ MX MZ NFX NFY RST TSTI TSTC TSTO AmbTmp SR SL
SA = data(:,4);
IA = data(:,5);
FX = data(:,9);
FZ = data(:,11);
SR = data(:,21);

% Isolate straight-line braking/driving points: near-zero slip angle and
% camber, and a friction-limited slip ratio (not free-rolling), so Fx
% reflects the tire's longitudinal capacity at each load rather than a
% partial-slip transient. FZ is negative (compression) in the SAE sign
% convention used by this file, so flip it to a positive load magnitude.
mask = abs(SA) < 1 & abs(IA) < 1 & FZ < -10 & abs(SR) > 0.08;
fx = abs(FX(mask));
fz = -FZ(mask);

% Bin by load and take the median |Fx| observed in each bin. The raw
% per-point max is dominated by single-sample transient spikes (very
% noisy, especially at low Fz where the ratio blows up); the median of
% the already slip-limited points is a much more stable estimate of the
% tire's sustained longitudinal capacity at that load.
edges = 0:10:300;
binIdx = discretize(fz, edges);
nbins = max(binIdx);
fzPeak = nan(nbins,1);
fxPeak = nan(nbins,1);
for k = 1:nbins
    inBin = binIdx == k;
    if sum(inBin) > 5
        fzPeak(k) = mean(fz(inBin));
        fxPeak(k) = median(fx(inBin));
    end
end
valid = ~isnan(fzPeak);
fzPeak = fzPeak(valid);
fxPeak = fxPeak(valid);

% Fit Fx = a*Fz + b*Fz^2 (no intercept, least squares)
A = [fzPeak, fzPeak.^2];
coeffs = A \ fxPeak;
a = coeffs(1);
b = coeffs(2);

fprintf('Fx = %.4f*Fz + %.6f*Fz^2\n', a, b);
fprintf('max |Fx| in filtered data: %.2f lb at Fz = %.2f lb\n', max(fx), fz(fx==max(fx)));

fzFit = linspace(0, max(fzPeak), 100);
fxFit = a*fzFit + b*fzFit.^2;

figure;
scatter(fz, fx, 4, [0.7 0.7 0.7], 'filled'); hold on;
scatter(fzPeak, fxPeak, 40, 'b', 'filled');
plot(fzFit, fxFit, 'r-', 'LineWidth', 2);
xlabel('F_z (lb)'); ylabel('F_x (lb)');
legend('all points (|SA|,|IA|<1{\circ}, |SR|>0.08)','median per load bin','quadratic fit','Location','best');
title('Round 9 Run 69 (Hoosier 18.0x6.0-10) - F_x vs F_z');
grid on;
