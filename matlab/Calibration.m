clear; clc; close all;
% Written By: Rasit Evduzen
% Date: 18-May-2026
% BNO055 9DOF IMU Calibration

%% Config
PORT           = 'COM18';
BAUD           = 115200;
GYRO_SAMPLES   = 500;
ACCEL_SAMPLES  = 200;
MAG_DURATION   = 30;

% Serial Port
s = serialport(PORT, BAUD);
configureTerminator(s, "CR/LF");
s.InputBufferSize = 500000;
readline(s); readline(s); readline(s);

fprintf('\n========================================\n');
fprintf('       BNO055 Calibration Tool\n');
fprintf('========================================\n\n');

% Storage
gyro_raw = zeros(GYRO_SAMPLES, 3);
mag_raw  = [];

%% Gyro Calibration
% Gyro Bias Estimation — Mathematical Formulation
%
% Gyroscope output model:
%   w_meas = w_true + b + n
%   where:
%     w_meas = measured angular rate (deg/s)
%     w_true = true angular rate (deg/s)
%     b      = constant bias (offset)
%     n      = zero-mean Gaussian noise
%
% When sensor is COMPLETELY STILL: w_true = 0
%   w_meas = b + n
%
% Bias estimate (mean of N samples):
%   b = (1/N) * sum(w_meas_i)   i = 1..N
%
% Apply calibration:
%   w_cal = w_meas - b

fprintf('---  1: Gyro Calibration ---\n');
fprintf('Place sensor on flat surface and keep COMPLETELY STILL.\n');
fprintf('Press Enter when ready...\n');
pause;

fprintf('Collecting %d samples...\n', GYRO_SAMPLES);
idx = 0;
while idx < GYRO_SAMPLES
    if s.NumBytesAvailable > 0
        try
            line = readline(s);
            vals = str2double(split(line, ','));
            if length(vals) ~= 18 || any(isnan(vals)); continue; end
            idx = idx + 1;
            gyro_raw(idx,1) = -vals(5) / 16;   % X axis inverted | 1 °/s = 16 LSB
            gyro_raw(idx,2) = -vals(6) / 16;   % Y axis inverted | 1 °/s = 16 LSB
            gyro_raw(idx,3) = -vals(7) / 16;   % Z axis inverted | 1 °/s = 16 LSB
            if mod(idx, 100) == 0
                fprintf('  %d / %d samples\n', idx, GYRO_SAMPLES);
            end
        catch; end
    end
end

gyro_offset = mean(gyro_raw, 1);
fprintf('Gyro offset: GX=%.4f  GY=%.4f  GZ=%.4f deg/s\n\n', ...
    gyro_offset(1), gyro_offset(2), gyro_offset(3));

%% Accel Calibration (LSE)
% Accel LSE Calibration — Mathematical Formulation
%
% Accelerometer output model:
%   a_meas = W * a_true + b + n
%   where:
%     a_meas = measured acceleration (m/s²)
%     a_true = true acceleration (m/s²)
%     W      = 3x3 scale + cross-axis error matrix
%     b      = 1x3 bias vector
%     n      = zero-mean Gaussian noise
%
% For 6 static positions, gravity is the known reference:
%   g_ref = [0,  0, -g]   (Z+ up)
%           [0,  0, +g]   (Z- up)
%           [-g, 0,  0]   (X+ up)
%           [+g, 0,  0]   (X- up)
%           [0, -g,  0]   (Y+ up)
%           [0, +g,  0]   (Y- up)
%
% LSE problem (N=6 equations, 4 unknowns per axis):
%   [a_means | ones] * [W; b] = g_ref
%   A_lse * params = g_ref
%   params = A_lse \ g_ref
%
% Extract:
%   W = params(1:3, :)   (3x3)
%   b = params(4,   :)   (1x3)
%
% Apply calibration:
%   a_cal = a_meas * W + b

fprintf('---  2: Accel Calibration ---\n');
fprintf('You will be asked to place the sensor in 6 positions.\n\n');

positions = {
    'Z+ UP   (sensor flat, chip facing up)',
    'Z- UP   (sensor upside down)',
    'X+ UP   (sensor on left side)',
    'X- UP   (sensor on right side)',
    'Y+ UP   (sensor on front edge)',
    'Y- UP   (sensor on back edge)'
};

accel_means = zeros(6, 3);

for pos = 1:6
    fprintf('Position %d/6: %s\n', pos, positions{pos});
    fprintf('Press Enter when ready and HOLD STILL...\n');
    pause;

    fprintf('Collecting %d samples...\n', ACCEL_SAMPLES);
    buf = zeros(ACCEL_SAMPLES, 3);
    idx = 0;
    while idx < ACCEL_SAMPLES
        if s.NumBytesAvailable > 0
            try
                line = readline(s);
                vals = str2double(split(line, ','));
                if length(vals) ~= 18 || any(isnan(vals)); continue; end
                idx = idx + 1;
                buf(idx,1) = -vals(2) / 100;   % X axis inverted | 1 m/s² = 100 LSB
                buf(idx,2) = -vals(3) / 100;   % Y axis inverted | 1 m/s² = 100 LSB
                buf(idx,3) = -vals(4) / 100;   % Z axis inverted | 1 m/s² = 100 LSB
            catch; end
        end
    end

    accel_means(pos,:) = mean(buf, 1);
    fprintf('  Mean: AX=%.4f  AY=%.4f  AZ=%.4f m/s²\n\n', ...
        accel_means(pos,1), accel_means(pos,2), accel_means(pos,3));
end

% Known gravity reference — NED convention (Z down)
g = 9.81;
g_ref = [
     0,    0,   -g;   % Z+ up  → az = -g
     0,    0,    g;   % Z- up  → az = +g
    -g,    0,    0;   % X+ up  → ax = -g
     g,    0,    0;   % X- up  → ax = +g
     0,   -g,    0;   % Y+ up  → ay = -g
     0,    g,    0;   % Y- up  → ay = +g
];

% LSE solution
A_lse  = [accel_means, ones(6,1)];
params = A_lse \ g_ref;

accel_W      = params(1:3,:);
accel_offset = params(4,:);
accel_cal_means = accel_means * accel_W + accel_offset;

fprintf('Accel offset: AX=%.4f  AY=%.4f  AZ=%.4f\n', ...
    accel_offset(1), accel_offset(2), accel_offset(3));
fprintf('Accel W matrix:\n');
disp(accel_W);

%% Mag Calibration (LSE ellipsoid)
% LSE Ellipsoid Fitting — Mathematical Formulation
%
% Ideal magnetometer output lies on a sphere:
%   (mx - ox)^2 + (my - oy)^2 + (mz - oz)^2 = r^2
%
% Real output is distorted by:
%   Hard iron:  additive offset (ox, oy, oz) from permanent magnets nearby
%   Soft iron:  multiplicative distortion (W) from ferromagnetic materials
%
% Distorted ellipsoid equation (general form):
%   x' * M * x + n' * x + d = 0
%   where x = [mx, my, mz]'
%
% Expanded design matrix (9 unknowns):
%   A = [mx^2, my^2, mz^2, 2mx*my, 2mx*mz, 2my*mz, 2mx, 2my, 2mz]
%   A * params = 1   (LSE problem)
%   params = A \ ones(N,1)
%
% Extract symmetric matrix M and vector n from params:
%   M = [p(1) p(4) p(5)]     n = [p(7)]
%       [p(4) p(2) p(6)]         [p(8)]
%       [p(5) p(6) p(3)]         [p(9)]
%
% Hard iron offset (ellipsoid center):
%   o = -M^{-1} * n
%
% Soft iron correction matrix W (maps ellipsoid → sphere):
%   Eigendecomposition: M = V * D * V'
%   W = V * diag(1/sqrt(diag(D))) * V'
%   W = W / W(1,1)   (normalize)
%
% Apply calibration:
%   m_cal = (m_raw - o') * W

fprintf('---  3: Mag Calibration ---\n');
fprintf('Rotate sensor slowly in ALL directions for %d seconds.\n', MAG_DURATION);
fprintf('Cover all orientations — figure-8, tumble, rotate.\n');
fprintf('Press Enter to start...\n');
pause;

fprintf('Collecting mag data for %d seconds...\n', MAG_DURATION);
t_start = tic;
mag_raw = [];
while toc(t_start) < MAG_DURATION
    if s.NumBytesAvailable > 0
        try
            line = readline(s);
            vals = str2double(split(line, ','));
            if length(vals) ~= 18 || any(isnan(vals)); continue; end
            mag_raw(end+1,:) = [-vals(8)/16, -vals(9)/16, -vals(10)/16]; %#ok
            % X Y Z axis inverted | 1 µT = 16 LSB
            remaining = MAG_DURATION - toc(t_start);
            if mod(size(mag_raw,1), 50) == 0
                fprintf('  %d samples | %.1f s remaining\n', size(mag_raw,1), remaining);
            end
        catch; end
    end
end

fprintf('Collected %d mag samples.\n\n', size(mag_raw,1));

fprintf('Computing LSE ellipsoid fit...\n');
mx = mag_raw(:,1); my = mag_raw(:,2); mz = mag_raw(:,3);

A = [mx.^2, my.^2, mz.^2, 2*mx.*my, 2*mx.*mz, 2*my.*mz, 2*mx, 2*my, 2*mz];
b_vec = ones(size(mx));
params = A \ b_vec;

M = [params(1), params(4), params(5);
     params(4), params(2), params(6);
     params(5), params(6), params(3)];
n_vec = [params(7); params(8); params(9)];

mag_offset  = -(M \ n_vec);
[V, D]      = eig(M);
soft_iron_W = V * diag(1./sqrt(diag(D))) * V';
soft_iron_W = soft_iron_W / soft_iron_W(1,1);

fprintf('Mag hard iron offset: MX=%.4f  MY=%.4f  MZ=%.4f uT\n', ...
    mag_offset(1), mag_offset(2), mag_offset(3));
fprintf('Soft iron matrix W:\n');
disp(soft_iron_W);

%% Save Calibration
calib.gyro_offset  = gyro_offset;
calib.accel_W      = accel_W;
calib.accel_offset = accel_offset;
calib.mag_offset   = mag_offset';
calib.mag_W        = soft_iron_W;
calib.date         = datestr(now, 'yyyymmdd_HHMMSS');
save('bno055_calib.mat', 'calib');
fprintf('\nCalibration saved to bno055_calib.mat\n');

%% Visualization
fprintf('\nPlotting results...\n');

channels   = {'X','Y','Z'};
colors     = {'r','g','b'};
pos_labels = {'Z+','Z-','X+','X-','Y+','Y-'};

%--- Figure 1: Gyro ---
figure('Name','Gyro Calibration','NumberTitle','off', ...
       'Position',[0 0 1920 1080],'Color','w');

for i = 1:3
    subplot(3,1,i);
    plot(gyro_raw(:,i), colors{i}, 'LineWidth', 2); hold on;
    yline(gyro_offset(i), 'k--', 'LineWidth', 1.5);
    hold off;
    title(sprintf('Gyro %s  Raw + Offset', channels{i}));
    ylabel('deg/s'); xlabel('Sample'); grid on;
    legend('Raw', sprintf('Offset = %.4f', gyro_offset(i)), ...
           'Location','northeast','FontSize',7);
end
sgtitle('Gyro Calibration');

%--- Figure 2: Accel ---
figure('Name','Accel Calibration','NumberTitle','off', ...
       'Position',[0 0 1920 1080],'Color','w');

for i = 1:3
    subplot(3,1,i);
    plot(1:6, accel_means(:,i),     colors{i},          'LineWidth', 2, ...
         'DisplayName','Raw'); hold on;
    plot(1:6, accel_cal_means(:,i), [colors{i} '--'],   'LineWidth', 1.5, ...
         'DisplayName','Calibrated');
    yline( g, 'k--', 'LineWidth', 1, 'HandleVisibility','off');
    yline(-g, 'k--', 'LineWidth', 1, 'HandleVisibility','off');
    yline( 0, 'k:',  'LineWidth', 1, 'HandleVisibility','off');
    hold off;
    title(sprintf('Accel %s  6 Positions', channels{i}));
    ylabel('m/s²'); xlabel('Position'); grid on;
    xticks(1:6); xticklabels(pos_labels);
    legend('Location','northeast','FontSize',7);
end
sgtitle('Accel Calibration (LSE)');

%--- Figure 3: Mag time series ---
mag_cal = (mag_raw - mag_offset') * soft_iron_W;
mx_c = mag_cal(:,1); my_c = mag_cal(:,2); mz_c = mag_cal(:,3);

figure('Name','Mag Calibration','NumberTitle','off', ...
       'Position',[0 0 1920 1080],'Color','w');

mag_raw_ch = {mx,   my,   mz  };
mag_cal_ch = {mx_c, my_c, mz_c};

for i = 1:3
    subplot(3,1,i);
    plot(mag_raw_ch{i}, colors{i}, 'LineWidth', 2, ...
         'DisplayName', sprintf('Raw M%s',  channels{i})); hold on;
    plot(mag_cal_ch{i}, 'k--',     'LineWidth', 2, ...
         'DisplayName', sprintf('Cal M%s',  channels{i})); hold off;
    title(sprintf('Mag %s', channels{i}));
    ylabel('uT'); xlabel('Sample'); grid on;
    legend('Location','northeast','FontSize',7);
end
sgtitle('Mag Calibration  Raw vs Calibrated');

%--- Figure 4: Mag 3D sphere ---
figure('Name','Mag 3D Sphere','NumberTitle','off', ...
       'Position',[0 0 1920 1080],'Color','w');

[cx_r, cy_r, cz_r, r_r] = fit_sphere(mx,   my,   mz  );
[cx_c, cy_c, cz_c, r_c] = fit_sphere(mx_c, my_c, mz_c);
[sx, sy, sz] = sphere(50);

hold on;
plot3(mx,   my,   mz,   'b.', 'MarkerSize', 2, 'DisplayName', 'Raw data');
surf(cx_r + r_r*sx, cy_r + r_r*sy, cz_r + r_r*sz, ...
    'FaceAlpha', 0.1, 'FaceColor', 'b', 'EdgeColor', 'none', ...
    'DisplayName', sprintf('Raw sphere r=%.1f', r_r));
plot3(mx_c, my_c, mz_c, 'r.', 'MarkerSize', 2, 'DisplayName', 'Calibrated data');
surf(cx_c + r_c*sx, cy_c + r_c*sy, cz_c + r_c*sz, ...
    'FaceAlpha', 0.1, 'FaceColor', 'r', 'EdgeColor', 'none', ...
    'DisplayName', sprintf('Calibrated sphere r=%.1f', r_c));
hold off;
axis equal; grid on; view(45,30);
xlabel('MX'); ylabel('MY'); zlabel('MZ');
legend('Location','northeast','FontSize',8);
title(sprintf('Raw vs Calibrated  |  Raw r=%.1f  Cal r=%.1f', r_r, r_c));

fprintf('Done!\n');

%% Functions
function [cx, cy, cz, r] = fit_sphere(x, y, z)
    A      = [2*x, 2*y, 2*z, ones(size(x))];
    b      = x.^2 + y.^2 + z.^2;
    params = A \ b;
    cx = params(1); cy = params(2); cz = params(3);
    r  = sqrt(params(4) + cx^2 + cy^2 + cz^2);
end
