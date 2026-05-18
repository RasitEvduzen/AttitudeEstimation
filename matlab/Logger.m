clear; clc; close all;
% Written By: Rasit Evduzen
% Date: 18-May-2026
% BNO055 9DOF IMU Data Logger

%% Config

PORT         = 'COM18';
BAUD         = 115200;
N            = 300;
UPDATE_EVERY = 10;
frame_len    = 10;


% Serial Port

s = serialport(PORT, BAUD);
configureTerminator(s, "CR/LF");
s.InputBufferSize = 500000;
readline(s); % Starting
readline(s); % Chip ID
readline(s); % CSV header


%% CSV Log

timestamp_str = datestr(now, 'yyyymmdd_HHMMSS');
filename = sprintf('bno055_log_%s.csv', timestamp_str);
fid = fopen(filename, 'w');
fprintf(fid, 'ts_us,ax,ay,az,gx,gy,gz,mx,my,mz,temp,roll,pitch,yaw,sys_cal,gyro_cal,acc_cal,mag_cal\n');
disp(['Logging to: ' filename]);


%% Ring (Circular) Buffer

ax_data   = zeros(1,N); ay_data   = zeros(1,N); az_data   = zeros(1,N);
gx_data   = zeros(1,N); gy_data   = zeros(1,N); gz_data   = zeros(1,N);
mx_data   = zeros(1,N); my_data   = zeros(1,N); mz_data   = zeros(1,N);
temp_data = zeros(1,N);
buf_idx   = 0;
buf_full  = false;
log_count = 0;
temp      = 0;
roll = 0; pitch = 0; yaw = 0;
sys_cal = 0; gyro_cal = 0; acc_cal = 0; mag_cal = 0;


%% Figure
figure('Name','BNO055 Data Logger','NumberTitle','off', 'Position',[0 0 1920 1080],'Color','w');

% Left  3D Attitude (BNO055 fusion)
sp_att = subplot(4,2,[1,3,5,7]);
hold(sp_att,'on'); grid(sp_att,'on');
axis(sp_att,[-frame_len frame_len -frame_len frame_len -frame_len frame_len]);
set(sp_att,'ZDir','reverse');
xlabel('X'); ylabel('Y'); zlabel('Z (down)');
title('BNO055 Attitude (fusion)'); view(3);
h_qx = quiver3(sp_att,0,0,0,frame_len,0,0,'r','LineWidth',2,'MaxHeadSize',0.5,'AutoScale','off');
h_qy = quiver3(sp_att,0,0,0,0,frame_len,0,'g','LineWidth',2,'MaxHeadSize',0.5,'AutoScale','off');
h_qz = quiver3(sp_att,0,0,0,0,0,frame_len,'b','LineWidth',2,'MaxHeadSize',0.5,'AutoScale','off');
legend(sp_att,'X','Y','Z','Location','northwest');

% Right row 1  Accel
sp_acc = subplot(4,2,2); hold on; grid on;
h_ax = plot(sp_acc,zeros(1,N),'r-');
h_ay = plot(sp_acc,zeros(1,N),'g-');
h_az = plot(sp_acc,zeros(1,N),'b-');
legend('AX','AY','AZ','Location','northwest','FontSize',7);
title('Accelerometer (m/s²)'); xlabel('Sample'); ylabel('m/s²');

% Right row 2  Gyro
sp_gyr = subplot(4,2,4); hold on; grid on;
h_gx = plot(sp_gyr,zeros(1,N),'r-');
h_gy = plot(sp_gyr,zeros(1,N),'g-');
h_gz = plot(sp_gyr,zeros(1,N),'b-');
legend('GX','GY','GZ','Location','northwest','FontSize',7);
title('Gyroscope (deg/s)'); xlabel('Sample'); ylabel('deg/s');

% Right row 3  Mag
sp_mag = subplot(4,2,6); hold on; grid on;
h_mx = plot(sp_mag,zeros(1,N),'r-');
h_my = plot(sp_mag,zeros(1,N),'g-');
h_mz = plot(sp_mag,zeros(1,N),'b-');
legend('MX','MY','MZ','Location','northwest','FontSize',7);
title('Magnetometer (uT)'); xlabel('Sample'); ylabel('uT');

% Right row 4  Temperature
sp_tmp = subplot(4,2,8); hold on; grid on;
h_temp = plot(sp_tmp,zeros(1,N),'k-','LineWidth',1.5);
title('Temperature (C)'); xlabel('Sample'); ylabel('C');

disp('Logging... Press Ctrl+C to stop.');


%% Main Loop
counter = 0;
try
    while true

        while s.NumBytesAvailable > 0
            try
                line = readline(s);
                vals = str2double(split(line, ','));
                if length(vals) ~= 18 || any(isnan(vals)); continue; end

                % Parse and scale
                ts       = vals(1);
                ax       = vals(2)  / 100;   % 1 m/s² = 100 LSB
                ay       = vals(3)  / 100;   % 1 m/s² = 100 LSB
                az       = vals(4)  / 100;   % 1 m/s² = 100 LSB
                gx       = vals(5)  / 16;    % 1 °/s = 16 LSB
                gy       = vals(6)  / 16;    % 1 °/s = 16 LSB
                gz       = vals(7)  / 16;    % 1 °/s = 16 LSB
                mx       = vals(8)  / 16;    % 1 µT = 16 LSB
                my       = vals(9)  / 16;    % 1 µT = 16 LSB
                mz       = vals(10) / 16;    % 1 µT = 16 LSB
                temp     = vals(11);         % Temperature in °C
                roll     = vals(12) / 16;    % 1° = 16 LSB
                pitch    = vals(13) / 16;    % 1° = 16 LSB
                yaw      = vals(14) / 16;    % 1° = 16 LSB
                sys_cal  = vals(15);         % 0-3, 3 = fully calibrated
                gyro_cal = vals(16);
                acc_cal  = vals(17);
                mag_cal  = vals(18);

                % Circular buffer
                buf_idx = mod(buf_idx, N) + 1;
                ax_data(buf_idx)   = ax;   ay_data(buf_idx)   = ay;   az_data(buf_idx)   = az;
                gx_data(buf_idx)   = gx;   gy_data(buf_idx)   = gy;   gz_data(buf_idx)   = gz;
                mx_data(buf_idx)   = mx;   my_data(buf_idx)   = my;   mz_data(buf_idx)   = mz;
                temp_data(buf_idx) = temp;
                if buf_idx == N; buf_full = true; end

                % Write to CSV
                fprintf(fid, '%.0f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.1f,%.4f,%.4f,%.4f,%d,%d,%d,%d\n', ...
                    ts, ax, ay, az, gx, gy, gz, mx, my, mz, temp, roll, pitch, yaw, ...
                    sys_cal, gyro_cal, acc_cal, mag_cal);
                log_count = log_count + 1;

                counter = counter + 1;
            catch;
            end
        end

        % Update plot
        if counter >= UPDATE_EVERY

            if buf_full
                idx_ord = [buf_idx+1:N, 1:buf_idx];
            else
                idx_ord = 1:buf_idx;
            end
            x_pts = 1:length(idx_ord);

            set(h_ax,'XData',x_pts,'YData',ax_data(idx_ord));
            set(h_ay,'XData',x_pts,'YData',ay_data(idx_ord));
            set(h_az,'XData',x_pts,'YData',az_data(idx_ord));
            set(h_gx,'XData',x_pts,'YData',gx_data(idx_ord));
            set(h_gy,'XData',x_pts,'YData',gy_data(idx_ord));
            set(h_gz,'XData',x_pts,'YData',gz_data(idx_ord));
            set(h_mx,'XData',x_pts,'YData',mx_data(idx_ord));
            set(h_my,'XData',x_pts,'YData',my_data(idx_ord));
            set(h_mz,'XData',x_pts,'YData',mz_data(idx_ord));
            set(h_temp,'XData',x_pts,'YData',temp_data(idx_ord));

            % 3D Attitude  BNO055 Euler
            r = roll  * pi/180;
            p = pitch * pi/180;
            y = yaw   * pi/180;
            R_vis = euler2rot([r; p; y])';
            set(h_qx,'UData',R_vis(1,1)*frame_len,'VData',R_vis(2,1)*frame_len,'WData',R_vis(3,1)*frame_len);
            set(h_qy,'UData',R_vis(1,2)*frame_len,'VData',R_vis(2,2)*frame_len,'WData',R_vis(3,2)*frame_len);
            set(h_qz,'UData',R_vis(1,3)*frame_len,'VData',R_vis(2,3)*frame_len,'WData',R_vis(3,3)*frame_len);

            title(sp_att, sprintf('R:%.1f  P:%.1f  Y:%.1f  |  T:%.1f C  |  CAL S:%d G:%d A:%d M:%d', ...
                roll, pitch, yaw, temp, sys_cal, gyro_cal, acc_cal, mag_cal));
            sgtitle(sprintf('BNO055 Logger  |  Samples: %d', log_count));

            drawnow;
            counter = 0;
        end
    end

catch ME
    fclose(fid);
    fprintf('Stopped. %d samples logged to: %s\n', log_count, filename);
    disp(ME.message);
end


%% Functions

function R = euler2rot(theta)
r = theta(1); p = theta(2); y = theta(3);
Rx = [1,0,0; 0,cos(r),-sin(r); 0,sin(r),cos(r)];
Ry = [cos(p),0,sin(p); 0,1,0; -sin(p),0,cos(p)];
Rz = [cos(y),-sin(y),0; sin(y),cos(y),0; 0,0,1];
R  = Rz * Ry * Rx;
end