clear all; close all; 

%% Read in bag file

tunnel1_bag = rosbag("tunnel1_vn100.bag");
circles_bag = rosbag("driving_in_circles.bag");

% Select topics
tunnel1_bag_imu = select(tunnel1_bag, 'Topic', '/imu');
tunnel1_bag_gps = select(tunnel1_bag, 'Topic', '/gps');
circles_bag_imu = select(circles_bag, 'Topic', '/imu');

% Read messages, return cell array of structures
tunnel1_imu_msgs = readMessages(tunnel1_bag_imu, 'DataFormat','struct');
tunnel1_gps_msgs = readMessages(tunnel1_bag_gps, 'DataFormat','struct');
circles_imu_msgs = readMessages(circles_bag_imu, 'DataFormat','struct');

% Get Time
tunnel1_times = tunnel1_bag_imu.MessageList.Time;
tunnel1_times = tunnel1_times - tunnel1_times(1);

% Extract IMU Data from Structs 
for ind = 1:size(tunnel1_imu_msgs)
      tunnel1_orientation(ind,1) = tunnel1_imu_msgs{ind}.IMU.Orientation.X;
      tunnel1_orientation(ind,2) = tunnel1_imu_msgs{ind}.IMU.Orientation.Y;
      tunnel1_orientation(ind,3) = tunnel1_imu_msgs{ind}.IMU.Orientation.Z;
      tunnel1_orientation(ind,4) = tunnel1_imu_msgs{ind}.IMU.Orientation.W;
      tunnel1_angvel(ind,1) = tunnel1_imu_msgs{ind}.IMU.AngularVelocity.X;
      tunnel1_angvel(ind,2) = tunnel1_imu_msgs{ind}.IMU.AngularVelocity.Y;
      tunnel1_angvel(ind,3) = tunnel1_imu_msgs{ind}.IMU.AngularVelocity.Z;
      tunnel1_accel(ind,1) = tunnel1_imu_msgs{ind}.IMU.LinearAcceleration.X;
      tunnel1_accel(ind,2) = tunnel1_imu_msgs{ind}.IMU.LinearAcceleration.Y;
      tunnel1_accel(ind,3) = tunnel1_imu_msgs{ind}.IMU.LinearAcceleration.Z;
      tunnel1_mag(ind,1) = tunnel1_imu_msgs{ind}.MagField.MagneticField_.X;
      tunnel1_mag(ind,2) = tunnel1_imu_msgs{ind}.MagField.MagneticField_.Y;
      tunnel1_mag(ind,3) = tunnel1_imu_msgs{ind}.MagField.MagneticField_.Z;
end

for ind = 1:size(circles_imu_msgs)
      circles_mag(ind,1) = circles_imu_msgs{ind}.MagField.MagneticField_.X;
      circles_mag(ind,2) = circles_imu_msgs{ind}.MagField.MagneticField_.Y;
      circles_mag(ind,3) = circles_imu_msgs{ind}.MagField.MagneticField_.Z;
end

% Convert Magnetometer Units to Gauss
circles_mag = circles_mag/10000;
tunnel1_mag = tunnel1_mag/10000;

% Extract GPS Data From Structs
for ind = 1:size(tunnel1_gps_msgs)
    tunnel1_utc(ind,:) = tunnel1_gps_msgs{ind}.UTC;
    tunnel1_utmeast(ind,:) = tunnel1_gps_msgs{ind}.UTMEasting;
    tunnel1_utmnorth(ind,:) = tunnel1_gps_msgs{ind}.UTMNorthing;
end

% Convert UTC to Secs
for ind = 1:length(tunnel1_utc)
    time_str = num2str(tunnel1_utc(ind));
    hours = time_str(1:2);
    mins = time_str(3:4);
    secs = time_str(5:6);
    tunnel1_gps_time(ind) = 3600*str2num(hours) + 60*str2num(mins) + str2num(secs);
end
tunnel1_gps_time = tunnel1_gps_time - tunnel1_gps_time(1);

%% Magnetometer Calibration

% Computer Hard and Soft Iron Magnetometer Coefficients
x = circles_mag(:,1); % Extract x and y data for readability
y = circles_mag(:,2);

% Find best fit ellipse using non-linear least squares optimization
init_weights = ones(5,1);
options = optimoptions('lsqnonlin','MaxIterations',2000,"Display",'iter');
Objective_Function = @(weights) (weights(1)*x.^2 + weights(2)*x.*y + weights(3)*y.^2 + weights(4)*x + weights(5)*y) - 1;
opt_weights = lsqnonlin(Objective_Function, init_weights, [], [], options);

% Compute Ellipse major axis, minor axis, and rotation angle
a = opt_weights(1); b = opt_weights(2); c = opt_weights(3); d = opt_weights(4); e = opt_weights(5); 
num = 2 * ((a*e^2 - b*d*e + c*d^2)/(4*a*c - b^2)+1);
% Major and Minor Axis
maj_ax = sqrt(num/(a + c + sqrt((a-c)^2+b^2)));
min_ax = sqrt(num/(a + c - sqrt((a-c)^2 + b^2))); 
% Center Coordinates
center_x = (b*e - 2*c*d)/(4*a*c - b^2); 
center_y = (b*d - 2*a*e)/(4*a*c - b^2);
% Rotation Angle
theta = 0.5 * atan(b/(a-c));

% Correct Raw Magnetometer data
sigma = min_ax/maj_ax;
S1 = sigma*cos(-theta)*cos(theta)-sin(-theta)*sin(theta);
S2 = sigma*cos(-theta)*sin(theta)+sin(-theta)*cos(theta);
S3 = -sigma*sin(-theta)*cos(theta)-cos(-theta)*sin(theta);
S4 = -sigma*sin(-theta)*sin(theta)+cos(-theta)*cos(theta);
S = [S1, S2; S3, S4];
corr_data = S*([x y]' - [center_x; center_y]);

% Plot magnetometer data before and after calibration
figure(1)
hold on
scatter(circles_mag(:,1),circles_mag(:,2),'r*')
scatter(corr_data(1,:),corr_data(2,:),'g*')

% Plot Ellipse approximations
t=0:pi/10:2*pi;
X = center_x + (maj_ax*cos(t)*cos(theta) - min_ax*sin(t)*sin(theta));
Y = center_y + (maj_ax*cos(t)*sin(theta) + min_ax*sin(t)*cos(theta));
corr_ellipse = S*([X; Y] - [center_x; center_y]);
plot(X,Y,'r--')
plot(corr_ellipse(1,:),corr_ellipse(2,:),'g--')
xlabel('X-Axis'), ylabel('Y-Axis')
legend('Uncorrect Data', 'Corrected Data', 'Uncorrected Ellipse','Corrected Ellipse')
%title('Corrected and Uncorrected Magnetometer Data'), fontname('Times New Roman'), grid on, axis equal, axis padded

%% Sensor Fusion

%  Raw Magnetometer Yaw Angle
tunnel1_mag_yaw = (180/pi) * (atan2(tunnel1_mag(:,1),tunnel1_mag(:,2)));

% Corrected Magnetometer Yaw Angle
corr_mag = S*(tunnel1_mag(:,1:2)' - [center_x; center_y]);
corr_mag_yaw = (180/pi) * (atan2(corr_mag(1,:),corr_mag(2,:)));
corr_mag_yaw = wrapTo180(corr_mag_yaw);

% Plot Raw and Corrected Magnetometer Yaw
figure(2) 
hold on 
scatter(tunnel1_mag(:,1),tunnel1_mag(:,2),'r*')
scatter(corr_mag(1,:),corr_mag(2,:),'g*')
plot(corr_ellipse(1,:),corr_ellipse(2,:),'g--')
plot(X,Y,'r--')
xlabel('X-Axis'), ylabel('Y-Axis')
legend('Uncorrect Data', 'Corrected Data', 'Uncorrected Ellipse','Corrected Ellipse')
%title('Corrected and Uncorrected Magnetometer Data'), fontname('Times New Roman'), grid on, axis equal, axis padded

figure(3)
hold on
plot(tunnel1_times,tunnel1_mag_yaw,'r-','LineWidth',1)
plot(tunnel1_times,corr_mag_yaw,'g-','LineWidth',1)
xlabel('Time Series (s)')
ylabel(['Yaw Angle (' char(176) ')'])
title('Raw and Calibrated Magnetometer Yaw Angle')
legend('Raw Magnetometer Yaw','Calibrated Magnetometer Yaw')
%fontname('Times New Roman')
grid on

% Integrate Angular Rate to get Yaw Angle
gyro_yaw = cumtrapz(tunnel1_times,tunnel1_angvel(:,3));
% Ensure Yaw Angle is between +-180
gyro_yaw = (180/pi) * wrapToPi(gyro_yaw);

% Plot Magnetometer Yaw and Gyro Yaw
figure(4)
subplot(2,1,1)
hold on
plot(tunnel1_times,corr_mag_yaw,'m-','LineWidth',1)
xlabel('Time Series (s)')
ylabel(['Yaw Angle (' char(176) ')'])
title('Yaw from Calibrated Magnetometer Data')
grid on
subplot(2,1,2)
plot(tunnel1_times,gyro_yaw,'b-','LineWidth',1)
xlabel('Time Series (s)')
ylabel(['Yaw Angle (' char(176) ')'])
title('Yaw from Integrated Angular Rate')
grid on
%fontname('Times New Roman')

% Apply Low Pass Filter to Magnetometer Data
fs = 1/mean(diff(tunnel1_times));
filtered_mag_yaw = lowpass(corr_mag_yaw,0.1,fs);

% Apply High Pass Filter to Gyro Data
% filtered_gyro_rate = highpass(driving_angvel(:,3),1e-8,fs);
[b,a] = butter(1, 0.004/(0.5/fs), "high");
filtered_gyro_rate = filtfilt(b,a,tunnel1_angvel(:,3));
filtered_gyro_yaw = cumtrapz(tunnel1_times,filtered_gyro_rate);
filtered_gyro_yaw = (180/pi) * wrapToPi(filtered_gyro_yaw);

% Plot Filtered Yaw Angles
figure(5)
subplot(2,1,1)
hold on
plot(tunnel1_times,corr_mag_yaw, '-r','LineWidth',1)
plot(tunnel1_times,filtered_mag_yaw, '-g','LineWidth',1)
xlabel('Time Series (s)')
ylabel(['Yaw Angle (' char(176) ')'])
legend('Unfiltered','Filtered')
title('Yaw from Calibrated Magnetometer Data')
grid on
%fontname('Times New Roman')
subplot(2,1,2)
hold on
plot(tunnel1_times,gyro_yaw, '-r','LineWidth',1')
plot(tunnel1_times,filtered_gyro_yaw, '-g','LineWidth',1')
xlabel('Time Series (s)')
ylabel(['Yaw Angle (' char(176) ')'])
legend('Unfiltered','Filtered')
title('Yaw from Integrated Angular Rate Data')
grid on
%fontname('Times New Roman')

% Apply Complimentary Filter
alpha = 0.9;
filtered_yaw = alpha*filtered_mag_yaw' + (1-alpha)*gyro_yaw;

% True Yaw Angle 
x = tunnel1_orientation(:,1);
y = tunnel1_orientation(:,2);
z = tunnel1_orientation(:,3);
w = tunnel1_orientation(:,4);
for i = 1:length(x)
    yaw(i) = (180/pi) * atan2(2*(w(i)*z(i)+x(i)*y(i)),1-2*(y(i)*y(i)+z(i)*z(i)));
end

figure(6)
hold on
plot(tunnel1_times,filtered_yaw,'-g','LineWidth',1')
plot(tunnel1_times,filtered_gyro_yaw, '-b','LineWidth',1')
plot(tunnel1_times,filtered_mag_yaw,'-m','LineWidth',1')
plot(tunnel1_times,yaw,'k-','LineWidth',1')
xlabel('Time Series (s)')
ylabel(['Yaw Angle (' char(176) ')'])
legend('Complimentary Filter','High Pass Filter - Gyro. Data','Low Pass Filter - Mag. Data', 'VN-100 Yaw')
title('Yaw Angle with Low Pass, High Pass and Complimentary Filter Compared to IMU Yaw')
grid on
%fontname('Times New Roman')

%% Forward Velocity Estimate

% Integrate Forward Acceleration to get an Estimate for Forward Velocity
raw_vel_x = cumtrapz(tunnel1_times,tunnel1_accel(:,1));
vel_x = cumtrapz(tunnel1_times,tunnel1_accel(:,1) - mean(tunnel1_accel(:,1)));
vel_y = cumtrapz(tunnel1_times,tunnel1_accel(:,2) - mean(tunnel1_accel(:,2)));

% Correct Velocity Data
forward_vel = detrend(vel_x);
forward_vel = 0.2*abs(forward_vel);

% Estimate Velocity From GPS Data
d_lat = diff(tunnel1_utmnorth);
d_long = diff(tunnel1_utmeast);
dx = sqrt(d_lat.^2 + d_long.^2);
dt = diff(tunnel1_gps_time)';
gps_vel = dx./dt;

% Plot Corrected and Uncorrected Velocity from Accel. and Velocity from GPS
figure(7)
hold on
plot(tunnel1_times, raw_vel_x, '-r','LineWidth',1)
plot(tunnel1_times, vel_x, '-y','LineWidth',1)
plot(tunnel1_times, forward_vel,'-g','LineWidth',1)
xlabel('Time Series (s)')
ylabel('Velocity (m/s)')
legend('Uncorrected','Acceleration Correction Applied','Velocity Correction Applied','Location','northwest')
title('Corrected and Uncorrected Velocity Estimate from Integrated Forward Acceleration')
%fontname('Times New Roman')
grid on
axis padded

figure(8)
subplot(2,1,1)
hold on
plot(tunnel1_times, forward_vel,'-m','LineWidth',1)
xlabel('Time Series (s)')
ylabel('Velocity (m/s)')
title('Velocity Estimate from Integrated Forward Acceleration')
%fontname('Times New Roman')
grid on
subplot(2,1,2)
hold on
plot(tunnel1_gps_time(1:20),gps_vel,'-b','LineWidth',1)
xlabel('Time Series (s)')
ylabel('Velocity (m/s)')
title('Velocity Estimate from GPS')
%fontname('Times New Roman')
grid on

%% Dead Reckoning 

% Integrate Forward Velocity to get an Estimate for Displacement
imu_displacement = cumtrapz(tunnel1_times,forward_vel);

% Scale UTM Northing and Easting
tunnel1_utmnorth = tunnel1_utmnorth - tunnel1_utmnorth(1);
tunnel1_utmeast = tunnel1_utmeast - tunnel1_utmeast(1);

% GPS Displacement 
gps_displacement = cumtrapz(tunnel1_gps_time(1:length(gps_vel)), gps_vel);

% Plot GPS and IMU Displacement over time
figure(9)
hold on 
subplot(2,1,1)
plot(tunnel1_times,imu_displacement,'m-','LineWidth',1)
xlabel('Time Series (s)')
ylabel('Displacement (m)')
title('Displacement Estimate from IMU')
%fontname('Times New Roman')
grid on
subplot(2,1,2)
plot(tunnel1_gps_time(1:20), gps_displacement,'b-','LineWidth',1)
xlabel('Time Series (s)')
ylabel('Displacement (m)')
title('Displacement Estimate from GPS')
%fontname('Times New Roman')
grid on

% Calculate omega*velocity
vel_x = cumtrapz(tunnel1_times,tunnel1_accel(:,1));
omega_xdot = tunnel1_angvel(:,3).*vel_x;
corr_omega_xdot = tunnel1_angvel(:,3).*forward_vel;
y_obs = lowpass(tunnel1_accel(:,2),0.01,fs);

figure(10)
subplot(3,1,1)
hold on
plot(tunnel1_times,omega_xdot, '-b', 'LineWidth',1)
plot(tunnel1_times,tunnel1_accel(:,2),'-m','LineWidth',1)
legend('\omega x','y_{obs}')
xlabel('Time Series (s)')
ylabel('Acceleration (m/s^{2})')
title('Y-Axis Acceleration - Uncorrected and Unfiltered')
%fontname('Times New Roman')
grid on
subplot(3,1,2)
hold on
plot(tunnel1_times,corr_omega_xdot, '-b', 'LineWidth',1)
plot(tunnel1_times,tunnel1_accel(:,2),'-m','LineWidth',1)
legend('\omega x','y_{obs}')
xlabel('Time Series (s)')
ylabel('Acceleration (m/s^{2})')
title('Y-Axis Acceleration - Corrected and Unfiltered')
%fontname('Times New Roman')
grid on
subplot(3,1,3)
hold on
plot(tunnel1_times,corr_omega_xdot, '-b', 'LineWidth',1)
plot(tunnel1_times,y_obs,'-m','LineWidth',1)
legend('\omega x','y_{obs}')
xlabel('Time Series (s)')
ylabel('Acceleration (m/s^{2})')
title('Y-Axis Acceleration - Corrected and Filtered')
%fontname('Times New Roman')
grid on

%% Trajectory Estimation

heading = deg2rad(filtered_mag_yaw)' - deg2rad(20);
delta_t = diff(tunnel1_times);
forward_vel = [0; delta_t].*tunnel1_accel(:,1);
forward_vel = 10*forward_vel;
v_easting = -forward_vel.*cos(heading);
v_northing = forward_vel.*sin(heading);

imu_northing = cumtrapz(tunnel1_times, v_northing);
imu_easting = cumtrapz(tunnel1_times, v_easting);

figure(11)
hold on
plot(tunnel1_utmeast,tunnel1_utmnorth,'-*','LineWidth',1)
plot(imu_easting,imu_northing,'LineWidth',1)
xlabel('Easting (m)')
ylabel('Northing (m)')
legend('GPS Trajectory','IMU Trajectory','Location','northwest')
title('IMU and GPS Easting anf Northing Trajectories')
%fontname('Times New Roman')
grid on

%% Extract second IMU
% Load data from the second IMU (MPU9250)
second_imu_data = readtable('tunnel_mpu9250.csv');

% Extract relevant columns
acceleration_second = [second_imu_data.Accel_x, second_imu_data.Accel_y, second_imu_data.Accel_z];
gyro_second = [second_imu_data.Gyro_x, second_imu_data.Gyro_y, second_imu_data.Gyro_z];
mag_second = [second_imu_data.Mag_x, second_imu_data.Mag_y, second_imu_data.Mag_z];

% Convert Magnetometer Units to Gauss
mag_second = mag_second / 10000;

% Assuming you have a constant sampling rate (replace 100 with your actual sampling rate)
sampling_rate_second = 100;
time_difference_second = 1 / sampling_rate_second;
timestamps_second = (0:(height(second_imu_data)-1)) * time_difference_second;

% Extract Magnetometer Data from Second IMU
x_second = mag_second(:,1); % Extract x and y data for readability
y_second = mag_second(:,2);

% Correct Raw Magnetometer data for the second IMU
sigma_second = min_ax / maj_ax;
S1_second = sigma_second * cos(-theta) * cos(theta) - sin(-theta) * sin(theta);
S2_second = sigma_second * cos(-theta) * sin(theta) + sin(-theta) * cos(theta);
S3_second = -sigma_second * sin(-theta) * cos(theta) - cos(-theta) * sin(theta);
S4_second = -sigma_second * sin(-theta) * sin(theta) + cos(-theta) * cos(theta);
S_second = [S1_second, S2_second; S3_second, S4_second];

t_second = 0:pi/10:2*pi;
X_second = center_x + (maj_ax * cos(t_second) * cos(theta) - min_ax * sin(t_second) * sin(theta));
Y_second = center_y + (maj_ax * cos(t_second) * sin(theta) + min_ax * sin(t_second) * cos(theta));
corr_ellipse_second = S_second * ([X_second; Y_second] - [center_x; center_y]);

%% Sensor Fusion for Second IMU

% Raw Magnetometer Yaw Angle for Second IMU
tunnel1_mag_yaw_second = (180/pi) * atan2(x_second, y_second);

% Corrected Magnetometer Yaw Angle for Second IMU
%corr_mag_second = S * ([second_imu_data.Mag_x; second_imu_data.Mag_y] - [center_x; center_y]);
corr_mag_second = S_second * ([x_second y_second]' - [center_x; center_y]);
corr_mag_yaw_second = (180/pi) * atan2(corr_mag_second(1,:), corr_mag_second(2,:));
corr_mag_yaw_second = wrapTo180(corr_mag_yaw_second);

% Plot Raw and Corrected Magnetometer Yaw for Second IMU
figure(12)
hold on
scatter(second_imu_data.Mag_x, second_imu_data.Mag_y, 'r*')
scatter(corr_mag_second(1,:), corr_mag_second(2,:), 'g*')
plot(x_second, y_second, 'r--')
plot(corr_ellipse_second(1,:), corr_ellipse_second(2,:), 'g--')
xlabel('X-Axis'), ylabel('Y-Axis')
legend('Uncorrected Data', 'Corrected Data', 'Uncorrected Ellipse', 'Corrected Ellipse')
title('Magnetometer Calibration for Second IMU')
grid on

% Plot Raw and Corrected Magnetometer Yaw for Second IMU
figure(13)
hold on
plot(timestamps_second, tunnel1_mag_yaw_second, 'r-', 'LineWidth', 1)
plot(timestamps_second, corr_mag_yaw_second, 'g-', 'LineWidth', 1)
xlabel('Time Series (s)')
ylabel(['Yaw Angle (' char(176) ')'])
title('Raw and Calibrated Magnetometer Yaw Angle for Second IMU')
legend('Raw Magnetometer Yaw', 'Calibrated Magnetometer Yaw')
grid on

% Integrate Angular Rate to get Yaw Angle for Second IMU
gyro_yaw_second = cumtrapz(timestamps_second, gyro_second(:,3));
% Ensure Yaw Angle is between +-180
gyro_yaw_second = (180/pi) * wrapToPi(gyro_yaw_second);

% Plot Magnetometer Yaw and Gyro Yaw for Second IMU
figure(14)
subplot(2,1,1)
hold on
plot(timestamps_second, corr_mag_yaw_second, 'm-', 'LineWidth', 1)
xlabel('Time Series (s)')
ylabel(['Yaw Angle (' char(176) ')'])
title('Yaw from Calibrated Magnetometer Data for Second IMU')
grid on
subplot(2,1,2)
plot(timestamps_second, gyro_yaw_second, 'b-', 'LineWidth', 1)
xlabel('Time Series (s)')
ylabel(['Yaw Angle (' char(176) ')'])
title('Yaw from Integrated Angular Rate for Second IMU')
grid on

% Apply Low Pass Filter to Magnetometer Data for Second IMU
fs_second = 1 / mean(diff(timestamps_second));
filtered_mag_yaw_second = lowpass(corr_mag_yaw_second, 0.1, fs_second);

% Apply High Pass Filter to Gyro Data for Second IMU
[b_second, a_second] = butter(1, 0.004/(0.5/fs_second), 'high');
filtered_gyro_rate_second = filtfilt(b_second, a_second, gyro_second(:,3));
filtered_gyro_yaw_second = cumtrapz(timestamps_second, filtered_gyro_rate_second);
filtered_gyro_yaw_second = (180/pi) * wrapToPi(filtered_gyro_yaw_second);

% Plot Filtered Yaw Angles for Second IMU
figure(15)
subplot(2,1,1)
hold on
plot(timestamps_second, corr_mag_yaw_second, '-r', 'LineWidth', 1)
plot(timestamps_second, filtered_mag_yaw_second, '-g', 'LineWidth', 1)
xlabel('Time Series (s)')
ylabel(['Yaw Angle (' char(176) ')'])
legend('Unfiltered', 'Filtered')
title('Yaw from Calibrated Magnetometer Data for Second IMU')
grid on
subplot(2,1,2)
hold on
plot(timestamps_second, gyro_yaw_second, '-r', 'LineWidth', 1)
plot(timestamps_second, filtered_gyro_yaw_second, '-g', 'LineWidth', 1)
xlabel('Time Series (s)')
ylabel(['Yaw Angle (' char(176) ')'])
legend('Unfiltered', 'Filtered')
title('Yaw from Integrated Angular Rate Data for Second IMU')
grid on

% Apply Complimentary Filter for Second IMU
alpha_second = 0.9;
filtered_yaw_second = alpha_second * filtered_mag_yaw_second' + (1 - alpha_second) * gyro_yaw_second;

% True Yaw Angle for the second IMU
x_second = second_imu_data.Roll; % Assuming Roll corresponds to x-axis in quaternion
y_second = second_imu_data.Pitch; % Assuming Pitch corresponds to y-axis in quaternion
z_second = second_imu_data.Yaw; % Assuming Yaw corresponds to z-axis in quaternion
w_second = zeros(size(x_second)); % Assuming w is not available in the provided data

yaw_second = zeros(size(x_second));
for i = 1:length(x_second)
    yaw_second(i) = (180/pi) * atan2(2*(w_second(i)*z_second(i)+x_second(i)*y_second(i)),1-2*(y_second(i)*y_second(i)+z_second(i)*z_second(i)));
end

figure(16)
hold on
plot(timestamps_second, filtered_yaw_second, '-g', 'LineWidth', 1)
plot(timestamps_second, filtered_gyro_yaw_second, '-b', 'LineWidth', 1)
plot(timestamps_second, filtered_mag_yaw_second, '-m', 'LineWidth', 1)
plot(timestamps_second, yaw_second, 'k-', 'LineWidth', 1)
xlabel('Time Series (s)')
ylabel(['Yaw Angle (' char(176) ')'])
legend('Complimentary Filter', 'High Pass Filter - Gyro. Data', 'Low Pass Filter - Mag. Data', 'Second IMU Yaw')
title('Yaw Angle with Low Pass, High Pass and Complimentary Filter Compared to Second IMU Yaw')
grid on

%% Forward Velocity Estimate for the Second IMU

% Integrate Forward Acceleration to get an Estimate for Forward Velocity
raw_vel_x_second = cumtrapz(timestamps_second, second_imu_data.Accel_x);
vel_x_second = cumtrapz(timestamps_second, second_imu_data.Accel_x - mean(second_imu_data.Accel_x));
vel_y_second = cumtrapz(timestamps_second, second_imu_data.Accel_y - mean(second_imu_data.Accel_y));

% Correct Velocity Data
forward_vel_second = detrend(vel_x_second);
forward_vel_second = 0.2 * abs(forward_vel_second);

% Estimate Velocity From GPS Data (Assuming similar data structure as the first part)
d_lat_second = diff(tunnel1_utmnorth);
d_long_second = diff(tunnel1_utmeast);
dx_second = sqrt(d_lat_second.^2 + d_long_second.^2);
dt_second = diff(tunnel1_gps_time)';
gps_vel_second = dx_second ./ dt_second;

% Plot Corrected and Uncorrected Velocity from Accel. and Velocity from GPS
figure(7)
hold on
plot(timestamps_second, raw_vel_x_second, '-r','LineWidth',1)
plot(timestamps_second, vel_x_second, '-y','LineWidth',1)
plot(timestamps_second, forward_vel_second,'-g','LineWidth',1)
xlabel('Time Series (s)')
ylabel('Velocity (m/s)')
legend('Uncorrected','Acceleration Correction Applied','Velocity Correction Applied','Location','northwest')
title('Corrected and Uncorrected Velocity Estimate from Integrated Forward Acceleration (Second IMU)')
grid on
axis padded

figure(8)
subplot(2,1,1)
hold on
plot(timestamps_second, forward_vel_second,'-m','LineWidth',1)
xlabel('Time Series (s)')
ylabel('Velocity (m/s)')
title('Velocity Estimate from Integrated Forward Acceleration (Second IMU)')
grid on

subplot(2,1,2)
hold on
plot(tunnel1_gps_time(1:20), gps_vel_second, '-b','LineWidth',1)
xlabel('Time Series (s)')
ylabel('Velocity (m/s)')
title('Velocity Estimate from GPS (Second IMU)')
grid on

%% Dead Reckoning for the Second IMU

% Integrate Forward Velocity to get an Estimate for Displacement
imu_displacement_second = cumtrapz(timestamps_second, forward_vel_second);

% Scale UTM Northing and Easting
tunnel1_utmnorth_second = tunnel1_utmnorth - tunnel1_utmnorth(1);
tunnel1_utmeast_second = tunnel1_utmeast - tunnel1_utmeast(1);

% GPS Displacement (Assuming similar data structure as the first part)
gps_displacement_second = cumtrapz(tunnel1_gps_time(1:length(gps_vel_second)), gps_vel_second);

% Plot GPS and IMU Displacement over time
figure(9)
hold on 
subplot(2,1,1)
plot(timestamps_second, imu_displacement_second, 'm-', 'LineWidth', 1)
xlabel('Time Series (s)')
ylabel('Displacement (m)')
title('Displacement Estimate from IMU (Second IMU)')
grid on

subplot(2,1,2)
plot(tunnel1_gps_time(1:20), gps_displacement_second, 'b-', 'LineWidth', 1)
xlabel('Time Series (s)')
ylabel('Displacement (m)')
title('Displacement Estimate from GPS (Second IMU)')
grid on

% Calculate omega*velocity
vel_x_second = cumtrapz(timestamps_second, second_imu_data.Accel_x);
omega_xdot_second = second_imu_data.Gyro_z .* vel_x_second;
corr_omega_xdot_second = second_imu_data.Gyro_z .* forward_vel_second;
y_obs_second = lowpass(second_imu_data.Accel_y, 0.01, fs);

figure(10)
subplot(3,1,1)
hold on
plot(timestamps_second, omega_xdot_second, '-b', 'LineWidth', 1)
plot(timestamps_second, second_imu_data.Accel_y, '-m', 'LineWidth', 1)
legend('\omega x', 'y_{obs}')
xlabel('Time Series (s)')
ylabel('Acceleration (m/s^{2})')
title('Y-Axis Acceleration - Uncorrected and Unfiltered (Second IMU)')
grid on

subplot(3,1,2)
hold on
plot(timestamps_second, corr_omega_xdot_second, '-b', 'LineWidth', 1)
plot(timestamps_second, second_imu_data.Accel_y, '-m', 'LineWidth', 1)
legend('\omega x', 'y_{obs}')
xlabel('Time Series (s)')
ylabel('Acceleration (m/s^{2})')
title('Y-Axis Acceleration - Corrected and Unfiltered (Second IMU)')
grid on

subplot(3,1,3)
hold on
plot(timestamps_second, corr_omega_xdot_second, '-b', 'LineWidth', 1)
plot(timestamps_second, y_obs_second, '-m', 'LineWidth', 1)
legend('\omega x', 'y_{obs}')
xlabel('Time Series (s)')
ylabel('Acceleration (m/s^{2})')
title('Y-Axis Acceleration - Corrected and Filtered (Second IMU)')
grid on

