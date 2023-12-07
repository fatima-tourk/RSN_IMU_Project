clear all; close all; 

%% Read in bag file

tunnel2_bag = rosbag("tunnel2_vn100.bag");
circles_bag = rosbag("driving_in_circles.bag");

% Select topics
tunnel2_bag_imu = select(tunnel2_bag, 'Topic', '/imu');
tunnel2_bag_gps = select(tunnel2_bag, 'Topic', '/gps');
circles_bag_imu = select(circles_bag, 'Topic', '/imu');

% Read messages, return cell array of structures
tunnel2_imu_msgs = readMessages(tunnel2_bag_imu, 'DataFormat','struct');
tunnel2_gps_msgs = readMessages(tunnel2_bag_gps, 'DataFormat','struct');
circles_imu_msgs = readMessages(circles_bag_imu, 'DataFormat','struct');

% Get Time
tunnel2_times = tunnel2_bag_imu.MessageList.Time;
tunnel2_times = tunnel2_times - tunnel2_times(1);

% Extract IMU Data from Structs 
for ind = 1:size(tunnel2_imu_msgs)
      tunnel2_orientation(ind,1) = tunnel2_imu_msgs{ind}.IMU.Orientation.X;
      tunnel2_orientation(ind,2) = tunnel2_imu_msgs{ind}.IMU.Orientation.Y;
      tunnel2_orientation(ind,3) = tunnel2_imu_msgs{ind}.IMU.Orientation.Z;
      tunnel2_orientation(ind,4) = tunnel2_imu_msgs{ind}.IMU.Orientation.W;
      tunnel2_angvel(ind,1) = tunnel2_imu_msgs{ind}.IMU.AngularVelocity.X;
      tunnel2_angvel(ind,2) = tunnel2_imu_msgs{ind}.IMU.AngularVelocity.Y;
      tunnel2_angvel(ind,3) = tunnel2_imu_msgs{ind}.IMU.AngularVelocity.Z;
      tunnel2_accel(ind,1) = tunnel2_imu_msgs{ind}.IMU.LinearAcceleration.X;
      tunnel2_accel(ind,2) = tunnel2_imu_msgs{ind}.IMU.LinearAcceleration.Y;
      tunnel2_accel(ind,3) = tunnel2_imu_msgs{ind}.IMU.LinearAcceleration.Z;
      tunnel2_mag(ind,1) = tunnel2_imu_msgs{ind}.MagField.MagneticField_.X;
      tunnel2_mag(ind,2) = tunnel2_imu_msgs{ind}.MagField.MagneticField_.Y;
      tunnel2_mag(ind,3) = tunnel2_imu_msgs{ind}.MagField.MagneticField_.Z;
end

for ind = 1:size(circles_imu_msgs)
      circles_mag(ind,1) = circles_imu_msgs{ind}.MagField.MagneticField_.X;
      circles_mag(ind,2) = circles_imu_msgs{ind}.MagField.MagneticField_.Y;
      circles_mag(ind,3) = circles_imu_msgs{ind}.MagField.MagneticField_.Z;
end

% Convert Magnetometer Units to Gauss
circles_mag = circles_mag/10000;
tunnel2_mag = tunnel2_mag/10000;

% Extract GPS Data From Structs
for ind = 1:size(tunnel2_gps_msgs)
    tunnel2_utc(ind,:) = tunnel2_gps_msgs{ind}.UTC;
    tunnel2_utmeast(ind,:) = tunnel2_gps_msgs{ind}.UTMEasting;
    tunnel2_utmnorth(ind,:) = tunnel2_gps_msgs{ind}.UTMNorthing;
end

% Convert UTC to Secs
for ind = 1:length(tunnel2_utc)
    time_str = num2str(tunnel2_utc(ind));
    hours = time_str(1:2);
    mins = time_str(3:4);
    secs = time_str(5:6);
    tunnel2_gps_time(ind) = 3600*str2num(hours) + 60*str2num(mins) + str2num(secs);
end
tunnel2_gps_time = tunnel2_gps_time - tunnel2_gps_time(1);

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
title('Corrected and Uncorrected Magnetometer Data'), fontname('Times New Roman'), grid on, axis equal, axis padded

%% Sensor Fusion

%  Raw Magnetometer Yaw Angle
tunnel2_mag_yaw = (180/pi) * (atan2(tunnel2_mag(:,1),tunnel2_mag(:,2)));

% Corrected Magnetometer Yaw Angle
corr_mag = S*(tunnel2_mag(:,1:2)' - [center_x; center_y]);
corr_mag_yaw = (180/pi) * (atan2(corr_mag(1,:),corr_mag(2,:)));
corr_mag_yaw = wrapTo180(corr_mag_yaw);

% Plot Raw and Corrected Magnetometer Yaw
figure(2) 
hold on 
scatter(tunnel2_mag(:,1),tunnel2_mag(:,2),'r*')
scatter(corr_mag(1,:),corr_mag(2,:),'g*')
plot(corr_ellipse(1,:),corr_ellipse(2,:),'g--')
plot(X,Y,'r--')
xlabel('X-Axis'), ylabel('Y-Axis')
legend('Uncorrect Data', 'Corrected Data', 'Uncorrected Ellipse','Corrected Ellipse')
title('Corrected and Uncorrected Magnetometer Data'), fontname('Times New Roman'), grid on, axis equal, axis padded

figure(3)
hold on
plot(tunnel2_times,tunnel2_mag_yaw,'r-','LineWidth',1)
plot(tunnel2_times,corr_mag_yaw,'g-','LineWidth',1)
xlabel('Time Series (s)')
ylabel(['Yaw Angle (' char(176) ')'])
title('Raw and Calibrated Magnetometer Yaw Angle')
legend('Raw Magnetometer Yaw','Calibrated Magnetometer Yaw')
fontname('Times New Roman')
grid on

% Integrate Angular Rate to get Yaw Angle
gyro_yaw = cumtrapz(tunnel2_times,tunnel2_angvel(:,3));
% Ensure Yaw Angle is between +-180
gyro_yaw = (180/pi) * wrapToPi(gyro_yaw);

% Plot Magnetometer Yaw and Gyro Yaw
figure(4)
subplot(2,1,1)
hold on
plot(tunnel2_times,corr_mag_yaw,'m-','LineWidth',1)
xlabel('Time Series (s)')
ylabel(['Yaw Angle (' char(176) ')'])
title('Yaw from Calibrated Magnetometer Data')
grid on
subplot(2,1,2)
plot(tunnel2_times,gyro_yaw,'b-','LineWidth',1)
xlabel('Time Series (s)')
ylabel(['Yaw Angle (' char(176) ')'])
title('Yaw from Integrated Angular Rate')
grid on
fontname('Times New Roman')

% Apply Low Pass Filter to Magnetometer Data
fs = 1/mean(diff(tunnel2_times));
filtered_mag_yaw = lowpass(corr_mag_yaw,0.1,fs);

% Apply High Pass Filter to Gyro Data
% filtered_gyro_rate = highpass(driving_angvel(:,3),1e-8,fs);
[b,a] = butter(1, 0.004/(0.5/fs), "high");
filtered_gyro_rate = filtfilt(b,a,tunnel2_angvel(:,3));
filtered_gyro_yaw = cumtrapz(tunnel2_times,filtered_gyro_rate);
filtered_gyro_yaw = (180/pi) * wrapToPi(filtered_gyro_yaw);

% Plot Filtered Yaw Angles
figure(5)
subplot(2,1,1)
hold on
plot(tunnel2_times,corr_mag_yaw, '-r','LineWidth',1)
plot(tunnel2_times,filtered_mag_yaw, '-g','LineWidth',1)
xlabel('Time Series (s)')
ylabel(['Yaw Angle (' char(176) ')'])
legend('Unfiltered','Filtered')
title('Yaw from Calibrated Magnetometer Data')
grid on
fontname('Times New Roman')
subplot(2,1,2)
hold on
plot(tunnel2_times,gyro_yaw, '-r','LineWidth',1')
plot(tunnel2_times,filtered_gyro_yaw, '-g','LineWidth',1')
xlabel('Time Series (s)')
ylabel(['Yaw Angle (' char(176) ')'])
legend('Unfiltered','Filtered')
title('Yaw from Integrated Angular Rate Data')
grid on
fontname('Times New Roman')

% Apply Complimentary Filter
alpha = 0.1;
filtered_yaw = alpha*filtered_mag_yaw' + (1-alpha)*gyro_yaw;

% True Yaw Angle 
x = tunnel2_orientation(:,1);
y = tunnel2_orientation(:,2);
z = tunnel2_orientation(:,3);
w = tunnel2_orientation(:,4);
for i = 1:length(x)
    yaw(i) = (180/pi) * atan2(2*(w(i)*z(i)+x(i)*y(i)),1-2*(y(i)*y(i)+z(i)*z(i)));
end

figure(6)
hold on
plot(tunnel2_times,filtered_yaw,'-g','LineWidth',1')
plot(tunnel2_times,filtered_gyro_yaw, '-b','LineWidth',1')
plot(tunnel2_times,filtered_mag_yaw,'-m','LineWidth',1')
plot(tunnel2_times,yaw,'k-','LineWidth',1')
xlabel('Time Series (s)')
ylabel(['Yaw Angle (' char(176) ')'])
legend('Complimentary Filter','High Pass Filter - Gyro. Data','Low Pass Filter - Mag. Data', 'VN-100 Yaw')
title('Yaw Angle with Low Pass, High Pass and Complimentary Filter Compared to IMU Yaw')
grid on
fontname('Times New Roman')

%% Forward Velocity Estimate

% Integrate Forward Acceleration to get an Estimate for Forward Velocity
raw_vel_x = cumtrapz(tunnel2_times,tunnel2_accel(:,1));
vel_x = cumtrapz(tunnel2_times,tunnel2_accel(:,1) - mean(tunnel2_accel(:,1)));
vel_y = cumtrapz(tunnel2_times,tunnel2_accel(:,2) - mean(tunnel2_accel(:,2)));

% Correct Velocity Data
forward_vel = detrend(vel_x);
forward_vel = 0.2*abs(forward_vel);

% Estimate Velocity From GPS Data
d_lat = diff(tunnel2_utmnorth);
d_long = diff(tunnel2_utmeast);
dx = sqrt(d_lat.^2 + d_long.^2);
dt = diff(tunnel2_gps_time)';
gps_vel = dx./dt;

% Plot Corrected and Uncorrected Velocity from Accel. and Velocity from GPS
figure(7)
hold on
plot(tunnel2_times, raw_vel_x, '-r','LineWidth',1)
plot(tunnel2_times, vel_x, '-y','LineWidth',1)
plot(tunnel2_times, forward_vel,'-g','LineWidth',1)
xlabel('Time Series (s)')
ylabel('Velocity (m/s)')
legend('Uncorrected','Acceleration Correction Applied','Velocity Correction Applied','Location','northwest')
title('Corrected and Uncorrected Velocity Estimate from Integrated Forward Acceleration')
fontname('Times New Roman')
grid on
axis padded

figure(8)
subplot(2,1,1)
hold on
plot(tunnel2_times, forward_vel,'-m','LineWidth',1)
xlabel('Time Series (s)')
ylabel('Velocity (m/s)')
title('Velocity Estimate from Integrated Forward Acceleration')
fontname('Times New Roman')
grid on
subplot(2,1,2)
hold on
plot(tunnel2_gps_time(1:163),gps_vel,'-b','LineWidth',1)
xlabel('Time Series (s)')
ylabel('Velocity (m/s)')
title('Velocity Estimate from GPS')
fontname('Times New Roman')
grid on

%% Dead Reckoning 

% Integrate Forward Velocity to get an Estimate for Displacement
imu_displacement = cumtrapz(tunnel2_times,forward_vel);

% Scale UTM Northing and Easting
tunnel2_utmnorth = tunnel2_utmnorth - tunnel2_utmnorth(1);
tunnel2_utmeast = tunnel2_utmeast - tunnel2_utmeast(1);

% GPS Displacement 
gps_displacement = cumtrapz(tunnel2_gps_time(1:length(gps_vel)), gps_vel);

% Plot GPS and IMU Displacement over time
figure(9)
hold on 
subplot(2,1,1)
plot(tunnel2_times,imu_displacement,'m-','LineWidth',1)
xlabel('Time Series (s)')
ylabel('Displacement (m)')
title('Displacement Estimate from IMU')
fontname('Times New Roman')
grid on
subplot(2,1,2)
plot(tunnel2_gps_time(1:163), gps_displacement,'b-','LineWidth',1)
xlabel('Time Series (s)')
ylabel('Displacement (m)')
title('Displacement Estimate from GPS')
fontname('Times New Roman')
grid on

% Calculate omega*velocity
vel_x = cumtrapz(tunnel2_times,tunnel2_accel(:,1));
omega_xdot = tunnel2_angvel(:,3).*vel_x;
corr_omega_xdot = tunnel2_angvel(:,3).*forward_vel;
y_obs = lowpass(tunnel2_accel(:,2),0.01,fs);

figure(10)
subplot(3,1,1)
hold on
plot(tunnel2_times,omega_xdot, '-b', 'LineWidth',1)
plot(tunnel2_times,tunnel2_accel(:,2),'-m','LineWidth',1)
legend('\omega x','y_{obs}')
xlabel('Time Series (s)')
ylabel('Acceleration (m/s^{2})')
title('Y-Axis Acceleration - Uncorrected and Unfiltered')
fontname('Times New Roman')
grid on
subplot(3,1,2)
hold on
plot(tunnel2_times,corr_omega_xdot, '-b', 'LineWidth',1)
plot(tunnel2_times,tunnel2_accel(:,2),'-m','LineWidth',1)
legend('\omega x','y_{obs}')
xlabel('Time Series (s)')
ylabel('Acceleration (m/s^{2})')
title('Y-Axis Acceleration - Corrected and Unfiltered')
fontname('Times New Roman')
grid on
subplot(3,1,3)
hold on
plot(tunnel2_times,corr_omega_xdot, '-b', 'LineWidth',1)
plot(tunnel2_times,y_obs,'-m','LineWidth',1)
legend('\omega x','y_{obs}')
xlabel('Time Series (s)')
ylabel('Acceleration (m/s^{2})')
title('Y-Axis Acceleration - Corrected and Filtered')
fontname('Times New Roman')
grid on

%% Trajectory Estimation

heading = deg2rad(filtered_mag_yaw)';
delta_t = diff(tunnel2_times);
forward_vel = [0; delta_t].*tunnel2_accel(:,1);
forward_vel = 10*forward_vel;
v_easting = -forward_vel.*cos(heading);
v_northing = forward_vel.*sin(heading);

imu_northing = cumtrapz(tunnel2_times, v_northing);
imu_easting = cumtrapz(tunnel2_times, v_easting);

figure(11)
hold on
plot(tunnel2_utmeast,tunnel2_utmnorth,'-*','LineWidth',1)
plot(imu_easting,imu_northing,'LineWidth',1)
xlabel('Easting (m)')
ylabel('Northing (m)')
legend('GPS Trajectory','IMU Trajectory','Location','northwest')
title('IMU and GPS Easting anf Northing Trajectories')
fontname('Times New Roman')
grid on
