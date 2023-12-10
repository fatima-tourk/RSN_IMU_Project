clear all; close all; 

%% Load data from VN-100

vn100_bag = rosbag("tunnel1_vn100.bag");

% Select topics
vn100_bag_imu = select(vn100_bag, 'Topic', '/imu');
bag_gps = select(vn100_bag, 'Topic', '/gps');

% Read messages, return cell array of structures
vn100_imu_msgs = readMessages(vn100_bag_imu, 'DataFormat','struct');
gps_msgs = readMessages(bag_gps, 'DataFormat','struct');

% Get Time
vn100_times = vn100_bag_imu.MessageList.Time;
vn100_times = vn100_times - vn100_times(1);

% Extract IMU Data from Structs 
for ind = 1:size(vn100_imu_msgs)
      vn100_orientation(ind,1) = vn100_imu_msgs{ind}.IMU.Orientation.X;
      vn100_orientation(ind,2) = vn100_imu_msgs{ind}.IMU.Orientation.Y;
      vn100_orientation(ind,3) = vn100_imu_msgs{ind}.IMU.Orientation.Z;
      vn100_orientation(ind,4) = vn100_imu_msgs{ind}.IMU.Orientation.W;
      vn100_angvel(ind,1) = vn100_imu_msgs{ind}.IMU.AngularVelocity.X;
      vn100_angvel(ind,2) = vn100_imu_msgs{ind}.IMU.AngularVelocity.Y;
      vn100_angvel(ind,3) = vn100_imu_msgs{ind}.IMU.AngularVelocity.Z;
      vn100_accel(ind,1) = vn100_imu_msgs{ind}.IMU.LinearAcceleration.X;
      vn100_accel(ind,2) = vn100_imu_msgs{ind}.IMU.LinearAcceleration.Y;
      vn100_accel(ind,3) = vn100_imu_msgs{ind}.IMU.LinearAcceleration.Z;
      vn100_mag(ind,1) = vn100_imu_msgs{ind}.MagField.MagneticField_.X;
      vn100_mag(ind,2) = vn100_imu_msgs{ind}.MagField.MagneticField_.Y;
      vn100_mag(ind,3) = vn100_imu_msgs{ind}.MagField.MagneticField_.Z;
end

% Convert Magnetometer Units to Gauss
vn100_mag = vn100_mag/10000;

% Extract GPS Data From Structs
for ind = 1:size(gps_msgs)
    utc(ind,:) = gps_msgs{ind}.UTC;
    utmeast(ind,:) = gps_msgs{ind}.UTMEasting;
    utmnorth(ind,:) = gps_msgs{ind}.UTMNorthing;
end

% Convert UTC to Secs
for ind = 1:length(utc)
    time_str = num2str(utc(ind));
    hours = time_str(1:2);
    mins = time_str(3:4);
    secs = time_str(5:6);
    gps_time(ind) = 3600*str2num(hours) + 60*str2num(mins) + str2num(secs);
end
gps_time = gps_time - gps_time(1);

%% Load data from MPU9250

mpu9250_imu_data = readtable('tunnel1_mpu9250.csv',ReadVariableNames=true);

% Extract relevant columns
mpu9250_accel = [mpu9250_imu_data.Accel_x, mpu9250_imu_data.Accel_y, mpu9250_imu_data.Accel_z];
mpu9250_angvel = [mpu9250_imu_data.Gyro_x, mpu9250_imu_data.Gyro_y, mpu9250_imu_data.Gyro_z];
mpu9250_mag = [mpu9250_imu_data.Mag_x, mpu9250_imu_data.Mag_y, mpu9250_imu_data.Mag_z];

% Convert Magnetometer Units to Gauss
mpu9250_mag = mpu9250_mag / 10000;

% Assuming you have a constant sampling rate (replace 100 with your actual sampling rate)
sampling_rate = 10;
time_difference = 1 / sampling_rate;
mpu9250_timestamps = (0:(height(mpu9250_imu_data)-1)) * time_difference;

%% Magnetometer Calibration

vn100_mag_corr = magCalibration(vn100_mag(:,1:2),1);
mpu9250_mag_corr = magCalibration(mpu9250_mag(:,1:2),2);

%% Plot Corrected and Uncorrected Magnetometer Yaw for Both IMUs

% Raw VN-100 Magnetometer Yaw Angle
vn100_mag_yaw = (180/pi) * (atan2(-vn100_mag(:,1),vn100_mag(:,2)));
vn100_mag_yaw = wrapTo180(vn100_mag_yaw);

% Corrected VN-100 Magnetometer Yaw Angle
vn100_mag_yaw_corr = (180/pi) * (atan2(-vn100_mag_corr(1,:),vn100_mag_corr(2,:)));
vn100_mag_yaw_corr = wrapTo180(vn100_mag_yaw_corr);

%  Raw MPU-9250 Magnetometer Yaw Angle
mpu9250_mag_yaw = (180/pi) * (atan2(-mpu9250_mag(:,1),mpu9250_mag(:,2)));
mpu9250_mag_yaw = wrapTo180(mpu9250_mag_yaw);

% Corrected Magnetometer Yaw Angle
mpu9250_mag_yaw_corr = (180/pi) * (atan2(-mpu9250_mag_corr(1,:),mpu9250_mag_corr(2,:)));
mpu9250_mag_yaw_corr = wrapTo180(mpu9250_mag_yaw_corr);

figure(3)
subplot(2,1,1)
hold on
plot(vn100_times,vn100_mag_yaw,'r-','LineWidth',1)
plot(vn100_times,vn100_mag_yaw_corr,'g-','LineWidth',1)
xlabel('Time Series (s)')
ylabel(['Yaw Angle (' char(176) ')'])
title('Raw and Calibrated Magnetometer Yaw Angle - VN-100')
legend('Raw Magnetometer Yaw','Calibrated Magnetometer Yaw')
fontname('Times New Roman')
grid on
subplot(2,1,2)
hold on
plot(mpu9250_timestamps,mpu9250_mag_yaw,'r-','LineWidth',1)
plot(mpu9250_timestamps,mpu9250_mag_yaw_corr,'g-','LineWidth',1)
xlabel('Time Series (s)')
ylabel(['Yaw Angle (' char(176) ')'])
title('Raw and Calibrated Magnetometer Yaw Angle - MPU-9250')
legend('Raw Magnetometer Yaw','Calibrated Magnetometer Yaw')
fontname('Times New Roman')
grid on

%% Apply Low Pass Filter to Magnetometer Data and Plot

% Apply Low Pass Filter to Magnetometer Data
fs = 1/mean(diff(vn100_times));
vn100_mag_yaw_filtered = lowpass(vn100_mag_yaw_corr,0.001,fs);
fs = 1/mean(diff(mpu9250_timestamps));
mpu9250_mag_yaw_filtered = lowpass(mpu9250_mag_yaw_corr,0.0001,fs);

figure(4)
subplot(2,1,1)
hold on
plot(vn100_times,vn100_mag_yaw,'r-','LineWidth',1)
plot(vn100_times,vn100_mag_yaw_filtered,'g-','LineWidth',1)
xlabel('Time Series (s)')
ylabel(['Yaw Angle (' char(176) ')'])
title('Raw and Calibrated Magnetometer Yaw Angle - VN-100')
legend('Raw Magnetometer Yaw','Calibrated Magnetometer Yaw')
fontname('Times New Roman')
grid on
subplot(2,1,2)
hold on
plot(mpu9250_timestamps,mpu9250_mag_yaw,'r-','LineWidth',1)
plot(mpu9250_timestamps,mpu9250_mag_yaw_filtered,'g-','LineWidth',1)
xlabel('Time Series (s)')
ylabel(['Yaw Angle (' char(176) ')'])
title('Raw and Calibrated Magnetometer Yaw Angle - MPU-9250')
legend('Raw Magnetometer Yaw','Calibrated Magnetometer Yaw')
fontname('Times New Roman')
grid on

%% Apply Complimentary Filter to Yaw Data

% Integrate Angular Rate to get Yaw Angle
vn100_gyro_yaw = cumtrapz(vn100_times,vn100_angvel(:,3));
vn100_gyro_yaw = (180/pi) * wrapToPi(vn100_gyro_yaw);
mpu9250_gyro_yaw = cumtrapz(mpu9250_timestamps,mpu9250_angvel(:,3));
mpu9250_gyro_yaw = (180/pi) * wrapToPi(mpu9250_gyro_yaw);

% Plot Magnetometer Yaw and Gyro Yaw
figure(5)
subplot(2,1,1)
hold on
plot(vn100_times,vn100_mag_yaw_filtered,'m-','LineWidth',1)
plot(vn100_times,vn100_gyro_yaw,'b-','LineWidth',1)
xlabel('Time Series (s)')
ylabel(['Yaw Angle (' char(176) ')'])
title('VN-100 Yaw')
legend('Magnetometer Yaw','Integrated Angular Rate')
grid on
subplot(2,1,2)
hold on
plot(mpu9250_timestamps,mpu9250_mag_yaw_filtered,'m-','LineWidth',1)
plot(mpu9250_timestamps,mpu9250_gyro_yaw,'b-','LineWidth',1)
xlabel('Time Series (s)')
ylabel(['Yaw Angle (' char(176) ')'])
title('MPU-9250 Yaw')
legend('Magnetometer Yaw','Integrated Angular Rate')
grid on
fontname('Times New Roman')

% Apply Complimentary Filter
alpha = 0.35;
vn100_yaw = alpha*vn100_mag_yaw_filtered' + (1-alpha)*vn100_gyro_yaw;
alpha = 0.8;
mpu9250_yaw = alpha*mpu9250_mag_yaw_filtered' + (1-alpha)*mpu9250_gyro_yaw;

% VN-100 EKF Yaw Angle 
x = vn100_orientation(:,1);
y = vn100_orientation(:,2);
z = vn100_orientation(:,3);
w = vn100_orientation(:,4);
for i = 1:length(x)
    yaw(i) = (180/pi) * atan2(2*(w(i)*z(i)+x(i)*y(i)),1-2*(y(i)*y(i)+z(i)*z(i)));
end

figure(6)
hold on
plot(vn100_times,vn100_yaw,'-m','LineWidth',1)
plot(mpu9250_timestamps,mpu9250_yaw,'-b','LineWidth',1)
plot(vn100_times,yaw,'k-','LineWidth',1')
xlabel('Time Series (s)')
ylabel(['Yaw Angle (' char(176) ')'])
legend('VN-100 Yaw - Complimentary Filt','MPU-9250 - Complimentary Filt', 'VN-100 - EKF')
title('Yaw Angle from Both IMUs')
grid on
fontname('Times New Roman')

%% Integrate Acceleration and Plot Forward Velocity

% Integrate Forward Acceleration to get an Estimate for Forward Velocity
vn100_vel = cumtrapz(vn100_times,vn100_accel(:,1));
mpu9250_vel = cumtrapz(mpu9250_timestamps,mpu9250_accel(:,1));

% Apply Corrections to Velocity Data
vn100_vel_corr = cumtrapz(vn100_times,vn100_accel(:,1) - mean(vn100_accel(:,1)));
vn100_vel_corr = abs(vn100_vel_corr);
mpu9250_vel_corr = cumtrapz(mpu9250_timestamps,mpu9250_accel(:,1) - mean(mpu9250_accel(:,1)));
mpu9250_vel_corr = abs(mpu9250_vel_corr);

% Estimating Velocity from GPS may not be useful since it's GPS denied
% Estimate Velocity From GPS Data
% d_lat = diff(utmnorth);
% d_long = diff(utmeast);
% dx = sqrt(d_lat.^2 + d_long.^2);
% dt = diff(gps_time)';
% gps_vel = dx./dt;

% Plot Corrected and Uncorrected Velocity from Accel. and Velocity from GPS
figure(7)
subplot(2,1,1)
hold on
plot(vn100_times, vn100_vel, '-m','LineWidth',1)
plot(mpu9250_timestamps, mpu9250_vel, '-b','LineWidth',1)
xlabel('Time Series (s)')
ylabel('Velocity (m/s)')
legend('VN-100','MPU-9250','Location','northwest')
title('Uncorrected Velocity Estimate')
grid on
subplot(2,1,2)
hold on
plot(vn100_times, vn100_vel_corr, '-m','LineWidth',1)
plot(mpu9250_timestamps, mpu9250_vel_corr, '-b','LineWidth',1)
xlabel('Time Series (s)')
ylabel('Velocity (m/s)')
legend('VN-100','MPU-9250','Location','northwest')
title('Corrected Velocity Estimate')
grid on
sgtitle('Velocity Estimate from Integrated Forward Acceleration')
fontname('Times New Roman')

%% Integrate Velcoity and Plot Displacement 

% Integrate Forward Velocity to get an Estimate for Displacement
vn100_displacement = cumtrapz(vn100_times,vn100_vel_corr);
mpu9250_displacement = cumtrapz(mpu9250_timestamps,mpu9250_vel_corr);

% GPS Displacement 
% gps_displacement = cumtrapz(gps_time(1:length(gps_vel)), gps_vel);

% Plot Displacement Over Time for Both IMUs
figure(8)
hold on 
plot(vn100_times,vn100_displacement,'m-','LineWidth',1)
plot(mpu9250_timestamps,mpu9250_displacement,'b-','LineWidth',1)
xlabel('Time Series (s)')
ylabel('Displacement (m)')
title('Displacement Estimate')
legend('VN-100','MPU-9250')
fontname('Times New Roman')
grid on

%% Trajectory Estimation

% Estimate trajectory from VN-100 EKF Yaw
ekf_heading = deg2rad(yaw');
ekf_vel_easting = vn100_vel_corr.*cos(ekf_heading);
ekf_vel_northing = vn100_vel_corr.*sin(ekf_heading);
ekf_northing = cumtrapz(vn100_times, ekf_vel_northing);
ekf_easting = cumtrapz(vn100_times, ekf_vel_easting);

% Estimate trajectory from VN-100 Complimentary Filter
vn100_heading = deg2rad(vn100_yaw) + deg2rad(90); % Align first leg of trajectory
vn100_vel_easting = vn100_vel_corr.*cos(vn100_heading);
vn100_vel_northing = vn100_vel_corr.*sin(vn100_heading);
vn100_northing = cumtrapz(vn100_times, vn100_vel_northing);
vn100_easting = cumtrapz(vn100_times, vn100_vel_easting);

% Estimate trajectory from MPU-9250 Complimentary Filter
mpu9250_heading = deg2rad(mpu9250_yaw) + deg2rad(90); % Align first leg of trajectory
mpu9250_vel_easting = mpu9250_vel_corr.*cos(mpu9250_heading);
mpu9250_vel_northing = mpu9250_vel_corr.*sin(mpu9250_heading);
mpu9250_northing = cumtrapz(mpu9250_timestamps, mpu9250_vel_northing);
mpu9250_easting = cumtrapz(mpu9250_timestamps, mpu9250_vel_easting);

%% Plot all trajectory Estimates Overlayed with GPS Data

% Scale GPS UTM Northing and Easting
utmnorth = utmnorth - utmnorth(1);
utmeast = utmeast - utmeast(1);

figure(9)
hold on
plot(utmeast,utmnorth,'-*','LineWidth',1)
plot(vn100_easting,vn100_northing,'-m','LineWidth',1)
plot(mpu9250_easting,mpu9250_northing,'-b','LineWidth',1)
plot(ekf_easting,ekf_northing,'-k','LineWidth',1)
xlabel('Easting (m)')
ylabel('Northing (m)')
legend('GPS Trajectory','VN-100 Complimentary Filter Trajectory','MPU-9250 Complimentary Filter Trajectory','VN-100 EKF Yaw','Location','northwest')
title('IMU and GPS Easting and Northing Trajectories')
fontname('Times New Roman')
grid on
axis equal

%% Plot GPS Data for Presentation

figure(10)
hold on
scatter(utmeast,utmnorth,'k*','LineWidth',1)
xlabel('Easting (m)')
ylabel('Northing (m)')
title('GPS Easting and Northing - Ted Williams Tunnel')
fontname('Times New Roman')
grid on
axis equal
axis padded

%% Define Sub-Functions

function [corr_data] = magCalibration(data,fignum)

    % Conduct PCA to fit Data inside an ellipse
    % Sample-estimates of mean vector and covariance matrix
    mu = mean(data,1);
    Sigma = 5*cov(data);
    
    % Subtract the estimated mean vector to make the data 0-mean
    xzm = data - mu;
    
    % Get the eigenvectors (Q) and eigenvalues (D) of the estimated covariance matrix
    [Q,D] = eig(Sigma);
    
    % Sort the eigenvalues from large to small, reorder eigenvectors accordingly as well.
    [d,ind] = sort(diag(D),'descend');
    Q = Q(:,ind);
    D = diag(d);
    
    % Scale eigenvectors
    VV = Q*sqrt(D);
    
    % Create ellipse 
    t = linspace(0,2*pi,100);
    e = [cos(t) ; sin(t)]; 
    y = VV*e + mu';
    
    % x and y vectors for readability
    x = y(1,:)';
    y = y(2,:)';
    
    % Find best fit ellipse using non-linear least squares optimization
    init_weights = ones(5,1);
    options = optimoptions('lsqnonlin','MaxIterations',2000,"Display",'iter');
    Objective_Function = @(weights) (weights(1)*x.^2 + weights(2)*x.*y + weights(3)*y.^2 + weights(4)*x + weights(5)*y) - 1;
    opt_weights = lsqnonlin(Objective_Function, init_weights, [], [], options);
    
    % Compute Ellipse major axis, minor axis, and rotation angle
    a = opt_weights(1); b = opt_weights(2); c = opt_weights(3); d = opt_weights(4); e = opt_weights(5); 
    num = 2 * ((a*e^2 - b*d*e + c*d^2)/(4*a*c - b^2)+1);
    % Major and Minor Axis
    min_ax = sqrt(num/(a + c + sqrt((a-c)^2+b^2)));
    maj_ax = sqrt(num/(a + c - sqrt((a-c)^2 + b^2))); 
    % Center Coordinates
    center_x = (b*e - 2*c*d)/(4*a*c - b^2); 
    center_y = (b*d - 2*a*e)/(4*a*c - b^2);
    % Rotation Angle
    theta = 0.5 * atan(b/(a-c));
    
    % Compute Hard and Soft Iron Magnetometer Coefficients
    sigma = min_ax/maj_ax;
    S1 = sigma*cos(-theta)*cos(theta)-sin(-theta)*sin(theta);
    S2 = sigma*cos(-theta)*sin(theta)+sin(-theta)*cos(theta);
    S3 = -sigma*sin(-theta)*cos(theta)-cos(-theta)*sin(theta);
    S4 = -sigma*sin(-theta)*sin(theta)+cos(-theta)*cos(theta);
    S = [S1, S2; S3, S4];
    
    % Correct Raw Magnetometer data
    corr_data = S*([data(:,1), data(:,2)]' - [center_x; center_y]);
    
    % Plot magnetometer data before and after calibration
    figure(fignum)
    hold on
    scatter(data(:,1),data(:,2),'r*')
    scatter(corr_data(1,:),corr_data(2,:),'g*')
    
    % Plot Ellipse approximations
    t=0:pi/10:2*pi;
    X = center_x + (maj_ax*cos(t)*cos(theta) - min_ax*sin(t)*sin(theta));
    Y = center_y + (maj_ax*cos(t)*sin(theta) + min_ax*sin(t)*cos(theta));
    corr_ellipse = S*([X; Y] - [center_x; center_y]);
    plot(X,Y,'k--')
    plot(corr_ellipse(1,:),corr_ellipse(2,:),'k--')
    xlabel('X-Axis'), ylabel('Y-Axis')
    legend('Uncorrect Data', 'Corrected Data', 'Uncorrected Ellipse','Corrected Ellipse')
    title('Corrected and Uncorrected Magnetometer Data'), fontname('Times New Roman'), grid on, axis equal, axis padded

end
