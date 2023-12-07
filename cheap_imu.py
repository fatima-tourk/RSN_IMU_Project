import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

# Read data from CSV
df = pd.read_csv('tunnel_mpu9250.csv')

# Extracting data columns
accel_x = df['Accel_x']
accel_y = df['Accel_y']
accel_z = df['Accel_z']
gyro_x = df['Gyro_x']
gyro_y = df['Gyro_y']
gyro_z = df['Gyro_z']
mag_x = df['Mag_x']
mag_y = df['Mag_y']
mag_z = df['Mag_z']
roll = df['Roll']
pitch = df['Pitch']
yaw = df['Yaw']

# Time array
time = np.arange(len(df))

# # Plotting each value over time
# plt.figure(figsize=(12, 8))

# plt.subplot(3, 3, 1)
# plt.plot(time, accel_x)
# plt.title('Acceleration X')

# plt.subplot(3, 3, 2)
# plt.plot(time, accel_y)
# plt.title('Acceleration Y')

# plt.subplot(3, 3, 3)
# plt.plot(time, accel_z)
# plt.title('Acceleration Z')

# plt.subplot(3, 3, 4)
# plt.plot(time, gyro_x)
# plt.title('Gyroscope X')

# plt.subplot(3, 3, 5)
# plt.plot(time, gyro_y)
# plt.title('Gyroscope Y')

# plt.subplot(3, 3, 6)
# plt.plot(time, gyro_z)
# plt.title('Gyroscope Z')

# plt.subplot(3, 3, 7)
# plt.plot(time, mag_x)
# plt.title('Magnetometer X')

# plt.subplot(3, 3, 8)
# plt.plot(time, mag_y)
# plt.title('Magnetometer Y')

# plt.subplot(3, 3, 9)
# plt.plot(time, mag_z)
# plt.title('Magnetometer Z')

# plt.tight_layout()
# plt.show()

velocity_x = np.cumsum(accel_x)
# velocity_y = np.cumsum(accel_y)
# velocity_z = np.cumsum(accel_z)

displacement_x = np.cumsum(velocity_x)
# displacement_y = np.cumsum(velocity_y)
# displacement_z = np.cumsum(velocity_z)

plt.figure(figsize=(12, 4))

plt.plot(time, velocity_x, label='velocity X')


#plt.plot(time, displacement_x, label='displacement X')
# plt.plot(time, displacement_y, label='Y')
# plt.plot(time, displacement_z, label='Z')
plt.title('Displacement over Time')
plt.legend()

# plt.subplot(1, 2, 2, projection='3d')
# plt.plot(displacement_x, displacement_y, displacement_z)
# plt.title('Trajectory')
# plt.xlabel('X')
# plt.ylabel('Y')

plt.tight_layout()
plt.show()
