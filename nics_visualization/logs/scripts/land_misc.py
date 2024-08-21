import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

no = "D"
print(no)

# Read the CSV files, skipping the first line
file1 = '../land2024/0801_land_uav_' + no + '.csv'
file2 = '../land2024/0801_land_led_' + no + '.csv'

data1 = pd.read_csv(file1, skiprows=1)
data2 = pd.read_csv(file2, skiprows=1)

# Extract the relevant columns for comparison
vx, vy, vz = data1['vx'], data1['vy'], data1['vz']
ugv_vx, ugv_vy, ugv_vyaw, t = data2['ugv_vx'], data2['ugv_vy'], data2['ugv_vyaw'], data2['t']
# st_x2, st_y2, st_z2, t2 = data2['st_x'], data2['st_y'], data2['st_z'], data2['t']


# Compute 3D velocity magnitude
velocity_3d = np.sqrt(vx**2 + vy**2 + vz**2)
mean_velocity_3d = np.mean(velocity_3d)
max_velocity_3d = np.max(velocity_3d)

# print(velocity_3d)
print("UAV V_mean: {}".format(mean_velocity_3d))
print("UAV V_max: {}".format(max_velocity_3d))


# plt.plot(t, velocity_3d, label='2D Velocity Magnitude (sqrt(vx^2 + vy^2))')
# plt.show()


# Compute 2D velocity magnitude
velocity_2d = np.sqrt(ugv_vx**2 + ugv_vy**2)
mean_velocity_2d = np.mean(velocity_2d)
max_velocity_2d = np.max(velocity_2d)

# print(velocity_2d)
# print(ugv_vx)
print("UGV V_mean: {}".format(mean_velocity_2d))
print("UGV V_max: {}".format(max_velocity_2d))
# print(mean_velocity_2d)
# print(max_velocity_2d)


# print(ugv_vyaw)
VR_mean = np.mean(np.abs(ugv_vyaw)) * 280 / np.pi * 180
print("UGV VR_mean: {}".format(VR_mean))
# print(np.max(ugv_vyaw) * 280)