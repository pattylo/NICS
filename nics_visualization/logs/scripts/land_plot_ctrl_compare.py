import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

no1 = "B"
no2 = "C"

file_path1 = '../thesis/0821_ctrl_tracking_' + no1 + ".csv"
data1 = pd.read_csv(file_path1, skiprows=1)
file_path2 = '../thesis/0821_ctrl_tracking_' + no2 + ".csv"
data2 = pd.read_csv(file_path2, skiprows=1)

# Print column names to check if they are read correctly
print("Column names:", data1.columns)

# Convert columns to numeric if needed
numeric_columns = ['set_x', 'set_y', 'set_z', 'uav_x', 'uav_y', 'uav_z', 'e_x', 'e_y', 'e_z']
for col in numeric_columns:
    data1[col] = pd.to_numeric(data1[col], errors='coerce')
    data2[col] = pd.to_numeric(data2[col], errors='coerce')

# Calculate RMSE for e_x, e_y, and e_z for data1 and data2
rmse_ex1 = np.sqrt(np.mean(data1['e_x'] ** 2))
rmse_ey1 = np.sqrt(np.mean(data1['e_y'] ** 2))
rmse_ez1 = np.sqrt(np.mean(data1['e_z'] ** 2))

rmse_ex2 = np.sqrt(np.mean(data2['e_x'] ** 2))
rmse_ey2 = np.sqrt(np.mean(data2['e_y'] ** 2))
rmse_ez2 = np.sqrt(np.mean(data2['e_z'] ** 2))

# Print RMSE for data1
print("RMSE for e_x (PID): {}".format(rmse_ex1))
print("RMSE for e_y (PID): {}".format(rmse_ey1))
print("RMSE for e_z (PID): {}".format(rmse_ez1))

# Print RMSE for data2
print("RMSE for e_x (PID+ESO): {}".format(rmse_ex2))
print("RMSE for e_y (PID+ESO): {}".format(rmse_ey2))
print("RMSE for e_z (PID+ESO): {}".format(rmse_ez2))

# Plot the data1 and data2
fig, axs = plt.subplots(3, 1, figsize=(15, 10))
t1 = data1['t'] - data1['t'][0]
t2 = data2['t'] - data2['t'][0]
plt.rcParams.update({'font.size': 18, 'xtick.labelsize': 18, 'ytick.labelsize': 18})

# Plot t vs set_x, uav_x for both data1 and data2
axs[0].plot(t1, data1['set_x'], label='set_x (PID)', linewidth=2.5)
axs[0].plot(t1, data1['uav_x'], label='uav_x (PID)', linewidth=2.5)
axs[0].plot(t2, data2['uav_x'], label='uav_x (PID+ESO)', linewidth=2.5, linestyle='--')
axs[0].set_xlabel('t (s)', fontsize=18)
axs[0].set_ylabel('x (m)', fontsize=18)
axs[0].tick_params(axis='both', which='major', labelsize=18)
axs[0].legend(loc='lower right')
axs[0].set_title('tracking plot')

# Plot t vs set_y, uav_y for both data1 and data2
axs[1].plot(t1, data1['set_y'], label='set_y (PID)', linewidth=2.5)
axs[1].plot(t1, data1['uav_y'], label='uav_y (PID)', linewidth=2.5)
axs[1].plot(t2, data2['uav_y'], label='uav_y (PID+ESO)', linewidth=2.5, linestyle='--')
axs[1].set_xlabel('t (s)', fontsize=18)
axs[1].set_ylabel('y (m)', fontsize=18)
axs[1].tick_params(axis='both', which='major', labelsize=18)
axs[1].legend(loc='lower right')

# Plot t vs set_z, uav_z for both data1 and data2
axs[2].plot(t1, data1['set_z'], label='set_z (PID)', linewidth=2.5)
axs[2].plot(t1, data1['uav_z'], label='uav_z (PID)', linewidth=2.5)
axs[2].plot(t2, data2['uav_z'], label='uav_z (PID+ESO)', linewidth=2.5, linestyle='--')
axs[2].set_xlabel('t (s)', fontsize=18)
axs[2].set_ylabel('z (m)', fontsize=18)
axs[2].tick_params(axis='both', which='major', labelsize=18)
axs[2].legend(loc='lower right')

plt.tight_layout()
plt.savefig('track_' + no1 + '_' + no2 + '.pdf', format='pdf')

# Plot combined e_x, e_y, e_z for both data1 and data2
plt.figure(figsize=(15, 5))
plt.plot(t1, data1['e_x'], label='e_x (PID)', color='purple', linewidth=2.5)
plt.plot(t1, data1['e_y'], label='e_y (PID)', color='green', linewidth=2.5)
plt.plot(t1, data1['e_z'], label='e_z (PID)', color='red', linewidth=2.5)
plt.plot(t2, data2['e_x'], label='e_x (PID+ESO)', color='purple', linestyle='--', linewidth=2.5)
plt.plot(t2, data2['e_y'], label='e_y (PID+ESO)', color='green', linestyle='--', linewidth=2.5)
plt.plot(t2, data2['e_z'], label='e_z (PID+ESO)', color='red', linestyle='--', linewidth=2.5)
plt.xlabel('t (s)', fontsize=18)
plt.ylabel('error (m)', fontsize=18)
plt.legend()
plt.title('error Plot')
plt.tight_layout()
plt.savefig('error_' + no1 + '_' + no2 + '.pdf', format='pdf')
