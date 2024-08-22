import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

no = "C"
print(no)

file_path = '../thesis/0821_ctrl_tracking_' + no + ".csv"
data = pd.read_csv(file_path, skiprows=1)

# Print column names to check if they are read correctly
print("Column names:", data.columns)

# Convert columns to numeric if needed
numeric_columns = ['set_x', 'set_y', 'set_z', 'uav_x', 'uav_y', 'uav_z', 'e_x', 'e_y', 'e_z']
for col in numeric_columns:
    data[col] = pd.to_numeric(data[col], errors='coerce')

# Calculate RMSE for e_x, e_y, and e_z
rmse_ex = np.sqrt(np.mean(data['e_x'] ** 2))
rmse_ey = np.sqrt(np.mean(data['e_y'] ** 2))
rmse_ez = np.sqrt(np.mean(data['e_z'] ** 2))
combined_rmse = np.sqrt((rmse_ex ** 2 + rmse_ey ** 2 + rmse_ez ** 2))
print(combined_rmse)


# Print RMSE (m) to the terminal
print("RMSE for e_x: {}".format(rmse_ex))

print("RMSE for e_y: {}".format(rmse_ey))

print("RMSE for e_z: {}".format(rmse_ez))

# Calculate combined (m) for e_x, e_y, and e_z
combined_errors = np.sqrt(data['e_x']**2 + data['e_y']**2 + data['e_z']**2)
# print(np.mean(combined_errors ** 2))

# Calculate RMSE for the combined errors
combined_rmse = np.sqrt(np.mean(combined_errors ** 2))

# Print combined RMSE (m) to the terminal
print("Combined RMSE: {}".format(combined_rmse))
print("MAX Tracking Error: {}".format(np.max(combined_errors)))
# print(np.sort(combined_errors))
print(np.var(combined_errors))

# exit()

# Plot the data
fig, axs = plt.subplots(3, 1, figsize=(15, 10))
t = data['t'] - data['t'][0]
plt.rcParams.update({'font.size': 18})  # Set the font size globally
plt.rcParams.update({'font.size': 18, 'xtick.labelsize': 18, 'ytick.labelsize': 18})


# Plot t vs set_x and t vs uav_x
axs[0].plot(t, data['set_x'], label='set_x', linewidth=2.5)
axs[0].plot(t, data['uav_x'], label='uav_x', linewidth=2.5)
axs[0].set_xlabel('Time (t)',fontsize=18)
axs[0].set_ylabel('X (m)',fontsize=18)
axs[0].tick_params(axis='both', which='major', labelsize=18)
axs[0].legend(loc='lower right',)
axs[0].set_title('Tracking Plot')
# axs[0].set_ylim([0.4, 1.4])

# Plot t vs set_y and t vs uav_y
axs[1].plot(t, data['set_y'], label='set_y', linewidth=2.5)
axs[1].plot(t, data['uav_y'], label='uav_y', linewidth=2.5)
axs[1].set_xlabel('Time (t)',fontsize=18)
axs[1].set_ylabel('Y (m)',fontsize=18)
axs[1].tick_params(axis='both', which='major', labelsize=18)
axs[1].legend(loc='lower right',)
# axs[1].set_title('t vs set_y and uav_y')
# axs[1].set_ylim([0.4, 1.4])

# Plot t vs set_z and t vs uav_z
axs[2].plot(t, data['set_z'], label='set_z', linewidth=2.5)
axs[2].plot(t, data['uav_z'], label='uav_z', linewidth=2.5)
axs[2].set_xlabel('Time (t)',fontsize=18)
axs[2].set_ylabel('Z (m)',fontsize=18)
axs[2].tick_params(axis='both', which='major', labelsize=18)
axs[2].legend(loc='lower right',)
# axs[2].set_title('t vs set_z and uav_z')
# axs[2].set_ylim([0.8, 1.4])

# # Plot t vs e_x
# axs[0, 1].plot(t, data['e_x'], label='e_x', color='purple')
# axs[0, 1].set_xlabel('Time (t)')
# axs[0, 1].set_ylabel('e_x (m)')
# axs[0, 1].legend()
# axs[0, 1].set_title('t vs e_x')
# axs[0, 1].set_ylim([-0.5, 0.5])

# # Plot t vs e_y
# axs[1, 1].plot(t, data['e_y'], label='e_y', color='green')
# axs[1, 1].set_xlabel('Time (t)')
# axs[1, 1].set_ylabel('e_y (m)')
# axs[1, 1].legend()
# axs[1, 1].set_title('t vs e_y')
# axs[1, 1].set_ylim([-0.5, 0.5])

# # Plot t vs e_z
# axs[2, 1].plot(t, data['e_z'], label='e_z', color='red')
# axs[2, 1].set_xlabel('Time (t)')
# axs[2, 1].set_ylabel('e_z (m)')
# axs[2, 1].legend()
# axs[2, 1].set_title('t vs e_z')
# axs[2, 1].set_ylim([-0.5, 0.5])

plt.tight_layout()
plt.savefig('track_'+ no + '.pdf', format='pdf')

# plt.show()

# Plot combined e_x, e_y, e_z
plt.figure(figsize=(15, 5))
plt.plot(t, data['e_x'], label='e_x', color='purple', linewidth=2.5)
plt.plot(t, data['e_y'], label='e_y', color='green', linewidth=2.5)
plt.plot(t, data['e_z'], label='e_z', color='red')
plt.xlabel('Time (t)',fontsize=18)
plt.ylabel('Error (m)',fontsize=18)
plt.legend()
plt.title('Error Plot')
plt.tight_layout()
plt.savefig('error_'+ no + '.pdf', format='pdf')
# plt.show()