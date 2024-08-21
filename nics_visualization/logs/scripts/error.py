import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

no = "D"
print(no)

file_path = '../land2024/0801_' + no + '_land_tracking.csv'
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


# Print RMSE values to the terminal
print("RMSE for e_x: {}".format(rmse_ex))

print("RMSE for e_y: {}".format(rmse_ey))

print("RMSE for e_z: {}".format(rmse_ez))

# Calculate combined values for e_x, e_y, and e_z
combined_errors = np.sqrt(data['e_x']**2 + data['e_y']**2 + data['e_z']**2)
# print(np.mean(combined_errors ** 2))

# Calculate RMSE for the combined errors
combined_rmse = np.sqrt(np.mean(combined_errors ** 2))

# Print combined RMSE values to the terminal
print("Combined RMSE: {}".format(combined_rmse))
print("MAX Tracking Error: {}".format(np.max(combined_errors)))
# print(np.sort(combined_errors))
print(np.var(combined_errors))

# exit()

# Plot the data
fig, axs = plt.subplots(3, 2, figsize=(15, 10))

# Plot t vs set_x and t vs uav_x
axs[0, 0].plot(data['t'], data['set_x'], label='set_x')
axs[0, 0].plot(data['t'], data['uav_x'], label='uav_x')
axs[0, 0].set_xlabel('Time (t)')
axs[0, 0].set_ylabel('X values')
axs[0, 0].legend()
axs[0, 0].set_title('t vs set_x and uav_x')


# Plot t vs set_y and t vs uav_y
axs[1, 0].plot(data['t'], data['set_y'], label='set_y')
axs[1, 0].plot(data['t'], data['uav_y'], label='uav_y')
axs[1, 0].set_xlabel('Time (t)')
axs[1, 0].set_ylabel('Y values')
axs[1, 0].legend()
axs[1, 0].set_title('t vs set_y and uav_y')

# Plot t vs set_z and t vs uav_z
axs[2, 0].plot(data['t'], data['set_z'], label='set_z')
axs[2, 0].plot(data['t'], data['uav_z'], label='uav_z')
axs[2, 0].set_xlabel('Time (t)')
axs[2, 0].set_ylabel('Z values')
axs[2, 0].legend()
axs[2, 0].set_title('t vs set_z and uav_z')

# Plot t vs e_x
axs[0, 1].plot(data['t'], data['e_x'], label='e_x', color='purple')
axs[0, 1].set_xlabel('Time (t)')
axs[0, 1].set_ylabel('e_x values')
axs[0, 1].legend()
axs[0, 1].set_title('t vs e_x')
axs[0, 1].set_ylim([-0.5, 0.5])

# Plot t vs e_y
axs[1, 1].plot(data['t'], data['e_y'], label='e_y', color='green')
axs[1, 1].set_xlabel('Time (t)')
axs[1, 1].set_ylabel('e_y values')
axs[1, 1].legend()
axs[1, 1].set_title('t vs e_y')
axs[1, 1].set_ylim([-0.5, 0.5])

# Plot t vs e_z
axs[2, 1].plot(data['t'], data['e_z'], label='e_z', color='red')
axs[2, 1].set_xlabel('Time (t)')
axs[2, 1].set_ylabel('e_z values')
axs[2, 1].legend()
axs[2, 1].set_title('t vs e_z')
axs[2, 1].set_ylim([-0.5, 0.5])

plt.tight_layout()
plt.show()

# Plot combined e_x, e_y, e_z
plt.figure(figsize=(10, 5))
plt.plot(data['t'], data['e_x'], label='e_x', color='purple')
plt.plot(data['t'], data['e_y'], label='e_y', color='green')
plt.plot(data['t'], data['e_z'], label='e_z', color='red')
plt.xlabel('Time (t)')
plt.ylabel('Error values')
plt.legend()
plt.title('Combined plot of e_x, e_y, and e_z')
plt.show()