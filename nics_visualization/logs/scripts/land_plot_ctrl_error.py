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
x1, y1, z1, t1 = data1['x'], data1['y'], data1['z'], data1['t']
st_x2, st_y2, st_z2, t2 = data2['st_x'], data2['st_y'], data2['st_z'], data2['t']

# Calculate the control errors
error_x = x1 - st_x2
error_y = y1 - st_y2
error_z = z1 - st_z2

# Calculate the total RMSE
combined_errors = np.sqrt(error_x**2 + error_y**2 + error_z**2)
total_rmse = np.sqrt(np.mean(error_x**2 + error_y**2 + error_z**2))

# Plot control errors
fig, axs = plt.subplots(3, 1, figsize=(10, 15))

# Plot error in x
axs[0].plot(t1, error_x, label='Error in x', color='red')
axs[0].set_ylabel('Error in x')
axs[0].legend()
axs[0].grid(True)

# Plot error in y
axs[1].plot(t1, error_y, label='Error in y', color='red')
axs[1].set_ylabel('Error in y')
axs[1].legend()
axs[1].grid(True)

# Plot error in z
axs[2].plot(t1, error_z, label='Error in z', color='red')
axs[2].set_xlabel('t')
axs[2].set_ylabel('Error in z')
axs[2].legend()
axs[2].grid(True)

print(f"Total RMSE: {total_rmse}")
print(f"Total MAX: {np.max(combined_errors)}")
print(f"Total VAR: {np.var(combined_errors)}")

plt.tight_layout()
plt.show()

# Print the total RMSE value

