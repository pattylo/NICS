import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

no = "A"
dataset_name = "circle"
print(dataset_name + "_" + no)

# Read the CSV files, skipping the first line
file1 = '../thesis/0821_rse_uav_' + dataset_name + "_" + no + '.csv'
file2 = '../thesis/0821_rse_led_' + dataset_name + "_" + no + '.csv'

data1 = pd.read_csv(file1, skiprows=1)
data2 = pd.read_csv(file2, skiprows=1)

# Extract the relevant columns for plotting
x1, y1, z1, o1, t1 = data1['x'], data1['y'], data1['z'], data1['ori'], data1['t']
x2, y2, z2, o2, t2 = data2['x'], data2['y'], data2['z'], data2['ori'], data2['t']

window_size = 4

# Apply the rolling mean to smooth the data
o2 = o2.rolling(window=window_size).mean()

# Plot x, y, z with respect to t
fig, axs = plt.subplots(4, 1, figsize=(10, 15))

# Plot x vs t
axs[0].plot(t1, x1, label='x (UAV)', color='blue')
axs[0].plot(t2, x2, label='x (LED)', color='orange')
axs[0].set_ylabel('x')
axs[0].legend()
axs[0].grid(True)

# Plot y vs t
axs[1].plot(t1, y1, label='y (UAV)', color='blue')
axs[1].plot(t2, y2, label='y (LED)', color='orange')
axs[1].set_ylabel('y')
axs[1].legend()
axs[1].grid(True)

# Plot z vs t
axs[2].plot(t1, z1, label='z (UAV)', color='blue')
axs[2].plot(t2, z2, label='z (LED)', color='orange')
axs[2].set_xlabel('t')
axs[2].set_ylabel('z')
axs[2].legend()
axs[2].grid(True)
axs[2].set_ylim(0,2)

# Plot ori vs t
axs[3].plot(t1, o1, label='ori (UAV)', color='blue')
axs[3].plot(t2, o2, label='ori (LED)', color='orange')
axs[3].set_xlabel('t')
axs[3].set_ylabel('ori')
axs[3].legend()
axs[3].grid(True)
# axs[3].set_ylim(0,2)

# Compute RMSE for combined x, y, z
xyz1 = np.vstack((x1, y1, z1)).T
xyz2 = np.vstack((x2, y2, z2)).T
rmse_xyz = np.sqrt(np.mean((xyz1 - xyz2)**2, axis=0)).mean()

# Compute RMSE for ori
rmse_ori = np.sqrt(np.mean((o1 - o2)**2))



# Compute the errors
e_x = x1 - x2
e_y = y1 - y2
e_z = z1 - z2

# Combine the errors into a single array
combined_errors = np.vstack((e_x, e_y, e_z)).T


# Calculate the standard deviation of the combined errors
std_combined_error = np.std(combined_errors)

print(combined_errors.size)
print(f"RMSE for combined x, y, z: {rmse_xyz}")
print(f"RMSE for ori: {rmse_ori}")
print(f'Standard Deviation of the combined errors: {std_combined_error}')

plt.tight_layout()
plt.show()
