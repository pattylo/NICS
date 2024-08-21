import pandas as pd
import matplotlib.pyplot as plt

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

# Plot x, y, z from file1 and st_x, st_y, st_z from file2
fig, axs = plt.subplots(3, 1, figsize=(10, 15))

# Plot x vs st_x
axs[0].plot(t1, x1, label='x (UAV)', color='blue')
axs[0].plot(t2, st_x2, label='st_x (LED)', color='orange')

# axs[0].plot(t2, x1 - st_x2, label='st_x (LED)', color='orange')

axs[0].set_ylabel('x / st_x')
axs[0].legend()
axs[0].grid(True)

# Plot y vs st_y
axs[1].plot(t1, y1, label='y (UAV)', color='blue')
axs[1].plot(t2, st_y2, label='st_y (LED)', color='orange')

# axs[1].plot(t2, y1 - st_y2, label='st_x (LED)', color='orange')

axs[1].set_ylabel('y / st_y')
axs[1].legend()
axs[1].grid(True)

# Plot z vs st_z
axs[2].plot(t1, z1, label='z (UAV)', color='blue')
axs[2].plot(t2, st_z2, label='st_z (LED)', color='orange')

# axs[2].plot(t2, z1 - st_z2, label='st_x (LED)', color='orange')

axs[2].set_xlabel('t')
axs[2].set_ylabel('z / st_z')
axs[2].legend()
axs[2].grid(True)
axs[2].set_ylim(0,1)

plt.tight_layout()
plt.show()
