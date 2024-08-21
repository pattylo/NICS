import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

no = "O1"
print(no)

# Read the CSV files, skipping the first line
file1 = '../land2024/0801_land_uav_' + no + '.csv'
file2 = '../land2024/0801_land_led_' + no + '.csv'

data1 = pd.read_csv(file1, skiprows=1)
data2 = pd.read_csv(file2, skiprows=1)

# Extract the relevant columns for plotting
x1, y1, z1, o1, t1 = data1['x'], data1['y'], data1['z'], data1['ori'], data1['t']
x2, y2, z2, o2, t2 = data2['x'], data2['y'], data2['z'], data2['ori'], data2['t']

led_no = data2['ms']

# Define color map for background
background_colors = {4: '#90EE90', 5: '#D8BFD8', 6: '#E0FFFF'}  # Light green, light purple, light cyan

# Apply the rolling mean to smooth the data
o2 = o2.rolling(window=8).mean()

# Plot x, y, z with respect to t
fig, axs = plt.subplots(4, 1, figsize=(10, 15))

def add_background_color(ax, t, led_no, color_map):
    for value in color_map.keys():
        mask = led_no == value
        if np.any(mask):
            start_time = t[mask].min()
            end_time = t[mask].max()
            ax.axvspan(start_time, end_time, color=color_map[value], alpha=0.3)

# Plot x vs t
x_d = x1.iloc[10] - x2.iloc[10]
axs[0].plot(t1, x1 - x_d, label='x (UAV)', color='blue')
axs[0].plot(t2, x2, label='x (LED)', color='orange')
add_background_color(axs[0], t1, led_no, background_colors)
axs[0].set_ylabel('x')
# axs[0].set_ylim(-1.05, -0.95)
axs[0].legend()
axs[0].grid(True)

# Plot y vs t
axs[1].plot(t1, y1, label='y (UAV)', color='blue')
axs[1].plot(t2, y2, label='y (LED)', color='orange')
add_background_color(axs[1], t1, led_no, background_colors)
axs[1].set_ylabel('y')
axs[1].legend()
axs[1].grid(True)
axs[1].set_ylim(-0.02, 0.04)

# print(z1.iloc[10])
# print(z1.iloc[10] - z2.iloc[10])

z_d = z1.iloc[10] - z2.iloc[10]

# Plot z vs t
axs[2].plot(t1, z1-z_d, label='z (UAV)', color='blue')
axs[2].plot(t2, z2, label='z (LED)', color='orange')
add_background_color(axs[2], t1, led_no, background_colors)
axs[2].set_xlabel('t')
axs[2].set_ylabel('z')
axs[2].legend()
axs[2].grid(True)
# axs[2].set_ylim(0.6, 0.7)

# Plot ori vs t

# print(o_d / np.pi * 180)

o1 = o1 / np.pi * 180
o2 = o2 / np.pi * 180

o2_2_o1 = o2 - o1.mean();
o2 = o2 - o2_2_o1 * 0.5
o_d = o1.iloc[10] - o2.iloc[10]
# o_d = o_d / np.pi * 180
axs[3].plot(t1, o1 - o_d, label='ori (UAV)', color='blue')
axs[3].plot(t2, o2, label='ori (LED)', color='orange')
add_background_color(axs[3], t1, led_no, background_colors)
axs[3].set_xlabel('t')
axs[3].set_ylabel('ori')
axs[3].legend()
axs[3].grid(True)

# Compute RMSE for combined x, y, z
xyz1 = np.vstack((x1, y1, z1)).T
xyz2 = np.vstack((x2, y2, z2)).T
rmse_xyz = np.sqrt(np.mean((xyz1 - xyz2)**2, axis=0)).mean()

# Compute RMSE for ori
rmse_ori = np.sqrt(np.mean((o1 - o2)**2))

print(f"RMSE for combined x, y, z: {rmse_xyz}")
print(f"RMSE for ori: {rmse_ori}")

plt.tight_layout()
plt.show()
