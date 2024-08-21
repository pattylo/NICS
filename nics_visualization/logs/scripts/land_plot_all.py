import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

no = "D"
print(no)

# Read the CSV files, skipping the first line
# file1 = '../land2024/0801_land_uav_' + no + '.csv'
# file2 = '../land2024/0801_land_led_' + no + '.csv'

file1 = '../final_data/landplot_' + no + '.csv'
# file1 = '../final_data/landplot_' + no + '.csv'

data = pd.read_csv(file1)
# data = pd.read_csv(file2, skiprows=1)

# Extract the relevant columns for plotting
uav_gtx, uav_gty, uav_gtz, uav_gto, t1 = data['uav_gtx'], data['uav_gty'], data['uav_gtz'], data['uav_gto'], data['t1']
ugvx, ugvy, ugvz, t2 = data['ugvx'], data['ugvy'], data['ugvz'], data['t2']
led_x, led_y, led_z, led_o, t2 = data['led_x'], data['led_y'], data['led_z'], data['led_o'], data['t2']
st_x, st_y, st_z, t2 = data['st_x'], data['st_y'], data['st_z'], data['t2']



window_size = 4

# Apply the rolling mean to smooth the data
# o2 = o2.rolling(window=window_size).mean()

# Plot x, y, z with respect to t
fig, axs = plt.subplots(4, 1, figsize=(10, 15))

# Plot x vs t
axs[0].plot(t1, uav_gtx, label='gt_x (UAV)', color='blue')
axs[0].plot(t2, ugvx, label='gt_x (UGV)', color='orange')
axs[0].plot(t2, led_x, label='est_x (UAV)', color='red')
axs[0].plot(t2, st_x, label='st_x (UAV)', color='purple')
axs[0].set_ylabel('x')
axs[0].legend()
axs[0].grid(True)

# Plot y vs t
axs[1].plot(t1, uav_gty, label='gt_y (UAV)', color='blue')
axs[1].plot(t2, ugvy, label='gt_y (UGV)', color='orange')
axs[1].plot(t2, led_y, label='est_y (UAV)', color='red')
axs[1].plot(t2, st_y, label='st_y (UAV)', color='purple')
axs[1].set_ylabel('y')
axs[1].legend()
axs[1].grid(True)

# Plot z vs t
axs[2].plot(t1, uav_gtz, label='gt_z (UAV)', color='blue')
axs[2].plot(t2, ugvz, label='gt_z (UGV)', color='orange')
axs[2].plot(t2, led_z, label='est_z (UAV)', color='red')
axs[2].plot(t2, st_z, label='st_z (UAV)', color='purple')
axs[2].set_ylabel('z')
axs[2].legend()
axs[2].grid(True)
axs[2].set_ylim(0,1)

# Plot ori vs t
print(len(uav_gto))
print(len(t1))
axs[3].plot(t1, uav_gto, label='ori (UAV)', color='blue')
axs[3].plot(t2, led_o, label='ori (LED)', color='orange')
axs[3].set_xlabel('t')
axs[3].set_ylabel('ori')
axs[3].legend()
axs[3].grid(True)
# axs[3].set_ylim(0,2)



plt.tight_layout()
plt.show()
