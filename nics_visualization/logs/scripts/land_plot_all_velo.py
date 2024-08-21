import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

no = "D"
print(no)


file1 = '../final_data/veloplot_' + no + '.csv'

data = pd.read_csv(file1)

# Extract the relevant columns for plotting
uav_gt_vx, uav_gt_vy, uav_gt_vz, t1 = data['uav_gt_vx'], data['uav_gt_vy'], data['uav_gt_vz'], data['t1']
led_vx, led_vy, led_vz, t2 = data['led_vx'], data['led_vy'], data['led_vz'], data['t2']

ugv_vx, ugv_vy, ugv_vz = data['ugv_vx'], data['ugv_vy'], data['ugv_vz']
ugv_vyaw = data['ugv_vyaw']

window_size = 4

# Apply the rolling mean to smooth the data
# o2 = o2.rolling(window=window_size).mean()

# Plot x, y, z with respect to t
fig, axs = plt.subplots(4, 1, figsize=(10, 15))

# Plot x vs t
axs[0].plot(t1, uav_gt_vx, label='gt_vx (UAV)', color='blue')
axs[0].plot(t2, ugv_vx, label='gt_vx (UGV)', color='orange')
axs[0].plot(t2, led_vx, label='est_vx (UAV)', color='red')
axs[0].set_ylabel('x')
axs[0].legend()
axs[0].grid(True)

# Plot y vs t
axs[1].plot(t1, uav_gt_vy, label='gt_vy (UAV)', color='blue')
axs[1].plot(t2, ugv_vy, label='gt_vy (UGV)', color='orange')
axs[1].plot(t2, led_vy, label='est_vy (UAV)', color='red')
axs[1].set_ylabel('y')
axs[1].legend()
axs[1].grid(True)

# Plot z vs t
axs[2].plot(t1, uav_gt_vz, label='gt_vz (UAV)', color='blue')
axs[2].plot(t2, ugv_vz, label='gt_vz (UGV)', color='orange')
axs[2].plot(t2, led_vz, label='est_vz (UAV)', color='red')
axs[2].set_ylabel('z')
axs[2].legend()
axs[2].grid(True)
# axs[2].set_ylim(0,1)

# Plot ori vs t
axs[3].plot(t1, ugv_vyaw, label='ori (UGV)', color='blue')
# axs[3].plot(t2, led_o, label='ori (LED)', color='orange')
axs[3].set_xlabel('t')
axs[3].set_ylabel('ori')
axs[3].legend()
axs[3].grid(True)
# axs[3].set_ylim(0,2)



plt.tight_layout()
plt.show()
