import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

dataset_name = "circle_dist"
files = ['A', 'B', 'D']
filenames = []

# Rolling mean window size
window_size = 10

for file in files:
    no = file
    filenames.append(['../thesis/0821_rse_uav_' + dataset_name + "_" + no + '.csv', '../thesis/0821_rse_led_' + dataset_name + "_" + no + '.csv', no])

# Initialize dictionaries to hold processed data
posi_data = {'x': [], 'y': [], 'z': [], 'ori_uav': [], 'ori_led': []}
velo_data = {'vx': [], 'vy': [], 'vz': []}

# Load and process the data
for file in filenames:
    file1, file2, no = file

    data1 = pd.read_csv(file1, skiprows=1)
    data2 = pd.read_csv(file2, skiprows=1)

    # Extract position data
    x1, y1, z1, o1, t1 = data1['x'], data1['y'], data1['z'], data1['ori'], data1['t']
    x2, y2, z2, o2, t2 = data2['x'], data2['y'], data2['z'], data2['ori'], data2['t']

    # Extract velocity data
    vx1, vy1, vz1 = data1['vx'], data1['vy'], data1['vz']
    vx2, vy2, vz2 = data2['vx'], data2['vy'], data2['vz']

    # Adjust orientation data and apply rolling mean
    o_d = o1.iloc[0] - o2.iloc[0]
    o2 += o_d
    o1 = o1.rolling(window=window_size).mean()
    o2 = o2.rolling(window=window_size).mean()

    # Apply rolling mean to velocities
    vx1 = vx1.rolling(window=window_size).mean()
    vy1 = vy1.rolling(window=window_size).mean()
    vz1 = vz1.rolling(window=window_size).mean()
    vx2 = vx2.rolling(window=window_size).mean()
    vy2 = vy2.rolling(window=window_size).mean()
    vz2 = vz2.rolling(window=window_size).mean()

    # Store processed data
    posi_data['x'].append((t1, x1, x2))
    posi_data['y'].append((t1, y1, y2))
    posi_data['z'].append((t1, z1, z2))
    posi_data['ori_uav'].append((t1, o1))
    posi_data['ori_led'].append((t2, o2))
    velo_data['vx'].append((t1, vx1, vx2))
    velo_data['vy'].append((t1, vy1, vy2))
    velo_data['vz'].append((t1, vz1, vz2))

# Plotting the first graph (posi_x, posi_y, posi_z, ori)
fig1, axs1 = plt.subplots(4, len(files), figsize=(25, 20))

metrics_posi = ['x', 'y', 'z', 'ori_uav', 'ori_led']

plt.rcParams.update({'font.size': 18, 'xtick.labelsize': 18, 'ytick.labelsize': 18})

for i, metric in enumerate(['x', 'y', 'z', 'ori']):
    for j in range(len(files)):
        # print(i)
        if i == 0:
            if j == 0:
                axs1[i, j].set_title('VD-RSE')
            if j == 1:
                axs1[i, j].set_title('IEKF')
            if j == 2:
                axs1[i, j].set_title('KF-less')
        if metric == 'ori':
            t1, ori_uav = posi_data['ori_uav'][j]
            t2, ori_led = posi_data['ori_led'][j]
            axs1[i, j].plot(t1, ori_uav, label=f'ori (UAV)', color='blue')
            axs1[i, j].plot(t2, ori_led, label=f'ori (LED)', color='orange')
            
            axs1[i, j].set_ylabel(metric + ' (dg)',fontsize=18)
        else:
            t1, data1, data2 = posi_data[metric][j]
            axs1[i, j].plot(t1, data1, label=f'{metric} (UAV)', color='blue')
            axs1[i, j].plot(t1, data2, label=f'{metric} (LED)', color='orange')
            axs1[i, j].set_ylabel(metric + ' (m)',fontsize=18)
        
        axs1[i, j].set_xlabel('t (s)',fontsize=18)
        axs1[i, j].legend(loc='lower right')
        axs1[i, j].grid(True)
        axs1[i, j].tick_params(axis='both', which='major', labelsize=18)

# Plotting the second graph (velo_x, velo_y, velo_z)
fig2, axs2 = plt.subplots(3, len(files), figsize=(15, 10))

metrics_velo = ['vx', 'vy', 'vz']
for i, metric in enumerate(metrics_velo):
    for j in range(len(files)):
        
        if i == 0:
            if j == 0:
                axs2[i, j].set_title('VD-RSE')
            if j == 1:
                axs2[i, j].set_title('IEKF')
            if j == 2:
                axs2[i, j].set_title('KF-less')
        t1, data1, data2 = velo_data[metric][j]
        axs2[i, j].plot(t1, data1, label=f'{metric} (UAV)', color='blue')
        axs2[i, j].plot(t1, data2, label=f'{metric} (LED)', color='orange')
        axs2[i, j].set_ylabel(metric + ' (m/s)')
        axs2[i, j].set_xlabel('t (s)')
        axs2[i, j].legend(loc='lower right')
        axs2[i, j].grid(True)

plt.tight_layout()
# Save the first figure (position and orientation)
fig1.savefig('s3_posi.pdf', dpi=300, bbox_inches='tight')

# Save the second figure (velocity)
fig2.savefig('s3_velo.pdf', dpi=300, bbox_inches='tight')

# Show the plots
# plt.show()
