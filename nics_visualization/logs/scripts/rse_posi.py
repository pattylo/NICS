import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

dataset_name = "circle_dist"

def process(filename1, filename2, no):
    # Read the CSV files, skipping the first line
    # print(filename1)
    # print(filename2)
    
    data1 = pd.read_csv(filename1, skiprows=1)
    data2 = pd.read_csv(filename2, skiprows=1)

    # Extract the relevant columns for plotting
    x1, y1, z1, o1, t1 = data1['x'], data1['y'], data1['z'], data1['ori'], data1['t']
    x2, y2, z2, o2, t2 = data2['x'], data2['y'], data2['z'], data2['ori'], data2['t']

    o_d = o1[1] - o2[1]
    o2 = o2 + o_d

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

    e_o = o1 - o2

    pe_square = e_x**2 + e_y**2 + e_z**2
    pe_square_sum = pe_square.sum()
    pe_rmse = np.sqrt(pe_square_sum / pe_square.size)
    pe_abs = np.sqrt(pe_square)

    oe_square = e_o ** 2
    oe_square_sum = oe_square.sum()
    oe_rmse = np.sqrt(oe_square_sum / oe_square.size)
    oe_abs = np.sqrt(oe_square)

    # print(ve_rmse)
        # print(np.std(ve_abs))
    print(dataset_name + "_" + no)

    print("posi")
    print(f"RMSE: {pe_rmse}")
    print(f'STD: {np.std(pe_abs)}')
    print(f'MAX: {np.max(pe_abs)}')
    print()
    print("ori")
    print(f"RMSE: {oe_rmse / np.pi * 180}")
    print(f'STD: {np.std(oe_abs) / np.pi * 180}')
    print(f'MAX: {np.max(oe_abs) / np.pi * 180}')

# files = ['A','B','C','D']
files = ['A','B','D']
filenames = []

for file in files:
    no = file
    filenames.append(['../thesis/0821_rse_uav_' + dataset_name + "_" + no + '.csv', '../thesis/0821_rse_led_' + dataset_name + "_" + no + '.csv', no])
# no = "D"

for file in filenames:
    file1, file2, no = file
    # print(file1)
    # print(file2)
    process(filename1=file1, filename2=file2,no=no)
    print()

exit()

# Combine the errors into a single array
combined_errors = np.vstack((e_x, e_y, e_z)).T


# Calculate the standard deviation of the combined errors
std_combined_error = np.std(combined_errors)
std_combined_ori = np.std(e_o)

print(dataset_name + "_" + no)
print(combined_errors.size)
print(f"RMSE for combined x, y, z: {rmse_xyz}")
print(f'Standard Deviation of the combined errors: {std_combined_error}')
print(f"RMSE for ori: {rmse_ori / np.pi * 180}")
print(f'Standard Deviation of the combined errors: {std_combined_ori / np.pi * 180}')


plt.tight_layout()
plt.show()
