import pandas as pd
import matplotlib.pyplot as plt
from sklearn.ensemble import IsolationForest
import numpy as np

dataset_name = "circle_dist"

def process(filename1, filename2, no):
    # Read the CSV files, skipping the first line
    # print(filename1)
    # print(filename2)
    
    data1 = pd.read_csv(filename1, skiprows=1)
    data2 = pd.read_csv(filename2, skiprows=1)

    # Extract the relevant columns for plotting
    vx1, vy1, vz1, t1 = data1['vx'], data1['vy'], data1['vz'], data1['t']
    vx2, vy2, vz2, t2 = data2['vx'], data2['vy'], data2['vz'], data2['t']


    # Define a function to remove outliers using IsolationForest
    def remove_outliers_isolation_forest(series):
        series = series.values.reshape(-1, 1)
        iso_forest = IsolationForest(contamination=0.05)
        yhat = iso_forest.fit_predict(series)
        mask = yhat != -1
        return pd.Series(series[mask].flatten())

    # Remove outliers from the data
    # vx2 = remove_outliers_isolation_forest(vx2)
    # vy2 = remove_outliers_isolation_forest(vy2)
    # vz2 = remove_outliers_isolation_forest(vz2)

    window_size = 1

    # Apply the rolling mean to smooth the data
    vx1 = vx1.rolling(window=window_size).mean()
    vy1 = vy1.rolling(window=window_size).mean()
    vz1 = vz1.rolling(window=window_size).mean()

    vx2 = vx2.rolling(window=window_size).mean()
    vy2 = vy2.rolling(window=window_size).mean()
    vz2 = vz2.rolling(window=window_size).mean()

    # Adjust vz2 if the difference between corresponding elements of vz1 and vz2 is larger than 0.25
    # for i in range(len(vz1)):
    #     if abs(vz1[i] - vz2[i]) > 0.15:
    #         print("jihi")
    #         vz2[i] = 0.2 * vz2[i] + 0.8 * vz1[i]  # This simplifies to vz2[i] = vz2[i]

    # for i in range(len(vx1)):
    #     if abs(vx1[i] - vx2[i]) > 0.5:
    #         print("jihi")
    #         vx2[i] = 0.2 * vx2[i] + 0.8 * vx1[i]  # This simplifies to vz2[i] = vz2[i]


    # If you meant to use a different formula, please specify it.

    # Plot vx, vy, vz with respect to t
    fig, axs = plt.subplots(3, 1, figsize=(10, 15))

    # Plot vx vs t
    axs[0].plot(t1, vx1, label='vx (UAV)', color='blue')
    axs[0].plot(t2, vx2, label='vx (LED)', color='orange')
    axs[0].set_ylabel('vx')
    axs[0].legend()
    axs[0].grid(True)

    # Plot vy vs t
    axs[1].plot(t1, vy1, label='vy (UAV)', color='blue')
    axs[1].plot(t2, vy2, label='vy (LED)', color='orange')
    axs[1].set_ylabel('vy')
    axs[1].legend()
    axs[1].grid(True)

    # Plot vz vs t
    axs[2].plot(t1, vz1, label='vz (UAV)', color='blue')
    axs[2].plot(t2, vz2, label='vz (LED)', color='orange')
    axs[2].set_xlabel('t')
    axs[2].set_ylabel('vz')
    axs[2].legend()
    axs[2].grid(True)
    axs[2].set_ylim(-0.5,0.5)


    # Compute the errors
    e_x = vx1 - vx2
    e_y = vy1 - vy2
    e_z = vz1 - vz2

    ve_square = e_x**2 + e_y**2 + e_z**2
    ve_square_sum = ve_square.sum()
    ve_rmse = np.sqrt(ve_square_sum / ve_square.size)
    ve_abs = np.sqrt(ve_square)

    # print(ve_rmse)
    # print(np.std(ve_abs))
    print(dataset_name + "_" + no)

    print(f"RMSE: {ve_rmse}")
    print(f'STD: {np.std(ve_abs)}')
    print(f'MAX: {np.max(ve_abs)}')

# files = ['A','B','C','D']
files = ['A', 'B','D']
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


