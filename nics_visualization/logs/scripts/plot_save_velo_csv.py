import pandas as pd

no = "D"

# Read the CSV files, skipping the first line
file1 = '../land2024/0801_land_uav_' + no + '.csv'
file2 = '../land2024/0801_land_led_' + no + '.csv'

data1 = pd.read_csv(file1, skiprows=1)
data2 = pd.read_csv(file2, skiprows=1)

# Extract the relevant columns for plotting
uav_gt_vx, uav_gt_vy, uav_gt_vz, t1 = data1['vx'], data1['vy'], data1['vz'], data1['t']

led_vx, led_vy, led_vz, t2 = data2['vx'], data2['vy'], data2['vz'], data2['t']

ugv_vx, ugv_vy = data2['ugv_vx'], data2['ugv_vy']
ugv_vz = 0 * data2['ugv_vx']
ugv_vyaw = 280 * data2['ugv_vyaw']

window_size = 20
uav_gt_vx = uav_gt_vx.rolling(window=window_size).mean()
uav_gt_vy = uav_gt_vy.rolling(window=window_size).mean()
uav_gt_vz = uav_gt_vz.rolling(window=window_size).mean()
led_vx = led_vx.rolling(window=window_size).mean()
led_vy = led_vy.rolling(window=window_size).mean()
led_vz = led_vz.rolling(window=window_size).mean()
ugv_vyaw = ugv_vyaw.rolling(window=window_size).mean()


# Create a new DataFrame with the relevant columns
new_data = pd.DataFrame({
    'uav_gt_vx': uav_gt_vx,
    'uav_gt_vy': uav_gt_vy,
    'uav_gt_vz': uav_gt_vz,
    't1': t1,
    'led_vx': led_vx,
    'led_vy': led_vy,
    'led_vz': led_vz,
    'ugv_vx': ugv_vx,
    'ugv_vy': ugv_vy,
    'ugv_vz': ugv_vz,
    'ugv_vyaw': ugv_vyaw,
    't2': t2
})

# Save the new DataFrame to a CSV file
output_file = '../final_data/veloplot_' + no + '.csv'
new_data.to_csv(output_file, index=False)

print(f"Data saved to {output_file}")
