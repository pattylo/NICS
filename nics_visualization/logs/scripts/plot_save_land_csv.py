import pandas as pd

no = "L"

# Read the CSV files, skipping the first line
file1 = '../land2024/0801_land_uav_' + no + '.csv'
file2 = '../land2024/0801_land_led_' + no + '.csv'

data1 = pd.read_csv(file1, skiprows=1)
data2 = pd.read_csv(file2, skiprows=1)

# Extract the relevant columns for plotting
uav_gtx, uav_gty, uav_gtz, uav_gto, t1 = data1['x'], data1['y'], data1['z'], data1['ori'], data1['t']
ugvx, ugvy, ugvz, t2 = data2['ugvx'], data2['ugvy'], data2['ugvz'], data2['t']
led_x, led_y, led_z, led_o, t2 = data2['x'], data2['y'], data2['z'], data2['ori'], data2['t']
st_x, st_y, st_z, t2 = data2['st_x'], data2['st_y'], data2['st_z'], data2['t']

uav_rr,uav_rp,uav_ry = data1['rr'], data1['rp'], data1['ry']
led_rr,led_rp,led_ry = data2['rr'], data2['rp'], data2['ry']

window_size = 4
led_o = led_o.rolling(window=window_size).mean()

# Create a new DataFrame with the relevant columns
new_data = pd.DataFrame({
    'uav_gtx': uav_gtx,
    'uav_gty': uav_gty,
    'uav_gtz': uav_gtz,
    'uav_gto': uav_gto,
    'uav_rr': uav_rr,
    'uav_rp': uav_rp,
    'uav_ry': uav_ry,
    't1': t1,
    'ugvx': ugvx,
    'ugvy': ugvy,
    'ugvz': ugvz,
    'led_x': led_x,
    'led_y': led_y,
    'led_z': led_z,
    'led_o': led_o,
    'st_x': st_x,
    'st_y': st_y,
    'st_z': st_z,
    'led_rr': led_rr,
    'led_rp': led_rp,
    'led_ry': led_ry,
    't2': t2
})

# Save the new DataFrame to a CSV file
output_file = '../final_data/landplot_' + no + '.csv'
new_data.to_csv(output_file, index=False)

print(f"Data saved to {output_file}")
