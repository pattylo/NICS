import pandas as pd
import matplotlib.pyplot as plt


file = "../ms/est_ms.csv"
data = pd.read_csv(file)

ini_ms = data['init_ms']
image_proc_ms = data['image_proc_ms']
corres_ms = data['corres_ms']
iekf_ms = data['iekf_ms']

# # Find unique values in the 'init_ms' column
# unique_values = ini_ms.unique()

# # Convert unique values to a list
# unique_values_array = list(unique_values)

# # Calculate mean, variance, and maximum value
# mean_value = ini_ms.mean()
# variance_value = ini_ms.var()
# max_value = ini_ms.max()

# # Print results
# print(f'The unique values are: {unique_values_array}')
# print(f'The number of unique values is: {len(unique_values_array)}')
# print(f'The mean value is: {mean_value}')
# print(f'The variance is: {variance_value}')
# print(f'The maximum value is: {max_value}')

online_total_ms = image_proc_ms + corres_ms + iekf_ms

print(f'The mean value is: {online_total_ms.mean()}')
print(f'The variance is: {online_total_ms.var()}')
print(f'The maximum value is: {online_total_ms.max()}')