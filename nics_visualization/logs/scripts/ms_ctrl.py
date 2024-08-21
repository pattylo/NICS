import pandas as pd
import matplotlib.pyplot as plt


file = "../ms/ctrl_ms.csv"
data = pd.read_csv(file)

sample_opt_ms = data['sample_opt_ms']
online_opt_ms = data['online_opt_ms']
ctrl_ms = data['ctrl_ms']

online_total_ms = online_opt_ms + ctrl_ms

print(f'The mean value is: {sample_opt_ms.mean()}')
print(f'The variance is: {sample_opt_ms.var()}')
print(f'The maximum value is: {sample_opt_ms.max()}')