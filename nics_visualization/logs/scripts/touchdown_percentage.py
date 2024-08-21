import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
from matplotlib.patches import Rectangle

# Reading the CSV file into a DataFrame
df = pd.read_csv('../touchdown_log/touchdown.csv')

# Extracting data based on mission type
missions = ['b', 'c', 'd', 'e', 'f', 'g']
legend_labels = {
    'b': 'Mission A', 
    'c': 'Mission B', 
    'd': 'Mission C', 
    'e': 'Mission D', 
    'f': 'Mission E', 
    'g': 'Mission F'
}
colors = {
    'b': 'blue', 
    'c': 'green', 
    'd': 'red', 
    'e': 'purple', 
    'f': 'orange', 
    'g': 'cyan'
}

# New origin
new_origin_x = -0.025
new_origin_y = +0.015

# Shift the coordinates and convert from meters to centimeters
df['shifted_touchdown_x'] = (df['touchdown_x'] - new_origin_x) * 100
df['shifted_touchdown_y'] = (df['touchdown_y'] - new_origin_y) * 100

# Define the shrinking factor
shrink = 5

# Define the bounds for the regions, including shrinkage
bounds = {
    'smallest': (-11.5+shrink, -11.5+shrink, (11.5-shrink)*2, (11.5-shrink)*2),
    'mid': (-16.5+shrink, -16.5+shrink, (16.5-shrink)*2, (16.5-shrink)*2),
    'largest': (-21.5+shrink, -21.5+shrink, (21.5-shrink)*2, (21.5-shrink)*2)
}

def in_bounds(x, y, bounds):
    return (bounds[0] <= x <= bounds[0] + bounds[2]) and (bounds[1] <= y <= bounds[1] + bounds[3])

# Initialize dictionaries to store counts
counts = {mission: {'total': 0, 'smallest': 0, 'mid': 0, 'largest': 0} for mission in missions}

for mission in missions:
    mission_data = df[df['mission_type'] == mission]
    counts[mission]['total'] = len(mission_data)
    
    # Count points within each bound
    for index, row in mission_data.iterrows():
        x, y = row['shifted_touchdown_x'], row['shifted_touchdown_y']
        if in_bounds(x, y, bounds['largest']):
            counts[mission]['largest'] += 1
            if in_bounds(x, y, bounds['mid']):
                counts[mission]['mid'] += 1
                if in_bounds(x, y, bounds['smallest']):
                    counts[mission]['smallest'] += 1

# Calculate percentages
percentages = {mission: {
    'smallest': (counts[mission]['smallest'] / counts[mission]['total']) * 100 if counts[mission]['total'] > 0 else 0,
    'mid': (counts[mission]['mid'] / counts[mission]['total']) * 100 if counts[mission]['total'] > 0 else 0,
    'largest': (counts[mission]['largest'] / counts[mission]['total']) * 100 if counts[mission]['total'] > 0 else 0
} for mission in missions}

# Print percentages
for mission in missions:
    print(f"Mission {legend_labels[mission]}:")
    print(f"  Percentage in 23 x 23 cm Pad: {percentages[mission]['smallest']:.2f}%")
    print(f"  Percentage in 33 x 33 cm Pad: {percentages[mission]['mid']:.2f}%")
    print(f"  Percentage in 43 x 43 cm Pad: {percentages[mission]['largest']:.2f}%")

