import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

# Reading the CSV file into a DataFrame
df = pd.read_csv('../touchdown_log/touchdown.csv')

# Define the new origin and convert coordinates to centimeters
new_origin_x = -0.025
new_origin_y = +0.015
df['shifted_touchdown_x'] = (df['touchdown_x'] - new_origin_x) * 100
df['shifted_touchdown_y'] = (df['touchdown_y'] - new_origin_y) * 100

# Compute the absolute value (distance from origin)
df['distance'] = np.sqrt(df['shifted_touchdown_x']**2 + df['shifted_touchdown_y']**2)

# Define mission type labels
mission_labels = {
    'b': 'Mission A',
    'c': 'Mission B',
    'd': 'Mission C',
    'e': 'Mission D',
    'f': 'Mission E',
    'g': 'Mission F'
}

# Prepare data for box plot
boxplot_data = {}
for mission in df['mission_type'].unique():
    mission_data = df[df['mission_type'] == mission]
    boxplot_data[mission_labels[mission]] = mission_data['distance']

# Create a box plot
plt.figure(figsize=(10, 6))  # Adjust size if needed

# Plot box plot for distances
plt.boxplot(
    [boxplot_data[label] for label in mission_labels.values()],
    labels=[label for label in mission_labels.values()],
    widths=0.5
)

# Add labels and title
plt.xlabel('Mission Type')
plt.ylabel('Distance from Origin (cm)')
plt.title('Box Plot of Distances from Origin by Mission Type')

# Add legend and adjust the layout
plt.tight_layout()

# Save the plot to a file
plt.savefig('boxplot_distances_mission_types.pdf', format='pdf')

# Show the plot
plt.show()
