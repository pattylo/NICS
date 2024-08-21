import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
from matplotlib.patches import Rectangle
from matplotlib import font_manager as fm

# Load the 'Times New Roman' TTF file
font_path = 'Times_New_Roman.ttf'
prop = fm.FontProperties(fname=font_path)

# Reading the CSV file into a DataFrame
df = pd.read_csv('../touchdown_log/touchdown.csv')

# Extracting data based on mission type
missions = ['b', 'c', 'd', 'e', 'f', 'g']
legend_labels = {
    'b': 'M_A', 
    'c': 'M_B', 
    'd': 'M_C', 
    'e': 'M_D', 
    'f': 'M_E', 
    'g': 'M_F'
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

# Create a figure with a specific size
plt.figure(figsize=(8, 8))  # 8x8 inches

# Define the shrinking factor
shrink = 5

# Define the bounds for the regions, including shrinkage
rectangles = [
    {'color': 'lightblue', 'bounds': (-11.5+shrink, -11.5+shrink, (11.5-shrink)*2, (11.5-shrink)*2)},  # Smallest square
    {'color': 'lightgreen', 'bounds': (-16.5+shrink, -16.5+shrink, (16.5-shrink)*2, (16.5-shrink)*2)},  # Mid square
    {'color': 'lightcoral', 'bounds': (-21.5+shrink, -21.5+shrink, (21.5-shrink)*2, (21.5-shrink)*2)}   # Largest square
]

# Plot rectangles and their edges
for rect in rectangles:
    plt.gca().add_patch(Rectangle(
        (rect['bounds'][0], rect['bounds'][1]),  # lower-left corner
        rect['bounds'][2],  # width
        rect['bounds'][3],  # height
        color=rect['color'],
        alpha=0.5  # Adjust transparency if needed
    ))
    
    # Add red-dotted lines on the edge of each rectangle
    x = [rect['bounds'][0], rect['bounds'][0] + rect['bounds'][2], 
         rect['bounds'][0] + rect['bounds'][2], rect['bounds'][0], rect['bounds'][0]]
    y = [rect['bounds'][1], rect['bounds'][1], 
         rect['bounds'][1] + rect['bounds'][3], rect['bounds'][1] + rect['bounds'][3], rect['bounds'][1]]
    
    plt.plot(x, y, 'r--')  # Red-dotted lines

# Initialize an empty list to store outside points for "failed mission"
outside_points = []

# Create a dictionary to keep track of plotted legends
legend_entries = {}

for mission in missions:
    mission_data = df[df['mission_type'] == mission]

    # Define the bounds for the regions including shrinkage
    inside_largest = (abs(mission_data['shifted_touchdown_y']) <= 21.5-shrink) & (abs(mission_data['shifted_touchdown_x']) <= 21.5-shrink)

    # Plot points inside the largest region
    inside_data = mission_data[inside_largest]
    if not inside_data.empty:
        plt.scatter(inside_data['shifted_touchdown_y'], inside_data['shifted_touchdown_x'], color=colors[mission], label=legend_labels[mission], s=160)
        legend_entries[legend_labels[mission]] = colors[mission]

    # Add outside data to the list for "failed mission"
    outside_data = mission_data[~inside_largest]
    if not outside_data.empty:
        outside_points.append(outside_data)

# Combine all outside points into a single DataFrame for plotting
if len(outside_points) > 0:
    all_outside_data = pd.concat(outside_points)
    print(all_outside_data['mission_type'])  # Print mission types of outside data
    
    # Plot each type of outside data with corresponding color
    for mission in missions:
        outside_mission_data = all_outside_data[all_outside_data['mission_type'] == mission]
        if not outside_mission_data.empty:
            plt.scatter(outside_mission_data['shifted_touchdown_y'], outside_mission_data['shifted_touchdown_x'],
                        color=colors[mission], label=f'Failed Mission {legend_labels[mission]}', s=160, marker='x')
            legend_entries[f'Failed Mission {legend_labels[mission]}'] = colors[mission]

# Invert the x-axis
plt.gca().invert_xaxis()

# Set axis limits (in cm)
plt.xlim(22.5, -22.5)
plt.ylim(-22.5, 22.5)

# Set aspect ratio to be equal (square)
plt.gca().set_aspect('equal', adjustable='box')

# Set font properties and sizes
plt.rcParams['font.family'] = prop.get_name()
plt.xlabel('Y-axis (cm)', fontproperties=prop, fontsize=18)
plt.ylabel('X-axis (cm)', fontproperties=prop, fontsize=18)

# Set font size for tick labels
for label in plt.gca().get_xticklabels() + plt.gca().get_yticklabels():
    label.set_fontproperties(prop)
    label.set_fontsize(18)

# Add legend only for plotted entries
plt.legend(loc='upper right', bbox_to_anchor=(1, 1), prop=prop, fontsize=18)

# Save the plot to a PDF file
# plt.savefig('plot_with_colored_regions_cm.pdf', format='pdf')
plt.savefig('plot_with_colored_regions_cm.pdf', format='pdf', bbox_inches='tight')


# Show the plot
# plt.show()
