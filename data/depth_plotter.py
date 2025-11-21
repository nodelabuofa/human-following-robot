#!/usr/bin/env python3
import numpy as np

# --- PATCH FOR NUMPY DEPRECATION ISSUES ---
if not hasattr(np, 'bool'):
    np.bool = bool
if not hasattr(np, 'object'):
    np.object = object

import rosbag
import pandas as pd
import ast
import matplotlib.pyplot as plt
import os
import glob

# --- Find the most recent rosbag file ---
script_dir = os.path.dirname(os.path.abspath(__file__))
subdirectories = [d for d in os.listdir(script_dir) if os.path.isdir(os.path.join(script_dir, d))]
if not subdirectories:
    raise FileNotFoundError(f"No subdirectories found in {script_dir}")
most_recent_dir = max([os.path.join(script_dir, d) for d in subdirectories], key=os.path.getctime)
bag_files = glob.glob(os.path.join(most_recent_dir, '**', '*.bag'), recursive=True)
if not bag_files:
    raise FileNotFoundError(f"No .bag files found in the most recent directory or its subdirectories: {most_recent_dir}")
bag_file = bag_files[0]
print(f"Processing ROSbag file: {bag_file}")

# Set the correct, absolute topic name
topic_name = '/aruco_corners_topic'

# --- Read data from ROS bag ---
data = []
with rosbag.Bag(bag_file, 'r') as bag:
    for topic, msg, t in bag.read_messages(topics=[topic_name]):
        data.append({
            'time': t.to_sec(),
            'data': str(msg.data)
        })

# --- Check if any messages were actually found ---
if not data:
    error_message = (
        f"Error: No messages were found on the topic '{topic_name}' in the bag file.\n"
        "Please verify the correct topic name by running:\n"
        f"rosbag info {bag_file}"
    )
    raise ValueError(error_message)

# --- Create a DataFrame from the extracted data ---
df = pd.DataFrame(data)

def extract_depths(data_str):
    try:
        values = ast.literal_eval(data_str)
        if isinstance(values, (tuple, list)) and len(values) % 3 == 0:
            return [values[i] for i in range(2, len(values), 3)]
        else:
            return [float('nan')] * 4
    except (ValueError, SyntaxError):
        return [float('nan')] * 4

# --- Process and plot the data ---
df['depths'] = df['data'].apply(extract_depths)
depth_df = pd.DataFrame(df['depths'].tolist(), index=df['time'])
depth_df.columns = [f'depth_{i+1}' for i in range(depth_df.shape[1])]

plt.figure(figsize=(12, 6))
for col in depth_df.columns:
    plt.plot(depth_df.index, depth_df[col], label=col)

plt.xlabel('Time')
plt.ylabel('Depth (m)')
plt.title('ArUco Marker Corner Depths Over Time from ROSbag')
plt.legend()
plt.xticks(rotation=45)
plt.tight_layout()

# --- MODIFIED: Save the plot to the bag file's directory ---
# Get the directory where the .bag file is located
bag_dir = os.path.dirname(bag_file)
# Get the base name of the .bag file (without the extension)
base_bag_name = os.path.splitext(os.path.basename(bag_file))[0]
# Construct the full save path
save_path = os.path.join(bag_dir, f"{base_bag_name}_depth_plot.png")

print(f"Saving plot to: {save_path}")
plt.savefig(save_path)

# --- Display the plot ---
plt.show()