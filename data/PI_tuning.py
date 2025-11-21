#!/usr/bin/env python3
import numpy as np

# --- PATCH FOR NUMPY DEPRECATION ISSUES ---
try:
    np.bool = np.bool_
except AttributeError:
    np.bool = bool
try:
    np.object = np.object_
except AttributeError:
    np.object = object

import rosbag
import pandas as pd
import matplotlib.pyplot as plt
import os
import glob

# --- Find the most recent rosbag file ---
script_dir = os.path.dirname(os.path.abspath(__file__))

# Recursively find ALL .bag files in the script directory or subdirectories
all_bag_files = glob.glob(os.path.join(script_dir, '**', '*.bag'), recursive=True)

if not all_bag_files:
    raise FileNotFoundError(f"No .bag files found in {script_dir} or any subdirectories.")

# Sort all found bag files by modification time (newest last)
bag_file = max(all_bag_files, key=os.path.getctime)

print(f"Processing ROSbag file: {bag_file}")

# Set the intended topic name
topic_name = '/PI_tuning_topic'

# --- Read data from ROS bag ---
data = []
bag = rosbag.Bag(bag_file, 'r')

# 1. Robust Topic Checking
info_dict = bag.get_type_and_topic_info()[1]
if topic_name not in info_dict:
    print(f"WARNING: Topic '{topic_name}' not found in bag.")
    # Try to find it without the leading slash
    alt_name = topic_name.lstrip('/')
    if alt_name in info_dict:
        topic_name = alt_name
        print(f"Found topic as '{topic_name}' instead.")
    else:
        available_topics = list(info_dict.keys())
        bag.close()
        raise ValueError(f"Topic '{topic_name}' is completely missing from the rosbag.\nAvailable topics: {available_topics}")

print(f"Reading from topic: {topic_name}")

# 2. Data Extraction with Debugging
msg_count = 0
for topic, msg, t in bag.read_messages(topics=[topic_name]):
    msg_count += 1
    # Check length to ensure it matches our new [P, I, v, w] format
    if len(msg.data) == 4:
        data.append({
            'time': t.to_sec(),
            'P_Sum': msg.data[0],
            'I_Sum': msg.data[1],
            'Linear_Vel': msg.data[2],
            'Angular_Vel': msg.data[3]
        })
    else:
        # This helps debug if you recorded with an old version of the node
        if msg_count == 1: 
            print(f"WARNING: Found message with length {len(msg.data)}, expected 4. Skipping incompatible messages.")

bag.close()

# --- Check if data was actually extracted ---
if not data:
    raise ValueError(f"Topic '{topic_name}' exists but no valid messages were extracted.\n"
                     f"Total messages seen: {msg_count}.\n"
                     "Possible Cause: The recorded messages have the wrong length (old code version?). Expected length 4.")

# --- Create DataFrame ---
tuning_df = pd.DataFrame(data)
tuning_df.set_index('time', inplace=True)

# Normalize time to start at 0 seconds
tuning_df.index = tuning_df.index - tuning_df.index[0]

# --- Plotting ---
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 10), sharex=True)

# Plot 1: Controller Error Sums
ax1.plot(tuning_df.index, tuning_df['P_Sum'], label='P Sum (Instant Error)', color='blue')
ax1.plot(tuning_df.index, tuning_df['I_Sum'], label='I Sum (Accumulated Error)', color='orange')
ax1.set_ylabel('Error Magnitude (Pixels)')
ax1.set_title(f'Controller Inputs (Bag: {os.path.basename(bag_file)})')
ax1.legend()
ax1.grid(True)

#!/usr/bin/env python3
import numpy as np

# --- PATCH FOR NUMPY DEPRECATION ISSUES ---
try:
    np.bool = np.bool_
except AttributeError:
    np.bool = bool
try:
    np.object = np.object_
except AttributeError:
    np.object = object

import rosbag
import pandas as pd
import matplotlib.pyplot as plt
import os
import glob

# --- Find the most recent rosbag file ---
script_dir = os.path.dirname(os.path.abspath(__file__))

# Recursively find ALL .bag files in the script directory or subdirectories
all_bag_files = glob.glob(os.path.join(script_dir, '**', '*.bag'), recursive=True)

if not all_bag_files:
    raise FileNotFoundError(f"No .bag files found in {script_dir} or any subdirectories.")

# Sort all found bag files by modification time (newest last)
bag_file = max(all_bag_files, key=os.path.getctime)

print(f"Processing ROSbag file: {bag_file}")

# Set the intended topic name
topic_name = '/PI_tuning_topic'

# --- Read data from ROS bag ---
data = []
bag = rosbag.Bag(bag_file, 'r')

# 1. Robust Topic Checking
info_dict = bag.get_type_and_topic_info()[1]
if topic_name not in info_dict:
    print(f"WARNING: Topic '{topic_name}' not found in bag.")
    # Try to find it without the leading slash
    alt_name = topic_name.lstrip('/')
    if alt_name in info_dict:
        topic_name = alt_name
        print(f"Found topic as '{topic_name}' instead.")
    else:
        available_topics = list(info_dict.keys())
        bag.close()
        raise ValueError(f"Topic '{topic_name}' is completely missing from the rosbag.\nAvailable topics: {available_topics}")

print(f"Reading from topic: {topic_name}")

# 2. Data Extraction with Debugging
msg_count = 0
for topic, msg, t in bag.read_messages(topics=[topic_name]):
    msg_count += 1
    # Check length to ensure it matches our new [P, I, v, w] format
    if len(msg.data) == 4:
        data.append({
            'time': t.to_sec(),
            'P_Sum': msg.data[0],
            'I_Sum': msg.data[1],
            'Linear_Vel': msg.data[2],
            'Angular_Vel': msg.data[3]
        })
    else:
        # This helps debug if you recorded with an old version of the node
        if msg_count == 1: 
            print(f"WARNING: Found message with length {len(msg.data)}, expected 4. Skipping incompatible messages.")

bag.close()

# --- Check if data was actually extracted ---
if not data:
    raise ValueError(f"Topic '{topic_name}' exists but no valid messages were extracted.\n"
                     f"Total messages seen: {msg_count}.\n"
                     "Possible Cause: The recorded messages have the wrong length (old code version?). Expected length 4.")

# --- Create DataFrame ---
tuning_df = pd.DataFrame(data)
tuning_df.set_index('time', inplace=True)

# Normalize time to start at 0 seconds
tuning_df.index = tuning_df.index - tuning_df.index[0]

# --- Plotting ---
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 10), sharex=True)

# Plot 1: Controller Error Sums
ax1.plot(tuning_df.index, tuning_df['P_Sum'], label='P Sum (Instant Error)', color='blue')
ax1.plot(tuning_df.index, tuning_df['I_Sum'], label='I Sum (Accumulated Error)', color='orange')
ax1.set_ylabel('Error Magnitude (Pixels)')
ax1.set_title(f'Controller Inputs (Bag: {os.path.basename(bag_file)})')
ax1.legend()
ax1.grid(True)

# Plot 2: Velocity Commands
ax2.plot(tuning_df.index, tuning_df['Linear_Vel'], label='Linear Vel (m/s)', color='green')
ax2.plot(tuning_df.index, tuning_df['Angular_Vel'], label='Angular Vel (rad/s)', color='red')
ax2.set_ylabel('Command Magnitude')
ax2.set_xlabel('Time (s)')
ax2.set_title('Controller Outputs: Twist Commands')
ax2.legend()
ax2.grid(True)

plt.xticks(rotation=45)
plt.tight_layout()

# saves plot in same directory as bag file
bag_dir = os.path.dirname(bag_file)
base_bag_name = os.path.splitext(os.path.basename(bag_file))[0]
save_path = os.path.join(bag_dir, f"{base_bag_name}_tuning_plot.png")

print(f"Saving plot to: {save_path}")
plt.savefig(save_path)


plt.show() # shows both in matplot tab


plt.xticks(rotation=45)
plt.tight_layout()

# --- Save the plot ---
bag_dir = os.path.dirname(bag_file)
base_bag_name = os.path.splitext(os.path.basename(bag_file))[0]
save_path = os.path.join(bag_dir, f"{base_bag_name}_tuning_plot.png")

print(f"Saving plot to: {save_path}")
plt.savefig(save_path)

# --- Display ---
plt.show()
