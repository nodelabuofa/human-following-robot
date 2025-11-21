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

try:
    np.bool
except AttributeError:
    np.bool = np.bool_
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider
import os
import glob

# --- CONFIGURATION ---
BAG_TOPIC_TUNING = '/PI_tuning_topic'
BAG_TOPIC_CORNERS = '/aruco_corners_topic'

# Camera & Target Parameters (From your controller code)
CX, CY = 467, 268
RES_W, RES_H = 960, 540
DESIRED_CORNERS_REL = {
    0: {'u': -150, 'v': 200.0},
    1: {'u': 150, 'v': 200.0},
    2: {'u': 130, 'v': -85},
    3: {'u': -130.0, 'v': -85.0}
}

def get_raw_desired_pixels():
    """Convert your relative controller targets back to raw pixel coordinates (u, v)."""
    raw_points = []
    for i in range(4):
        u_des = DESIRED_CORNERS_REL[i]['u']
        v_des = DESIRED_CORNERS_REL[i]['v']
        
        # Inverse of your controller logic:
        # u = u_0 - cx  ->  u_0 = u + cx
        raw_u = u_des + CX
        
        # v = (540 - v_0) - cy  ->  v + cy = 540 - v_0  ->  v_0 = 540 - (v + cy)
        raw_v = RES_H - (v_des + CY)
        raw_points.append((raw_u, raw_v))
    return np.array(raw_points)

def find_bag_file():
    script_dir = os.path.dirname(os.path.abspath(__file__))
    all_bag_files = glob.glob(os.path.join(script_dir, '**', '*.bag'), recursive=True)
    if not all_bag_files:
        raise FileNotFoundError("No .bag files found.")
    return max(all_bag_files, key=os.path.getctime)

def extract_data(bag_path):
    tuning_data = []
    corners_data = []
    
    print(f"Processing: {os.path.basename(bag_path)}")
    bag = rosbag.Bag(bag_path, 'r')
    
    # Get available topics to handle missing slashes
    info = bag.get_type_and_topic_info()[1]
    topics_to_read = []
    
    # Robust topic name matching
    topic_map = {}
    for target in [BAG_TOPIC_TUNING, BAG_TOPIC_CORNERS]:
        if target in info:
            topics_to_read.append(target)
            topic_map[target] = target
        elif target.lstrip('/') in info:
            alt = target.lstrip('/')
            topics_to_read.append(alt)
            topic_map[target] = alt
        else:
            print(f"WARNING: Topic {target} not found in bag.")

    for topic, msg, t in bag.read_messages(topics=topics_to_read):
        timestamp = t.to_sec()
        
        # 1. Extract Controller Data
        if topic == topic_map.get(BAG_TOPIC_TUNING):
            if len(msg.data) == 4:
                tuning_data.append({
                    'time': timestamp,
                    'P_Sum': msg.data[0],
                    'I_Sum': msg.data[1],
                    'Linear_Cmd': msg.data[2],
                    'Angular_Cmd': msg.data[3]
                })

        # 2. Extract Vision Data (ArUco Corners)
        elif topic == topic_map.get(BAG_TOPIC_CORNERS):
            # msg.data layout: [internal_timestamp, u0, v0, d0, u1, v1, d1, ...]
            raw_arr = np.array(msg.data)
            if len(raw_arr) >= 13: # 1 + 4*3
                # Extract pairs of (u, v). We skip 'depth' (indexes 3, 6, 9, 12)
                # Data starts at index 1. 
                # Corner 0: idx 1, 2
                # Corner 1: idx 4, 5
                # Corner 2: idx 7, 8
                # Corner 3: idx 10, 11
                row = {'time': timestamp}
                
                # Store corners as u0, v0, u1, v1 ...
                for i in range(4):
                    base_idx = 1 + (i * 3)
                    row[f'u{i}'] = raw_arr[base_idx]
                    row[f'v{i}'] = raw_arr[base_idx + 1]
                
                corners_data.append(row)

    bag.close()
    return pd.DataFrame(tuning_data), pd.DataFrame(corners_data)

# --- MAIN EXECUTION ---
bag_file = find_bag_file()
df_tune, df_vis = extract_data(bag_file)

if df_tune.empty:
    raise ValueError("No tuning data found.")
if df_vis.empty:
    print("WARNING: No ArUco corner data found. Visualization will be static.")
    # Create dummy vision data to prevent crash
    df_vis = pd.DataFrame({'time': df_tune['time'], 'u0':0, 'v0':0})

# Normalize Time
start_time = min(df_tune['time'].iloc[0], df_vis['time'].iloc[0])
df_tune['time'] -= start_time
df_vis['time'] -= start_time

# Merge Data (Synchronize Vision to Controller frequency)
# We use merge_asof to find the closest vision frame for every controller step
df_merged = pd.merge_asof(df_tune.sort_values('time'), 
                          df_vis.sort_values('time'), 
                          on='time', 
                          direction='nearest')

# --- PLOTTING ---
fig = plt.figure(figsize=(14, 10))
gs = fig.add_gridspec(3, 2, height_ratios=[2, 1, 1])

# 1. Virtual Camera View (Top Full Width)
ax_cam = fig.add_subplot(gs[0, :])
ax_cam.set_title(f"Virtual Camera View (Bag: {os.path.basename(bag_file)})")
ax_cam.set_xlim(0, RES_W)
ax_cam.set_ylim(RES_H, 0) # Invert Y to match computer vision coordinates
ax_cam.set_aspect('equal')
ax_cam.grid(True, linestyle='--', alpha=0.5)

# Plot Static Desired Targets
targets = get_raw_desired_pixels()
ax_cam.scatter(targets[:, 0], targets[:, 1], c='green', marker='x', s=100, label='Desired (Ref)', zorder=10)

# Initialize Moving "Actual" Markers
# We initialize with the first frame of data
current_u = [df_merged.iloc[0][f'u{i}'] for i in range(4)] if 'u0' in df_merged else []
current_v = [df_merged.iloc[0][f'v{i}'] for i in range(4)] if 'v0' in df_merged else []
scat_actual = ax_cam.scatter(current_u, current_v, c='red', marker='o', s=80, label='Actual (Live)')
ax_cam.legend(loc='upper right')

# 2. Controller Error Sums
ax_err = fig.add_subplot(gs[1, :])
l1, = ax_err.plot(df_merged['time'], df_merged['P_Sum'], 'b-', label='P Sum', alpha=0.6)
l2, = ax_err.plot(df_merged['time'], df_merged['I_Sum'], 'orange', label='I Sum', alpha=0.6)
line_err = ax_err.axvline(x=0, color='red', linestyle='--') # The sliding cursor
ax_err.set_ylabel('Error Sum')
ax_err.legend(loc='upper right')
ax_err.grid(True)

# 3. Controller Commands
ax_cmd = fig.add_subplot(gs[2, :], sharex=ax_err)
ax_cmd.plot(df_merged['time'], df_merged['Linear_Cmd'], 'g-', label='Linear (m/s)', alpha=0.6)
ax_cmd.plot(df_merged['time'], df_merged['Angular_Cmd'], 'r-', label='Angular (rad/s)', alpha=0.6)
line_cmd = ax_cmd.axvline(x=0, color='red', linestyle='--') # The sliding cursor
ax_cmd.set_ylabel('Twist Cmd')
ax_cmd.set_xlabel('Time (s)')
ax_cmd.legend(loc='upper right')
ax_cmd.grid(True)

# --- SLIDER SETUP ---
plt.subplots_adjust(bottom=0.15) # Make room for slider
ax_slider = plt.axes([0.15, 0.02, 0.7, 0.03])
time_slider = Slider(
    ax=ax_slider,
    label='Time (s)',
    valmin=0,
    valmax=df_merged['time'].max(),
    valinit=0,
)

def update(val):
    target_time = time_slider.val
    
    # Find index closest to slider time
    # searchsorted is fast for finding insertion points in sorted arrays
    idx = np.searchsorted(df_merged['time'], target_time)
    if idx >= len(df_merged):
        idx = len(df_merged) - 1
        
    # Update Vertical Lines
    line_err.set_xdata([df_merged['time'].iloc[idx]])
    line_cmd.set_xdata([df_merged['time'].iloc[idx]])
    
    # Update Camera Scatter Points
    if 'u0' in df_merged:
        new_offsets = []
        for i in range(4):
            u = df_merged.iloc[idx][f'u{i}']
            v = df_merged.iloc[idx][f'v{i}']
            new_offsets.append([u, v])
        scat_actual.set_offsets(new_offsets)
        
    fig.canvas.draw_idle()

time_slider.on_changed(update)

plt.show()