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
from matplotlib.widgets import Slider
import os
import glob

# --- CONFIGURATION ---

BAG_TOPIC_TUNING = '/PI_tuning_topic'
BAG_TOPIC_CORNERS = '/aruco_corners_topic'

# Camera & Target Parameters (From your controller code)
CX, CY = 933, 536
RES_W, RES_H = 1920, 1080
DESIRED_CORNERS_REL = {
    0: {'u': -300, 'v': 400.0},  # Target for top left corner
    1: {'u': 300, 'v': 400.0},   # Target for top right corner
    2: {'u': 260, 'v': -170},    # Target for bottom right corner
    3: {'u': -260.0, 'v': -170}    # Target for bottom left corner
}

def get_raw_desired_pixels():
    """Convert your relative controller targets back to raw pixel coordinates (u, v)."""
    raw_points = []
    for i in range(4):
        u_des = DESIRED_CORNERS_REL[i]['u']
        v_des = DESIRED_CORNERS_REL[i]['v']
        raw_u = u_des + CX
        raw_v = RES_H - (v_des + CY)
        raw_points.append((raw_u, raw_v))
    return np.array(raw_points)

def find_bag_file():
    script_dir = os.path.dirname(os.path.abspath(__file__))
    all_bag_files = glob.glob(os.path.join(script_dir, '**', '*.bag'), recursive=True)
    if not all_bag_files:
        raise FileNotFoundError("No .bag files found.")
    return max(all_bag_files, key=os.path.getctime)

def extract_and_save_csv(bag_path):
    tuning_data = []
    corners_data = []
    bag = rosbag.Bag(bag_path, 'r')
    info = bag.get_type_and_topic_info()[1]

    topic_map = {}
    for target in [BAG_TOPIC_TUNING, BAG_TOPIC_CORNERS]:
        if target in info:
            topic_map[target] = target
        elif target.lstrip('/') in info:
            topic_map[target] = target.lstrip('/')
        else:
            print(f"WARNING: Topic {target} not found in bag.")

    for topic, msg, t in bag.read_messages(topics=list(topic_map.values())):
        timestamp = t.to_sec()
        if topic == topic_map.get(BAG_TOPIC_TUNING):
            if len(msg.data) == 4:
                tuning_data.append({
                    'time': timestamp,
                    'P_Sum': msg.data[0],
                    'I_Sum': msg.data[1],
                    'Linear_Cmd': msg.data[2],
                    'Angular_Cmd': msg.data[3]
                })
        elif topic == topic_map.get(BAG_TOPIC_CORNERS):
            raw_arr = np.array(msg.data)
            if len(raw_arr) >= 13:
                row = {'time': timestamp}
                for i in range(4):
                    base_idx = 1 + (i * 3)
                    row[f'u{i}'] = raw_arr[base_idx]
                    row[f'v{i}'] = raw_arr[base_idx + 1]
                corners_data.append(row)

    bag.close()

    base_dir = os.path.dirname(bag_path)
    base_name = os.path.splitext(os.path.basename(bag_path))[0]

    df_tune = pd.DataFrame(tuning_data)
    df_corners = pd.DataFrame(corners_data)

    tune_csv = os.path.join(base_dir, 'PI_tuning.csv')
    corners_csv = os.path.join(base_dir, 'aruco_corners.csv')

    df_tune.to_csv(tune_csv, index=False)
    df_corners.to_csv(corners_csv, index=False)

    print(f"Saved tuning data CSV to: {tune_csv}")
    print(f"Saved corners data CSV to: {corners_csv}")

    return tune_csv, corners_csv

def load_and_visualize(tune_csv, corners_csv):
    df_tune = pd.read_csv(tune_csv)
    df_vis = pd.read_csv(corners_csv)

    # Normalize Time
    start_time = min(df_tune['time'].iloc[0], df_vis['time'].iloc[0])
    df_tune['time'] -= start_time
    df_vis['time'] -= start_time

    # Merge Data (Synchronize Vision to Controller frequency)
    df_merged = pd.merge_asof(df_tune.sort_values('time'), 
                              df_vis.sort_values('time'), 
                              on='time', 
                              direction='nearest')

    # --- PLOTTING ---
    fig = plt.figure(figsize=(14, 10))
    gs = fig.add_gridspec(3, 2, height_ratios=[2, 1, 1])

    # Virtual Camera View
    ax_cam = fig.add_subplot(gs[0, :])
    ax_cam.set_title(f"Virtual Camera View ({os.path.basename(tune_csv)})")
    ax_cam.set_xlim(0, RES_W)
    ax_cam.set_ylim(RES_H, 0) # Invert Y to match image coords
    ax_cam.set_aspect('equal')
    ax_cam.grid(True, linestyle='--', alpha=0.5)

    # Static Desired Targets
    targets = get_raw_desired_pixels()
    ax_cam.scatter(targets[:, 0], targets[:, 1], c='green', marker='x', s=100, label='Desired (Ref)', zorder=10)

    # Actual Markers Init
    current_u = [df_merged.iloc[0][f'u{i}'] for i in range(4)] if 'u0' in df_merged else []
    current_v = [df_merged.iloc[0][f'v{i}'] for i in range(4)] if 'v0' in df_merged else []
    scat_actual = ax_cam.scatter(current_u, current_v, c='red', marker='o', s=80, label='Actual (Live)')
    ax_cam.legend(loc='upper right')

    # Controller Error Sums
    ax_err = fig.add_subplot(gs[1, :])
    ax_err.plot(df_merged['time'], df_merged['P_Sum'], 'b-', label='P Sum', alpha=0.6)
    ax_err.plot(df_merged['time'], df_merged['I_Sum'], 'orange', label='I Sum', alpha=0.6)
    line_err = ax_err.axvline(x=0, color='red', linestyle='--') # Cursor
    ax_err.set_ylabel('Error Sum')
    ax_err.legend(loc='upper right')
    ax_err.grid(True)

    # --- Controller Commands with dual y-axes ---
    ax_cmd = fig.add_subplot(gs[2, :], sharex=ax_err)

    # Left y-axis: linear velocity
    line_lin, = ax_cmd.plot(df_merged['time'], df_merged['Linear_Cmd'],
                            'g-', label='Linear (m/s)', alpha=0.6)
    ax_cmd.set_ylabel('Linear Cmd (m/s)', color='g')
    ax_cmd.tick_params(axis='y', labelcolor='g')
    ax_cmd.set_xlabel('Time (s)')
    ax_cmd.grid(True)

    # Right y-axis: angular velocity
    ax_cmd2 = ax_cmd.twinx()
    line_ang, = ax_cmd2.plot(df_merged['time'], df_merged['Angular_Cmd'],
                             'r-', label='Angular (rad/s)', alpha=0.6)
    ax_cmd2.set_ylabel('Angular Cmd (rad/s)', color='r')
    ax_cmd2.tick_params(axis='y', labelcolor='r')

    # Shared vertical cursor (same x on both axes)
    line_cmd = ax_cmd.axvline(x=0, color='k', linestyle='--')

    # Combined legend from both axes
    lines = [line_lin, line_ang]
    labels = [l.get_label() for l in lines]
    ax_cmd.legend(lines, labels, loc='upper right')

    # Slider Setup
    plt.subplots_adjust(bottom=0.15)
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
        idx = np.searchsorted(df_merged['time'], target_time)
        if idx >= len(df_merged):
            idx = len(df_merged) - 1
        line_err.set_xdata([df_merged['time'].iloc[idx]])
        line_cmd.set_xdata([df_merged['time'].iloc[idx]])
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

if __name__ == '__main__':
    # Extract CSVs from bag (replace this step with loading CSVs if already saved)
    bag_file = find_bag_file()
    tune_csv, corners_csv = extract_and_save_csv(bag_file)

    # Load CSVs and visualize with slider
    load_and_visualize(tune_csv, corners_csv)
