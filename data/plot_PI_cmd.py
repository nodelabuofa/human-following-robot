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
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider
import os

# Adjust these paths to where your CSV files are stored
tune_csv = 'PI_tuning.csv'
corners_csv = 'aruco_corners.csv'

# Same constants and function as before
CX, CY = 933, 536
RES_W, RES_H = 1920, 1080
DESIRED_CORNERS_REL = {
    0: {'u': -300, 'v': 400.0},  # Target for top left corner
    1: {'u': 300, 'v': 400.0},   # Target for top right corner
    2: {'u': 260, 'v': -170},    # Target for bottom right corner
    3: {'u': -260.0, 'v': -170}    # Target for bottom left corner
}

def get_raw_desired_pixels():
    raw_points = []
    for i in range(4):
        u_des = DESIRED_CORNERS_REL[i]['u']
        v_des = DESIRED_CORNERS_REL[i]['v']
        raw_u = u_des + CX
        raw_v = RES_H - (v_des + CY)
        raw_points.append((raw_u, raw_v))
    return np.array(raw_points)

def load_and_visualize(tune_csv, corners_csv):
    df_tune = pd.read_csv(tune_csv)
    df_vis = pd.read_csv(corners_csv)

    start_time = min(df_tune['time'].iloc[0], df_vis['time'].iloc[0])
    df_tune['time'] -= start_time
    df_vis['time'] -= start_time

    df_merged = pd.merge_asof(df_tune.sort_values('time'), 
                              df_vis.sort_values('time'), 
                              on='time', 
                              direction='nearest')

    fig = plt.figure(figsize=(14, 10))
    gs = fig.add_gridspec(3, 2, height_ratios=[2, 1, 1])

    ax_cam = fig.add_subplot(gs[0, :])
    ax_cam.set_title(f"Virtual Camera View ({os.path.basename(tune_csv)})")
    ax_cam.set_xlim(0, RES_W)
    ax_cam.set_ylim(RES_H, 0)
    ax_cam.set_aspect('equal')
    ax_cam.grid(True, linestyle='--', alpha=0.5)

    targets = get_raw_desired_pixels()
    ax_cam.scatter(targets[:, 0], targets[:, 1], c='green', marker='x', s=100, label='Desired (Ref)', zorder=10)

    current_u = [df_merged.iloc[0][f'u{i}'] for i in range(4)] if 'u0' in df_merged else []
    current_v = [df_merged.iloc[0][f'v{i}'] for i in range(4)] if 'v0' in df_merged else []
    scat_actual = ax_cam.scatter(current_u, current_v, c='red', marker='o', s=80, label='Actual (Live)')
    ax_cam.legend(loc='upper right')

    ax_err = fig.add_subplot(gs[1, :])
    ax_err.plot(df_merged['time'], df_merged['P_Sum'], 'b-', label='P Sum', alpha=0.6)
    ax_err.plot(df_merged['time'], df_merged['I_Sum'], 'orange', label='I Sum', alpha=0.6)
    line_err = ax_err.axvline(x=0, color='red', linestyle='--')
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

if __name__ == "__main__":
    load_and_visualize(tune_csv, corners_csv)
