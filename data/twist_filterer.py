#!/home/jetson-nano/catkin_ws/src/aruco-course-correction/data/venv/bin/python3
import pandas as pd
import numpy as np
if not hasattr(np, 'bool'):
    np.bool = bool
import ast
import matplotlib.pyplot as plt

# Read the CSV, skipping the header lines that don't contain data
csv_file = 'Nov10-2025_aruco/17-04-55/aruco_corners_topic.csv'

# Read with no header, then manually set columns after removing the first 4 lines
raw = pd.read_csv(csv_file, header=None, skiprows=4, names=['time', 'layout_dim', 'layout_data_offset', 'data'])

# Only keep 'time' and 'data'
df = raw[['time', 'data']].copy()

def extract_depths(data_str):
    try:
        values = ast.literal_eval(data_str)
        # Ensure values is a tuple/list and length is multiple of 3
        if isinstance(values, (tuple, list)) and len(values) % 3 == 0:
            return [values[i] for i in range(2, len(values), 3)]
        else:
            return [float('nan')] * 4  # Or however many depth values expected
    except (ValueError, SyntaxError):
        # Return NaNs or some sentinel value if parsing fails
        return [float('nan')] * 4


# Extract depths
df['depths'] = df['data'].apply(extract_depths)

# Unroll depths into separate columns for graphing
depth_df = pd.DataFrame(df['depths'].tolist(), index=df['time'])
depth_df.columns = [f'depth_{i+1}' for i in range(depth_df.shape[1])]

# Plot all depths over time
plt.figure(figsize=(12,6))
for col in depth_df.columns:
    plt.plot(depth_df.index, depth_df[col], label=col)
plt.xlabel('Time')
plt.ylabel('Depth (m)')
plt.title('ArUco Marker Corner Depths Over Time')
plt.legend()
plt.xticks(rotation=45)
plt.tight_layout()
plt.show()

