#!/usr/bin/env python3

import rosbag
from sensor_msgs.msg import Image
import cv2
import os
from cv_bridge import CvBridge

# --- Configuration ---
# Set the path to your bag file
BAG_FILE = './Nov10-2025_aruco/19-44-14/2025-11-10-19-44-59.bag'  # <-- CHANGE THIS

# Set the output video file name
OUTPUT_VIDEO_FILE = '19-24-29.mp4'

# Set the image topic you want to extract
IMAGE_TOPIC = '/annotated_image_topic'  # <-- CHANGE THIS if needed

# Set the desired frame rate for the output video
FRAME_RATE = 30.0
# -------------------

# Instantiate CvBridge
bridge = CvBridge()
video = None

try:
    # Open the bag file for reading
    with rosbag.Bag(BAG_FILE, 'r') as bag:
        print(f"Reading messages from topic '{IMAGE_TOPIC}' in bag file '{BAG_FILE}'...")
        
        # Iterate through messages on the specified topic
        for topic, msg, t in bag.read_messages(topics=[IMAGE_TOPIC]):
            try:
                # Convert the ROS Image message to an OpenCV image (NumPy array)
                cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            except Exception as e:
                print(f"Warning: Could not convert image message. Error: {e}")
                continue

            # --- Initialize the VideoWriter on the first valid frame ---
            if video is None:
                # Get the frame dimensions from the first image
                height, width, _ = cv_image.shape
                
                # Define the codec and create VideoWriter object
                # 'mp4v' is a widely compatible codec for .mp4 files
                fourcc = cv2.VideoWriter_fourcc(*'mp4v')
                video = cv2.VideoWriter(OUTPUT_VIDEO_FILE, fourcc, FRAME_RATE, (width, height))
                print(f"Video dimensions set to {width}x{height} at {FRAME_RATE} fps.")
                print(f"Writing to '{OUTPUT_VIDEO_FILE}'...")

            # Write the current frame to the video file
            video.write(cv_image)

    if video is not None:
        print("Video creation complete.")
    else:
        print(f"No images found on topic '{IMAGE_TOPIC}'. No video was created.")

finally:
    # --- Release the VideoWriter object ---
    if video is not None:
        video.release()


