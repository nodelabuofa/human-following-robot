#!/usr/bin/env python3

"""
Computes steering and throttle commands 
based on where the QR/ArUco code is vs. where it should be
"""
import rospy

from std_msgs.msg import Float32MultiArray, Float32
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
import numpy as np

from human_following_robot import library as lib # my custom library(.py) file with functions and config params

## GLOBALS/CONFIG
camera_config = lib.CameraConfig()
controller_config = lib.ControllerConfig()
desired_corners = lib.DESIRED_CORNERS

current_time = None
last_time = None
max_dt = rospy.Duration(0.15)
error_history = np.zeros((controller_config.integral_history_size, 8), dtype=np.float32) # stores visual error history
integral_error_sum = np.zeros(8, dtype=np.float32)

# global so helper functions can access
aruco_corners_sub = None
updated_twist_pub = None
PID_tuning_pub = None



def main():
    global aruco_corners_sub, updated_twist_pub, PID_tuning_pub
    rospy.init_node('visual_error_controller')

    aruco_corners_sub = rospy.Subscriber('aruco_corners_topic', Float32MultiArray, interaction_matrix_callback)
    
    updated_twist_pub = rospy.Publisher('/updated_twist_topic', Twist, queue_size=10)
    PID_tuning_pub = rospy.Publisher('/PI_tuning_topic', Float32MultiArray, queue_size=10)

    rospy.loginfo("Visual Error Controller running")
    rospy.spin() # run forever


### CALLBACKS (when you get mail, ring my doorbell)


# relates motion of QR code in video frame to motion in real world
def interaction_matrix_callback(aruco_corners_msg): 
    global camera_config, last_time, current_time, max_dt, error_history, integral_error_sum, controller_config, updated_twist_pub, PID_tuning_pub

    aruco_corners_array = np.array(aruco_corners_msg.data, dtype=np.float32) # convert from Float32MultiArray back to numpy array
    
    current_frame_errors = np.zeros(8, dtype=np.float32)
    interaction_matrices = []
    active_mask = []

    for i in range(4): # 0, 1, 2, 3
        error, depth, skip = lib.compute_unpack_errors(
            i, aruco_corners_array, desired_corners, camera_config)
        if skip == False:
            interaction_matrix = lib.compute_interaction_matrix(error, depth, camera_config)

            current_frame_errors[2*i:2*i+2] = error
            interaction_matrices.append(interaction_matrix)
            active_mask.extend([True, True])
        else:
            active_mask.extend([False, False])

    if not interaction_matrices:
        return

    stacked_interaction_matrices = np.vstack(interaction_matrices) # shape: (2*n,6)
    active_mask = np.array(active_mask)

    current_time = rospy.Time.now()
    if last_time is None:
        last_time = current_time
        return # Skip first frame to initialize last_time

    dt = current_time - last_time # ie. \Delta t
    last_time = current_time # changes for next loop

    if dt < max_dt:
        # Roll the history array down to make space for the new error
        error_history = np.roll(error_history, shift=1, axis=0) # shifts vertically (axis=0) down 1 row
        # Add the new error to the top of the history
        error_history[0] = current_frame_errors * dt.to_sec() 

        # Calculate the integral sum from the moving window
        integral_error_sum = np.sum(error_history, axis=0) # sums them vertically

    
    P = controller_config.proportional_gain * current_frame_errors
    I = controller_config.integral_gain * integral_error_sum

    inverse_stacked_interaction_matrices = np.linalg.pinv(stacked_interaction_matrices) # shape: (6,2*n)

    # v_twist is 6DOF linear + angular velocity vector [v_x, v_y, v_z, \omega_x, \omega_y, \omega_z]
    # Multiply inverse interaction matrix with the masked error vector
    v_twist = -1 * inverse_stacked_interaction_matrices @ (P[active_mask] + I[active_mask]) # matrix vector multiplication
    updated_twist = Twist()

    updated_twist = lib.differential_wheel_speed(v_twist, updated_twist, controller_config)
    updated_twist_pub.publish(updated_twist)

    # bundle for plotting: [P_sum, I_sum, Linear_Vel_Cmd, Angular_Vel_Cmd]
    tuning_msg = Float32MultiArray()
    tuning_msg.data = [
        float(np.nansum(np.abs(P))),      # absolute sum of P entries (8x1)
        float(np.nansum(np.abs(I))),      # absolute sum of I entries (8x1)
        float(v_twist[2] * controller_config.throttle_gain),        # Linear velocity (vz in camera frame)
        float(updated_twist.angular.y)         # Angular velocity (omega_y in camera frame)
    ]
    PID_tuning_pub.publish(tuning_msg)
    
try:
    main()
except rospy.ROSInterruptException:
    pass

