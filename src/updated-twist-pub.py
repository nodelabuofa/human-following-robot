#!/usr/bin/env python3

# """
# Computes velocity commands based on visual servo error
# Publishes emergency stop command from MSI gaming controller
# """

import rospy

from std_msgs.msg import Float32MultiArray, Float32
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

import numpy as np

class UpdatedTwistPub:
    def __init__(self):
        rospy.init_node('updated_twist_pub')

        # Subscribe to aruco_corners_topic
        self.aruco_corner_sub = rospy.Subscriber('aruco_corners_topic', Float32MultiArray, self.interaction_matrix_callback)
        rospy.loginfo("Subscribed to aruco_corners_topic")

        # publish to updated_twist_topic
        self.updated_twist_pub = rospy.Publisher('/updated_twist_topic', Twist, queue_size=10)
        self.PI_tuning_pub = rospy.Publisher('/PI_tuning_topic', Float32MultiArray, queue_size=10)


        # ArUco must be in particular orientation
        self.desired_corners = {
            0: {'u': -150, 'v': 200.0},  # Target for top left corner
            1: {'u': 150, 'v': 200.0},   # Target for top right corner
            2: {'u': 130, 'v': -85},    # Target for bottom right corner
            3: {'u': -130.0, 'v': -85.0}    # Target for bottom left corner
        }

        # for PI controller
        self.proportional_gain = 0.5

        self.integral_error_sum = np.zeros(8, dtype=np.float32)
        self.last_time = rospy.Time.now() # kinda attribute, kinda method, when "called", executes rospy.Time.now()
        self.error_history = np.zeros((3, 8), dtype=np.float32) # 5 rows (last 5 successful detection frames) by 8 entries (8 entry long error vector)
        self.integral_gain = 0.1 # This is the Ki gain, tune as needed
        self.max_dt = rospy.Duration(0.15) # max time between frames to prevent integral windup


    def interaction_matrix_callback(self, aruco_corners_msg):
        """
        Updates entries of image Jacobian interaction matrix
        
        Args:
            aruco_corners_msg (Float32MultiArray): similar to list, content: [x1, y1, d1, x2, ..., y4, d4]
        Returns:
        """

        # intrinsic ZED mini camera parameters found using rostopic echo /zedm/zed_node/left/camera_info
        # pixel resolution 960 x 540
        f = 750 # focal length, UPDATE
        rho = 0.000002 # physical individual square pixel sensor width AND height conversion
        cx = 467 # found by
        cy = 268

        aruco_corners_data = np.array(aruco_corners_msg.data, dtype=np.float32)

        # Extract the original timestamp from the message
        start_timestamp = aruco_corners_data[0]
        aruco_corners_data = aruco_corners_data[1:] # remove timestamp so can process properly
        # rospy.loginfo(f'Pseudo Inverse of Interaction Matrix L:\n{L_inv}')

        # how much time has passed (used for simple numerical integration for PI later)
        current_time = rospy.Time.now()
        dt = current_time - self.last_time # ie. \Delta t
        self.last_time = current_time # changes for next loop

        interaction_matrices = []
        error_vectors = []
        corner_mask = []
        # control gain
        steering_gain = 1
        throttle_gain = -0.50
        DEAD_ZONE_THRESHOLD = 8  # as it gets closer

        # unpacks (x,y,d) to (u,v,Z) for all 4 corners
        for i in range(4): # 0, 1, 2, 3
            u_0 = aruco_corners_data[i*3 + 0]
            v_0 = aruco_corners_data[i*3 + 1]
            Z = aruco_corners_data[i*3 + 2]

            # flipping so origin at bottom right of visual feed
            v_flipped = 540 - v_0

            # convert from origin at bottom left to origin at center
            u = u_0 - cx
            v = v_flipped - cy

            u_desired = self.desired_corners[i]['u']
            v_desired = self.desired_corners[i]['v']

            u_error = u - u_desired
            v_error = v - v_desired

            e = np.array([u_error, v_error], dtype=np.float32) # error vector for this corner
            error_vectors.append(e) # append it before dropping corners so if ArUco detected w/odepth from ZED, still adds

            if Z == 0 or np.isnan(Z) or np.isinf(Z) or Z <= 0.1:
                corner_mask.extend([False, False])
                continue


            corner_mask.extend([True, True])

            # 2x6 empty interaction matrix
            L = np.zeros((2, 6), dtype=np.float32)

            # First row
            L[0, 0] = -f / Z
            L[0, 1] = 0
            L[0, 2] = u_error / Z
            L[0, 3] = (u_error * v_error) / f
            L[0, 4] = -f * rho  - ((u_error ** 2) / f)
            L[0, 5] = v_error

            # Second row
            L[1, 0] = 0
            L[1, 1] = -f / Z
            L[1, 2] = v_error / Z
            L[1, 3] = f * rho + ((v_error ** 2) / f)
            L[1, 4] = -(u_error * v_error) / f
            L[1, 5] = -u_error

            interaction_matrices.append(L)

        if not interaction_matrices:
            return # don't try anything if no corners found
        
        L_stacked = np.vstack(interaction_matrices) # shape: (8,6)

        # if np.isnan(L_stacked).any():
        #     # rospy.logerr("Missing depth values, matrix not computed.")
        #     return
        # else:
        current_stacked_error = np.concatenate(error_vectors) # shape: (8,1)

        L_stacked_inv = np.linalg.pinv(L_stacked) # shape: (6,8)

        if dt < self.max_dt:
            # Roll the history array to make space for the new error
            self.error_history = np.roll(self.error_history, shift=1, axis=0) # shifts vertically (axis=0) down 1 row, bringing bottom (5 frame old error vector) to top
            
            scaled_error = current_stacked_error.copy()
            scaled_error[np.abs(scaled_error) < DEAD_ZONE_THRESHOLD] = 0
            # Add the new error to the top of the history
            self.error_history[0] = scaled_error * dt.to_sec() # replaces first row (not simply top left entry) with numerical integral

            # Calculate the integral sum from the moving window
            self.integral_error_sum = np.sum(self.error_history, axis=0) # sums them vertically

         # --- Tuning Data Computation ---
        P = self.proportional_gain * current_stacked_error
        I = self.integral_gain * self.integral_error_sum

        # Apply mask for control calculation
        active_mask = np.array(corner_mask)
        masked_P = P[active_mask]
        masked_I = I[active_mask]
        
        # PI control law
        v_twist = -1 * L_stacked_inv @ (masked_P + masked_I)

        updated_twist = Twist() # special geometry_msgs datatype digestible by ROS, float64
                
        if abs(v_twist[2]) < 1e-2: # prevents division by zero and huge steering values
            updated_twist.angular.y = 0.0
        else:
            if v_twist[4] > 6: # max angular velocity of kinematic bicycle model v/L, max lin velocity is 1 m/s, wheel to wheel length 0.1778, so max omega approx 6
                v_twist[4] = 6
            if v_twist[4] < -6:
                v_twist[4] = -6

            updated_twist.angular.y = float(np.arctan(v_twist[4] * 0.1778 / v_twist[2])) * steering_gain
            v_twist[4] = updated_twist.angular.y
        updated_twist.linear.z = float(v_twist[2])

        # linear.x is left wheel, linear.y is right wheel
        # left is positive radian max, right is negative radian max
        # 0.1778 is vehicle length (wheel center to center), 0.1667 wheelbase (rear tires, tread center to center)
        if updated_twist.angular.y == 0:
            # z axis portrudes perpendicular from lens, so z from computed twist is desired linear velocity of end effector/camera
            updated_twist.linear.x = float(v_twist[2]) * throttle_gain
            updated_twist.linear.y = float(v_twist[2]) * throttle_gain
        else:
            turning_radius = 0.1778 / np.tan(updated_twist.angular.y) # 0.
            ackerman_differential_ratio = (abs(turning_radius) - (0.1667/2)) / (abs(turning_radius) + (0.1667/2))
            
            if turning_radius > 0: # turning left
                updated_twist.linear.y = float(v_twist[2] * throttle_gain) # outer (right) wheel has max velocity
                updated_twist.linear.x = float(float(v_twist[2]) * ackerman_differential_ratio * throttle_gain)
            else: # turning right
                updated_twist.linear.x = float(v_twist[2] * throttle_gain) # outer (left) wheel has max velocity
                updated_twist.linear.y = float(float(v_twist[2]) * ackerman_differential_ratio * throttle_gain)

        # bundle for plotting: [P_sum, I_sum, Linear_Vel_Cmd, Angular_Vel_Cmd]
        tuning_msg = Float32MultiArray()
        tuning_msg.data = [
            float(np.nansum(np.abs(P))),      # absolute sum of P entries (8x1)
            float(np.nansum(np.abs(I))),      # absolute sum of I entries (8x1)
            float(v_twist[2] * throttle_gain),        # Linear velocity (vz in camera frame)
            float(v_twist[4])         # Angular velocity (omega_y in camera frame)
        ]
        self.PI_tuning_pub.publish(tuning_msg)

        self.updated_twist_pub.publish(updated_twist)
        return
    
    def run(self):
        rospy.spin()


if __name__ == '__main__':
    try:
        node = UpdatedTwistPub()
        node.run()
    except rospy.ROSInterruptException:
        pass
