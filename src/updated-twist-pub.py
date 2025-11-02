#!/usr/bin/env python3

# """
# Image-Based Visual Servoing Controller Node
# Computes velocity commands based on visual feature error
# """

import rospy

from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist

import numpy as np

class UpdatedTwistPub:
    def __init__(self):
        rospy.init_node('updated_twist_pub')

        # Subscribe to aruco_corners_topic
        self.aruco_corner_sub = rospy.Subscriber('aruco_corners_topic', Float32MultiArray, self.interaction_matrix_callback)
        rospy.loginfo("Subscribed to aruco_corners_topic")

        # publish to updated_twist_topic
        self.updated_twist_pub = rospy.Publisher('/updated_twist_topic', Twist, queue_size=10)
        self.error_pub = rospy.Publisher('/servo_error_topic', Float32MultiArray, queue_size=10 )

        # ArUco must be in particular orientation
        self.desired_corners = {
            0: {'u': -100.0, 'v': 100.0},  # Target for top left corner
            1: {'u': 100.0, 'v': 100.0},   # Target for top right corner
            2: {'u': 100.0, 'v': -50.0},    # Target for bottom right corner
            3: {'u': -100.0, 'v': -50.0}    # Target for bottom left corner
        }
        

    def interaction_matrix_callback(self, aruco_corners_msg):
        """
        Updates entries of image Jacobian interaction matrix
        
        Args:
            aruco_corners_msg (Float32MultiArray): similar to list, content: [x1, y1, d1, x2, ..., y4, d4]
        Returns:
        """

        # intrinsic ZED mini camera parameters
        f = 366 # focal length, UPDATE
        rho = 0.000002 # physical individual square pixel sensor width AND height conversion
        cx = 315
        cy = 178

        aruco_corners_data = np.array(aruco_corners_msg.data, dtype=np.float32)
        # rospy.loginfo(f'Pseudo Inverse of Interaction Matrix L:\n{L_inv}')

        interaction_matrices = []
        error_vectors = []

        # control gain
        steering_gain = 0.5
        throttle_gain = -1.25

        # unpacks (x,y,d) to (u,v,Z) for all 4 corners
        for i in range(4): # 0, 1, 2, 3
            u_0 = aruco_corners_data[i*3 + 0]
            v_0 = aruco_corners_data[i*3 + 1]
            Z = aruco_corners_data[i*3 + 2]

            # flipping so origin at bottom right of visual feed
            v_flipped = 360 - v_0

            # convert from origin at bottom left to origin at center
            u = u_0 - cx
            v = v_flipped - cy

            u_desired = self.desired_corners[i]['u']
            v_desired = self.desired_corners[i]['v']

            u_error = u - u_desired
            v_error = v - v_desired

            e = np.array([u_error, v_error], dtype=np.float32) # error vector for this corner

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
            error_vectors.append(e)
        
        L_stacked = np.vstack(interaction_matrices) # shape: (8,6)

        if np.isnan(L_stacked).any():
            # rospy.logerr("Missing depth values, matrix not computed.")
            return
        else:
            e_stacked = np.concatenate(error_vectors) # shape: (8,1)

            L_stacked_inv = np.linalg.pinv(L_stacked) # shape: (6,8)

            v_twist = -1 * L_stacked_inv @ e_stacked  # Shape: (6,1)

            updated_twist = Twist() # special geometry_msgs datatype digestible by ROS, float64
                    
            updated_twist.angular.x = float(v_twist[3])
            updated_twist.angular.y = float(np.arctan(v_twist[4] * 0.1825 / v_twist[2])) * steering_gain
            updated_twist.angular.z = float(v_twist[5])

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
                
                if turning_radius > 0: # turning left
                    updated_twist.linear.y = float(v_twist[2] * throttle_gain) # outer (right) wheel has max velocity
                    updated_twist.linear.x = float(((turning_radius - (0.1667 / 2)) / (turning_radius + (0.1667 / 2))) * float(v_twist[2]) * throttle_gain)
                else: # turning right
                    updated_twist.linear.x = float(v_twist[2] * throttle_gain) # outer (left) wheel has max velocity
                    updated_twist.linear.y = float(((turning_radius - (0.1667 / 2)) / (turning_radius + (0.1667 / 2))) * float(v_twist[2]) * throttle_gain)

            servo_error_msg = Float32MultiArray() # empty array datatype compatible with ROS
            servo_error_msg.data = e_stacked
            self.error_pub.publish(servo_error_msg)

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