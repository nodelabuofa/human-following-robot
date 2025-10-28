#!/usr/bin/env python3

# """
# Image-Based Visual Servoing Controller Node
# Computes velocity commands based on visual feature error
# """

import rospy

from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist

import numpy as np

turning_radius = 0

class UpdatedTwistPub:
    def __init__(self):
        rospy.init_node('updated_twist_pub')

        # Subscribe to aruco_corners_topic
        self.aruco_corner_sub = rospy.Subscriber('aruco_corners_topic', Float32MultiArray, self.interaction_matrix_callback)
        rospy.loginfo("Subscribed to aruco_corners_topic")

        # publish to updated_twist_topic
        self.updated_twist_pub = rospy.Publisher('/updated_twist_topic', Twist, queue_size=10)

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

        # unpack (x,y,d) to (u,v,Z)
        u_0 = aruco_corners_data[0] 
        v_0 = aruco_corners_data[1]
        Z = aruco_corners_data[2]

        # flip origin from top left to bottom left to follow math conventions (360 is pixel resolution height)
        v_flipped = 360 - v_0

        # convert from origin at bottom left to origin at center
        u = cx - u_0
        v = cy - v_flipped

        # specify target position of ArUco marker
        u_desired = -100
        v_desired = -50
        u_bar = u_desired - u
        v_bar = v_desired - v

        # 2x6 empty matrix
        L = np.zeros((2, 6), dtype=np.float32)

        # First row
        L[0, 0] = -f / Z
        L[0, 1] = 0
        L[0, 2] = u_bar / Z
        L[0, 3] = u_bar * v_bar / f
        L[0, 4] = -f * rho  - ((u_bar ** 2) / f)
        L[0, 5] = v_bar

        # Second row
        L[1, 0] = 0
        L[1, 1] = -f / Z
        L[1, 2] = v_bar / Z
        L[1, 3] = f * rho + (v_bar ** 2) / f
        L[1, 4] = -(u_bar * v_bar) / f
        L[1, 5] = -u_bar

        L_inv = np.linalg.pinv(L)
        # rospy.loginfo(f'Pseudo Inverse of Interaction Matrix L:\n{L_inv}')

        # control gain
        lambda_gain = 25

        e = np.array([u_bar, v_bar], dtype=np.float32)

         # Compute twist
        v_twist = -lambda_gain * L_inv @ e  # Shape: (6,1)

        rospy.loginfo(f'Updated Twist Vector:\n{v_twist}')


        updated_twist = Twist() # special geometry_msgs datatype digestible by ROS, float64
        
        updated_twist.angular.x = float(v_twist[3])
        updated_twist.angular.y = float(np.arctan(v_twist[4] * 0.1825 / v_twist[2]))
        updated_twist.angular.z = float(v_twist[5])

        updated_twist.linear.z = float(v_twist[2])

        # linear.x is left wheel, linear.y is right wheel
        # left is positive radian max, right is negative radian max
        # 0.1778 is vehicle length (wheel center to center), 0.1667 wheelbase (rear tires, tread center to center)
        if updated_twist.angular.y == 0:
            # z axis portrudes perpendicular from lens, so z from computed twist is desired linear velocity of end effector/camera
            updated_twist.linear.x = float(v_twist[2])
            updated_twist.linear.y = float(v_twist[2])
        else:
            turning_radius = 0.1778 / np.tan(updated_twist.angular.y) # 0.
            
            if turning_radius > 0: # turning left
                updated_twist.linear.y = float(v_twist[2]) # outer (right) wheel has max velocity
                updated_twist.linear.x = float(((turning_radius - (0.1667 / 2)) / (turning_radius + (0.1667 / 2))) * float(v_twist[2]))
            else: # turning right
                updated_twist.linear.x = float(v_twist[2]) # outer (left) wheel has max velocity
                updated_twist.linear.y = float(((turning_radius - (0.1667 / 2)) / (turning_radius + (0.1667 / 2))) * float(v_twist[2]))

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
