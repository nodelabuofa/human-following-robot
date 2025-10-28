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
        updated_twist.linear.x = float(v_twist[0])
        updated_twist.linear.y = float(v_twist[1])
        updated_twist.linear.z = float(v_twist[2])
        updated_twist.angular.x = float(v_twist[3])
        updated_twist.angular.y = float(np.arctan(v_twist[4] * 0.1825 / v_twist[2]))
        updated_twist.angular.z = float(v_twist[5])

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



















# import rospy
# import numpy as np
# from std_msgs.msg import Float32MultiArray
# from geometry_msgs.msg import Twist

# class UpdatedTwistPub:
#     def __init__(self):
#         # Initialize ROS node
#         rospy.init_node('ibvs_controller_node', anonymous=True)
        
#         # Parameters
#         self.control_gain = rospy.get_param('~lambda_gain', 0.5)  # Control gain λ
#         self.focal_length = rospy.get_param('~focal_length', 700.0)  # pixels
#         self.depth_estimate = rospy.get_param('~depth_estimate', 1.0)  # meters (Z)
        
#         # Desired corner positions (ground truth) - set during calibration
#         # Format: [x1, y1, x2, y2, x3, y3, x4, y4]
#         self.desired_corners = rospy.get_param('~desired_corners', 
#                                                [320, 200, 400, 200, 400, 280, 320, 280])
#         self.desired_corners = np.array(self.desired_corners, dtype=np.float32)
        
#         # Image center (for normalized coordinates)
#         self.image_width = rospy.get_param('~image_width', 672)
#         self.image_height = rospy.get_param('~image_height', 376)
#         self.u0 = self.image_width / 2.0
#         self.v0 = self.image_height / 2.0
        
#         # Pixel size (if known, otherwise set to 1.0)
#         self.pixel_size = rospy.get_param('~pixel_size', 1.0)
        
#         # Velocity limits for safety
#         self.max_linear_vel = rospy.get_param('~max_linear_vel', 0.5)  # m/s
#         self.max_angular_vel = rospy.get_param('~max_angular_vel', 1.0)  # rad/s
        
#         # Publisher for velocity commands
#         self.twist_pub = rospy.Publisher('updated-twist-topic', Twist, queue_size=10)
        
#         # Subscriber to ArUco corners
#         self.corner_sub = rospy.Subscriber('aruco-corners-topic', Float32MultiArray, 
#                                           self.corner_callback)
        
#         rospy.loginfo("IBVS Controller Node Initialized")
#         rospy.loginfo(f"Control gain λ: {self.control_gain}")
#         rospy.loginfo(f"Desired corners: {self.desired_corners}")
        
#     def compute_interaction_matrix(self, corners):
#         """
#         Compute the interaction matrix L for point features in IBVS
#         Based on: ṡ = Lv, where s are image features and v is camera velocity
        
#         For a point (u, v) in pixels:
#         L = [-f/Z    0      ū/Z     ūv̄/f    -(f+ū²/f)   v̄  ]
#             [ 0     -f/Z    v̄/Z   f+v̄²/f    -ūv̄/f     -ū ]
        
#         where ū = (u - u0) * pixel_size, v̄ = (v - v0) * pixel_size
#         """
#         num_points = len(corners) // 2
#         L = np.zeros((2 * num_points, 6), dtype=np.float32)
        
#         f = self.focal_length
#         Z = self.depth_estimate
        
#         for i in range(num_points):
#             u = corners[2*i]
#             v = corners[2*i + 1]
            
#             # Convert to normalized coordinates
#             u_bar = (u - self.u0) * self.pixel_size
#             v_bar = (v - self.v0) * self.pixel_size
            
#             # Fill interaction matrix for this point (2 rows)
#             # Row for u coordinate
#             L[2*i, 0] = -f / Z
#             L[2*i, 1] = 0
#             L[2*i, 2] = u_bar / Z
#             L[2*i, 3] = u_bar * v_bar / f
#             L[2*i, 4] = -(f + (u_bar**2) / f)
#             L[2*i, 5] = v_bar
            
#             # Row for v coordinate
#             L[2*i+1, 0] = 0
#             L[2*i+1, 1] = -f / Z
#             L[2*i+1, 2] = v_bar / Z
#             L[2*i+1, 3] = f + (v_bar**2) / f
#             L[2*i+1, 4] = -u_bar * v_bar / f
#             L[2*i+1, 5] = -u_bar
        
#         return L
    
#     def corner_callback(self, msg):
#         """
#         Callback for ArUco corner positions
#         Computes control law: v = -λ * L+ * e
#         """
#         try:
#             # Current corner positions
#             current_corners = np.array(msg.data, dtype=np.float32)
            
#             # Check if we have correct number of corners
#             if len(current_corners) != len(self.desired_corners):
#                 rospy.logwarn(f"Corner count mismatch: got {len(current_corners)}, expected {len(self.desired_corners)}")
#                 return
            
#             # Compute visual error: e = s - s*
#             error = current_corners - self.desired_corners
            
#             # Compute interaction matrix
#             L = self.compute_interaction_matrix(current_corners)
            
#             # Compute pseudoinverse of interaction matrix
#             # L+ = L^T (L L^T)^-1
#             L_pinv = np.linalg.pinv(L)
            
#             # Compute velocity command: v = -λ * L+ * e
#             velocity = -self.control_gain * L_pinv @ error
            
#             # velocity is [vx, vy, vz, wx, wy, wz]
#             # For Ackermann steering ground robot, we primarily use:
#             # - vx (forward velocity)
#             # - wz (angular velocity / yaw rate)
            
#             # Create Twist message
#             twist_msg = Twist()
#             twist_msg.linear.x = np.clip(velocity[0], -self.max_linear_vel, self.max_linear_vel)
#             twist_msg.linear.y = 0.0  # No lateral motion for Ackermann
#             twist_msg.linear.z = 0.0  # No vertical motion
            
#             twist_msg.angular.x = 0.0
#             twist_msg.angular.y = 0.0
#             twist_msg.angular.z = np.clip(velocity[5], -self.max_angular_vel, self.max_angular_vel)
            
#             # Publish velocity command
#             self.twist_pub.publish(twist_msg)
            
#             # Log error magnitude
#             error_norm = np.linalg.norm(error)
#             rospy.loginfo_throttle(1.0, f"Visual error: {error_norm:.2f} pixels, "
#                                        f"Cmd - linear: {twist_msg.linear.x:.3f} m/s, "
#                                        f"angular: {twist_msg.angular.z:.3f} rad/s")
            
#         except Exception as e:
#             rospy.logerr(f"Error in corner callback: {str(e)}")
    
#     def run(self):
#         """Keep the node running"""
#         rospy.spin()

# if __name__ == '__main__':
#     try:
#         node = IBVSControllerNode()
#         node.run()
#     except rospy.ROSInterruptException:
#         pass
