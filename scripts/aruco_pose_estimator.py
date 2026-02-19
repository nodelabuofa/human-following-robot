#!/usr/bin/env python3

"""
ArUco Pose Estimation MWP
Detects ArUco markers and publishes their 3D pose in the Camera Frame.
"""
import rospy
import cv2
import numpy as np
import math
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import Float32MultiArray

MARKER_SIZE = 0.182  # meters
MARKER_ID_TO_DETECT = 1

class ArucoPoseNode:
    def __init__(self):
        rospy.init_node('aruco_pose_estimator')
        
        # 1. Camera Intrinsics
        fx = 749.6
        fy = 749.6
        cx = 466.7
        cy = 268.1
        
        self.camera_matrix = np.array([
            [fx, 0, cx],
            [0, fy, cy],
            [0, 0, 1.0]
        ], dtype=float)
        
        self.dist_coeffs = np.zeros((5,1)) 

        # 2. ArUco Setup
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_50)
        self.aruco_params = cv2.aruco.DetectorParameters_create()
        self.aruco_params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        self.aruco_params.cornerRefinementWinSize = 15

        # 3. ROS Setup
        self.bridge = CvBridge()
        self.annotated_image_pub = rospy.Publisher('aruco_debug_image', Image, queue_size=1)

        self.pose_pub = rospy.Publisher('/aruco_pose_topic', Float32MultiArray, queue_size=1)
        
        self.rgb_sub = rospy.Subscriber(
            '/zedm/zed_node/left/image_rect_color', 
            Image, 
            self.rgb_callback
        )
        rospy.loginfo("ArUco Pose Estimation Started")

    def is_rotation_matrix(self, R):
        Rt = np.transpose(R)
        shouldBeIdentity = np.dot(Rt, R)
        I = np.identity(3, dtype=R.dtype)
        n = np.linalg.norm(I - shouldBeIdentity)
        return n < 1e-6

    def rotation_matrix_to_euler_angles(self, R):
        # Calculates rotation matrix to euler angles
        # The result is the same as MATLAB except the order
        # of the euler angles ( x and z are swapped ).
        assert(self.is_rotation_matrix(R))

        sy = math.sqrt(R[0,0] * R[0,0] +  R[1,0] * R[1,0])
        singular = sy < 1e-6

        if not singular:
            x = math.atan2(R[2,1] , R[2,2])
            y = math.atan2(-R[2,0], sy)
            z = math.atan2(R[1,0], R[0,0])
        else:
            x = math.atan2(-R[1,2], R[1,1])
            y = math.atan2(-R[2,0], sy)
            z = 0

        # Returns Roll (X), Pitch (Y), Yaw (Z) in radians
        # For Camera Frame:
        # X = Pitch (Tilt up/down)
        # Y = Heading (Pan left/right)
        # Z = Roll (Tilt sideways)
        return np.array([x, y, z])

    def rgb_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            corners, ids, _ = cv2.aruco.detectMarkers(
                cv_image, 
                self.aruco_dict, 
                parameters=self.aruco_params
            )

            if ids is not None and MARKER_ID_TO_DETECT in ids:
                cv2.aruco.drawDetectedMarkers(cv_image, corners, ids)
                idx = np.where(ids == MARKER_ID_TO_DETECT)[0][0]
                target_corners = corners[idx]

                # --- ESTIMATE POSE ---
                rvecs, tvecs, _obj_points = cv2.aruco.estimatePoseSingleMarkers(
                    target_corners, 
                    MARKER_SIZE, 
                    self.camera_matrix, 
                    self.dist_coeffs
                )
                
                rvec = rvecs[0][0]
                tvec = tvecs[0][0]

                # --- CALCULATE HEADING ---
                # 1. Convert rotation vector (rvec) to rotation matrix
                R_mat, _ = cv2.Rodrigues(rvec)
                
                # 2. Convert matrix to Euler angles (radians)
                # In standard OpenCV Cam frame: Y-axis is vertical (down), so Y-rotation is Heading
                angles_rad = self.rotation_matrix_to_euler_angles(R_mat)
                angles_deg = np.degrees(angles_rad)
                
                pitch_x = angles_deg[0]
                heading_y = angles_deg[1]
                roll_z = angles_deg[2]

                # Print Pose + Heading
                rospy.loginfo_throttle(0.5, 
                    f"Pose [x, y, z]: {tvec[0]:.2f}, {tvec[1]:.2f}, {tvec[2]:.2f} | "
                    f"Yaw/Heading: {heading_y:.1f} deg"
                )

                pose_msg = Float32MultiArray()
                pose_msg.data = [tvec[0], tvec[1], heading_y] # X, Y, Z, Heading
                self.pose_pub.publish(pose_msg)

                # --- VISUALIZATION ---
                cv2.aruco.drawDetectedMarkers(cv_image, corners, ids)
                try:
                    cv2.aruco.drawAxis(cv_image, self.camera_matrix, self.dist_coeffs, rvec, tvec, 0.1)
                except AttributeError:
                    cv2.drawFrameAxes(cv_image, self.camera_matrix, self.dist_coeffs, rvec, tvec, 0.1)

            try:
                cv2.imshow("Image", cv_image)
                cv2.waitKey(1)
            except Exception as e:
                rospy.logerr(f"Img Pub Fail: {e}")

        except Exception as e:
            rospy.logerr(f"Processing Fail: {e}")

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    node = ArucoPoseNode()
    node.run()