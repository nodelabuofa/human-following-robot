#!/usr/bin/env python3

"""
ArUco Marker Detection Node
Detects ArUco markers and publishes corner positions to 'aruco-corners-topic'
"""
import rospy
import cv2
import numpy as np
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class ArucoCornersPub:
    def __init__(self): # constructor
        rospy.init_node('aruco_corners_pub')

        # aruco corners publisher
        self.aruco_corners_pub = rospy.Publisher('aruco_corners_topic', Float32MultiArray, queue_size=10)
        self.annotated_image_pub = rospy.Publisher('annotated_image_topic', Image, queue_size=10)

        # zed mini subscriber
        self.RGB_sub = rospy.Subscriber('/zedm/zed_node/left/image_rect_color', Image, self.RGB_callback) 
        rospy.loginfo("Subscribed to ZED Mini RGB feed")
        self.depth_sub = rospy.Subscriber('/zedm/zed_node/depth/depth_registered', Image, self.depth_callback)

        # instantiates CvBridge object, that converts RGB to format digestible by OpenCV
        self.bridge = CvBridge()

        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_50) # specifies I'm using 5x5 Aruco code, ID between 0-50
        self.aruco_params = cv2.aruco.DetectorParameters_create()
        self.aruco_params.adaptiveThreshWinSizeMin = 3
        self.aruco_params.adaptiveThreshWinSizeMax = 23
        self.aruco_params.adaptiveThreshWinSizeStep = 10
        self.aruco_params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        self.aruco_params.minMarkerDistanceRate = 0.025
        # Can also adjust errorCorrectionRate, minMarkerPerimeterRate, etc.

        # ArUco must be in particular orientation
        self.desired_corners = {
            0: {'u': -100.0, 'v': 150.0},  # Target for top left corner
            1: {'u': 90, 'v': 150.0},   # Target for top right corner
            2: {'u': 90, 'v': -50},    # Target for bottom right corner
            3: {'u': -100.0, 'v': -50.0}    # Target for bottom left corner
        }

    def depth_callback(self, msg):
        try:
            # convert to OpenCV float32 array (depth in meters)
            self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1') # 32 bit float, one channel (RGB is 3 high, depth is 1 high)
        except Exception as e:
            rospy.logerr(f"Depth image conversion failed: {e}")



    def RGB_callback(self, msg):
        # rospy.loginfo("Received an image message")
        # rospy.loginfo(f"Image frame id: {msg.header.frame_id}, height: {msg.height}, width: {msg.width}")
        # if self.depth_image is None:
        #     return # wait until depth frame available

        f = 366 # focal length, UPDATE
        rho = 0.000002 # physical individual square pixel sensor width AND height conversion
        cx = 315
        cy = 178

        desired_pixel_corners = []

        for i in range(4): # 0, 1, 2, 3
            u_desired = self.desired_corners[i]['u']
            v_desired = self.desired_corners[i]['v']

            # only for plotting outline of desired servo position
            u_pixel = int(u_desired + cx)
            v_pixel = int(360 - (v_desired + cy))
            desired_pixel_corners.append([u_pixel, v_pixel])

        pts = np.array(desired_pixel_corners, np.int32)
        pts = pts.reshape((-1, 1, 2))

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            # rospy.loginfo(f"Converted image shape: {cv_image.shape}, dtype: {cv_image.dtype}")
            cv2.polylines(cv_image, [pts], isClosed=True, color=(0, 255, 0), thickness=2)

            corners, ids, rejected = cv2.aruco.detectMarkers(
                cv_image, 
                self.aruco_dict, 
                parameters=self.aruco_params
            )

            for i, corner in enumerate(corners):
                corner_coordinates = corner.reshape((4, 2))
                corner_depths_array = [] # looks like [(x1, y1, d1), ... , (x4, y4, d4)]

                for corner in corner_coordinates:
                    x, y = int(corner[0]), int(corner[1])
                    d = float(self.depth_image[y, x]) # (y,x) because (row, column)
                    corner_depths_array.append((x,y,d))

                # flatten (x,y) coordinates to single list [x1, y1, x2, y2, x3, y3, x4, y4]
                flat_corners = np.array(corner_depths_array).flatten()
                flat_corners_list = flat_corners.tolist()

                # publish to ROS
                aruco_corners_msg = Float32MultiArray()
                aruco_corners_msg.data = flat_corners_list

                self.aruco_corners_pub.publish(aruco_corners_msg)

                # format corners to 2 decimal places
                formatted_corners = ', '.join([f"{val:.2f}" if isinstance(val, float) else str(val) for val in flat_corners])
                # print(f"Marker ID {ids[i][0]} corners: [{formatted_corners}]")

            # Draw detected markers only if any are found
            if ids is not None and len(ids) > 0:
                cv2.aruco.drawDetectedMarkers(cv_image, corners, ids)
            #     rospy.loginfo("YES: ArUco marker detected")
            # else:
            #     rospy.loginfo("NO: No marker detected")

            pts = np.array(desired_pixel_corners, np.int32)
            pts = pts.reshape((-1, 1, 2))
            cv2.polylines(cv_image, [pts], isClosed=True, color=(0, 255, 0), thickness=2)

            # Visualize after drawing (or not drawing) the markers
            cv2.imshow("Image", cv_image)
            cv2.waitKey(1)

            try:
                annotated_image_msg = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
                self.annotated_image_pub.publish(annotated_image_msg)
            except Exception as e:
                rospy.logerr(f"Failed to convert annotated image: {e}")

        except Exception as e:
            rospy.logerr(f"Failed to convert image: {e}")

    
    def run(self):
        rospy.spin()


if __name__ == '__main__':
    try:
        node = ArucoCornersPub()
        node.run()
    except rospy.ROSInterruptException:
        pass
