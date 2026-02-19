#!/usr/bin/env python3

### Detects QR/ArUco codes and publishes corner pixel positions and corresponding depths to 'aruco_corners_topic'


import rospy # ROS for python
import cv2   # OpenCV, finds ArUco codes
import numpy as np
from std_msgs.msg import Float32MultiArray, Header # for ROS topic transportation
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from human_following_robot import library as lib # my custom library(.py) file with functions and config params


## GLOBALS/CONFIG
camera_config = lib.CameraConfig()
cv_bridge = CvBridge() # converts color/RGB and depth arrays to digestible format for OpenCV

desired_corners = lib.DESIRED_CORNERS

aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_50) # 5x5 black/white square size ArUco/QR code, ID < 50
aruco_params = lib.get_aruco_parameters()

depth_image = None
color_image = None

# global to allow helper functions to access
aruco_corners_pub = None
annotated_image_pub = None


def main():
    global aruco_corners_pub
    global annotated_image_pub
    rospy.init_node('aruco_corners_finder')

    # constantly retrieves and passes new color and depth images to these two 'callback' function
    rospy.Subscriber('/zedm/zed_node/left/image_rect_color', Image, aruco_finder_callback)
    rospy.Subscriber('/zedm/zed_node/depth/depth_registered', Image, depth_callback)

    # constantly uploads pixel position and depth of corners, plus video feed, 
    aruco_corners_pub = rospy.Publisher('aruco_corners_topic', Float32MultiArray, queue_size=10) 
    annotated_image_pub = rospy.Publisher('annotated_image_topic', Image, queue_size=10)

    rospy.loginfo("ArUco finder running")
    rospy.spin() # run forever


## CALLBACKS (when you get mail, ring my doorbell)
def aruco_finder_callback(msg):
    global color_image, aruco_dict, aruco_corners_pub, annotated_image_pub
    color_image = cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8') # openCV accepts 8 bit BGR, not RGB

    corners, ids, rejected = cv2.aruco.detectMarkers( # corners is pixel position, ID identifies ArUco code, rejected idk
                image = color_image, 
                dictionary=aruco_dict, 
                parameters = aruco_params
            )
    if depth_image is None: # hasn't loaded yet
        return
    packaged_corners = lib.package_corners(corners, depth_image) # outputs [x1, y1, d1, ..., x4, y4, d4]
    publish_corners(packaged_corners, aruco_corners_pub)

    # draw current and desired ArUco code positions on image and upload video feed
    absolute_desired_corners = lib.centered_to_corner_origin(desired_corners, camera_config)
    annotated_image = lib.annotate_image(color_image, corners, ids, absolute_desired_corners)
    publish_annotated_image(annotated_image, annotated_image_pub)

    cv2.imshow("Image", annotated_image)
    cv2.waitKey(1)



def depth_callback(msg):
    global depth_image
    try:
        depth_image = cv_bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')
        if depth_image is None:
            rospy.loginfo_once("Depth image not available yet, skipping frame") # gets impatient during bootup
            return
    except Exception as e:
        rospy.logerr(f"Depth image conversion failed: {e}")



## HELPER FUNCTIONS (used inside callbacks, involve ROS so couldn't be put in library)

def publish_corners(packaged_corners_param, aruco_corners_pub_param):
    aruco_corners_msg = Float32MultiArray()
    aruco_corners_msg.data = packaged_corners_param
    aruco_corners_pub_param.publish(aruco_corners_msg)

def publish_annotated_image(annotated_image_param, annotated_image_pub_param):
    annotated_image_msg = cv_bridge.cv2_to_imgmsg(annotated_image_param, "bgr8")
    annotated_image_pub_param.publish(annotated_image_msg)



try:
    main()
except rospy.ROSInterruptException:
    pass

